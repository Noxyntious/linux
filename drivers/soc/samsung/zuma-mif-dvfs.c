// SPDX-License-Identifier: GPL-2.0-only
/*
 * Memory interface (MIF) and internal bus (INT) frequency scaling for the
 * Google Tensor G3 (zuma).
 *
 * Out of the bootloader both buses sit at a fixed rate (MIF 2028 MHz, INT
 * 356 MHz on the Pixel 8 Pro) whatever the load: too fast for an idle phone,
 * too slow for the GPU at full tilt.  Android scales them with a devfreq on
 * the bus performance counters plus per-master bandwidth votes, neither of
 * which exists here.  This driver follows the masters that drive memory
 * traffic instead: each CPU cluster's and the GPU's current frequency maps to
 * the MIF rate it needs, the highest demand wins, and INT follows MIF through
 * the vendor's MIF->INT pairing.  Both rates are ACPM DVFS domains, set with
 * clk_set_rate().
 *
 * Raising is immediate (a cpufreq/devfreq transition schedules the work);
 * lowering waits until demand has stayed lower for a hold time, so a busy
 * scheduler does not bounce the DRAM clock (each switch costs a few tens of
 * microseconds of memory traffic stall).
 *
 * Over system sleep the buses are put back at the boot rate, the state the
 * suspend path was validated with; resume re-evaluates.
 *
 * Tunables in /sys/module/zuma_mif_dvfs/parameters/: enable, min_khz,
 * max_khz (take effect at once), hold_ms; cur_mif_khz/cur_int_khz/switches
 * are read-only.  enable=0 puts the buses back at the boot rate.
 */

#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/devfreq.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/workqueue.h>

struct zmif_step {
	u32 upto_khz;	/* master frequency up to and including */
	u32 mif_khz;	/* MIF rate it needs */
};

/*
 * MIF ladder of the Android devfreq (kHz): 421 546 676 845 1014 1352 1539
 * 1716 2028 2288 2730 3172 3744.  The demand tables are this port's: a core
 * at its top OPP streaming from DRAM can use roughly the MIF the vendor's
 * CPU->MIF DVFS-manager constraints grant it, a lightly clocked one cannot.
 */
static const struct zmif_step zmif_little[] = {	/* A510, 324..1704 MHz */
	{  955000,  421000 },
	{ 1328000,  845000 },
	{ U32_MAX, 1352000 },
};

static const struct zmif_step zmif_mid[] = {	/* A715, 402..2367 MHz */
	{  910000,  421000 },
	{ 1328000,  845000 },
	{ 1572000, 1352000 },
	{ 1945000, 2028000 },
	{ U32_MAX, 2730000 },
};

static const struct zmif_step zmif_big[] = {	/* X3, 500..2914 MHz */
	{  880000,  421000 },
	{ 1164000,  845000 },
	{ 1557000, 1539000 },
	{ 2049000, 2028000 },
	{ 2556000, 2730000 },
	{ U32_MAX, 3172000 },
};

static const struct zmif_step zmif_gpu[] = {	/* Mali-G715, 150..890 MHz */
	{  151000,  421000 },
	{  302000,  845000 },
	{  401000, 1352000 },
	{  572000, 2028000 },
	{  701000, 2730000 },
	{ U32_MAX, 3744000 },
};

static const u32 zmif_ladder[] = {
	421000, 546000, 676000, 845000, 1014000, 1352000, 1539000,
	1716000, 2028000, 2288000, 2730000, 3172000, 3744000,
};

/* Vendor mif_int_map (MIF kHz at or above -> INT kHz). */
static const struct zmif_step zmif_int_map[] = {
	{ 3744000, 664000 },
	{ 3172000, 533000 },
	{ 2028000, 356000 },
	{ 1539000, 178000 },
};

#define ZMIF_INT_MIN_KHZ	178000

enum { ZMIF_LITTLE, ZMIF_MID, ZMIF_BIG, ZMIF_GPU, ZMIF_MASTERS };

static const struct zmif_step *const zmif_tables[ZMIF_MASTERS] = {
	zmif_little, zmif_mid, zmif_big, zmif_gpu,
};

static const unsigned int zmif_table_len[ZMIF_MASTERS] = {
	ARRAY_SIZE(zmif_little), ARRAY_SIZE(zmif_mid),
	ARRAY_SIZE(zmif_big), ARRAY_SIZE(zmif_gpu),
};

struct zmif {
	struct device *dev;
	struct clk *mif;
	struct clk *intc;
	struct devfreq *gpu;
	struct notifier_block cpufreq_nb;
	struct notifier_block gpu_nb;
	struct delayed_work work;
	struct mutex lock;
	u32 demand[ZMIF_MASTERS];	/* MIF kHz each master wants */
	unsigned long boot_mif, boot_int;
	unsigned long last_high;	/* jiffies the current rate was last needed */
	bool suspended;
};

static int zmif_param_set_uint(const char *val, const struct kernel_param *kp);
static int zmif_param_set_bool(const char *val, const struct kernel_param *kp);

static const struct kernel_param_ops zmif_uint_ops = {
	.set = zmif_param_set_uint,
	.get = param_get_uint,
};

static const struct kernel_param_ops zmif_bool_ops = {
	.set = zmif_param_set_bool,
	.get = param_get_bool,
};

/* writing any of these re-evaluates the rate at once */
static bool zmif_enable = true;
module_param_cb(enable, &zmif_bool_ops, &zmif_enable, 0644);
static unsigned int zmif_min_khz = 421000;
module_param_cb(min_khz, &zmif_uint_ops, &zmif_min_khz, 0644);
static unsigned int zmif_max_khz = 3744000;
module_param_cb(max_khz, &zmif_uint_ops, &zmif_max_khz, 0644);
static unsigned int zmif_hold_ms = 100;
module_param_named(hold_ms, zmif_hold_ms, uint, 0644);
static unsigned int zmif_cur_mif;
module_param_named(cur_mif_khz, zmif_cur_mif, uint, 0444);
static unsigned int zmif_cur_int;
module_param_named(cur_int_khz, zmif_cur_int, uint, 0444);
static unsigned int zmif_switches;
module_param_named(switches, zmif_switches, uint, 0444);

static struct zmif *zmif_inst;

static u32 zmif_lookup(unsigned int master, u32 khz)
{
	const struct zmif_step *t = zmif_tables[master];
	unsigned int i;

	for (i = 0; i < zmif_table_len[master]; i++)
		if (khz <= t[i].upto_khz)
			return t[i].mif_khz;
	return t[zmif_table_len[master] - 1].mif_khz;
}

/* Smallest ladder rate >= khz, clamped to the ladder. */
static u32 zmif_ceil(u32 khz)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(zmif_ladder); i++)
		if (zmif_ladder[i] >= khz)
			return zmif_ladder[i];
	return zmif_ladder[ARRAY_SIZE(zmif_ladder) - 1];
}

static u32 zmif_int_for(u32 mif_khz)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(zmif_int_map); i++)
		if (mif_khz >= zmif_int_map[i].upto_khz)
			return zmif_int_map[i].mif_khz;
	return ZMIF_INT_MIN_KHZ;
}

static int zmif_set(struct zmif *z, unsigned long mif_hz, unsigned long int_hz)
{
	unsigned long cur_mif = clk_get_rate(z->mif);
	int ret;

	/*
	 * Order the pair so INT never runs ahead of a MIF it is paired with
	 * from the vendor map: up = MIF first, down = INT first.
	 */
	if (mif_hz >= cur_mif) {
		ret = clk_set_rate(z->mif, mif_hz);
		if (!ret)
			ret = clk_set_rate(z->intc, int_hz);
	} else {
		ret = clk_set_rate(z->intc, int_hz);
		if (!ret)
			ret = clk_set_rate(z->mif, mif_hz);
	}
	if (ret) {
		dev_warn_ratelimited(z->dev, "MIF %lu / INT %lu kHz: %d\n",
				     mif_hz / 1000, int_hz / 1000, ret);
		return ret;
	}

	zmif_cur_mif = clk_get_rate(z->mif) / 1000;
	zmif_cur_int = clk_get_rate(z->intc) / 1000;
	zmif_switches++;
	return 0;
}

static void zmif_work(struct work_struct *work)
{
	struct zmif *z = container_of(to_delayed_work(work), struct zmif, work);
	u32 want = 0, lo, hi;
	unsigned int i;

	mutex_lock(&z->lock);
	if (z->suspended)
		goto out;

	if (!READ_ONCE(zmif_enable)) {
		if (zmif_cur_mif != z->boot_mif / 1000)
			zmif_set(z, z->boot_mif, z->boot_int);
		goto out;
	}

	for (i = 0; i < ZMIF_MASTERS; i++)
		want = max(want, READ_ONCE(z->demand[i]));

	lo = zmif_ceil(READ_ONCE(zmif_min_khz));
	hi = max(zmif_ceil(READ_ONCE(zmif_max_khz)), lo);
	want = clamp(zmif_ceil(want), lo, hi);

	if (want >= zmif_cur_mif) {
		z->last_high = jiffies;
		if (want > zmif_cur_mif)
			zmif_set(z, want * 1000UL, zmif_int_for(want) * 1000UL);
	} else {
		unsigned long due = READ_ONCE(z->last_high) +
				    msecs_to_jiffies(READ_ONCE(zmif_hold_ms));

		if (time_before(jiffies, due)) {
			/* look again when the hold runs out */
			mod_delayed_work(system_power_efficient_wq, &z->work,
					 due - jiffies);
			goto out;
		}
		zmif_set(z, want * 1000UL, zmif_int_for(want) * 1000UL);
		z->last_high = jiffies;
	}
out:
	mutex_unlock(&z->lock);
}

static void zmif_update(struct zmif *z, unsigned int master, u32 khz)
{
	u32 d = zmif_lookup(master, khz);

	/* the current rate is still wanted: restart the hold */
	if (d >= READ_ONCE(zmif_cur_mif))
		WRITE_ONCE(z->last_high, jiffies);
	if (READ_ONCE(z->demand[master]) == d)
		return;
	WRITE_ONCE(z->demand[master], d);
	/* raise now; a lower demand is weighed (and held) by the work */
	mod_delayed_work(system_power_efficient_wq, &z->work, 0);
}

static int zmif_cpufreq_notify(struct notifier_block *nb, unsigned long event,
			       void *data)
{
	struct zmif *z = container_of(nb, struct zmif, cpufreq_nb);
	struct cpufreq_freqs *f = data;
	unsigned int cpu = f->policy->cpu, master;

	if (event != CPUFREQ_POSTCHANGE)
		return NOTIFY_OK;

	master = cpu >= 8 ? ZMIF_BIG : cpu >= 4 ? ZMIF_MID : ZMIF_LITTLE;
	zmif_update(z, master, f->new);
	return NOTIFY_OK;
}

static int zmif_gpu_notify(struct notifier_block *nb, unsigned long event,
			   void *data)
{
	struct zmif *z = container_of(nb, struct zmif, gpu_nb);
	struct devfreq_freqs *f = data;

	if (event != DEVFREQ_POSTCHANGE)
		return NOTIFY_OK;

	zmif_update(z, ZMIF_GPU, f->new / 1000);
	return NOTIFY_OK;
}

static void zmif_kick_work(void)
{
	if (zmif_inst)
		mod_delayed_work(system_power_efficient_wq, &zmif_inst->work, 0);
}

static int zmif_param_set_uint(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_uint(val, kp);

	if (!ret)
		zmif_kick_work();
	return ret;
}

static int zmif_param_set_bool(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_bool(val, kp);

	if (!ret)
		zmif_kick_work();
	return ret;
}

static int zmif_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cpufreq_policy *policy;
	struct zmif *z;
	int ret, cpu;

	z = devm_kzalloc(dev, sizeof(*z), GFP_KERNEL);
	if (!z)
		return -ENOMEM;
	z->dev = dev;
	mutex_init(&z->lock);
	INIT_DELAYED_WORK(&z->work, zmif_work);

	z->mif = devm_clk_get(dev, "mif");
	if (IS_ERR(z->mif))
		return dev_err_probe(dev, PTR_ERR(z->mif), "no MIF clock\n");
	z->intc = devm_clk_get(dev, "int");
	if (IS_ERR(z->intc))
		return dev_err_probe(dev, PTR_ERR(z->intc), "no INT clock\n");

	/* The GPU is the biggest DRAM user; wait for its devfreq. */
	z->gpu = devfreq_get_devfreq_by_phandle(dev, "google,gpu-devfreq", 0);
	if (IS_ERR(z->gpu)) {
		ret = PTR_ERR(z->gpu);
		if (ret == -ENODEV)
			ret = -EPROBE_DEFER;
		return dev_err_probe(dev, ret, "GPU devfreq not there yet\n");
	}

	/* every cluster must have its cpufreq policy before we follow it */
	for (cpu = 0; cpu <= 8; cpu += 4) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			return dev_err_probe(dev, -EPROBE_DEFER,
					     "cpufreq policy %d not there yet\n",
					     cpu);
		z->demand[cpu >= 8 ? ZMIF_BIG : cpu >= 4 ? ZMIF_MID :
			  ZMIF_LITTLE] = zmif_lookup(cpu >= 8 ? ZMIF_BIG :
						     cpu >= 4 ? ZMIF_MID :
						     ZMIF_LITTLE, policy->cur);
		cpufreq_cpu_put(policy);
	}

	z->boot_mif = clk_get_rate(z->mif);
	z->boot_int = clk_get_rate(z->intc);
	if (!z->boot_mif || !z->boot_int)
		return dev_err_probe(dev, -EIO, "cannot read the bus rates\n");
	zmif_cur_mif = z->boot_mif / 1000;
	zmif_cur_int = z->boot_int / 1000;
	z->last_high = jiffies;
	platform_set_drvdata(pdev, z);

	z->cpufreq_nb.notifier_call = zmif_cpufreq_notify;
	ret = cpufreq_register_notifier(&z->cpufreq_nb,
					CPUFREQ_TRANSITION_NOTIFIER);
	if (ret)
		return dev_err_probe(dev, ret, "cpufreq notifier\n");

	z->gpu_nb.notifier_call = zmif_gpu_notify;
	ret = devfreq_register_notifier(z->gpu, &z->gpu_nb,
					DEVFREQ_TRANSITION_NOTIFIER);
	if (ret) {
		cpufreq_unregister_notifier(&z->cpufreq_nb,
					    CPUFREQ_TRANSITION_NOTIFIER);
		return dev_err_probe(dev, ret, "GPU devfreq notifier\n");
	}

	zmif_inst = z;
	dev_info(dev, "following CPU + GPU, boot MIF %lu INT %lu kHz\n",
		 z->boot_mif / 1000, z->boot_int / 1000);
	mod_delayed_work(system_power_efficient_wq, &z->work, 0);
	return 0;
}

static int zmif_suspend(struct device *dev)
{
	struct zmif *z = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&z->work);
	mutex_lock(&z->lock);
	z->suspended = true;
	if (zmif_cur_mif != z->boot_mif / 1000)
		zmif_set(z, z->boot_mif, z->boot_int);
	mutex_unlock(&z->lock);
	return 0;
}

static int zmif_resume(struct device *dev)
{
	struct zmif *z = dev_get_drvdata(dev);

	mutex_lock(&z->lock);
	z->suspended = false;
	/* ACPM may have moved the rates over the sleep: read them back */
	zmif_cur_mif = clk_get_rate(z->mif) / 1000;
	zmif_cur_int = clk_get_rate(z->intc) / 1000;
	z->last_high = jiffies;
	mutex_unlock(&z->lock);
	mod_delayed_work(system_power_efficient_wq, &z->work, 0);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(zmif_pm_ops, zmif_suspend, zmif_resume);

static const struct of_device_id zmif_of_match[] = {
	{ .compatible = "google,zuma-mif-dvfs" },
	{ }
};
MODULE_DEVICE_TABLE(of, zmif_of_match);

static struct platform_driver zmif_driver = {
	.probe = zmif_probe,
	.driver = {
		.name = "zuma-mif-dvfs",
		.of_match_table = zmif_of_match,
		.pm = pm_sleep_ptr(&zmif_pm_ops),
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(zmif_driver);

MODULE_DESCRIPTION("Tensor G3 MIF/INT bus frequency scaling");
MODULE_LICENSE("GPL");
