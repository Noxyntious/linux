// SPDX-License-Identifier: GPL-2.0
/*
 * Cirrus Logic CS40L26 haptic driver, ROM-firmware mode.
 *
 * The CS40L26 boots a ROM firmware on its HALO DSP that already holds a
 * small waveform table (39 effects) and a buzz generator, and plays them
 * when a trigger index is posted to its mailbox. This driver drives that
 * ROM firmware only: no RAM firmware/wavetable load (cs40l26.wmfw/.bin) yet.
 *
 * Effects (input force feedback):
 *  - FF_PERIODIC with an FF_CUSTOM waveform, custom_data = { bank, index },
 *    bank 1 = ROM (the downstream and CS40L50 convention), index 0..38;
 *  - FF_RUMBLE and the other FF_PERIODIC waveforms: the ROM buzz generator,
 *    re-triggered until the effect's length has run out.
 *
 * Register map and mailbox protocol from the downstream Google/Cirrus
 * cs40l26 driver (android kernel/google-modules/amplifiers).
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/workqueue.h>

#define CS40L26_DEVID			0x0
#define CS40L26_REVID			0x4
#define CS40L26_DSP_VIRTUAL1_MBOX_1	0x13020

#define CS40L26_DEVID_A			0x40A260
#define CS40L26_DEVID_B			0x40A26B
#define CS40L26_DEVID_L27_A		0x40A270
#define CS40L26_DEVID_L27_B		0x40A27B
#define CS40L26_REVID_B2		0xB2

/* ROM firmware state, per silicon revision */
#define CS40L26_A1_DSP_HALO_STATE	0x02800fa8
#define CS40L26_A1_PM_CUR_STATE		0x02800370
#define CS40L26_B2_DSP_HALO_STATE	0x02806af8
#define CS40L26_B2_PM_CUR_STATE		0x02801f98
#define CS40L26_DSP_HALO_STATE_RUN	2
#define CS40L26_A1_PM_STATE_LOCKS	0x02800378
#define CS40L26_DSP_LOCK3_OFFSET	8
#define CS40L26_DSP_LOCK3		BIT(1)
#define CS40L26_DSP_STATE_MASK		GENMASK(7, 0)
#define CS40L26_DSP_STATE_SHUTDOWN	1
#define CS40L26_DSP_STATE_STANDBY	2
#define CS40L26_DSP_STATE_ACTIVE	3
#define CS40L26_MBOX_PREVENT_HIBER	0x02000003

/* Boost and errata registers (rev A1/B0/B1) */
#define CS40L26_TEST_KEY_CTRL		0x40
#define CS40L26_GLOBAL_ENABLES		0x2014
#define CS40L26_GLOBAL_EN		BIT(0)
#define CS40L26_ERROR_RELEASE		0x2034
/* ERROR_RELEASE bits (downstream enum cs40l26_err_rls) */
#define CS40L26_AMP_SHORT_ERR_RLS	1
#define CS40L26_BST_SHORT_ERR_RLS	2
#define CS40L26_BST_OVP_ERR_RLS		3
#define CS40L26_BST_UVP_ERR_RLS		4
#define CS40L26_TEMP_ERR_RLS		6
/* IRQ1 latched-event registers: write 1 to clear */
#define CS40L26_IRQ1_EINT_1		0x10010
#define CS40L26_IRQ1_EINT_2		0x10014
#define CS40L26_IRQ1_STS_1		0x10090
#define CS40L26_BST_OVP_ERR		BIT(20)
#define CS40L26_BST_DCM_UVP_ERR		BIT(21)
#define CS40L26_BST_SHORT_ERR_INT	BIT(22)
#define CS40L26_BST_IPK_FLAG		BIT(23)
#define CS40L26_TEMP_ERR		BIT(26)
#define CS40L26_AMP_ERR			BIT(27)
#define CS40L26_ERR_INTS		(CS40L26_BST_OVP_ERR | CS40L26_BST_DCM_UVP_ERR | \
					 CS40L26_BST_SHORT_ERR_INT | CS40L26_TEMP_ERR | \
					 CS40L26_AMP_ERR)
#define CS40L26_BST_SHORT_ERR		BIT(2)
#define CS40L26_PLL_REFCLK_DETECT_0	0x2c28
#define CS40L26_VBST_CTL_1		0x3800
#define CS40L26_BST_CTL_MASK		GENMASK(7, 0)
#define CS40L26_VBST_CTL_2		0x3804
#define CS40L26_BST_CTL_SEL_MASK	GENMASK(1, 0)
#define CS40L26_BST_CTL_LIM_EN		BIT(2)
#define CS40L26_BST_IPK_CTL		0x3808
#define CS40L26_TEST_LBST		0x391c
#define CS40L26_DISABLE_EXPL_MODE	0x0100c080
#define CS40L26_BST_UV_MIN		2500000
#define CS40L26_BST_UV_MAX		11000000
#define CS40L26_BST_UV_STEP		50000
#define CS40L26_BST_IPK_UA_MIN		800000
#define CS40L26_BST_IPK_UA_MAX		4800000
#define CS40L26_BST_IPK_UA_STEP		50000

#define CS40L26_ROM_INDEX_START		0x01800000
#define CS40L26_ROM_NUM_WAVES		39
#define CS40L26_BUZZGEN_INDEX		0x01800080
#define CS40L26_STOP_PLAYBACK		0x05000000

#define CS40L26_ROM_BANK_ID		1
#define CS40L26_CUSTOM_DATA_SIZE	2

#define CS40L26_RETRIES			20
#define CS40L26_RETRY_US		1000
#define CS40L26_RUMBLE_PERIOD_MS	50
#define CS40L26_MAX_EFFECTS		16

struct cs40l26_effect {
	u32 trigger;
	bool repeat;		/* buzz: re-trigger for the effect's length */
	u16 length_ms;
};

struct cs40l26 {
	struct device *dev;
	struct regmap *regmap;
	struct input_dev *input;
	struct gpio_desc *reset;
	struct mutex lock;	/* serialises mailbox traffic */
	u32 revid;
	u32 halo_state_reg;
	u32 bst_ipk;		/* BST_IPK_CTL as programmed: a reset chip reads other */
	unsigned int err_releases;
	struct dentry *dbgfs;

	struct cs40l26_effect effects[CS40L26_MAX_EFFECTS];

	/* playback requests from atomic context */
	spinlock_t req_lock;
	int req_id;
	bool req_on;
	struct work_struct play_work;
	struct delayed_work repeat_work;
	unsigned long repeat_end;
	u32 repeat_trigger;
};

static const struct regmap_config cs40l26_regmap = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.max_register = 0x3ffffff,
};

/* The chip NAKs the first transaction while hibernating; it wakes on it. */
static int cs40l26_read(struct cs40l26 *cs, u32 reg, u32 *val)
{
	int i, ret;

	for (i = 0; i < CS40L26_RETRIES; i++) {
		ret = regmap_read(cs->regmap, reg, val);
		if (!ret)
			return 0;
		usleep_range(CS40L26_RETRY_US, CS40L26_RETRY_US + 100);
	}
	return ret;
}

static int cs40l26_write(struct cs40l26 *cs, u32 reg, u32 val)
{
	int i, ret;

	for (i = 0; i < CS40L26_RETRIES; i++) {
		ret = regmap_write(cs->regmap, reg, val);
		if (!ret)
			return 0;
		usleep_range(CS40L26_RETRY_US, CS40L26_RETRY_US + 100);
	}
	return ret;
}

/* Post a command and wait for the DSP to clear the mailbox (its ACK). */
static int cs40l26_mailbox(struct cs40l26 *cs, u32 cmd)
{
	u32 val = cmd;
	int i, ret;

	ret = cs40l26_write(cs, CS40L26_DSP_VIRTUAL1_MBOX_1, cmd);
	if (ret) {
		dev_err(cs->dev, "mailbox write %#x failed: %d\n", cmd, ret);
		return ret;
	}

	for (i = 0; i < CS40L26_RETRIES; i++) {
		ret = cs40l26_read(cs, CS40L26_DSP_VIRTUAL1_MBOX_1, &val);
		if (ret)
			return ret;
		if (!val)
			return 0;
		usleep_range(CS40L26_RETRY_US, CS40L26_RETRY_US + 100);
	}

	dev_err(cs->dev, "mailbox %#x not acknowledged (%#x)\n", cmd, val);
	return -ETIMEDOUT;
}

/* Downstream cs40l26_error_release(): clear, set, clear the release bit. */
static int cs40l26_error_release(struct cs40l26 *cs, unsigned int rls)
{
	u32 sts;
	int ret;

	ret = cs40l26_read(cs, CS40L26_ERROR_RELEASE, &sts);
	if (!ret)
		ret = cs40l26_write(cs, CS40L26_ERROR_RELEASE, sts & ~BIT(rls));
	if (!ret)
		ret = cs40l26_write(cs, CS40L26_ERROR_RELEASE, sts | BIT(rls));
	if (!ret)
		ret = cs40l26_write(cs, CS40L26_ERROR_RELEASE, sts & ~BIT(rls));
	return ret;
}

/*
 * The amplifier latches a protection fault (boost over/undervoltage, boost
 * or amp short, die overtemperature) and stays silent until the host runs
 * the release sequence; downstream does it from the chip's IRQ, which this
 * driver does not wire up. Without it the haptics stopped mid-typing and
 * came back only with the next fault-free reset (field report 2026-11-19).
 * Poll the latched events after each playback instead. Caller holds lock.
 */
static void cs40l26_check_errors(struct cs40l26 *cs)
{
	static const struct { u32 bit; unsigned int rls; const char *what; } errs[] = {
		{ CS40L26_BST_OVP_ERR, CS40L26_BST_OVP_ERR_RLS, "boost overvoltage" },
		{ CS40L26_BST_DCM_UVP_ERR, CS40L26_BST_UVP_ERR_RLS, "boost undervoltage" },
		{ CS40L26_BST_SHORT_ERR_INT, CS40L26_BST_SHORT_ERR_RLS, "boost short" },
		{ CS40L26_TEMP_ERR, CS40L26_TEMP_ERR_RLS, "die overtemperature" },
		{ CS40L26_AMP_ERR, CS40L26_AMP_SHORT_ERR_RLS, "amp short" },
	};
	u32 eint;
	int i;

	if (cs40l26_read(cs, CS40L26_IRQ1_EINT_1, &eint) || !(eint & CS40L26_ERR_INTS))
		return;

	cs40l26_write(cs, CS40L26_IRQ1_EINT_1, eint & CS40L26_ERR_INTS);
	for (i = 0; i < ARRAY_SIZE(errs); i++) {
		if (!(eint & errs[i].bit))
			continue;
		cs->err_releases++;
		dev_warn_ratelimited(cs->dev, "%s, releasing (EINT_1 %#x, #%u)\n",
				     errs[i].what, eint, cs->err_releases);
		cs40l26_error_release(cs, errs[i].rls);
	}
}

static void cs40l26_repeat_work(struct work_struct *work)
{
	struct cs40l26 *cs = container_of(to_delayed_work(work), struct cs40l26,
					  repeat_work);

	mutex_lock(&cs->lock);
	if (time_before(jiffies, cs->repeat_end)) {
		cs40l26_mailbox(cs, cs->repeat_trigger);
		schedule_delayed_work(&cs->repeat_work,
				      msecs_to_jiffies(CS40L26_RUMBLE_PERIOD_MS));
	} else {
		cs40l26_mailbox(cs, CS40L26_STOP_PLAYBACK);
		cs40l26_check_errors(cs);
	}
	mutex_unlock(&cs->lock);
}

static void cs40l26_play_work(struct work_struct *work)
{
	struct cs40l26 *cs = container_of(work, struct cs40l26, play_work);
	struct cs40l26_effect *e;
	unsigned long flags;
	bool on;
	int id;

	spin_lock_irqsave(&cs->req_lock, flags);
	id = cs->req_id;
	on = cs->req_on;
	spin_unlock_irqrestore(&cs->req_lock, flags);

	cancel_delayed_work_sync(&cs->repeat_work);

	mutex_lock(&cs->lock);
	if (!on || id < 0 || id >= CS40L26_MAX_EFFECTS) {
		cs40l26_mailbox(cs, CS40L26_STOP_PLAYBACK);
		goto out;
	}

	e = &cs->effects[id];
	if (cs40l26_mailbox(cs, e->trigger)) {
		cs40l26_check_errors(cs);
		goto out;
	}
	if (!e->repeat)
		cs40l26_check_errors(cs);

	if (e->repeat) {
		cs->repeat_trigger = e->trigger;
		cs->repeat_end = jiffies + msecs_to_jiffies(e->length_ms ?: 1000);
		schedule_delayed_work(&cs->repeat_work,
				      msecs_to_jiffies(CS40L26_RUMBLE_PERIOD_MS));
	}
out:
	mutex_unlock(&cs->lock);
}

static int cs40l26_upload_effect(struct input_dev *dev, struct ff_effect *effect,
				 struct ff_effect *old)
{
	struct cs40l26 *cs = input_get_drvdata(dev);
	struct cs40l26_effect *e;
	s16 data[CS40L26_CUSTOM_DATA_SIZE];

	if (effect->id < 0 || effect->id >= CS40L26_MAX_EFFECTS)
		return -EINVAL;
	e = &cs->effects[effect->id];

	switch (effect->type) {
	case FF_PERIODIC:
		if (effect->u.periodic.waveform != FF_CUSTOM) {
			e->trigger = CS40L26_BUZZGEN_INDEX;
			e->repeat = true;
			e->length_ms = effect->replay.length;
			return 0;
		}

		if (effect->u.periodic.custom_len != CS40L26_CUSTOM_DATA_SIZE)
			return -EINVAL;
		if (copy_from_user(data, effect->u.periodic.custom_data,
				   sizeof(data)))
			return -EFAULT;
		if (data[0] != CS40L26_ROM_BANK_ID) {
			dev_dbg(cs->dev, "bank %d needs the RAM firmware\n", data[0]);
			return -EOPNOTSUPP;
		}
		if (data[1] < 0 || data[1] >= CS40L26_ROM_NUM_WAVES)
			return -EINVAL;

		e->trigger = CS40L26_ROM_INDEX_START + data[1];
		e->repeat = false;
		e->length_ms = 0;
		return 0;

	case FF_RUMBLE:
		if (!effect->u.rumble.strong_magnitude &&
		    !effect->u.rumble.weak_magnitude)
			return -EINVAL;
		e->trigger = CS40L26_BUZZGEN_INDEX;
		e->repeat = true;
		e->length_ms = effect->replay.length;
		return 0;

	default:
		return -EINVAL;
	}
}

static int cs40l26_playback(struct input_dev *dev, int effect_id, int value)
{
	struct cs40l26 *cs = input_get_drvdata(dev);
	unsigned long flags;

	spin_lock_irqsave(&cs->req_lock, flags);
	cs->req_id = effect_id;
	cs->req_on = value > 0;
	spin_unlock_irqrestore(&cs->req_lock, flags);

	schedule_work(&cs->play_work);
	return 0;
}

static int cs40l26_erase(struct input_dev *dev, int effect_id)
{
	struct cs40l26 *cs = input_get_drvdata(dev);

	if (effect_id >= 0 && effect_id < CS40L26_MAX_EFFECTS)
		memset(&cs->effects[effect_id], 0, sizeof(cs->effects[0]));
	return 0;
}

static void cs40l26_cancel(void *data)
{
	struct cs40l26 *cs = data;

	cancel_work_sync(&cs->play_work);
	cancel_delayed_work_sync(&cs->repeat_work);
	mutex_lock(&cs->lock);
	cs40l26_mailbox(cs, CS40L26_STOP_PLAYBACK);
	mutex_unlock(&cs->lock);
}

static int cs40l26_identify(struct cs40l26 *cs)
{
	u32 devid, state = 0;
	int i, ret;

	ret = cs40l26_read(cs, CS40L26_DEVID, &devid);
	if (ret)
		return dev_err_probe(cs->dev, ret, "no response\n");

	switch (devid & GENMASK(23, 0)) {
	case CS40L26_DEVID_A:
	case CS40L26_DEVID_B:
	case CS40L26_DEVID_L27_A:
	case CS40L26_DEVID_L27_B:
		break;
	default:
		return dev_err_probe(cs->dev, -ENODEV, "unknown device %#x\n", devid);
	}

	ret = cs40l26_read(cs, CS40L26_REVID, &cs->revid);
	if (ret)
		return ret;
	cs->revid &= 0xff;

	cs->halo_state_reg = cs->revid == CS40L26_REVID_B2 ?
			     CS40L26_B2_DSP_HALO_STATE : CS40L26_A1_DSP_HALO_STATE;

	/* the ROM firmware takes a moment to start after reset */
	for (i = 0; i < 100; i++) {
		ret = cs40l26_read(cs, cs->halo_state_reg, &state);
		if (!ret && state == CS40L26_DSP_HALO_STATE_RUN)
			break;
		usleep_range(1000, 1100);
	}
	if (state != CS40L26_DSP_HALO_STATE_RUN)
		return dev_err_probe(cs->dev, -ENODEV,
				     "ROM firmware not running (HALO_STATE %#x)\n",
				     state);

	dev_info(cs->dev, "CS40L%02X rev %02X, ROM firmware running\n",
		 (devid >> 4) & 0xff, cs->revid);
	return 0;
}

static int cs40l26_update_bits(struct cs40l26 *cs, u32 reg, u32 mask, u32 val)
{
	u32 old;
	int ret;

	ret = cs40l26_read(cs, reg, &old);
	if (ret)
		return ret;
	return cs40l26_write(cs, reg, (old & ~mask) | (val & mask));
}

/*
 * Keep the chip out of hibernation. Hibernation drops register settings that
 * are not in the DSP's power-on write sequence, and finding that sequence
 * needs the RAM firmware's symbol table, which this driver does not load
 * yet. Staying in standby keeps the boost limits and errata below in force.
 */
static int cs40l26_prevent_hibernate(struct cs40l26 *cs)
{
	u32 state, locks;
	int i, ret;

	for (i = 0; i < 10; i++) {
		ret = cs40l26_mailbox(cs, CS40L26_MBOX_PREVENT_HIBER);
		if (ret)
			return ret;
		ret = cs40l26_read(cs, CS40L26_A1_PM_CUR_STATE, &state);
		if (ret)
			return ret;
		state &= CS40L26_DSP_STATE_MASK;
		if (state == CS40L26_DSP_STATE_ACTIVE)
			return 0;
		if (state == CS40L26_DSP_STATE_STANDBY) {
			ret = cs40l26_read(cs, CS40L26_A1_PM_STATE_LOCKS +
					   CS40L26_DSP_LOCK3_OFFSET, &locks);
			if (ret)
				return ret;
			if (locks & CS40L26_DSP_LOCK3)
				return 0;
		}
		usleep_range(5000, 5100);
	}
	return -ETIMEDOUT;
}

/* Wait until the DSP is idle (not playing) before touching the boost. */
static int cs40l26_wait_dsp_idle(struct cs40l26 *cs)
{
	u32 state = 0;
	int i, ret;

	for (i = 0; i < 20; i++) {
		ret = cs40l26_read(cs, CS40L26_A1_PM_CUR_STATE, &state);
		if (ret)
			return ret;
		state &= CS40L26_DSP_STATE_MASK;
		if (state == CS40L26_DSP_STATE_STANDBY ||
		    state == CS40L26_DSP_STATE_SHUTDOWN)
			return 0;
		usleep_range(20000, 20100);
	}
	dev_err(cs->dev, "DSP never went idle (state %u)\n", state);
	return -EBUSY;
}

/*
 * Boost converter short test, as the downstream driver runs it on rev A1
 * parts: boost to VP in fixed mode, enable, and check for a short.
 */
static int cs40l26_lbst_short_test(struct cs40l26 *cs)
{
	u32 vbst1, vbst2, err;
	int ret, ret2;

	ret = cs40l26_read(cs, CS40L26_VBST_CTL_1, &vbst1);
	if (!ret)
		ret = cs40l26_read(cs, CS40L26_VBST_CTL_2, &vbst2);
	if (!ret)
		ret = cs40l26_update_bits(cs, CS40L26_VBST_CTL_1,
					  CS40L26_BST_CTL_MASK, 0);	/* VP */
	if (!ret)
		ret = cs40l26_update_bits(cs, CS40L26_VBST_CTL_2,
					  CS40L26_BST_CTL_SEL_MASK, 0);	/* fixed */
	if (ret)
		return ret;

	ret = cs40l26_update_bits(cs, CS40L26_GLOBAL_ENABLES,
				  CS40L26_GLOBAL_EN, CS40L26_GLOBAL_EN);
	if (ret)
		return ret;

	usleep_range(10000, 10100);	/* boost converter powered up */

	ret = cs40l26_read(cs, CS40L26_ERROR_RELEASE, &err);

	ret2 = cs40l26_update_bits(cs, CS40L26_GLOBAL_ENABLES,
				   CS40L26_GLOBAL_EN, 0);
	if (ret)
		return ret;
	if (ret2)
		return ret2;

	if (err & CS40L26_BST_SHORT_ERR) {
		dev_crit(cs->dev, "FATAL: boost shorted at startup, haptics disabled\n");
		return -ENOTRECOVERABLE;
	}

	ret = cs40l26_write(cs, CS40L26_VBST_CTL_1, vbst1);
	if (!ret)
		ret = cs40l26_write(cs, CS40L26_VBST_CTL_2, vbst2);
	return ret;
}

static int cs40l26_boost_config(struct cs40l26 *cs)
{
	u32 uv, ua, ctl, ipk;
	int ret;

	if (device_property_read_u32(cs->dev, "cirrus,bst-ctl-microvolt", &uv) ||
	    uv < CS40L26_BST_UV_MIN || uv > CS40L26_BST_UV_MAX)
		uv = CS40L26_BST_UV_MAX;
	ctl = (uv - CS40L26_BST_UV_MIN) / CS40L26_BST_UV_STEP;

	if (device_property_read_u32(cs->dev, "cirrus,bst-ipk-microamp", &ua) ||
	    ua < CS40L26_BST_IPK_UA_MIN || ua > CS40L26_BST_IPK_UA_MAX)
		ua = 4500000;
	ipk = ua / CS40L26_BST_IPK_UA_STEP - 16;

	cs->bst_ipk = ipk;
	ret = cs40l26_write(cs, CS40L26_VBST_CTL_1, ctl);
	if (!ret)
		ret = cs40l26_update_bits(cs, CS40L26_VBST_CTL_2,
					  CS40L26_BST_CTL_LIM_EN,
					  CS40L26_BST_CTL_LIM_EN);
	if (!ret)
		ret = cs40l26_write(cs, CS40L26_BST_IPK_CTL, ipk);
	if (ret)
		return ret;

	dev_info(cs->dev, "boost %u mV, peak current %u mA\n",
		 uv / 1000, ua / 1000);
	return 0;
}

static int cs40l26_safety_init(struct cs40l26 *cs)
{
	static const struct reg_sequence a1_errata[] = {
		{ CS40L26_PLL_REFCLK_DETECT_0, 0 },
		{ CS40L26_TEST_KEY_CTRL, 0x55 },
		{ CS40L26_TEST_KEY_CTRL, 0xaa },
		{ CS40L26_TEST_LBST, CS40L26_DISABLE_EXPL_MODE },
		{ CS40L26_TEST_KEY_CTRL, 0 },
	};
	bool expl_mode = !device_property_present(cs->dev,
						  "cirrus,bst-expl-mode-disable");
	int i, ret;

	if (cs->revid >= CS40L26_REVID_B2) {
		dev_warn(cs->dev, "rev %02X: no safety init for this revision yet\n",
			 cs->revid);
		return 0;
	}

	ret = cs40l26_prevent_hibernate(cs);
	if (ret)
		return dev_err_probe(cs->dev, ret, "cannot keep the DSP awake\n");

	ret = cs40l26_wait_dsp_idle(cs);
	if (ret)
		return ret;

	/* downstream: explicit boost mode skips the short test and errata */
	if (!expl_mode) {
		ret = cs40l26_lbst_short_test(cs);
		if (ret)
			return ret;
	}

	for (i = 0; i < (expl_mode ? 1 : ARRAY_SIZE(a1_errata)); i++) {
		ret = cs40l26_write(cs, a1_errata[i].reg, a1_errata[i].def);
		if (ret)
			return ret;
	}

	return cs40l26_boost_config(cs);
}

static int cs40l26_status_show(struct seq_file *m, void *unused)
{
	static const struct { const char *name; u32 reg; } regs[] = {
		{ "IRQ1_EINT_1", CS40L26_IRQ1_EINT_1 },
		{ "IRQ1_EINT_2", CS40L26_IRQ1_EINT_2 },
		{ "IRQ1_STS_1", CS40L26_IRQ1_STS_1 },
		{ "ERROR_RELEASE", CS40L26_ERROR_RELEASE },
		{ "PM_CUR_STATE", CS40L26_A1_PM_CUR_STATE },
		{ "PM_STATE_LOCK3", CS40L26_A1_PM_STATE_LOCKS + CS40L26_DSP_LOCK3_OFFSET },
		{ "VBST_CTL_1", CS40L26_VBST_CTL_1 },
		{ "BST_IPK_CTL", CS40L26_BST_IPK_CTL },
		{ "MBOX_1", CS40L26_DSP_VIRTUAL1_MBOX_1 },
	};
	struct cs40l26 *cs = m->private;
	u32 val;
	int i;

	mutex_lock(&cs->lock);
	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		if (cs40l26_read(cs, regs[i].reg, &val))
			seq_printf(m, "%-15s read error\n", regs[i].name);
		else
			seq_printf(m, "%-15s %#010x\n", regs[i].name, val);
	}
	seq_printf(m, "%-15s %#x (expected)\n", "bst_ipk", cs->bst_ipk);
	seq_printf(m, "%-15s %u\n", "err_releases", cs->err_releases);
	mutex_unlock(&cs->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(cs40l26_status);

static void cs40l26_debugfs_remove(void *data)
{
	debugfs_remove_recursive(data);
}

static int cs40l26_suspend(struct device *dev)
{
	struct cs40l26 *cs = dev_get_drvdata(dev);

	cancel_work_sync(&cs->play_work);
	cancel_delayed_work_sync(&cs->repeat_work);
	mutex_lock(&cs->lock);
	cs40l26_mailbox(cs, CS40L26_STOP_PLAYBACK);
	mutex_unlock(&cs->lock);
	return 0;
}

/*
 * If the chip lost power in system sleep, its ROM firmware rebooted with the
 * default boost limits and hibernation allowed: redo the safety setup.
 */
static int cs40l26_resume(struct device *dev)
{
	struct cs40l26 *cs = dev_get_drvdata(dev);
	u32 ipk = 0;
	int ret = 0;

	mutex_lock(&cs->lock);
	if (cs40l26_read(cs, CS40L26_BST_IPK_CTL, &ipk) || ipk != cs->bst_ipk) {
		dev_info(cs->dev, "lost its setup in sleep (BST_IPK %#x), redoing it\n",
			 ipk);
		ret = cs40l26_identify(cs);
		if (!ret)
			ret = cs40l26_safety_init(cs);
		if (ret)
			dev_err(cs->dev, "re-init after resume failed: %d\n", ret);
	}
	cs40l26_check_errors(cs);
	mutex_unlock(&cs->lock);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(cs40l26_pm, cs40l26_suspend, cs40l26_resume);

static int cs40l26_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ff_device *ff;
	struct cs40l26 *cs;
	int ret;

	cs = devm_kzalloc(dev, sizeof(*cs), GFP_KERNEL);
	if (!cs)
		return -ENOMEM;

	cs->dev = dev;
	mutex_init(&cs->lock);
	spin_lock_init(&cs->req_lock);
	INIT_WORK(&cs->play_work, cs40l26_play_work);
	INIT_DELAYED_WORK(&cs->repeat_work, cs40l26_repeat_work);

	cs->regmap = devm_regmap_init_i2c(client, &cs40l26_regmap);
	if (IS_ERR(cs->regmap))
		return PTR_ERR(cs->regmap);

	/* reset pulse: the ROM firmware boots fresh, in a known state */
	cs->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(cs->reset))
		return PTR_ERR(cs->reset);
	if (cs->reset) {
		usleep_range(2000, 2200);
		gpiod_set_value_cansleep(cs->reset, 0);
		usleep_range(3000, 3200);
	}

	ret = cs40l26_identify(cs);
	if (ret)
		return ret;

	ret = cs40l26_safety_init(cs);
	if (ret)
		return ret;

	cs->input = devm_input_allocate_device(dev);
	if (!cs->input)
		return -ENOMEM;

	cs->input->name = "cs40l26_input";
	cs->input->id.bustype = BUS_I2C;
	input_set_drvdata(cs->input, cs);

	input_set_capability(cs->input, EV_FF, FF_PERIODIC);
	input_set_capability(cs->input, EV_FF, FF_CUSTOM);
	input_set_capability(cs->input, EV_FF, FF_SINE);
	input_set_capability(cs->input, EV_FF, FF_RUMBLE);

	ret = input_ff_create(cs->input, CS40L26_MAX_EFFECTS);
	if (ret)
		return ret;

	ff = cs->input->ff;
	ff->upload = cs40l26_upload_effect;
	ff->playback = cs40l26_playback;
	ff->erase = cs40l26_erase;

	ret = devm_add_action_or_reset(dev, cs40l26_cancel, cs);
	if (ret)
		return ret;

	i2c_set_clientdata(client, cs);
	cs->dbgfs = debugfs_create_dir(dev_name(dev), NULL);
	debugfs_create_file("status", 0400, cs->dbgfs, cs, &cs40l26_status_fops);
	ret = devm_add_action_or_reset(dev, cs40l26_debugfs_remove, cs->dbgfs);
	if (ret)
		return ret;

	mutex_lock(&cs->lock);
	cs40l26_check_errors(cs);	/* anything latched before we were here */
	mutex_unlock(&cs->lock);

	return input_register_device(cs->input);
}

static const struct of_device_id cs40l26_of_match[] = {
	{ .compatible = "cirrus,cs40l26a" },
	{ .compatible = "cirrus,cs40l26b" },
	{ .compatible = "cirrus,cs40l27a" },
	{ .compatible = "cirrus,cs40l27b" },
	{ }
};
MODULE_DEVICE_TABLE(of, cs40l26_of_match);

static const struct i2c_device_id cs40l26_id[] = {
	{ "cs40l26a" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cs40l26_id);

static struct i2c_driver cs40l26_driver = {
	.driver = {
		.name = "cs40l26",
		.of_match_table = cs40l26_of_match,
		.pm = pm_sleep_ptr(&cs40l26_pm),
	},
	.probe = cs40l26_probe,
	.id_table = cs40l26_id,
};
module_i2c_driver(cs40l26_driver);

MODULE_DESCRIPTION("Cirrus Logic CS40L26 haptics (ROM firmware)");
MODULE_LICENSE("GPL");
