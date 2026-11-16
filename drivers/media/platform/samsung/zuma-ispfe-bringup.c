// SPDX-License-Identifier: GPL-2.0-only
/*
 * Tensor G3 ISP front end (CSIS + PDMA) bring-up: hold the block's power
 * domain and bus clocks on demand, so its registers can be studied and the
 * vendor programming replayed from userspace. Not a media driver.
 *
 * sysfs: on (0/1). While on, the isp-fe registers are safe to access.
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

struct ispfe_bringup {
	struct device *dev;
	struct clk_bulk_data *clks;
	int nclks;
	void __iomem *pmu;	/* ISPFE_CONFIGURATION/STATUS, read only */
	struct mutex lock;
	bool on;
};

static int ispfe_set(struct ispfe_bringup *p, bool on)
{
	int ret;

	if (on == p->on)
		return 0;

	/*
	 * CMU_TOP's ISPFE_NOC clock feeds the block's bus interface, so it
	 * has to run before the domain comes up and stay up until it is down.
	 */
	if (on) {
		ret = clk_bulk_prepare_enable(p->nclks, p->clks);
		if (ret)
			return ret;
		ret = pm_runtime_resume_and_get(p->dev);
		if (ret) {
			clk_bulk_disable_unprepare(p->nclks, p->clks);
			return ret;
		}
	} else {
		pm_runtime_put_sync(p->dev);
		clk_bulk_disable_unprepare(p->nclks, p->clks);
	}
	p->on = on;
	dev_info(p->dev, "ISP-FE %s, PMU status %#x, clk %lu Hz\n",
		 on ? "on" : "off", p->pmu ? readl(p->pmu + 4) : ~0U,
		 p->nclks ? clk_get_rate(p->clks[0].clk) : 0);
	return 0;
}

static ssize_t on_show(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	struct ispfe_bringup *p = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", p->on);
}

static ssize_t on_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct ispfe_bringup *p = dev_get_drvdata(dev);
	bool on;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;

	mutex_lock(&p->lock);
	ret = ispfe_set(p, on);
	mutex_unlock(&p->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(on);

static struct attribute *ispfe_attrs[] = {
	&dev_attr_on.attr,
	NULL,
};
ATTRIBUTE_GROUPS(ispfe);

static int ispfe_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ispfe_bringup *p;
	struct resource *res;
	int ret;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->dev = dev;
	mutex_init(&p->lock);

	p->nclks = devm_clk_bulk_get_all(dev, &p->clks);
	if (p->nclks < 0)
		return dev_err_probe(dev, p->nclks, "clocks\n");
	if (!p->nclks)
		return dev_err_probe(dev, -EINVAL, "no ISPFE_NOC clock\n");

	/* Shared with pd_ispfe: map without claiming the region. */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	p->pmu = devm_ioremap(dev, res->start, resource_size(res));
	if (!p->pmu)
		dev_warn(dev, "PMU %pR not mapped, no status readback\n", res);

	platform_set_drvdata(pdev, p);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	dev_info(dev, "ISP-FE bring-up, %d clocks; echo 1 > on\n", p->nclks);
	return 0;
}

static void ispfe_remove(struct platform_device *pdev)
{
	struct ispfe_bringup *p = platform_get_drvdata(pdev);

	mutex_lock(&p->lock);
	ispfe_set(p, false);
	mutex_unlock(&p->lock);
}

static const struct of_device_id ispfe_of_match[] = {
	{ .compatible = "google,zuma-ispfe-bringup" },
	{ }
};
MODULE_DEVICE_TABLE(of, ispfe_of_match);

static struct platform_driver ispfe_driver = {
	.driver = {
		.name = "zuma-ispfe-bringup",
		.of_match_table = ispfe_of_match,
		.dev_groups = ispfe_groups,
	},
	.probe = ispfe_probe,
	.remove = ispfe_remove,
};
module_platform_driver(ispfe_driver);

MODULE_DESCRIPTION("Tensor G3 ISP front end power bring-up");
MODULE_LICENSE("GPL");
