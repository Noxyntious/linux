// SPDX-License-Identifier: GPL-2.0-only
/*
 * Tensor G3 ISP front end (CSIS + PDMA) bring-up: hold the block's power
 * domain and bus clocks on demand, so its registers can be studied and the
 * vendor programming replayed from userspace. Not a media driver.
 *
 * sysfs: on (0/1). While on, the isp-fe registers are safe to access.
 *
 * After power-on the ISP-FE sub-blocks sit in reset and any access to them
 * (e.g. +0x50000, version 0x1003aa) raises an SError. Android pulses the
 * block reset first (+0x20008 = 0xf, +0x20010 = 1, wait for +0x2003c == 0);
 * do the same. Offset 0 of the isp-fe window is a decode hole.
 */

#define ISPFE_RST_SEL		0x20008
#define ISPFE_RST_TRIG		0x20010
#define ISPFE_RST_BUSY		0x2003c

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/iopoll.h>
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
	void __iomem *ispfe;	/* isp-fe core, 0x1c300000 */
	struct mutex lock;
	bool on;
};

static int ispfe_reset(struct ispfe_bringup *p)
{
	u32 val;
	int ret;

	writel(0xf, p->ispfe + ISPFE_RST_SEL);
	writel(1, p->ispfe + ISPFE_RST_TRIG);
	ret = readl_poll_timeout(p->ispfe + ISPFE_RST_BUSY, val, !val, 10, 10000);
	if (ret)
		dev_err(p->dev, "ISP-FE reset stuck, %#x = %#x\n", ISPFE_RST_BUSY, val);
	return ret;
}

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
		ret = ispfe_reset(p);
		if (ret) {
			pm_runtime_put_sync(p->dev);
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

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res)
		return dev_err_probe(dev, -EINVAL, "no isp-fe window\n");
	p->ispfe = devm_ioremap(dev, res->start, resource_size(res));
	if (!p->ispfe)
		return -ENOMEM;

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
