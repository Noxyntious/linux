// SPDX-License-Identifier: GPL-2.0-only
/*
 * Pixel camera sensor bring-up: power a CIS through the sequence in its
 * device tree node and identify it. Not a V4L2 driver (yet) -- just enough
 * to prove the sensor answers on mainline before the real driver exists.
 *
 * DT (all optional except the I2C address):
 *   pixel,supply-names    supplies to enable, in order ("<name>-supply")
 *   pixel,supply-delays-us  delay after each supply (same length)
 *   enable-gpios          raised after the supplies (shared enable)
 *   clocks                MCLK, set to clock-frequency (default 24 MHz)
 *   pinctrl "mclk_on"/"mclk_off"  MCLK pad mux
 *   reset-gpios           active low, released last
 *   pixel,reset-delay-us  wait after releasing reset
 *
 * sysfs: cis_power (0/1); on power-up registers 0x0000-0x0003 are read and
 * logged (Samsung/Sony sensors keep their model ID there).
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>

#define CIS_MAX_SUPPLIES	8

struct cis_bringup {
	struct i2c_client *client;
	struct mutex lock;
	bool powered;

	int nsupplies;
	struct regulator *supply[CIS_MAX_SUPPLIES];
	const char *supply_name[CIS_MAX_SUPPLIES];
	u32 supply_delay[CIS_MAX_SUPPLIES];

	struct gpio_desc *enable;
	struct gpio_desc *reset;
	struct clk *mclk;
	u32 mclk_rate;
	u32 reset_delay;
	struct pinctrl *pinctrl;
	struct pinctrl_state *mclk_on, *mclk_off;
};

static int cis_read(struct cis_bringup *c, u16 reg, u8 *buf, int len)
{
	u8 addr[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msgs[2] = {
		{ .addr = c->client->addr, .len = 2, .buf = addr },
		{ .addr = c->client->addr, .flags = I2C_M_RD, .len = len,
		  .buf = buf },
	};
	int ret = i2c_transfer(c->client->adapter, msgs, 2);

	return ret == 2 ? 0 : (ret < 0 ? ret : -EIO);
}

static void cis_power_off(struct cis_bringup *c)
{
	int i;

	gpiod_set_value_cansleep(c->reset, 1);
	usleep_range(1000, 1500);
	if (c->mclk)
		clk_disable_unprepare(c->mclk);
	if (c->mclk_off)
		pinctrl_select_state(c->pinctrl, c->mclk_off);
	gpiod_set_value_cansleep(c->enable, 0);
	for (i = c->nsupplies - 1; i >= 0; i--)
		regulator_disable(c->supply[i]);
	c->powered = false;
}

static int cis_power_on(struct cis_bringup *c)
{
	struct device *dev = &c->client->dev;
	u8 id[4];
	int i, ret;

	gpiod_set_value_cansleep(c->reset, 1);

	for (i = 0; i < c->nsupplies; i++) {
		ret = regulator_enable(c->supply[i]);
		if (ret) {
			dev_err(dev, "enable %s: %d\n", c->supply_name[i], ret);
			goto err_supplies;
		}
		dev_info(dev, "%s on (%d uV)\n", c->supply_name[i],
			 regulator_get_voltage(c->supply[i]));
		if (c->supply_delay[i])
			fsleep(c->supply_delay[i]);
	}

	gpiod_set_value_cansleep(c->enable, 1);
	usleep_range(500, 1000);

	if (c->mclk_on)
		pinctrl_select_state(c->pinctrl, c->mclk_on);
	if (c->mclk) {
		ret = clk_set_rate(c->mclk, c->mclk_rate);
		if (ret)
			dev_warn(dev, "MCLK set_rate(%u): %d\n", c->mclk_rate, ret);
		ret = clk_prepare_enable(c->mclk);
		if (ret) {
			dev_err(dev, "MCLK enable: %d\n", ret);
			goto err_enable;
		}
		dev_info(dev, "MCLK %lu Hz\n", clk_get_rate(c->mclk));
	}
	usleep_range(1000, 1500);

	gpiod_set_value_cansleep(c->reset, 0);
	fsleep(c->reset_delay);
	c->powered = true;

	ret = cis_read(c, 0x0000, id, sizeof(id));
	if (ret)
		dev_err(dev, "no I2C answer at 0x%02x: %d\n", c->client->addr, ret);
	else
		dev_info(dev, "ID regs 0x0000-0x0003: %*ph\n", 4, id);
	return 0;	/* stay powered either way, for poking */

err_enable:
	gpiod_set_value_cansleep(c->enable, 0);
err_supplies:
	while (--i >= 0)
		regulator_disable(c->supply[i]);
	return ret;
}

static ssize_t cis_power_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct cis_bringup *c = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", c->powered);
}

static ssize_t cis_power_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct cis_bringup *c = dev_get_drvdata(dev);
	bool on;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;

	mutex_lock(&c->lock);
	if (on && !c->powered)
		ret = cis_power_on(c);
	else if (!on && c->powered)
		cis_power_off(c);
	mutex_unlock(&c->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(cis_power);

static struct attribute *cis_attrs[] = {
	&dev_attr_cis_power.attr,
	NULL,
};
ATTRIBUTE_GROUPS(cis);

static int cis_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct cis_bringup *c;
	int i, n;

	c = devm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;
	c->client = client;
	mutex_init(&c->lock);

	n = of_property_count_strings(dev->of_node, "pixel,supply-names");
	if (n > CIS_MAX_SUPPLIES)
		return -EINVAL;
	for (i = 0; i < n; i++) {
		of_property_read_string_index(dev->of_node, "pixel,supply-names",
					      i, &c->supply_name[i]);
		c->supply[i] = devm_regulator_get(dev, c->supply_name[i]);
		if (IS_ERR(c->supply[i]))
			return dev_err_probe(dev, PTR_ERR(c->supply[i]),
					     "supply %s\n", c->supply_name[i]);
	}
	c->nsupplies = n > 0 ? n : 0;
	of_property_read_u32_array(dev->of_node, "pixel,supply-delays-us",
				   c->supply_delay, c->nsupplies);

	c->enable = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(c->enable))
		return dev_err_probe(dev, PTR_ERR(c->enable), "enable gpio\n");
	c->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(c->reset))
		return dev_err_probe(dev, PTR_ERR(c->reset), "reset gpio\n");

	c->mclk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(c->mclk))
		return dev_err_probe(dev, PTR_ERR(c->mclk), "mclk\n");
	c->mclk_rate = 24000000;
	of_property_read_u32(dev->of_node, "clock-frequency", &c->mclk_rate);
	c->reset_delay = 10000;
	of_property_read_u32(dev->of_node, "pixel,reset-delay-us",
			     &c->reset_delay);

	c->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(c->pinctrl)) {
		c->mclk_on = pinctrl_lookup_state(c->pinctrl, "mclk_on");
		c->mclk_off = pinctrl_lookup_state(c->pinctrl, "mclk_off");
		if (IS_ERR(c->mclk_on))
			c->mclk_on = NULL;
		if (IS_ERR(c->mclk_off))
			c->mclk_off = NULL;
		if (c->mclk_off)
			pinctrl_select_state(c->pinctrl, c->mclk_off);
	}

	i2c_set_clientdata(client, c);
	dev_info(dev, "bring-up CIS at 0x%02x, %d supplies; echo 1 > cis_power\n",
		 client->addr, c->nsupplies);
	return 0;
}

static void cis_remove(struct i2c_client *client)
{
	struct cis_bringup *c = i2c_get_clientdata(client);

	mutex_lock(&c->lock);
	if (c->powered)
		cis_power_off(c);
	mutex_unlock(&c->lock);
}

static const struct of_device_id cis_of_match[] = {
	{ .compatible = "google,pixel-cis-bringup" },
	{ }
};
MODULE_DEVICE_TABLE(of, cis_of_match);

static struct i2c_driver cis_driver = {
	.driver = {
		.name = "pixel-cis-bringup",
		.of_match_table = cis_of_match,
		.dev_groups = cis_groups,
	},
	.probe = cis_probe,
	.remove = cis_remove,
};
module_i2c_driver(cis_driver);

MODULE_DESCRIPTION("Pixel camera sensor power bring-up");
MODULE_LICENSE("GPL");
