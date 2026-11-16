// SPDX-License-Identifier: GPL-2.0-only
/*
 * GPIOs of the Samsung S2MPG15 PMIC (Pixel 8 Pro "sub" PMIC).
 *
 * Ten pins, reached through ACPM access type 0x0c (the "gpio" regmap the
 * sec-acpm MFD attaches to its device). Layout read from the vendor kernel's
 * regmap debugfs while the camera stack toggled the pins:
 *   0x06/0x07  current pin levels, pins 0-7 / 8-9
 *   0x08 + n   pin n config: 0x40 = output low, 0x60 = output high
 * On the Pixel 8 Pro the pins enable the SLG51002 camera PMIC (0, 1, 3) and
 * the camera sensors (2, 4).
 */

#include <linux/bits.h>
#include <linux/gpio/driver.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define S2MPG15_GPIO_NGPIO	10
#define S2MPG15_GPIO_LEVEL(n)	(0x06 + (n) / 8)
#define S2MPG15_GPIO_CFG(n)	(0x08 + (n))
#define S2MPG15_GPIO_CFG_OE	BIT(6)
#define S2MPG15_GPIO_CFG_OUT	BIT(5)

struct s2mpg15_gpio {
	struct gpio_chip gc;
	struct regmap *regmap;
};

static int s2mpg15_gpio_get_direction(struct gpio_chip *gc, unsigned int off)
{
	struct s2mpg15_gpio *g = gpiochip_get_data(gc);
	unsigned int val;
	int ret;

	ret = regmap_read(g->regmap, S2MPG15_GPIO_CFG(off), &val);
	if (ret)
		return ret;

	return (val & S2MPG15_GPIO_CFG_OE) ? GPIO_LINE_DIRECTION_OUT :
					     GPIO_LINE_DIRECTION_IN;
}

static int s2mpg15_gpio_get(struct gpio_chip *gc, unsigned int off)
{
	struct s2mpg15_gpio *g = gpiochip_get_data(gc);
	unsigned int val;
	int ret;

	ret = regmap_read(g->regmap, S2MPG15_GPIO_LEVEL(off), &val);
	if (ret)
		return ret;

	return !!(val & BIT(off % 8));
}

static int s2mpg15_gpio_set(struct gpio_chip *gc, unsigned int off, int value)
{
	struct s2mpg15_gpio *g = gpiochip_get_data(gc);

	return regmap_update_bits(g->regmap, S2MPG15_GPIO_CFG(off),
				  S2MPG15_GPIO_CFG_OUT,
				  value ? S2MPG15_GPIO_CFG_OUT : 0);
}

static int s2mpg15_gpio_direction_output(struct gpio_chip *gc,
					 unsigned int off, int value)
{
	struct s2mpg15_gpio *g = gpiochip_get_data(gc);

	return regmap_update_bits(g->regmap, S2MPG15_GPIO_CFG(off),
				  S2MPG15_GPIO_CFG_OE | S2MPG15_GPIO_CFG_OUT,
				  S2MPG15_GPIO_CFG_OE |
				  (value ? S2MPG15_GPIO_CFG_OUT : 0));
}

static int s2mpg15_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s2mpg15_gpio *g;

	g = devm_kzalloc(dev, sizeof(*g), GFP_KERNEL);
	if (!g)
		return -ENOMEM;

	g->regmap = dev_get_regmap(dev->parent, "gpio");
	if (!g->regmap)
		return dev_err_probe(dev, -ENODEV, "no gpio regmap\n");

	device_set_node(dev, dev_fwnode(dev));
	g->gc.label = "s2mpg15-gpio";
	g->gc.parent = dev;
	g->gc.owner = THIS_MODULE;
	g->gc.base = -1;
	g->gc.ngpio = S2MPG15_GPIO_NGPIO;
	g->gc.can_sleep = true;
	g->gc.get_direction = s2mpg15_gpio_get_direction;
	g->gc.direction_output = s2mpg15_gpio_direction_output;
	g->gc.get = s2mpg15_gpio_get;
	g->gc.set = s2mpg15_gpio_set;

	return devm_gpiochip_add_data(dev, &g->gc, g);
}

static const struct platform_device_id s2mpg15_gpio_id[] = {
	{ "s2mpg15-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(platform, s2mpg15_gpio_id);

static const struct of_device_id s2mpg15_gpio_of_match[] = {
	{ .compatible = "samsung,s2mpg15-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mpg15_gpio_of_match);

static struct platform_driver s2mpg15_gpio_driver = {
	.driver = {
		.name = "s2mpg15-gpio",
		.of_match_table = s2mpg15_gpio_of_match,
	},
	.probe = s2mpg15_gpio_probe,
	.id_table = s2mpg15_gpio_id,
};
module_platform_driver(s2mpg15_gpio_driver);

MODULE_DESCRIPTION("Samsung S2MPG15 PMIC GPIO driver");
MODULE_LICENSE("GPL");
