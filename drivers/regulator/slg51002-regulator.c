// SPDX-License-Identifier: GPL-2.0-only
/*
 * Dialog SLG51002 camera PMIC (Pixel 8 Pro), enable-only.
 *
 * Eight LDOs and four load switches ("gpio1".."gpio4"). The output voltages
 * are left as the PMIC holds them: this driver only switches rails, and it
 * reads each LDO's VSEL at probe and refuses to register any LDO whose
 * voltage is not exactly what the device tree asks for. The register map was
 * read from the vendor kernel (regmap debugfs and I2C traces):
 *   0x1105..0x1107  pattern ID 03 31 b1
 *   0x110d          LDO enable matrix, bit n-1 = ldo<n>
 *   0x1710 + n-1    load switch gpio<n>, bit 0 = on
 *   VSEL            (VSEL + 0x60/0x61 = allowed selector range)
 *                   ldo1..5 at 0x2000/0x2200/0x2300/0x2500/0x2700,
 *                   1.2 V + 10 mV/step; ldo6..8 at 0x2900/0x3100/0x3200,
 *                   0.4 V + 5 mV/step
 * Three S2MPG15 GPIOs (cs, bb, buck) power the chip itself; they are raised
 * at probe.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#define SLG51002_PATN_ID_B0	0x1105
#define SLG51002_LDO_EN		0x110d
#define SLG51002_SW_EN(n)	(0x1710 + (n))

#define SLG51002_NUM_LDO	8
#define SLG51002_NUM_SW		4

static const u16 slg51002_vsel_reg[SLG51002_NUM_LDO] = {
	0x2000, 0x2200, 0x2300, 0x2500, 0x2700, 0x2900, 0x3100, 0x3200,
};

static int slg51002_ldo_uV(int i, unsigned int vsel)
{
	return i < 5 ? 1200000 + vsel * 10000 : 400000 + vsel * 5000;
}

/* Program LDO @i to exactly @uV (it must be off). Returns the VSEL or < 0. */
static int slg51002_ldo_program(struct device *dev, struct regmap *regmap,
				int i, int uV)
{
	int base = i < 5 ? 1200000 : 400000;
	int step = i < 5 ? 10000 : 5000;
	unsigned int en, vsel, rb;
	u8 range[2];
	int ret;

	if (uV < base || (uV - base) % step)
		return -EINVAL;
	vsel = (uV - base) / step;

	ret = regmap_bulk_read(regmap, slg51002_vsel_reg[i] + 0x60, range, 2);
	if (ret)
		return ret;
	if (vsel < range[0] || vsel > range[1] || vsel > 0xff)
		return -ERANGE;

	ret = regmap_read(regmap, SLG51002_LDO_EN, &en);
	if (ret)
		return ret;
	if (en & BIT(i))
		return -EBUSY;

	ret = regmap_write(regmap, slg51002_vsel_reg[i], vsel);
	if (ret)
		return ret;
	ret = regmap_read(regmap, slg51002_vsel_reg[i], &rb);
	if (ret)
		return ret;
	if (rb != vsel) {
		dev_err(dev, "ldo%d: wrote VSEL 0x%02x, reads 0x%02x\n",
			i + 1, vsel, rb);
		return -EIO;
	}
	return vsel;
}

static const struct regulator_ops slg51002_switch_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static const struct regulator_ops slg51002_ldo_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
};

static const struct regmap_range slg51002_ranges[] = {
	regmap_reg_range(0x1105, 0x1119),
	regmap_reg_range(0x1500, 0x1506),
	regmap_reg_range(0x1700, 0x173f),
	regmap_reg_range(0x2000, 0x32c2),
};

static const struct regmap_access_table slg51002_access = {
	.yes_ranges = slg51002_ranges,
	.n_yes_ranges = ARRAY_SIZE(slg51002_ranges),
};

static const struct regmap_config slg51002_regmap = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x32c2,
	.rd_table = &slg51002_access,
	.wr_table = &slg51002_access,
	.cache_type = REGCACHE_NONE,
};

static int slg51002_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *regs_np, *np;
	struct regmap *regmap;
	u8 id[3];
	int i, ret;

	/* cs, bb, buck: the chip's own enables, all on while it is in use. */
	for (i = 0; i < 3; i++) {
		static const char * const names[] = { "dlg,cs", "dlg,bb", "dlg,buck" };
		struct gpio_desc *g;

		g = devm_gpiod_get_optional(dev, names[i], GPIOD_OUT_HIGH);
		if (IS_ERR(g))
			return dev_err_probe(dev, PTR_ERR(g), "%s gpio\n", names[i]);
	}

	regmap = devm_regmap_init_i2c(client, &slg51002_regmap);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	/* It takes a while to answer after bb/buck come up. */
	for (i = 0; i < 20; i++) {
		usleep_range(5000, 6000);
		ret = regmap_bulk_read(regmap, SLG51002_PATN_ID_B0, id,
				       sizeof(id));
		if (!ret)
			break;
	}
	if (ret)
		return dev_err_probe(dev, ret, "no answer\n");
	if (id[0] != 0x03 || id[1] != 0x31 || id[2] != 0xb1)
		return dev_err_probe(dev, -ENODEV, "unexpected ID %*ph\n",
				     3, id);

	regs_np = of_get_child_by_name(dev->of_node, "regulators");
	if (!regs_np)
		return dev_err_probe(dev, -EINVAL, "no regulators node\n");

	for_each_child_of_node(regs_np, np) {
		struct regulator_config cfg = { .dev = dev, .regmap = regmap };
		struct regulator_init_data *init;
		struct regulator_desc *desc;
		struct regulator_dev *rdev;
		unsigned int idx, vsel;
		bool is_ldo;
		int uV;

		if (sscanf(np->name, "ldo%u", &idx) == 1 &&
		    idx >= 1 && idx <= SLG51002_NUM_LDO)
			is_ldo = true;
		else if (sscanf(np->name, "gpio%u", &idx) == 1 &&
			 idx >= 1 && idx <= SLG51002_NUM_SW)
			is_ldo = false;
		else
			continue;
		idx--;

		desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
		if (!desc) {
			of_node_put(np);
			ret = -ENOMEM;
			goto out;
		}
		desc->name = devm_kasprintf(dev, GFP_KERNEL, "slg51002-%s",
					    np->name);
		desc->of_match = np->name;
		desc->type = REGULATOR_VOLTAGE;
		desc->owner = THIS_MODULE;

		init = of_get_regulator_init_data(dev, np, desc);
		if (!init) {
			of_node_put(np);
			ret = -ENOMEM;
			goto out;
		}

		if (is_ldo) {
			ret = regmap_read(regmap, slg51002_vsel_reg[idx], &vsel);
			if (ret) {
				of_node_put(np);
				goto out;
			}
			uV = slg51002_ldo_uV(idx, vsel);
			if (init->constraints.min_uV != init->constraints.max_uV ||
			    !init->constraints.min_uV) {
				dev_err(dev, "%s: needs one fixed voltage in DT, NOT registering it\n",
					np->name);
				continue;
			}
			if (uV != init->constraints.min_uV) {
				int ret2 = slg51002_ldo_program(dev, regmap, idx,
							init->constraints.min_uV);

				if (ret2 < 0) {
					dev_err(dev, "%s: is %d uV, cannot set %d uV (%d): NOT registering it\n",
						np->name, uV,
						init->constraints.min_uV, ret2);
					continue;
				}
				dev_info(dev, "%s: %d -> %d uV (VSEL 0x%02x -> 0x%02x)\n",
					 np->name, uV, init->constraints.min_uV,
					 vsel, ret2);
				vsel = ret2;
				uV = init->constraints.min_uV;
			} else {
				dev_info(dev, "%s: %d uV (VSEL 0x%02x)\n",
					 np->name, uV, vsel);
			}
			desc->ops = &slg51002_ldo_ops;
			desc->n_voltages = 1;
			desc->min_uV = uV;
			desc->enable_reg = SLG51002_LDO_EN;
			desc->enable_mask = BIT(idx);
		} else {
			desc->ops = &slg51002_switch_ops;
			desc->enable_reg = SLG51002_SW_EN(idx);
			desc->enable_mask = BIT(0);
		}

		/* The voltage is fixed here, never applied. */
		init->constraints.apply_uV = false;
		init->constraints.valid_ops_mask &= ~REGULATOR_CHANGE_VOLTAGE;
		cfg.init_data = init;
		cfg.of_node = np;

		rdev = devm_regulator_register(dev, desc, &cfg);
		if (IS_ERR(rdev)) {
			of_node_put(np);
			ret = dev_err_probe(dev, PTR_ERR(rdev), "%s\n",
					    np->name);
			goto out;
		}
	}
	ret = 0;
out:
	of_node_put(regs_np);
	return ret;
}

static const struct of_device_id slg51002_of_match[] = {
	{ .compatible = "dlg,slg51002" },
	{ }
};
MODULE_DEVICE_TABLE(of, slg51002_of_match);

static struct i2c_driver slg51002_driver = {
	.driver = {
		.name = "slg51002",
		.of_match_table = slg51002_of_match,
	},
	.probe = slg51002_probe,
};
module_i2c_driver(slg51002_driver);

MODULE_DESCRIPTION("Dialog SLG51002 camera PMIC (enable-only)");
MODULE_LICENSE("GPL");
