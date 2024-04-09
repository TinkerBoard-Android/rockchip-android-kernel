/*
 * it5201.c - ITE CC logic for USB Type-C applications
 *
 * Copyright 2024 ASUS
 * Author: TzuWen Chang <tzuwen_chang@asus.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>


/* IT5201_REG_GCR */
#define VBUS_INTERRUPT (1 << 6)
/* IT5201_REG_IOCPCR */
#define TYPEC_INTERRUPT (1 << 7)
/* IT5201_REG_TCFSMCR0 */
#define TYPEC_ROLE_DRP (1 << 4)
/* IT5201_REG_CC_STATUS */
#define IS_NOT_CONNECTED(val) (val == 0)
#define IS_UFP_ATTATCHED(val) (val&0x1)
#define IS_DFP_ATTATCHED(val) (val&0x2)

struct it5201_info {
	struct device *dev;
	struct regmap *regmap;
};

enum it5201_reg {
	IT5201_REG_CID7R = 0x0,
	IT5201_REG_CID6R = 0x1,
	IT5201_REG_CID5R = 0x2,
	IT5201_REG_CID4R = 0x3,
	IT5201_REG_CID3R = 0x4,
	IT5201_REG_CID2R = 0x5,
	IT5201_REG_CID1R = 0x6,
	IT5201_REG_CID0R = 0x7,
	IT5201_REG_CVR = 0xf,
	IT5201_REG_GCR = 0x10,
	IT5201_REG_VCR = 0x13,
	IT5201_REG_TCFSMCR0 = 0x14,
	IT5201_REG_TCFSMCR1 = 0x15,
	IT5201_REG_TCASR = 0x16,
	IT5201_REG_CCSRCVR = 0x18,
	IT5201_REG_CCSNKVR = 0x19,
	IT5201_REG_IOCPCR = 0x1a,
	IT5201_REG_SPSR = 0x1b,
};

static const struct regmap_config it5201_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = IT5201_REG_SPSR,
	.cache_type = REGCACHE_NONE,
};

static int it5201_setup_i2c_det(
	struct it5201_info *info, struct i2c_client *i2c)
{
	int ret;
	unsigned int val = 0;

	ret = regmap_write(info->regmap, IT5201_REG_TCFSMCR0, TYPEC_ROLE_DRP);
	if (ret) {
		dev_err(info->dev, "failed to set TypeC Role Setting to DRP:%d\n",
			ret);
	}

	ret = regmap_read(info->regmap, IT5201_REG_TCFSMCR0, &val);
	if (!ret)
		dev_info(info->dev, "ITE IT5201: TypeC FSM Control Reg:0x%x\n", val);

	return 0;
}

static int it5201_i2c_probe(
	struct i2c_client *i2c, const struct i2c_device_id *id)
{

	struct device_node *np = i2c->dev.of_node;
	struct it5201_info *info;
	int ret = 0;

	if (!np) {
		return -EINVAL;
	}

	info = devm_kzalloc(&i2c->dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		return -ENOMEM;
	}

	i2c_set_clientdata(i2c, info);
	info->dev = &i2c->dev;

	info->regmap = devm_regmap_init_i2c(i2c, &it5201_regmap_config);
	if (IS_ERR(info->regmap)) {
		ret = PTR_ERR(info->regmap);
		dev_err(info->dev, "failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	it5201_setup_i2c_det(info, i2c);

	return 0;
}

static void it5201_i2c_remove(struct i2c_client *i2c)
{
	return;
}

static const struct i2c_device_id it5201_id[] = {
	{ "it5201", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, it5201_id);

#ifdef CONFIG_OF
static const struct of_device_id it5201_of_match[] = {
	{
		.compatible = "ite,it5201",
	},
	{},
};
MODULE_DEVICE_TABLE(of, it5201_of_match);
#endif

static struct i2c_driver it5201_i2c_driver = {
    .driver =
        {
            .name = "it5201",
            .of_match_table = of_match_ptr(it5201_of_match),
        },
    .probe = it5201_i2c_probe,
    .remove = it5201_i2c_remove,
    .id_table = it5201_id,
};
module_i2c_driver(it5201_i2c_driver);

MODULE_DESCRIPTION("ITE IT5201 CC logic driver for USB Type-C");
MODULE_AUTHOR("TzuWen Chang <TzuWen_Chang@asus.com>");
MODULE_LICENSE("GPL v2");
