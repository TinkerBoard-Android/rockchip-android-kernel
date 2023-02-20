// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for Renesas 9-series PCIe clock generator driver
 *
 * The following series can be supported:
 *   - 9FGV/9DBV/9DMV/9FGL/9DML/9QXL/9SQ
 * Currently supported:
 *   - 9FGV0441
 *
 * Copyright (C) 2022 Marek Vasut <marex@denx.de>
 */

#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>

enum tb3_rs9_model {
	RENESAS_9FGV0441,
};

/* Structure to describe features of a particular 9-series model */
struct tb3_rs9_chip_info {
	const enum tb3_rs9_model	model;
	unsigned int		num_clks;
};

static int tb3_rs9_probe(struct i2c_client *client)
{
	int  ret;

	ret = i2c_smbus_write_byte_data(client, 0x82 , 0xfe);
	if (ret < 0)
		return ret;

	pr_info("rs9: read 0x82 Slew rate control value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x82));

	return ret;
}


static const struct tb3_rs9_chip_info tb3_renesas_9fgv0441_info = {
	.model		= RENESAS_9FGV0441,
	.num_clks	= 4,
};

static const struct i2c_device_id tb3_rs9_id[] = {
	{ "9fgv0441tb3", .driver_data = RENESAS_9FGV0441 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tb3_rs9_id);

static const struct of_device_id tb3_clk_rs9_of_match[] = {
	{ .compatible = "renesas,9fgv0441tb3", .data = &tb3_renesas_9fgv0441_info },
	{ }
};
MODULE_DEVICE_TABLE(of, tb3_clk_rs9_of_match);

static struct i2c_driver tb3_rs9_driver = {
	.driver = {
		.name = "clk-renesas-pcie-9series",
		.of_match_table = tb3_clk_rs9_of_match,
	},
	.probe_new	= tb3_rs9_probe,
	.id_table	= tb3_rs9_id,
};
module_i2c_driver(tb3_rs9_driver);

MODULE_AUTHOR("Yi-Hsin Hung <yi-hsin_hung@asus.com>");
MODULE_DESCRIPTION("Renesas 9-series PCIe clock generator driver for ASUS TB3");
MODULE_LICENSE("GPL");
