// SPDX-License-Identifier: GPL-2.0+
/*
 * Nuvoton NCT7716 Thermal sensor driver of Hardware monitor framework.
 *
 * Copyright (C) 2023 Nuvoton Technology Corp.
 */

#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/hwmon.h>

#define NCT7716_TEMPERATURE_REG		0x00
#define NCT7716_ALERTSTATUS_REG		0x02
#define NCT7716_CONFIG_REG		0x03
#define NCT7716_OS_MODE			0x0F

#define NCT7716_R_CONVRATE_REG		0x04
#define NCT7716_W_CONVRATE_REG		0x0A
#define NCT7716_R_HALERT_REG		0x05
#define NCT7716_W_HALERT_REG		0x0B

#define NCT7716_CID_REG			0xFD
#define NCT7716_VID_REG			0xFE
#define NCT7716_DID_REG			0xFF

#define NCT7716_DEVICE_ID_MASK		0xF0
#define NCT7716_CHIP_ID			0x50
#define NCT7716_VENDOR_ID		0x50
#define NCT7716_DEVICE_ID		(0x91 & NCT7716_DEVICE_ID_MASK)

#define STS_LTHA_BIT			BIT(6)

static const struct acpi_device_id nct7716_acpi_ids[] = {
	{ "NCT7716", 20 },
	{}
};
MODULE_DEVICE_TABLE(acpi, nct7716_acpi_ids);

static const struct i2c_device_id nct7716_ids[] = {
	{ "nct7716", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, nct7716_ids);

struct nct7716_chip_info {
	struct regmap *regmap;
	struct mutex access_lock;
};

static bool nct7716_readable_reg(struct device *dev, unsigned int reg)
{
	if (reg != NCT7716_OS_MODE)
		return true;

	return false;
}
static bool nct7716_writeable_reg(struct device *dev, unsigned int reg)
{
	if (reg != NCT7716_TEMPERATURE_REG || reg != NCT7716_ALERTSTATUS_REG)
		return true;
	return false;
}
static bool nct7716_volatile_reg(struct device *dev, unsigned int reg)
{
	if (reg == NCT7716_TEMPERATURE_REG || reg != NCT7716_ALERTSTATUS_REG)
		return true;

	return false;
}

static const struct regmap_config nct7716_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = nct7716_writeable_reg,
	.readable_reg = nct7716_readable_reg,
	.volatile_reg = nct7716_volatile_reg,
};

static const struct hwmon_channel_info *nct7716_info[] = {
	HWMON_CHANNEL_INFO(chip, HWMON_C_UPDATE_INTERVAL),
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT | HWMON_T_MAX | HWMON_T_ALARM),
	NULL
};

static int nct7716_read(struct device *dev, enum hwmon_sensor_types type,
			u32 attr, int channel, long *temp)
{
	struct nct7716_chip_info *chip = dev_get_drvdata(dev);
	signed char val;
	int reg, err;

	switch (type) {
	case hwmon_chip:
		switch (attr) {
		case hwmon_chip_update_interval:
			reg = NCT7716_R_CONVRATE_REG;	/* Read the ConvRate from Conversion Rate Register */
			break;

		default:
			return -EOPNOTSUPP;
		}
		err = regmap_raw_read(chip->regmap, reg, &val, 1);
		if (err < 0)
			return err;
		*temp = val;
		break;

	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
			reg = NCT7716_TEMPERATURE_REG;
			break;

		case hwmon_temp_max:
			reg = NCT7716_R_HALERT_REG;
			break;

		case hwmon_temp_alarm:
			reg = NCT7716_ALERTSTATUS_REG;
			mutex_lock(&chip->access_lock);
			err = regmap_raw_read(chip->regmap, reg, &val, 1);
			mutex_unlock(&chip->access_lock);
			if (err < 0)
				return err;
			*temp = !!(val & STS_LTHA_BIT);	/* Read the STS_LTHA from Alert Status Register */
			return 0;

		default:
			return -EOPNOTSUPP;
		}
		err = regmap_raw_read(chip->regmap, reg, &val, 1);
		if (err < 0)
			return err;

		*temp = val * 1000;		/* Convert temperature to milliCelsius */
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int nct7716_write(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long temp)
{
	struct nct7716_chip_info *chip = dev_get_drvdata(dev);
	int reg;

	switch (type) {
	case hwmon_chip:
		switch (attr) {
		case hwmon_chip_update_interval:
			reg = NCT7716_W_CONVRATE_REG;	/* Write ConvRate to Conversion Rate Register */
			break;

		default:
			return -EOPNOTSUPP;
		}
		break;

	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_max:
			reg = NCT7716_W_HALERT_REG;
			break;

		default:
			return -EOPNOTSUPP;
		}
		break;

	default:
		return -EOPNOTSUPP;
	}

	return regmap_write(chip->regmap, reg, temp);
}

static umode_t nct7716_is_visible(const void *data, enum hwmon_sensor_types type,
				  u32 attr, int channel)
{
	switch (type) {
	case hwmon_chip:
		switch (attr) {
		case hwmon_chip_update_interval:
			return 0644;

		default:
			return -EOPNOTSUPP;
		}
		break;

	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
		case hwmon_temp_alarm:
			return 0444;

		case hwmon_temp_max:
			return 0644;

		default:
			return -EOPNOTSUPP;
		}
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static const struct hwmon_ops nct7716_hwmon_ops = {
	.is_visible = nct7716_is_visible,
	.read = nct7716_read,
	.write = nct7716_write,
};

static const struct hwmon_chip_info nct7716_chip_info = {
	.ops = &nct7716_hwmon_ops,
	.info = nct7716_info,
};

static bool nct7716_check_id(struct nct7716_chip_info *chip)
{
	int ret, chip_id, vendor_id, device_id;

	ret = regmap_read(chip->regmap, NCT7716_CID_REG, &chip_id);
	if (ret < 0)
		return false;

	ret = regmap_read(chip->regmap, NCT7716_VID_REG, &vendor_id);
	if (ret < 0)
		return false;

	ret = regmap_read(chip->regmap, NCT7716_DID_REG, &device_id);
	if (ret < 0)
		return false;

	device_id &= NCT7716_DEVICE_ID_MASK;
	if (chip_id == NCT7716_CHIP_ID &&
	    vendor_id == NCT7716_VENDOR_ID &&
	    device_id == NCT7716_DEVICE_ID)
		return true;

	return false;
}

static int nct7716_probe(struct i2c_client *client)
{
	struct nct7716_chip_info *chip;
	struct device *hwmon_dev;

	chip = devm_kzalloc(&client->dev, sizeof(struct nct7716_chip_info),
			    GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	mutex_init(&chip->access_lock);
	chip->regmap = devm_regmap_init_i2c(client, &nct7716_regmap);

	if (!nct7716_check_id(chip)) {
		dev_err(&client->dev, "No NCT7716 device\n");
		return -ENODEV;
	}

	hwmon_dev = devm_hwmon_device_register_with_info(&client->dev,
							 client->name,
							 chip,
							 &nct7716_chip_info,
							 NULL);
	if (IS_ERR(hwmon_dev)) {
		dev_err(&client->dev, "Register HM device failed\n");
		return PTR_ERR(hwmon_dev);
	}

	dev_info(&client->dev, "nct7716 probe finished");

	return 0;
}

static const struct of_device_id nct7716_of_match_table[] = {
        { .compatible = "nuvoton,nct7716", },
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nct7716_of_match_table);

static struct i2c_driver nct7716_driver = {
	.driver = {
		.name	= "nct7716",
		.acpi_match_table = nct7716_acpi_ids,
                .of_match_table = nct7716_of_match_table,
	},
	.probe_new	= nct7716_probe,
	.id_table	= nct7716_ids,
};

static int __init nct7716_init(void)
{
	return i2c_add_driver(&nct7716_driver);
}
module_init(nct7716_init);

static void __exit nct7716_exit(void)
{
	i2c_del_driver(&nct7716_driver);
}
module_exit(nct7716_exit);

MODULE_DESCRIPTION("Thermal sensor driver for NCT7716");
MODULE_AUTHOR("Tzu-Ming Yu <tmyu0@nuvoton.com>");
MODULE_LICENSE("GPL");
