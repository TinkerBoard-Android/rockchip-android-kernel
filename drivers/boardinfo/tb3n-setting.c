#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/driver.h>
#include <linux/iio/consumer.h>
#include <linux/proc_fs.h>
#include "tb3n-setting.h"

static int projectid = -1, boardid = -1, ddrid = -1, emmcid = -1, odmid = -1;
static char *boardinfo, *boardver, *ddr, *emmc, *odm;
static int emmc0_gpio = 0, emmc1_gpio = 0, init_done = 0;

static int all_show(struct seq_file *m, void *v)
{
	seq_printf(m, "boardinfo:\t%s\n", boardinfo);
	seq_printf(m, "projectid:\t%d\n", projectid);
	seq_printf(m, "boardver:\t%s\n", boardver);
	seq_printf(m, "boardid:\t%d\n", boardid);
	seq_printf(m, "ddr:\t\t%s\n", ddr);
	seq_printf(m, "ddrid:\t\t%d\n", ddrid);
	seq_printf(m, "emmc:\t\t%s\n", emmc);
	seq_printf(m, "emmcid:\t\t%d\n", emmcid);
	seq_printf(m, "odm:\t\t%s\n", odm);
	seq_printf(m, "odmid:\t\t%d\n", odmid);
	return 0;
}

static int info_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", boardinfo);
	return 0;
}

static int projectid_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", projectid);
	return 0;
}

static int ver_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", boardver);
	return 0;
}

static int boardid_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", boardid);
	return 0;
}

static int ddr_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", ddr);
	return 0;
}

static int ddrid_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", ddrid);
	return 0;
}

static int emmc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", emmc);
	return 0;
}

static int emmcid_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", emmcid);
	return 0;
}

static int odm_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", odm);
	return 0;
}

static int odmid_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", odmid);
	return 0;
}

int tb3n_gpios(struct device *dev)
{
	int ret;
	struct proc_dir_entry* file;
	int id0, id1;

	emmc0_gpio = of_get_named_gpio(dev->of_node, "emmc0-gpios", 0);
	if (!gpio_is_valid(emmc0_gpio)) {
		printk("[boardinfo] No emmc0-gpio pin available in boardinfo\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, emmc0_gpio, GPIOF_DIR_IN, "GPIO_EMMC0");
		if (ret < 0) {
			printk("[boardinfo] Failed to request EMMC0 gpio: %d\n", ret);
			return ret;
		}
	}

	emmc1_gpio = of_get_named_gpio(dev->of_node, "emmc1-gpios", 0);
	if (!gpio_is_valid(emmc1_gpio)) {
		printk("[boardinfo] No emmc1-gpio pin available in boardinfo\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, emmc1_gpio, GPIOF_DIR_IN, "GPIO_EMMC1");
		if (ret < 0) {
			printk("[boardinfo] Failed to request EMMC1 gpio: %d\n", ret);
			return ret;
		}
	}

	id0 = gpio_get_value(emmc0_gpio);
	id1 = gpio_get_value(emmc1_gpio);

	emmcid = (id1 << 1) + id0;

	switch(emmcid) {
		case 0:
			emmc = "16GB";
			break;
		case 1:
			emmc = "32GB";
			break;
		case 2:
			emmc = "64GB";
			break;
		case 3:
			emmc = "NONE";
			break;
		default:
			emmc = "unknown";
	}

	file = proc_create_single("emmc", 0444, NULL, emmc_show);
	if (!file)
		return -ENOMEM;

	file = proc_create_single("emmcid", 0444, NULL, emmcid_show);
	if (!file)
		return -ENOMEM;

	if (!init_done) {
		file = proc_create_single("boardinfo_all", 0444, NULL, all_show);
		if (!file)
			return -ENOMEM;
		else
			init_done = 1;
	}

	return 0;
}

void tb3n_gpios_free(void)
{
	gpio_free(emmc0_gpio);
	gpio_free(emmc1_gpio);
}

int tb3n_adcs(struct device *dev, const char *compatible, int *hwid, int *pid, int *oid)
{
	int ret, vresult;
	struct iio_channel *channels;
	struct proc_dir_entry* file;
	const char *channel_name;

	if (device_property_read_string(dev, "io-channel-names", &channel_name)) {
		printk("[boardinfo][iio] Failed to read io-channel-names");
		return -ENODEV;
	}

	channels = devm_iio_channel_get(dev, channel_name);
	if (IS_ERR(channels)) {
		printk("[boardinfo][iio] Failed to get channels\n");
		return -ENODEV;
	} else {
		ret = iio_read_channel_processed(channels, &vresult);
		if (ret < 0) {
			printk("[boardinfo][iio] Failed to read channel result\n");
			return ret;
		}
	}

	if (vresult < 1950 && vresult > 1650)
		ret = 18;
	else if (vresult < 1650 && vresult > 1350)
		ret = 15;
	else if (vresult < 1350 && vresult > 1050)
		ret = 12;
	else if (vresult < 1050 && vresult > 750)
		ret = 9;
	else if (vresult < 750 && vresult > 450)
		ret = 6;
	else if (vresult < 450 && vresult > 150)
		ret = 3;
	else if (vresult < 150)
		ret = 0;

	if (strcmp(compatible, "RK3568-ADC1-PCBID") == 0) {
		boardid = ret;
		*hwid = boardid;

		switch(boardid) {
			case 18:
				boardver = "1.00";
				break;
			case 15:
				boardver = "1.01";
				break;
			case 12:
				boardver = "1.02";
				break;
			case 9:
			case 6:
			case 3:
			case 0:
			default:
				boardver = "unknown";
		}

		file = proc_create_single("boardver", 0444, NULL, ver_show);
		if (!file)
			return -ENOMEM;

		file = proc_create_single("boardid", 0444, NULL, boardid_show);
		if (!file)
			return -ENOMEM;
	} else if (strcmp(compatible, "RK3568-ADC3-RAMID") == 0) {
		ddrid = ret;

		switch(ddrid) {
			case 18:
				ddr = "2GB";
				break;
			case 15:
				ddr = "4GB";
				break;
			case 12:
				ddr = "8GB";
				break;
			case 9:
			case 6:
			case 3:
			case 0:
			default:
				ddr = "unknown";
		}

		file = proc_create_single("ddr", 0444, NULL, ddr_show);
		if (!file)
			return -ENOMEM;

		file = proc_create_single("ddrid", 0444, NULL, ddrid_show);
		if (!file)
			return -ENOMEM;
	} else if (strcmp(compatible, "RK3568-ADC4-ODMID") == 0) {
		odmid = ret;
		*oid = odmid;

		switch(odmid) {
			case 18:
				odm = "Tinker Board 3N";
				break;
			case 15:
				odm = "Sanden";
				break;
			case 12:
			case 9:
			case 6:
			case 3:
			case 0:
			default:
				odm = "unknown";
		}

		file = proc_create_single("odm", 0444, NULL, odm_show);
		if (!file)
			return -ENOMEM;

		file = proc_create_single("odmid", 0444, NULL, odmid_show);
		if (!file)
			return -ENOMEM;
	} else if (strcmp(compatible, "RK3568-ADC5-PRJID") == 0) {
		projectid = ret;
		*pid = projectid;

		switch(projectid) {
			case 18:
				if (odmid == 15)
					boardinfo = "Sanden - SKU1";
				else
					boardinfo = "Tinker Board 3N - SKU1";
				break;
			case 15:
				boardinfo = "Tinker Board 3N - SKU2";
				break;
			case 12:
				boardinfo = "Tinker Board 3N - SKU3";
				break;
			case 9:
			case 6:
			case 3:
			case 0:
			default:
				boardinfo = "unknown";
		}

		file = proc_create_single("boardinfo", 0444, NULL, info_show);
		if (!file)
			return -ENOMEM;

		file = proc_create_single("projectid", 0444, NULL, projectid_show);
		if (!file)
			return -ENOMEM;
	}

	if (!init_done) {
		file = proc_create_single("boardinfo_all", 0444, NULL, all_show);
		if (!file)
			return -ENOMEM;
		else
			init_done = 1;
	}

	return 0;
}
