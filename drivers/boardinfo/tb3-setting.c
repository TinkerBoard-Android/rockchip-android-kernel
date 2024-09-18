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
#include "tb3-setting.h"

static int projectid = -1, boardid = -1, ddrid = -1, emmcid = -1;
static char *boardver, *ddr, *emmc;
static char boardinfo[32];
static int emmc0_gpio = 0, emmc1_gpio = 0, emmc2_gpio = 0, ddr0_gpio = 0, ddr1_gpio = 0, ddr2_gpio = 0, init_done = 0;

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
	return 0;
}

static int all_open(struct inode *inode, struct file *file)
{
	return single_open(file, all_show, NULL);
}

static struct file_operations all_ops = {
	.owner	= THIS_MODULE,
	.open	= all_open,
	.read	= seq_read,
};

static int info_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", boardinfo);
	return 0;
}

static int info_open(struct inode *inode, struct file *file)
{
	return single_open(file, info_show, NULL);
}

static struct file_operations info_ops = {
	.owner	= THIS_MODULE,
	.open	= info_open,
	.read	= seq_read,
};

static int projectid_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", projectid);
	return 0;
}

static int projectid_open(struct inode *inode, struct file *file)
{
	return single_open(file, projectid_show, NULL);
}

static struct file_operations projectid_ops = {
	.owner	= THIS_MODULE,
	.open	= projectid_open,
	.read	= seq_read,
};

static int ver_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", boardver);
	return 0;
}

static int ver_open(struct inode *inode, struct file *file)
{
	return single_open(file, ver_show, NULL);
}

static struct file_operations ver_ops = {
	.owner	= THIS_MODULE,
	.open	= ver_open,
	.read	= seq_read,
};

static int boardid_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", boardid);
	return 0;
}

static int boardid_open(struct inode *inode, struct file *file)
{
	return single_open(file, boardid_show, NULL);
}

static struct file_operations boardid_ops = {
	.owner	= THIS_MODULE,
	.open	= boardid_open,
	.read	= seq_read,
};

static int ddr_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", ddr);
	return 0;
}

static int ddr_open(struct inode *inode, struct file *file)
{
	return single_open(file, ddr_show, NULL);
}

static struct file_operations ddr_ops = {
	.owner	= THIS_MODULE,
	.open	= ddr_open,
	.read	= seq_read,
};

static int ddrid_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", ddrid);
	return 0;
}

static int ddrid_open(struct inode *inode, struct file *file)
{
	return single_open(file, ddrid_show, NULL);
}

static struct file_operations ddrid_ops = {
	.owner	= THIS_MODULE,
	.open	= ddrid_open,
	.read	= seq_read,
};

static int emmc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", emmc);
	return 0;
}

static int emmc_open(struct inode *inode, struct file *file)
{
	return single_open(file, emmc_show, NULL);
}

static struct file_operations emmc_ops = {
	.owner	= THIS_MODULE,
	.open	= emmc_open,
	.read	= seq_read,
};

static int emmcid_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", emmcid);
	return 0;
}

static int emmcid_open(struct inode *inode, struct file *file)
{
	return single_open(file, emmcid_show, NULL);
}

static struct file_operations emmcid_ops = {
	.owner	= THIS_MODULE,
	.open	= emmcid_open,
	.read	= seq_read,
};

int tb3_gpios(struct device *dev)
{
	int ret;
	struct proc_dir_entry* file;
	int id0, id1, id2;

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

	emmc2_gpio = of_get_named_gpio(dev->of_node, "emmc2-gpios", 0);
	if (!gpio_is_valid(emmc2_gpio)) {
		printk("[boardinfo] No emmc2-gpio pin available in boardinfo\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, emmc2_gpio, GPIOF_DIR_IN, "GPIO_EMMC2");
		if (ret < 0) {
			printk("[boardinfo] Failed to request EMMC2 gpio: %d\n", ret);
			return ret;
		}
	}

	id0 = gpio_get_value(emmc0_gpio);
	id1 = gpio_get_value(emmc1_gpio);
	id2 = gpio_get_value(emmc2_gpio);

	emmcid = (id2 << 2) + (id1 << 1) + id0;

	switch(emmcid) {
		case 0:
			emmc = "NONE";
			break;
		case 1:
			emmc = "16GB";
			break;
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		default:
			emmc = "unknown";
	}

	file = proc_create("emmc", 0444, NULL, &emmc_ops);
	if (!file)
		return -ENOMEM;

	file = proc_create("emmcid", 0444, NULL, &emmcid_ops);
	if (!file)
		return -ENOMEM;

	ddr0_gpio = of_get_named_gpio(dev->of_node, "ddr0-gpios", 0);
	if (!gpio_is_valid(ddr0_gpio)) {
		printk("[boardinfo] No ddr0-gpio pin available in boardinfo\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, ddr0_gpio, GPIOF_DIR_IN, "GPIO_DDR0");
		if (ret < 0) {
			printk("[boardinfo] Failed to request DDR0 gpio: %d\n", ret);
			return ret;
		}
	}

	ddr1_gpio = of_get_named_gpio(dev->of_node, "ddr1-gpios", 0);
	if (!gpio_is_valid(ddr1_gpio)) {
		printk("[boardinfo] No ddr1-gpio pin available in boardinfo\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, ddr1_gpio, GPIOF_DIR_IN, "GPIO_DDR1");
		if (ret < 0) {
			printk("[boardinfo] Failed to request DDR1 gpio: %d\n", ret);
			return ret;
		}
	}

	ddr2_gpio = of_get_named_gpio(dev->of_node, "ddr2-gpios", 0);
	if (!gpio_is_valid(ddr2_gpio)) {
		printk("[boardinfo] No ddr2-gpio pin available in boardinfo\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, ddr2_gpio, GPIOF_DIR_IN, "GPIO_DDR2");
		if (ret < 0) {
			printk("[boardinfo] Failed to request DDR2 gpio: %d\n", ret);
			return ret;
		}
	}

	id0 = gpio_get_value(ddr0_gpio);
	id1 = gpio_get_value(ddr1_gpio);
	id2 = gpio_get_value(ddr2_gpio);

	ddrid = (id2 << 2) + (id1 << 1) + id0;

	switch(ddrid) {
		case 0:
			ddr = "2GB";
			break;
		case 1:
			ddr = "4GB";
			break;
		case 2:
			ddr = "1GB";
			break;
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		default:
			ddr = "unknown";
	}

	file = proc_create("ddr", 0444, NULL, &ddr_ops);
	if (!file)
		return -ENOMEM;

	file = proc_create("ddrid", 0444, NULL, &ddrid_ops);
	if (!file)
		return -ENOMEM;

	if (!init_done) {
		file = proc_create("boardinfo_all", 0444, NULL, &all_ops);
		if (!file)
			return -ENOMEM;
		else
			init_done = 1;
	}

	return 0;
}

void tb3_gpios_free(void)
{
	gpio_free(emmc0_gpio);
	gpio_free(emmc1_gpio);
	gpio_free(emmc2_gpio);
	gpio_free(ddr0_gpio);
	gpio_free(ddr1_gpio);
	gpio_free(ddr2_gpio);
}

int tb3_adcs(struct device *dev, const char *compatible, const char *board, int *hwid, int *pid)
{
	int ret, raw, vref, bits, vresult;
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
		ret = iio_read_channel_raw(channels, &raw);
		if (ret < 0) {
			printk("[boardinfo][iio] Failed to read channel raw\n");
			return ret;
		}

		ret = iio_read_channel_scale(channels, &vref, &bits);
		if (ret < 0) {
			printk("[boardinfo][iio] Failed to read channel scale\n");
			return ret;
		}
	}

	vresult = vref * raw / ((2 << (bits - 1)) - 1);

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

	if (strcmp(compatible, "RK3566-ADC1-PCBID") == 0) {
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

		file = proc_create("boardver", 0444, NULL, &ver_ops);
		if (!file)
			return -ENOMEM;

		file = proc_create("boardid", 0444, NULL, &boardid_ops);
		if (!file)
			return -ENOMEM;
	} else if (strcmp(compatible, "RK3566-ADC3-PRJID") == 0) {
		projectid = ret;
		*pid = projectid;

		switch(projectid) {
			case 18:
				snprintf(boardinfo, strlen(board) + strlen(" - SKU1") + 1, "%s - SKU1", board);
				break;
			case 15:
				snprintf(boardinfo, strlen(board) + strlen(" - SKU2") + 1, "%s - SKU2", board);
				break;
			case 12:
				snprintf(boardinfo, strlen(board) + strlen(" - SKU3") + 1, "%s - SKU3", board);
				break;
			case 9:
			case 6:
			case 3:
			case 0:
			default:
				snprintf(boardinfo, strlen(board) + 1, "%s", board);
		}

		file = proc_create("boardinfo", 0444, NULL, &info_ops);
		if (!file)
			return -ENOMEM;

		file = proc_create("projectid", 0444, NULL, &projectid_ops);
		if (!file)
			return -ENOMEM;
	}

	if (!init_done) {
		file = proc_create("boardinfo_all", 0444, NULL, &all_ops);
		if (!file)
			return -ENOMEM;
		else
			init_done = 1;
	}

	return 0;
}
