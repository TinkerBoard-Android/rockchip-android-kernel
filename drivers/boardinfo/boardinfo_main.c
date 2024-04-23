#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>

#include "tb-setting.h"
#include "tb2-setting.h"
#include "tb3-setting.h"
#include "tb3n-setting.h"

static const char *model;
static int hwid = -1, pid = -1, odmid = -1;

static const struct of_device_id of_board_info_match[] = {
	{ .compatible = "board-info", },
	{ .compatible = "RK3568-ADC1-PCBID", },
	{ .compatible = "RK3568-ADC3-RAMID", },
	{ .compatible = "RK3568-ADC4-ODMID", },
	{ .compatible = "RK3568-ADC5-PRJID", },
	{ .compatible = "RK3566-ADC1-PCBID", },
	{ .compatible = "RK3566-ADC3-PRJID", },
	{},
};
MODULE_DEVICE_TABLE(of, of_board_info_match);

static int board_info_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const char *compatible;
	int ret;

	if (device_property_read_string(dev, "compatible", &compatible)) {
		printk("[boardinfo] Failed to read compatible");
		return -ENODEV;
	}
	printk("boardinfo: initialized %s\n", compatible);

	if (strcmp(compatible, "board-info") == 0) {
		if (device_property_read_string(dev, "model", &model))
			model = "unknow";

		if (!strcmp("rk3288", model))
			ret = tb_gpios(dev, &hwid, &pid);
		else if (!strcmp("rk3399", model))
			ret = tb2_gpios(dev, &hwid, &pid);
		else if (!strcmp("rk3568", model))
			ret = tb3n_gpios(dev);
		else if (!strcmp("rk3566", model))
			ret = tb3_gpios(dev);
		else
			ret = 0;
	} else {
		if (!strcmp("rk3568", model))
			ret = tb3n_adcs(dev, compatible, &hwid, &pid, &odmid);
		else if (!strcmp("rk3566", model))
			ret = tb3_adcs(dev, compatible, &hwid, &pid);
		else
			ret = 0;
	}

	if (ret < 0)
		return -EPROBE_DEFER;

	return 0;
}

int get_board_model(void)
{
	if (!strcmp("rk3288", model))
		return 3288;
	else if (!strcmp("rk3399", model))
		return 3399;
	else if (!strcmp("rk3568", model))
		return 3568;
	else if (!strcmp("rk3566", model))
		return 3566;
	else
		return -1;
}
EXPORT_SYMBOL_GPL(get_board_model);

int get_board_id(void)
{
	return hwid;
}
EXPORT_SYMBOL_GPL(get_board_id);

int get_project_id(void)
{
	return pid;
}
EXPORT_SYMBOL_GPL(get_project_id);

int get_odm_id(void)
{
	return odmid;
}
EXPORT_SYMBOL_GPL(get_odm_id);

static int board_info_remove(struct platform_device *pdev)
{
	if (!strcmp("rk3288", model))
		tb_gpios_free();
	else if (!strcmp("rk3399", model))
		tb2_gpios_free();
	else if (!strcmp("rk3568", model))
		tb3n_gpios_free();
	else if (!strcmp("rk3566", model))
		tb3_gpios_free();

	return 0;
}

static struct platform_driver boardinfo_driver = {
	.probe          = board_info_probe,
	.remove		= board_info_remove,
	.driver = {
		.name   = "board-info",
#ifdef CONFIG_OF_GPIO
		.of_match_table = of_match_ptr(of_board_info_match),
#endif
	},
};

module_platform_driver(boardinfo_driver);

MODULE_ALIAS("platform:boardinfo");
MODULE_AUTHOR("Frank Chiang <frank_chiang@asus.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver to set Board Information");
