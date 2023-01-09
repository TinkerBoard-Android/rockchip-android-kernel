#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>

#include "tb-setting.h"
#include "tb2-setting.h"
#include "tb3-setting.h"

static const char *model;
static int hwid = -1, pid = -1, odmid = -1;

static const struct of_device_id of_board_info_match[] = {
	{ .compatible = "board-info", },
	{ .compatible = "ADC1-PCBID", },
	{ .compatible = "ADC3-RAMID", },
	{ .compatible = "ADC4-ODMID", },
	{ .compatible = "ADC5-PRJID", },
	{},
};
MODULE_DEVICE_TABLE(of, of_board_info_match);

static int board_info_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const char *compatible;

	if (device_property_read_string(dev, "compatible", &compatible)) {
		printk("[boardinfo] Failed to read compatible");
		return -ENODEV;
	}
	printk("boardinfo: initialized %s\n", compatible);

	if (strcmp(compatible, "board-info") == 0) {
		if (device_property_read_string(dev, "model", &model))
			model = "unknow";

		if (!strcmp(model, "rk3288"))
			tb_gpios(dev, &hwid, &pid);
		else if (!strcmp(model, "rk3399"))
			tb2_gpios(dev, &hwid, &pid);
		else if (!strcmp(model, "rk3568"))
			tb3_gpios(dev);
	} else {
		tb3_adcs(dev, compatible, &hwid, &pid, &odmid);
	}

	return 0;
}

int get_board_model(void)
{
	if (!strcmp(model, "rk3288"))
		return 3288;
	else if (!strcmp(model, "rk3399"))
		return 3399;
	else if (!strcmp(model, "rk3568"))
		return 3568;
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
	if (!strcmp(model, "rk3288"))
		tb_gpios_free();
	else if (!strcmp(model, "rk3399"))
		tb2_gpios_free();
	else if (!strcmp(model, "rk3568"))
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
