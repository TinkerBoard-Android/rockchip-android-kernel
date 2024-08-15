// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * gpio-fan-tb3.c - Hwmon driver for fans connected to GPIO lines.
 *
 * Copyright (C) 2010 LaCie
 *
 * Author: Simon Guinot <sguinot@lacie.com>
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/hwmon.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_platform.h>

struct gpio_fan_data {
	struct device		*dev;
	struct device		*hwmon_dev;
	struct mutex		lock; /* lock GPIOs operations. */
	struct gpio_desc	*speed_gpio;
	struct gpio_desc	*power_gpio;
};

static ssize_t fan_power_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gpio_fan_data *fan_data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", gpiod_get_value(fan_data->power_gpio));
}

static ssize_t fan_power_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct gpio_fan_data *fan_data = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 2, &val) || val > 1)
		return -EINVAL;

	mutex_lock(&fan_data->lock);

	gpiod_set_value(fan_data->power_gpio, val);

	mutex_unlock(&fan_data->lock);

	return count;
}

static ssize_t fan_speed_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gpio_fan_data *fan_data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", gpiod_get_value(fan_data->speed_gpio));
}

static ssize_t fan_speed_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct gpio_fan_data *fan_data = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 2, &val) || val > 1)
		return -EINVAL;

	mutex_lock(&fan_data->lock);

	gpiod_set_value(fan_data->speed_gpio, val);

	mutex_unlock(&fan_data->lock);

	return count;
}

static DEVICE_ATTR_RW(fan_power);
static DEVICE_ATTR_RW(fan_speed);

static umode_t gpio_fan_tb3_is_visible(struct kobject *kobj,
				   struct attribute *attr, int index)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct gpio_fan_data *data = dev_get_drvdata(dev);

	if (!data->power_gpio || !data->speed_gpio)
		return 0;

	return attr->mode;
}

static struct attribute *gpio_fan_tb3_attributes[] = {
	&dev_attr_fan_power.attr,
	&dev_attr_fan_speed.attr,
	NULL
};

static const struct attribute_group gpio_fan_tb3_group = {
	.attrs = gpio_fan_tb3_attributes,
	.is_visible = gpio_fan_tb3_is_visible,
};

static const struct attribute_group *gpio_fan_tb3_groups[] = {
	&gpio_fan_tb3_group,
	NULL
};

/* Must be called with fan_data->lock held, except during initialization. */
static void set_fan_power(struct gpio_fan_data *fan_data, int power)
{
	gpiod_set_value_cansleep(fan_data->power_gpio, power);
}

/*
 * Translate OpenFirmware node properties into platform_data
 */
static int gpio_fan_get_of_data(struct gpio_fan_data *fan_data)
{
	struct device *dev = fan_data->dev;

	/* Speed GPIO if one exists */
	fan_data->speed_gpio = devm_gpiod_get_optional(dev, "speed", GPIOD_OUT_LOW);
	if (IS_ERR(fan_data->speed_gpio))
		return PTR_ERR(fan_data->speed_gpio);

	/* Power GPIO if one exists */
	fan_data->power_gpio = devm_gpiod_get_optional(dev, "power", GPIOD_OUT_HIGH);
	if (IS_ERR(fan_data->power_gpio))
		return PTR_ERR(fan_data->power_gpio);

	return 0;
}

static const struct of_device_id of_gpio_fan_match[] = {
	{ .compatible = "gpio-fan-tb3", },
	{},
};
MODULE_DEVICE_TABLE(of, of_gpio_fan_match);

static void gpio_fan_stop(void *data)
{
	set_fan_power(data, 0);
}

static int gpio_fan_probe(struct platform_device *pdev)
{
	int err;
	struct gpio_fan_data *fan_data;
	struct device *dev = &pdev->dev;

	pr_info("%s +++\n", __func__);

	fan_data = devm_kzalloc(dev, sizeof(struct gpio_fan_data),
				GFP_KERNEL);
	if (!fan_data)
		return -ENOMEM;

	fan_data->dev = dev;
	err = gpio_fan_get_of_data(fan_data);
	if (err)
		return err;

	platform_set_drvdata(pdev, fan_data);
	mutex_init(&fan_data->lock);

	/* Configure control GPIOs if available. */
	if (fan_data->power_gpio) {
		err = devm_add_action_or_reset(dev, gpio_fan_stop, fan_data);
		if (err)
			return err;
	}

	/* Make this driver part of hwmon class. */
	fan_data->hwmon_dev =
		devm_hwmon_device_register_with_groups(dev,
						       "gpio_fan_tb3", fan_data,
						       gpio_fan_tb3_groups);
	if (IS_ERR(fan_data->hwmon_dev))
		return PTR_ERR(fan_data->hwmon_dev);

	pr_info("%s ---\n", __func__);

	return 0;
}

static void gpio_fan_shutdown(struct platform_device *pdev)
{
	struct gpio_fan_data *fan_data = platform_get_drvdata(pdev);

	pr_info("%s +++\n", __func__);

	if (fan_data->power_gpio)
		set_fan_power(fan_data, 0);

	pr_info("%s ---\n", __func__);
}

#ifdef CONFIG_PM_SLEEP
static int gpio_fan_suspend(struct device *dev)
{
	struct gpio_fan_data *fan_data = dev_get_drvdata(dev);

	pr_info("%s +++\n", __func__);

	if (fan_data->power_gpio)
		set_fan_power(fan_data, 0);

	pr_info("%s ---\n", __func__);

	return 0;
}

static int gpio_fan_resume(struct device *dev)
{
	struct gpio_fan_data *fan_data = dev_get_drvdata(dev);

	pr_info("%s +++\n", __func__);

	if (fan_data->power_gpio)
		set_fan_power(fan_data, 1);

	pr_info("%s ---\n", __func__);

	return 0;
}

static SIMPLE_DEV_PM_OPS(gpio_fan_pm, gpio_fan_suspend, gpio_fan_resume);
#define GPIO_FAN_PM	(&gpio_fan_pm)
#else
#define GPIO_FAN_PM	NULL
#endif

static struct platform_driver gpio_fan_tb3_driver = {
	.probe		= gpio_fan_probe,
	.shutdown	= gpio_fan_shutdown,
	.driver	= {
		.name	= "gpio-fan-tb3",
		.pm	= GPIO_FAN_PM,
		.of_match_table = of_match_ptr(of_gpio_fan_match),
	},
};

module_platform_driver(gpio_fan_tb3_driver);

MODULE_AUTHOR("Simon Guinot <sguinot@lacie.com>");
MODULE_DESCRIPTION("GPIO FAN driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:gpio-fan");
