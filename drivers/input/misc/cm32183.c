/* drivers/input/misc/cm32183.c - cm32183 optical sensors driver
 *
 * Copyright (C) 2021 Vishay Capella Microsystems Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/lightsensor.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
//#include <asm/mach-types.h>
#include <linux/cm32183.h>
#include <asm/setup.h>
#include <linux/jiffies.h>
#include <linux/platform_data/at24.h>

#define D(x...) pr_info(x)

#define I2C_RETRY_COUNT 10

#define LS_POLLING_DELAY 600

#define REL_ALS REL_X
#define REL_WHITE REL_Y

static void report_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(report_work, report_do_work);

struct cm32183_info {
	struct class *cm32183_class;
	struct device *ls_dev;
	struct input_dev *ls_input_dev;
	struct i2c_client *i2c_client;
	struct workqueue_struct *lp_wq;

	int als_enable;
	int (*power)(int, uint8_t); /* power to the chip */

	int lightsensor_opened;
	int polling_delay;
	uint16_t ls_cmd;
};
struct cm32183_info *lp_info;
int enable_log = 0;
static struct mutex als_enable_mutex, als_disable_mutex, als_get_adc_mutex;
static int lightsensor_enable(struct cm32183_info *lpi);
static int lightsensor_disable(struct cm32183_info *lpi);

static uint16_t cm32183_adc_als, cm32183_adc_white;
int cali = 65535;

static int I2C_RxData(uint16_t slaveAddr, uint8_t cmd, uint8_t *rxData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = 1,
		 .buf = &cmd,
		 },
		{
		 .addr = slaveAddr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(lp_info->i2c_client->adapter, msg, 2) > 0)
			break;

		msleep(10);
	}
	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[ERR][CM32183 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int I2C_TxData(uint16_t slaveAddr, uint8_t *txData, int length)
{
	uint8_t loop_i;

	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(lp_info->i2c_client->adapter, msg, 1) > 0)
			break;

		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[ERR][CM32183 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int _cm32183_I2C_Read_Word(uint16_t slaveAddr, uint8_t cmd, uint16_t *pdata)
{
	uint8_t buffer[2];
	int ret = 0;

	if (pdata == NULL)
		return -EFAULT;

	ret = I2C_RxData(slaveAddr, cmd, buffer, 2);
	if (ret < 0) {
		pr_err(
			"[ERR][CM32183 error]%s: I2C_RxData fail [0x%x, 0x%x]\n",
			__func__, slaveAddr, cmd);
		return ret;
	}

	*pdata = (buffer[1] << 8) | buffer[0];
#if 0
	/* Debug use */
	printk(KERN_DEBUG "[CM32183] %s: I2C_RxData[0x%x, 0x%x] = 0x%x\n",
		__func__, slaveAddr, cmd, *pdata);
#endif
	return ret;
}

static int _cm32183_I2C_Write_Word(uint16_t SlaveAddress, uint8_t cmd, uint16_t data)
{
	char buffer[3];
	int ret = 0;
#if 0
	/* Debug use */
	printk(KERN_DEBUG
	"[CM32183] %s: _cm32183_I2C_Write_Word[0x%x, 0x%x, 0x%x]\n",
		__func__, SlaveAddress, cmd, data);
#endif
	buffer[0] = cmd;
	buffer[1] = (uint8_t)(data & 0xff);
	buffer[2] = (uint8_t)((data & 0xff00) >> 8);

	ret = I2C_TxData(SlaveAddress, buffer, 3);
	if (ret < 0) {
		pr_err("[ERR][CM32183 error]%s: I2C_TxData fail\n", __func__);
		return -EIO;
	}

	return ret;
}

static void report_lsensor_input_event(struct cm32183_info *lpi, bool resume)
{/*when resume need report a data, so the paramerter need to quick reponse*/
	//uint32_t als_val, white_val;

	mutex_lock(&als_get_adc_mutex);

	_cm32183_I2C_Read_Word(CM32183_ADDR, CM32183_ALS_DATA, &cm32183_adc_als);
	_cm32183_I2C_Read_Word(CM32183_ADDR, CM32183_W_DATA, &cm32183_adc_white);


	input_report_rel(lpi->ls_input_dev, REL_ALS, (int) ((cm32183_adc_als + 1) * cali / 65535));
	input_report_rel(lpi->ls_input_dev, REL_WHITE, (int) cm32183_adc_white + 1);
	input_sync(lpi->ls_input_dev);
	//D("[LS][CM32183] %s als w : %x %x \n", __func__, cm32183_adc_als, cm32183_adc_white);

	mutex_unlock(&als_get_adc_mutex);
}

static void report_do_work(struct work_struct *work)
{
	struct cm32183_info *lpi = lp_info;

	if (enable_log)
		D("[CM32183] %s\n", __func__);

	report_lsensor_input_event(lpi, 0);

	queue_delayed_work(lpi->lp_wq, &report_work, lpi->polling_delay);
}

static int als_power(int enable)
{
	struct cm32183_info *lpi = lp_info;

	if (lpi->power)
		lpi->power(LS_PWR_ON, 1);

	return 0;
}

static int lightsensor_enable(struct cm32183_info *lpi)
{
	int ret = 0;

	mutex_lock(&als_enable_mutex);
	D("[LS][CM32183] %s\n", __func__);

	lpi->ls_cmd &= CM32183_CONF_SD_MASK;
	ret = _cm32183_I2C_Write_Word(CM32183_ADDR, CM32183_CONF, lpi->ls_cmd);
	if (ret < 0) {
		pr_err(
		"[LS][CM32183 error]%s: set auto light sensor fail\n",
		__func__);
	} else {
		msleep(200);/*wait for 200 ms for the first report*/
		report_lsensor_input_event(lpi, 1);/*resume, IOCTL and DEVICE_ATTR*/
	}

	queue_delayed_work(lpi->lp_wq, &report_work, lpi->polling_delay);
	lpi->als_enable = 1;
	mutex_unlock(&als_enable_mutex);

	return ret;
}

static int lightsensor_disable(struct cm32183_info *lpi)
{
	int ret = 0;

	mutex_lock(&als_disable_mutex);

	D("[LS][CM32183] %s\n", __func__);

	lpi->ls_cmd |= CM32183_CONF_SD;
	ret = _cm32183_I2C_Write_Word(CM32183_ADDR, CM32183_CONF, lpi->ls_cmd);
	if (ret < 0){
		pr_err("[LS][CM32183 error]%s: disable auto light sensor fail\n",
			__func__);
	}

	cancel_delayed_work_sync(&report_work);
	lpi->als_enable = 0;
	mutex_unlock(&als_disable_mutex);

	return ret;
}

static int lightsensor_open(struct inode *inode, struct file *file)
{
	struct cm32183_info *lpi = lp_info;
	int rc = 0;

	D("[LS][CM32183] %s\n", __func__);
	if (lpi->lightsensor_opened) {
		pr_err("[LS][CM32183 error]%s: already opened\n", __func__);
		rc = -EBUSY;
	}
	lpi->lightsensor_opened = 1;
	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	struct cm32183_info *lpi = lp_info;

	D("[LS][CM32183] %s\n", __func__);
	lpi->lightsensor_opened = 0;
	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int rc, val;
	struct cm32183_info *lpi = lp_info;

	switch (cmd) {
		case LIGHTSENSOR_IOCTL_ENABLE:
			if (get_user(val, (unsigned long __user *)arg)) {
				rc = -EFAULT;
				break;
			}
			D("[LS][CM32183] %s LIGHTSENSOR_IOCTL_ENABLE, value = %d\n",
				__func__, val);
			rc = val ? lightsensor_enable(lpi) : lightsensor_disable(lpi);
			break;
		case LIGHTSENSOR_IOCTL_GET_ENABLED:
			val = lpi->als_enable;
			D("[LS][CM32183] %s LIGHTSENSOR_IOCTL_GET_ENABLED, enabled %d\n",
				__func__, val);
			rc = put_user(val, (unsigned long __user *)arg);
			break;
		default:
			pr_err("[LS][CM32183 error]%s: invalid cmd %d\n",
				__func__, _IOC_NR(cmd));
			rc = -EINVAL;
	}

	return rc;
}

static const struct file_operations lightsensor_fops = {
	.owner = THIS_MODULE,
	.open = lightsensor_open,
	.release = lightsensor_release,
	.unlocked_ioctl = lightsensor_ioctl
};

static struct miscdevice lightsensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &lightsensor_fops
};

static ssize_t ls_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cm32183_info *lpi = lp_info;

	ret = sprintf(buf, "Light sensor Auto Enable = %d\n",
			lpi->als_enable);

	return ret;
}

static ssize_t ls_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret = 0;
	int ls_auto;
	struct cm32183_info *lpi = lp_info;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1)
		return -EINVAL;

	if (ls_auto) {
		ret = lightsensor_enable(lpi);
	} else {
		ret = lightsensor_disable(lpi);
	}

	D("[LS][CM32183] %s: lpi->als_enable = %d, ls_auto = %d\n",
		__func__, lpi->als_enable, ls_auto);

	if (ret < 0)
		pr_err(
		"[LS][CM32183 error]%s: set auto light sensor fail\n",
		__func__);

	return count;
}

static ssize_t ls_poll_delay_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cm32183_info *lpi = lp_info;

	ret = sprintf(buf, "Light sensor Poll Delay = %d ms\n",
			jiffies_to_msecs(lpi->polling_delay));

	return ret;
}

static ssize_t ls_poll_delay_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int new_delay;
	struct cm32183_info *lpi = lp_info;

	sscanf(buf, "%d", &new_delay);

	D("new delay = %d ms, old delay = %d ms\n",
		new_delay, jiffies_to_msecs(lpi->polling_delay));

	lpi->polling_delay = msecs_to_jiffies(new_delay);

	if( lpi->als_enable ){
		lightsensor_disable(lpi);
		lightsensor_enable(lpi);
	}

	return count;
}

static ssize_t ls_conf_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cm32183_info *lpi = lp_info;
	return sprintf(buf, "CONF_SETTING = %x\n", lpi->ls_cmd);
}

static ssize_t ls_conf_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int value = 0;
	struct cm32183_info *lpi = lp_info;
	sscanf(buf, "0x%x", &value);

	lpi->ls_cmd = value;
	printk(KERN_INFO "[LS]set CM32183_CONF = %x\n", lpi->ls_cmd);
	_cm32183_I2C_Write_Word(CM32183_ADDR, CM32183_CONF, lpi->ls_cmd);
	return count;
}

static ssize_t ls_als_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", cm32183_adc_als * cali / 65535);
}

static ssize_t ls_white_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", cm32183_adc_white);
}

static ssize_t calibration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", cali);
}

static ssize_t cal_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 value[5];
	at24_read_eeprom(value, 0x20, 5);
	return sprintf(buf, "%d %d %d %d %d\n", value[0], value[1],
			value[2], value[3], value[4]);
}

static ssize_t calibration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int stdls;
	char tmp[10];
	sscanf(buf, "%d %s", &stdls, tmp);
	cali = 65535;

	if (!strncmp(tmp, "-setcv", 6)) {
		cali = stdls;
		return count;
	}

	if (stdls < 0) {
		printk("Std light source: [%d] < 0 !!!\nCheck again, please.\n\
		Set calibration factor to 65535.\n", stdls);
		return -EBUSY;
	}

	mutex_lock(&als_get_adc_mutex);
	_cm32183_I2C_Read_Word(CM32183_ADDR, CM32183_ALS_DATA, &cm32183_adc_als);
	if (cm32183_adc_als != 0) {
		pr_info("cm32183_adc_als %d\n", cm32183_adc_als);
		cali = stdls * 65535 / cm32183_adc_als;
		pr_info("cali is %d\n", cali);
	} else
		pr_err("Lux value is zero!");
	mutex_unlock(&als_get_adc_mutex);
	return count;
}

static struct device_attribute dev_attr_light_enable =
__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP, ls_enable_show, ls_enable_store);

static struct device_attribute dev_attr_light_poll_delay =
__ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP, ls_poll_delay_show, ls_poll_delay_store);

static struct device_attribute dev_attr_light_conf =
__ATTR(conf, S_IRUGO | S_IWUSR | S_IWGRP, ls_conf_show, ls_conf_store);

static struct device_attribute dev_attr_light_als =
__ATTR(in_intensity_green, S_IRUGO | S_IRUSR | S_IRGRP, ls_als_show, NULL);

static struct device_attribute dev_attr_light_white =
__ATTR(in_intensity_ir, S_IRUGO | S_IRUSR | S_IRGRP, ls_white_show, NULL);

static struct device_attribute dev_attr_calibration =
__ATTR(calibration, S_IRUGO | S_IWUSR | S_IWGRP, calibration_show, calibration_store);

static struct device_attribute dev_attr_cal =
__ATTR(cal, S_IRUGO | S_IRUSR | S_IRGRP, cal_show, NULL);

static struct attribute *light_sysfs_attrs[] = {
&dev_attr_light_enable.attr,
&dev_attr_light_poll_delay.attr,
&dev_attr_light_conf.attr,
&dev_attr_light_als.attr,
&dev_attr_light_white.attr,
&dev_attr_calibration.attr,
&dev_attr_cal.attr,
NULL
};

static struct attribute_group light_attribute_group = {
.attrs = light_sysfs_attrs,
};

static int lightsensor_setup(struct cm32183_info *lpi)
{
	int ret;

	lpi->ls_input_dev = input_allocate_device();
	if (!lpi->ls_input_dev) {
		pr_err(
			"[LS][CM32183 error]%s: could not allocate ls input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->ls_input_dev->name = "cm32183-ls";

	input_set_capability(lpi->ls_input_dev, EV_REL, REL_ALS);
	input_set_capability(lpi->ls_input_dev, EV_REL, REL_WHITE);

	ret = input_register_device(lpi->ls_input_dev);
	if (ret < 0) {
		pr_err("[LS][CM32183 error]%s: can not register ls input device\n",
				__func__);
		goto err_free_ls_input_device;
	}

	return ret;

err_free_ls_input_device:
	input_free_device(lpi->ls_input_dev);
	return ret;
}

static int cm32183_setup(struct cm32183_info *lpi)
{
	int ret = 0;
 	uint16_t adc_data;

	als_power(1);
	msleep(5);

	lpi->ls_cmd = CM32183_CONF_IT_200MS | CM32183_CONF_CH_EN;

	// Shut down CM32183
	lpi->ls_cmd |= CM32183_CONF_SD;
	ret = _cm32183_I2C_Write_Word(CM32183_ADDR, CM32183_CONF, lpi->ls_cmd);
	if(ret < 0)
		return ret;

	// Enable CM32183
	lpi->ls_cmd &= CM32183_CONF_SD_MASK;
	ret = _cm32183_I2C_Write_Word(CM32183_ADDR, CM32183_CONF, lpi->ls_cmd);
	if(ret < 0)
		return ret;

	msleep(200);

	// Get initial ALS light data
	ret = _cm32183_I2C_Read_Word(CM32183_ADDR, CM32183_ALS_DATA, &adc_data);
	if (ret < 0) {
		pr_err(
			"[LS][CM32183 error]%s: _cm32183_I2C_Read_Word for GREEN fail\n",
			__func__);
		return -EIO;
	}

	// Get initial WHITE light data
	ret = _cm32183_I2C_Read_Word(CM32183_ADDR, CM32183_W_DATA, &adc_data);
	if (ret < 0) {
		pr_err(
			"[LS][CM32183 error]%s: _cm32183_I2C_Read_Word for BLUE fail\n",
			__func__);
		return -EIO;
	}

	return ret;
}

static int cm32183_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	struct cm32183_info *lpi;

	D("[CM32183] %s\n", __func__);

	lpi = kzalloc(sizeof(struct cm32183_info), GFP_KERNEL);
	if (!lpi)
		return -ENOMEM;

	lpi->i2c_client = client;

	i2c_set_clientdata(client, lpi);

	lpi->power = NULL; //if necessary, add power function here for sensor chip

	lpi->polling_delay = msecs_to_jiffies(LS_POLLING_DELAY);
	lp_info = lpi;

	mutex_init(&als_enable_mutex);
	mutex_init(&als_disable_mutex);
	mutex_init(&als_get_adc_mutex);

	ret = lightsensor_setup(lpi);
	if (ret < 0) {
		pr_err("[LS][CM32183 error]%s: lightsensor_setup error!!\n",
			__func__);
		goto err_lightsensor_setup;
	}

	lpi->lp_wq = create_singlethread_workqueue("cm32183_wq");
	if (!lpi->lp_wq) {
		pr_err("[CM32183 error]%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}

	ret = cm32183_setup(lpi);
	if (ret < 0) {
		pr_err("[ERR][CM32183 error]%s: cm32183_setup error!\n", __func__);
		goto err_cm32183_setup;
	}

	lpi->cm32183_class = class_create(THIS_MODULE, "optical_sensors");
	if (IS_ERR(lpi->cm32183_class)) {
		ret = PTR_ERR(lpi->cm32183_class);
		lpi->cm32183_class = NULL;
		goto err_create_class;
	}

	lpi->ls_dev = device_create(lpi->cm32183_class,
				NULL, 0, "%s", "lightsensor");
	if (unlikely(IS_ERR(lpi->ls_dev))) {
		ret = PTR_ERR(lpi->ls_dev);
		lpi->ls_dev = NULL;
		goto err_create_ls_device;
	}

	/* register the attributes */
        ret = sysfs_create_group(&lpi->i2c_client->dev.kobj,
	&light_attribute_group);
        ret = sysfs_create_group(&lpi->ls_input_dev->dev.kobj,
        &light_attribute_group);
	if (ret) {
		pr_err("[LS][CM32183 error]%s: could not create sysfs group\n", __func__);
		goto err_sysfs_create_group_light;
	}

	lpi->als_enable = 0;
	D("[CM32183] %s: Probe success!\n", __func__);

	return ret;

err_sysfs_create_group_light:
	device_unregister(lpi->ls_dev);
err_create_ls_device:
	class_destroy(lpi->cm32183_class);
err_create_class:
err_cm32183_setup:
	destroy_workqueue(lpi->lp_wq);
	mutex_destroy(&als_enable_mutex);
	mutex_destroy(&als_disable_mutex);
	mutex_destroy(&als_get_adc_mutex);
	input_unregister_device(lpi->ls_input_dev);
	input_free_device(lpi->ls_input_dev);
err_create_singlethread_workqueue:
	misc_deregister(&lightsensor_misc);
err_lightsensor_setup:
//err_platform_data_null:
	kfree(lpi);
	return ret;
}

static const struct i2c_device_id cm32183_i2c_id[] = {
	{CM32183_I2C_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static struct of_device_id cm32183_match_table[] = {
	{ .compatible = "capella,cm32183",},
	{ },
};
#else
#define cm32183_match_table NULL
#endif

static struct i2c_driver cm32183_driver = {
	.id_table = cm32183_i2c_id,
	.probe = cm32183_probe,
	.driver = {
		.name = CM32183_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(cm32183_match_table),
	},
};

static int __init cm32183_init(void)
{
	return i2c_add_driver(&cm32183_driver);
}

static void __exit cm32183_exit(void)
{
	i2c_del_driver(&cm32183_driver);
}

module_init(cm32183_init);
module_exit(cm32183_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("RGB sensor device driver for CM32183");
MODULE_AUTHOR("Frank Hsieh <frank.hsieh@vishay.com>");
