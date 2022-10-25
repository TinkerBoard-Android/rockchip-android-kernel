/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Luca Hsu <luca_hsu@ilitek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301 USA.
 *
 */

#include "ilitek_ts.h"
#include "ilitek_common.h"
#include "ilitek_protocol.h"
#include <linux/firmware.h>

#ifdef ILITEK_TOOL
struct dev_data {
	dev_t devno;
	struct cdev cdev;
	struct class *class;
};

static struct dev_data ilitek_dev;
static struct proc_dir_entry *ilitek_proc;
static struct proc_dir_entry *ilitek_proc_entry;
static struct proc_dir_entry *ilitek_proc_irq_enable;
uint32_t set_update_len = UPGRADE_LENGTH_BLV1_8;

#define ILITEK_IOCTL_BASE                       100
#define ILITEK_IOCTL_I2C_WRITE_DATA             _IOWR(ILITEK_IOCTL_BASE, 0, uint8_t*)
#define ILITEK_IOCTL_I2C_WRITE_LENGTH           _IOWR(ILITEK_IOCTL_BASE, 1, int32_t)
#define ILITEK_IOCTL_I2C_READ_DATA              _IOWR(ILITEK_IOCTL_BASE, 2, uint8_t*)
#define ILITEK_IOCTL_I2C_READ_LENGTH            _IOWR(ILITEK_IOCTL_BASE, 3, int32_t)
#define ILITEK_IOCTL_USB_WRITE_DATA             _IOWR(ILITEK_IOCTL_BASE, 4, uint8_t*)
#define ILITEK_IOCTL_USB_WRITE_LENGTH           _IOWR(ILITEK_IOCTL_BASE, 5, int32_t)
#define ILITEK_IOCTL_USB_READ_DATA              _IOWR(ILITEK_IOCTL_BASE, 6, uint8_t*)
#define ILITEK_IOCTL_USB_READ_LENGTH            _IOWR(ILITEK_IOCTL_BASE, 7, int32_t)
#define ILITEK_IOCTL_DRIVER_INFORMATION		_IOWR(ILITEK_IOCTL_BASE, 8, int32_t)
#define ILITEK_IOCTL_USB_UPDATE_RESOLUTION      _IOWR(ILITEK_IOCTL_BASE, 9, int32_t)
#define ILITEK_IOCTL_I2C_INT_FLAG	        _IOWR(ILITEK_IOCTL_BASE, 10, int32_t)
#define ILITEK_IOCTL_I2C_UPDATE                 _IOWR(ILITEK_IOCTL_BASE, 11, int32_t)
#define ILITEK_IOCTL_STOP_READ_DATA             _IOWR(ILITEK_IOCTL_BASE, 12, int32_t)
#define ILITEK_IOCTL_START_READ_DATA            _IOWR(ILITEK_IOCTL_BASE, 13, int32_t)
#define ILITEK_IOCTL_GET_INTERFANCE		_IOWR(ILITEK_IOCTL_BASE, 14, int32_t)	//default setting is i2c interface
#define ILITEK_IOCTL_I2C_SWITCH_IRQ		_IOWR(ILITEK_IOCTL_BASE, 15, int32_t)
#define ILITEK_IOCTL_UPDATE_FLAG		_IOWR(ILITEK_IOCTL_BASE, 16, int32_t)
#define ILITEK_IOCTL_I2C_UPDATE_FW		_IOWR(ILITEK_IOCTL_BASE, 18, int32_t)
#define ILITEK_IOCTL_RESET			_IOWR(ILITEK_IOCTL_BASE, 19, int32_t)
#define ILITEK_IOCTL_INT_STATUS			_IOWR(ILITEK_IOCTL_BASE, 20, int32_t)

#ifdef ILITEK_TUNING_MESSAGE
extern bool ilitek_debug_flag;
#define ILITEK_IOCTL_DEBUG_SWITCH		_IOWR(ILITEK_IOCTL_BASE, 21, int32_t)
#endif

#define ILITEK_IOCTL_I2C_INT_CLR		_IOWR(ILITEK_IOCTL_BASE, 22, int32_t)
#define ILITEK_IOCTL_I2C_INT_POLL		_IOWR(ILITEK_IOCTL_BASE, 23, bool*)

#define ILITEK_DEVICE_NODE_PERMISSON			0755

static int32_t ilitek_file_open(struct inode *inode, struct file *filp)
{
	ts->operation_protection = true;
	tp_msg("operation_protection = %d\n", ts->operation_protection);
	return 0;
}

static ssize_t ilitek_file_write(struct file *filp, const char *buf, size_t size, loff_t *f_pos)
{
	int32_t ret = 0, count = 0;
	uint8_t buffer[512] = { 0 };
	uint32_t *data;
	char *token = NULL, *cur = NULL;

	ret = copy_from_user(buffer, buf, size - 1);
	if (ret < 0) {
		tp_err("copy data from user space, failed");
		return -1;
	}

	token = cur = buffer;

	data = kcalloc(size, sizeof(u32), GFP_KERNEL);

	while ((token = strsep(&cur, ",")) != NULL) {
		//data[count] = str2hex(token);
		sscanf(token,"%x", &data[count]);
		tp_msg("data[%d] = %x\n", count, data[count]);
		count++;
	}

	if (buffer[size - 2] == 'I' && (size == 20 || size == 52) && buffer[0] == 0x77 && buffer[1] == 0x77) {

		tp_msg("IOCTL_WRITE CMD = %d\n", buffer[2]);
		switch (buffer[2]) {
		case 13:
			//ilitek_irq_enable();
			tp_msg("ilitek_irq_enable. do nothing\n");
			break;
		case 12:
			//ilitek_irq_disable();
			tp_msg("ilitek_irq_disable. do nothing\n");
			break;
		case 19:
			ilitek_reset(ts->reset_time);
			break;
#ifdef ILITEK_TUNING_MESSAGE
		case 21:
			tp_msg("ilitek The ilitek_debug_flag = %d.\n", buffer[3]);
			if (buffer[3] == 0) {
				ilitek_debug_flag = false;
			} else if (buffer[3] == 1) {
				ilitek_debug_flag = true;
			}
			break;
#endif
		case 15:
			if (buffer[3] == 0)
				ilitek_irq_disable();
			else
				ilitek_irq_enable();

			break;
		case 16:
			ts->operation_protection = buffer[3];
			tp_msg("ts->operation_protection = %d\n", ts->operation_protection);
			break;
		case 18:
			ilitek_irq_disable();
			mutex_lock(&ts->ilitek_mutex);
			ret = ilitek_i2c_write(&buffer[3], 33);
			mutex_unlock(&ts->ilitek_mutex);
			ilitek_irq_enable();
			if (ret < 0)
				tp_err("i2c write error, ret %d, addr %x\n", ret, ts->client->addr);

			return ret;
			break;
		default:
			return -1;
		}
	}

	if (buffer[size - 2] == 'W') {
		ilitek_irq_disable();
		mutex_lock(&ts->ilitek_mutex);
		ret = ilitek_i2c_write(buffer, size - 2);
		mutex_unlock(&ts->ilitek_mutex);
		ilitek_irq_enable();
		if (ret < 0) {
			tp_err("i2c write error, ret %d, addr %x\n", ret, ts->client->addr);
			return ret;
		}
	} else if (strcmp(buffer, "unhandle_irq") == 0) {
		ts->unhandle_irq = !ts->unhandle_irq;
		tp_msg("ts->unhandle_irq = %d.\n", ts->unhandle_irq);
	} else if (strcmp(buffer, "dbg_debug") == 0) {
		ts->ilitek_log_level_value = ILITEK_DEBUG_LOG_LEVEL;
		tp_msg("ilitek_log_level_value = %d.\n", ts->ilitek_log_level_value);
	} else if (strcmp(buffer, "dbg_info") == 0) {
		ts->ilitek_log_level_value = ILITEK_INFO_LOG_LEVEL;
		tp_msg("ilitek_log_level_value = %d.\n", ts->ilitek_log_level_value);
	} else if (strcmp(buffer, "dbg_err") == 0) {
		ts->ilitek_log_level_value = ILITEK_ERR_LOG_LEVEL;
		tp_msg("ilitek_log_level_value = %d.\n", ts->ilitek_log_level_value);
	} else if (strcmp(buffer, "dbg_num") == 0) {
		tp_msg("ilitek_log_level_value = %d.\n", ts->ilitek_log_level_value);
	}
#ifdef ILITEK_TUNING_MESSAGE
	else if (strcmp(buffer, "truning_dbg_flag") == 0) {
		ilitek_debug_flag = !ilitek_debug_flag;
		tp_msg(" %s debug_flag message(%X).\n", ilitek_debug_flag ? "Enabled" : "Disabled", ilitek_debug_flag);
	}
#endif
	else if (strcmp(buffer, "irq_status") == 0) {
		tp_msg("gpio_get_value(i2c.irq_gpio) = %d.\n", gpio_get_value(ts->irq_gpio));
	} else if (strcmp(buffer, "enable") == 0) {
		ilitek_irq_enable();
		tp_msg("irq enable\n");
	} else if (strcmp(buffer, "disable") == 0) {
		ilitek_irq_disable();
		tp_msg("irq disable\n");
	} else if (strcmp(buffer, "info") == 0) {
		ilitek_irq_disable();
		mutex_lock(&ts->ilitek_mutex);
		ilitek_read_tp_info(false);
		mutex_unlock(&ts->ilitek_mutex);
		ilitek_irq_enable();
	} else if (strcmp(buffer, "reset") == 0) {
		ilitek_reset(ts->reset_time);
	} else if (strcmp(buffer, "update_protocol") == 0) {
		ilitek_irq_disable();
		mutex_lock(&ts->ilitek_mutex);
		ret = api_init_func();
		mutex_unlock(&ts->ilitek_mutex);
		ilitek_irq_enable();
	}
	else if (strncmp(buffer, "setlen,", 6) == 0) {
		set_update_len = data[1];
		tp_msg( "set_update_len=%d\n", set_update_len);
	}

	tp_dbg("ilitek return count = %d\n", (int32_t)size);
	kfree(data);
	return size;
}

/*
   description
   ioctl function for character device driver
   prarmeters
   inode
   file node
   filp
   file pointer
   cmd
   command
   arg
   arguments
   return
   status
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
static long ilitek_file_ioctl(struct file *filp, uint32_t cmd, unsigned long arg)
#else
static int32_t ilitek_file_ioctl(struct inode *inode, struct file *filp, uint32_t cmd, unsigned long arg)
#endif
{
	static uint8_t *buffer;
	static int32_t len = 0;
	int32_t ret = ILITEK_SUCCESS;
	int tmp;

	buffer = kmalloc(ILITEK_IOCTL_MAX_TRANSFER, GFP_KERNEL);
	memset(buffer, 0 , ILITEK_IOCTL_MAX_TRANSFER);
	switch (cmd) {
	case ILITEK_IOCTL_I2C_WRITE_DATA:
		if (copy_from_user(buffer, (uint8_t *)arg, len)) {
			tp_err("copy data from user space, failed\n");
			ret = -EFAULT;
			break;
		}

		mutex_lock(&ts->ilitek_mutex);
		ret = ilitek_i2c_write_and_read(buffer, len, 0, NULL, 0);
		mutex_unlock(&ts->ilitek_mutex);
		if (ret < 0)
			tp_err("i2c write failed, cmd: %x\n", buffer[0]);
		break;
	case ILITEK_IOCTL_I2C_READ_DATA:
		mutex_lock(&ts->ilitek_mutex);
		ret = ilitek_i2c_write_and_read(NULL, 0, 0, buffer, len);
		mutex_unlock(&ts->ilitek_mutex);
		if (ret < 0) {
			tp_err("i2c read failed, buf: %x\n", buffer[0]);
			break;
		}

		if (copy_to_user((uint8_t *)arg, buffer, len)) {
			ret = -EFAULT;
			tp_err("copy data to user space, failed\n");
		}
		break;
	case ILITEK_IOCTL_I2C_WRITE_LENGTH:
	case ILITEK_IOCTL_I2C_READ_LENGTH:
		len = arg;
		break;
	case ILITEK_IOCTL_DRIVER_INFORMATION:
		memcpy(buffer, ilitek_driver_information, 7);
		if (copy_to_user((uint8_t *)arg, buffer, 7))
			ret = -EFAULT;
		break;
	case ILITEK_IOCTL_I2C_UPDATE:
		break;
	case ILITEK_IOCTL_I2C_INT_FLAG:
		buffer[0] = !(gpio_get_value(ts->irq_gpio));
		if (copy_to_user((uint8_t *)arg, buffer, 1)) {
			tp_err("copy data to user space, failed\n");
			ret = -EFAULT;
			break;
		}
		tp_dbg("ILITEK_IOCTL_I2C_INT_FLAG = %d.\n", buffer[0]);
		break;
	case ILITEK_IOCTL_START_READ_DATA:
		ilitek_irq_enable();
		ts->unhandle_irq = false;
		tp_msg("enable_irq and ts->unhandle_irq = false.\n");
		break;
	case ILITEK_IOCTL_STOP_READ_DATA:
		ilitek_irq_disable();
		ts->unhandle_irq = true;
		tp_msg("disable_irq and ts->unhandle_irq = true.\n");
		break;
	case ILITEK_IOCTL_RESET:
		ilitek_reset(ts->reset_time);
		break;
	case ILITEK_IOCTL_INT_STATUS:
		if (put_user(gpio_get_value(ts->irq_gpio), (int32_t *)arg))
			ret = -EFAULT;
		break;
#ifdef ILITEK_TUNING_MESSAGE
	case ILITEK_IOCTL_DEBUG_SWITCH:
		if (copy_from_user(buffer, (uint8_t *)arg, 1)) {
			ret = -EFAULT;
			break;
		}
		tp_msg("ilitek The debug_flag = %d.\n", buffer[0]);
		if (buffer[0] == 0)
			ilitek_debug_flag = false;
		else if (buffer[0] == 1)
			ilitek_debug_flag = true;
		break;
#endif
	case ILITEK_IOCTL_I2C_SWITCH_IRQ:
		if (copy_from_user(buffer, (uint8_t *)arg, 1)) {
			ret = -EFAULT;
			break;
		}

		if (buffer[0] == 0)
			ilitek_irq_disable();
		else
			ilitek_irq_enable();

		break;
	case ILITEK_IOCTL_UPDATE_FLAG:
		ts->operation_protection = arg;
		tp_msg("operation_protection = %d\n", ts->operation_protection);
		break;
	case ILITEK_IOCTL_I2C_UPDATE_FW:
		if (copy_from_user(buffer, (uint8_t *)arg, 35)) {
			tp_err("copy data from user space, failed\n");
			ret = -EFAULT;
			break;
		}

		ilitek_irq_disable();
		mutex_lock(&ts->ilitek_mutex);
		ret = ilitek_i2c_write_and_read(buffer, buffer[34], 0, NULL, 0);
		mutex_unlock(&ts->ilitek_mutex);
		ilitek_irq_enable();

		if (ret < 0)
			tp_err("i2c write, failed\n");

		break;
	case ILITEK_IOCTL_I2C_INT_CLR:
		tp_msg("ILITEK_IOCTL_I2C_INT_CLR, set get_INT false\n");
		atomic_set(&ts->get_INT, 0);
		break;
	case ILITEK_IOCTL_I2C_INT_POLL:
		tmp = atomic_read(&ts->get_INT);
		tp_msg("ILITEK_IOCTL_I2C_INT_POLL, get_INT: %d\n", tmp);

		if (copy_to_user((uint8_t *)arg, &tmp, 1)) {
			tp_err("copy data to user space, failed\n");
			ret = -EFAULT;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

	kfree(buffer);
	return (ret < 0) ? ret : 0;
}

/*
   description
   read function for character device driver
   prarmeters
   filp
   file pointer
   buf
   buffer
   count
   buffer length
   f_pos
   offset
   return
   status
 */
static ssize_t ilitek_file_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	uint8_t *tmp;
	int32_t ret;
	long rc;

	//tp_msg("%s enter count = %d\n", __func__, count);

	if (count > 8192)
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL)
		return -ENOMEM;
	ilitek_irq_disable();
	mutex_lock(&ts->ilitek_mutex);
	ret = ilitek_i2c_read(tmp, count);
	mutex_unlock(&ts->ilitek_mutex);
	ilitek_irq_enable();
	if (ret < 0) {
		tp_err("i2c read error, ret %d,addr %x\n", ret, ts->client->addr);
	}
	rc = copy_to_user(buf, tmp, count);

	kfree(tmp);
	tmp = NULL;
	return ret > 0 ? count : ret;
}

/*
   description
   close function
   prarmeters
   inode
   inode
   filp
   file pointer
   return
   status
 */
static int32_t ilitek_file_close(struct inode *inode, struct file *filp)
{
	ts->operation_protection = false;
	tp_msg("operation_protection = %d\n", ts->operation_protection);
	return 0;
}

// declare file operations
static struct file_operations ilitek_fops = {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
	.unlocked_ioctl = ilitek_file_ioctl,
#else
	.ioctl = ilitek_file_ioctl,
#endif
	.read = ilitek_file_read,
	.write = ilitek_file_write,
	.open = ilitek_file_open,
	.release = ilitek_file_close,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static struct proc_ops ilitek_proc_ops = {
	.proc_ioctl = ilitek_file_ioctl,
	.proc_read = ilitek_file_read,
	.proc_write = ilitek_file_write,
	.proc_open = ilitek_file_open,
	.proc_release = ilitek_file_close,
};
#endif

static ssize_t ilitek_irq_enable_read(struct file *pFile, char __user *buf, size_t nCount, loff_t *pPos)
{
	int32_t ret = 0;
	uint8_t tmpbuf[128] = { 0 };
	tp_msg("\n");
	if (*pPos != 0) {
		return 0;
	}
	tp_msg("irq trigger count %d, irq handle count %d\n", ts->irq_trigger_count, ts->irq_handle_count);
	nCount = scnprintf(tmpbuf, PAGE_SIZE, "irq trigger count %d, irq handle count %d\n", ts->irq_trigger_count, ts->irq_handle_count);
	ret = copy_to_user(buf, tmpbuf, nCount);
	*pPos += nCount;

	ilitek_irq_enable();

	return nCount;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 6, 0)
static struct file_operations ilitek_irq_enable_fops = {
	.read = ilitek_irq_enable_read,
	.write = NULL,
};
#else
static struct proc_ops ilitek_irq_enable_fops = {
	.proc_read = ilitek_irq_enable_read,
	.proc_write = NULL,
};
#endif

int32_t ilitek_check_busy(int32_t count, int32_t delay, int32_t type)
{
	int32_t i;
	uint8_t buf[2];

	for (i = 0; i < count; i++) {
		buf[0] = CMD_GET_SYS_BUSY;
		if (ilitek_i2c_write_and_read(buf, 1, 1, buf, 1) < 0)
			return ILITEK_I2C_TRANSFER_ERR;

		if ((buf[0] & (ILITEK_TP_SYSTEM_READY + type)) == ILITEK_TP_SYSTEM_READY)
			return ILITEK_SUCCESS;

		msleep(delay);
	}
	tp_msg("check_busy is busy,0x%x\n", buf[0]);
	return ILITEK_FAIL;
}

void tp_msg_arr(const char *tag, const uint8_t *buf, int size)
{
	int i;

	tp_msg("%s :\n", tag);
	for (i = 0; i < size; i++) {
		if (i % 16 == 15) {
			printk(KERN_CONT"[0x%02X]\n", buf[i]);
		} else {
			printk(KERN_CONT"[0x%02X]", buf[i]);
		}
	}
	printk("\n");
}

static ssize_t ilitek_update_with_hex_read(struct file *pFile, char __user *buf,
					   size_t nCount, loff_t *pPos)
{
	int32_t error = 0;
	uint8_t tmpbuf[256];

	tp_msg("\n");

	if (*pPos)
		return 0;

	memset(tmpbuf, 0, sizeof(tmpbuf));

	if ((error = ilitek_upgrade_firmware()) < 0) {
		nCount = scnprintf(tmpbuf, PAGE_SIZE,
				   "upgrade failed, err: %d\n", error);
		goto err_return;
	}

	nCount = scnprintf(tmpbuf, PAGE_SIZE,
			   "upgrade success, fw version: %d.%d.%d.%d.%d.%d.%d.%d\n",
			   ts->fw_ver[0], ts->fw_ver[1],
			   ts->fw_ver[2], ts->fw_ver[3],
			   ts->fw_ver[4], ts->fw_ver[5],
			   ts->fw_ver[6], ts->fw_ver[7]);

err_return:
	if (copy_to_user(buf, tmpbuf, nCount))
		return -EFAULT;

	*pPos += nCount;
	return nCount;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 6, 0)
static struct file_operations ilitek_proc_fops_fwupdate = {
	.read = ilitek_update_with_hex_read,
};
#else
static struct proc_ops ilitek_proc_fops_fwupdate = {
	.proc_read = ilitek_update_with_hex_read,
};
#endif

static ssize_t ilitek_firmware_version_read(struct file *pFile, char __user *buf, size_t nCount, loff_t *pPos)
{
	int32_t ret = 0;
	uint8_t tmpbuf[256] = { 0 };

	tp_msg("\n");

	if (*pPos != 0)
		return 0;

	ilitek_irq_disable();
	mutex_lock(&ts->ilitek_mutex);
	ret = ilitek_read_tp_info(false);
	mutex_unlock(&ts->ilitek_mutex);
	ilitek_irq_enable();

	if (ret < 0) {
		tp_err("ilitek_read_tp_info err ret = %d\n", ret);
		nCount = scnprintf(tmpbuf, PAGE_SIZE, "ilitek firmware version read error ret = %d\n", ret);

	} else {
		nCount = scnprintf(tmpbuf, PAGE_SIZE, "ilitek firmware version is %d.%d.%d.%d.%d.%d.%d.%d\n",
 				   ts->fw_ver[0], ts->fw_ver[1], ts->fw_ver[2],
 				   ts->fw_ver[3], ts->fw_ver[4], ts->fw_ver[5],
 				   ts->fw_ver[6], ts->fw_ver[7]);
	}
	*pPos += nCount;
	ret = copy_to_user(buf, tmpbuf, nCount);
	return nCount;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 6, 0)
static struct file_operations ilitek_proc_fops_fwversion = {
	.read = ilitek_firmware_version_read,
	.write = NULL,
};
#else
static struct proc_ops ilitek_proc_fops_fwversion = {
	.proc_read = ilitek_firmware_version_read,
	.proc_write = NULL,
};
#endif

static ssize_t ilitek_setmode_0(struct file *pFile, char __user *buf, size_t nCount, loff_t *pPos)
{
	int ret = ILITEK_SUCCESS;
	ilitek_irq_disable();
	mutex_lock(&ts->ilitek_mutex);
	ret = api_set_testmode(true);
	ret = api_set_funcmode(ILITEK_TP_SET_MODE_0);
	ret = api_set_testmode(false);
	mutex_unlock(&ts->ilitek_mutex);
	ilitek_irq_enable();
	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 6, 0)
static struct file_operations ilitek_proc_fops_setmode_0 = {
	.read = ilitek_setmode_0,
	.write = NULL,
};
#else
static struct proc_ops ilitek_proc_fops_setmode_0 = {
	.proc_read = ilitek_setmode_0,
	.proc_write = NULL,
};
#endif

static ssize_t ilitek_setmode_1(struct file *pFile, char __user *buf, size_t nCount, loff_t *pPos)
{
	int ret = ILITEK_SUCCESS;
	ilitek_irq_disable();
	mutex_lock(&ts->ilitek_mutex);
	ret = api_set_testmode(true);
	ret = api_set_funcmode(ILITEK_TP_SET_MODE_1);
	ret = api_set_testmode(false);
	mutex_unlock(&ts->ilitek_mutex);
	ilitek_irq_enable();
	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 6, 0)
static struct file_operations ilitek_proc_fops_setmode_1 = {
	.read = ilitek_setmode_1,
	.write = NULL,
};
#else
static struct proc_ops ilitek_proc_fops_setmode_1 = {
	.proc_read = ilitek_setmode_1,
	.proc_write = NULL,
};
#endif

static ssize_t ilitek_setmode_2(struct file *pFile, char __user *buf, size_t nCount, loff_t *pPos)
{
	int ret = ILITEK_SUCCESS;
	ilitek_irq_disable();
	mutex_lock(&ts->ilitek_mutex);
	ret = api_set_testmode(true);
	ret = api_set_funcmode(ILITEK_TP_SET_MODE_2);
	ret = api_set_testmode(false);
	mutex_unlock(&ts->ilitek_mutex);
	ilitek_irq_enable();
	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 6, 0)
static struct file_operations ilitek_proc_fops_setmode_2 = {
	.read = ilitek_setmode_2,
	.write = NULL,
};
#else
static struct proc_ops ilitek_proc_fops_setmode_2 = {
	.proc_read = ilitek_setmode_2,
	.proc_write = NULL,
};
#endif

static ssize_t ilitek_gesture_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (ts->gesture_status)
		return scnprintf(buf, PAGE_SIZE, "gesture: on, state: %hhu\n",
				 ts->gesture_status);

	return scnprintf(buf, PAGE_SIZE, "gesture: off\n");
}

static ssize_t ilitek_gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	uint8_t type;

	sscanf(buf, "%hhu", &type);

	tp_msg("[%s] gesture: before/now (%hhx, %hhx)\n",
		__func__, ts->gesture_status, type);

	if (type < 0 || type >= Gesture_Undefined)
		return -EINVAL;

	if (!ts->gesture_status && type)
		ilitek_register_gesture(ts, true);
	else if (ts->gesture_status && !type)
		ilitek_register_gesture(ts, false);

	ts->gesture_status = type;

	return size;
}
static DEVICE_ATTR(gesture, 0664, ilitek_gesture_show, ilitek_gesture_store);

static ssize_t ilitek_firmware_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;

	tp_msg("\n");

	ilitek_irq_disable();
	mutex_lock(&ts->ilitek_mutex);
	ret = ilitek_read_tp_info(false);
	mutex_unlock(&ts->ilitek_mutex);
	ilitek_irq_enable();

	if (ret < 0) {
		tp_err("ilitek_read_tp_info err ret = %d\n", ret);
		return scnprintf(buf, PAGE_SIZE, "ilitek firmware version read error ret = %d\n", ret);
	}

	return scnprintf(buf, PAGE_SIZE, "ilitek firmware version is %d.%d.%d.%d.%d.%d.%d.%d\n",
 			 ts->fw_ver[0], ts->fw_ver[1], ts->fw_ver[2],
 			 ts->fw_ver[3], ts->fw_ver[4], ts->fw_ver[5],
 			 ts->fw_ver[6], ts->fw_ver[7]);
}

static DEVICE_ATTR(firmware_version, 0664, ilitek_firmware_version_show, NULL);

static ssize_t ilitek_module_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	uint8_t tmpbuf[64];

	tp_msg("\n");
	ilitek_irq_disable();
	mutex_lock(&ts->ilitek_mutex);
	ret = api_ptl_set_cmd(GET_MCU_VER, NULL, tmpbuf);
	mutex_unlock(&ts->ilitek_mutex);
	ilitek_irq_enable();
	return scnprintf(buf, PAGE_SIZE, "%s\n", ts->product_id);
}

static DEVICE_ATTR(product_id, 0664, ilitek_module_name_show, NULL);

static ssize_t ilitek_update_fw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int error;

	tp_msg("\n");

	if ((error = ilitek_upgrade_firmware()) < 0)
		return scnprintf(buf, PAGE_SIZE, "upgrade failed, err: %d\n",
				 error);

	return scnprintf(buf, PAGE_SIZE, "upgrade success, fw version: %d.%d.%d.%d.%d.%d.%d.%d\n",
			   ts->fw_ver[0], ts->fw_ver[1],
			   ts->fw_ver[2], ts->fw_ver[3],
			   ts->fw_ver[4], ts->fw_ver[5],
			   ts->fw_ver[6], ts->fw_ver[7]);
}

static DEVICE_ATTR(update_fw, 0664, ilitek_update_fw_show, NULL);

static struct attribute *ilitek_sysfs_attrs_ctrl[] = {
	&dev_attr_firmware_version.attr,
	&dev_attr_product_id.attr,
	&dev_attr_gesture.attr,
	&dev_attr_update_fw.attr,
	NULL
};

static struct attribute_group ilitek_attribute_group[] = {
	{.attrs = ilitek_sysfs_attrs_ctrl},
};

int ilitek_create_sysfsnode(void)
{
	int ret = 0;
	struct i2c_client *client = ts->client;

	ts->ilitek_func_kobj = kobject_create_and_add("touchscreen", NULL);
	if (ts->ilitek_func_kobj == NULL) {
		tp_err("kobject_create_and_add failed\n");
	} else {
		ret = sysfs_create_group(ts->ilitek_func_kobj, ilitek_attribute_group);
		if (ret < 0) {
			tp_err("sysfs_create_group failed\n");
			kobject_put(ts->ilitek_func_kobj);
		}
	}
	ret = sysfs_create_group(&client->dev.kobj, ilitek_attribute_group);
	if (ret < 0) {
		tp_err("sysfs_create_group failed\n");
		kobject_put(&client->dev.kobj);
	}
	return ret;
}


int32_t ilitek_create_tool_node(void)
{
	int32_t ret = 0;

	ret = alloc_chrdev_region(&ilitek_dev.devno, 0, 1, "ilitek_file");
	if (ret) {
		tp_err("can't allocate chrdev\n");
	} else {
		tp_msg("register chrdev(%d, %d)\n", MAJOR(ilitek_dev.devno), MINOR(ilitek_dev.devno));

		// initialize character device driver
		cdev_init(&ilitek_dev.cdev, &ilitek_fops);
		ilitek_dev.cdev.owner = THIS_MODULE;
		ret = cdev_add(&ilitek_dev.cdev, ilitek_dev.devno, 1);
		if (ret < 0) {
			tp_err("add character device error, ret %d\n", ret);
		} else {
			ilitek_dev.class = class_create(THIS_MODULE, "ilitek_file");
			if (IS_ERR(ilitek_dev.class))
				tp_err("create class, error\n");

			device_create(ilitek_dev.class, NULL, ilitek_dev.devno, NULL, "ilitek_ctrl");
		}
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 6, 0)
	ilitek_proc = proc_create("ilitek_ctrl", ILITEK_DEVICE_NODE_PERMISSON, NULL, &ilitek_fops);
#else
	ilitek_proc = proc_create("ilitek_ctrl", ILITEK_DEVICE_NODE_PERMISSON, NULL, &ilitek_proc_ops);
#endif

	if (!ilitek_proc)
		tp_err("proc_create(ilitek_ctrl, ILITEK_DEVICE_NODE_PERMISSON, NULL, &ilitek_fops) fail\n");

	ilitek_proc_irq_enable = proc_create("ilitek_irq_enable", ILITEK_DEVICE_NODE_PERMISSON, NULL, &ilitek_irq_enable_fops);
	if (!ilitek_proc_irq_enable)
		tp_err("proc_create(ilitek_irq_enable, ILITEK_DEVICE_NODE_PERMISSON, NULL, &ilitek_irq_enable_fops) fail\n");

	ilitek_proc_entry = proc_mkdir("ilitek", NULL);
	if (!ilitek_proc_entry) {
		tp_err("Error, failed to creat procfs.\n");
		return -EINVAL;
	}

	if (!proc_create("firmware_version", ILITEK_DEVICE_NODE_PERMISSON, ilitek_proc_entry, &ilitek_proc_fops_fwversion)) {
		tp_err("Error, failed to creat procfs firmware_version.\n");
		remove_proc_entry("firmware_version", ilitek_proc_entry);
	}
	if (!proc_create("update_fw", ILITEK_DEVICE_NODE_PERMISSON, ilitek_proc_entry, &ilitek_proc_fops_fwupdate)) {
		tp_err("Error, failed to creat procfs update_fw.\n");
		remove_proc_entry("update_fw", ilitek_proc_entry);
	}

	if (!proc_create("setmode_0", ILITEK_DEVICE_NODE_PERMISSON, ilitek_proc_entry, &ilitek_proc_fops_setmode_0)) {
		tp_err("Error, failed to creat procfs setmode_0.\n");
		remove_proc_entry("setmode_0", ilitek_proc_entry);
	}
	if (!proc_create("setmode_1", ILITEK_DEVICE_NODE_PERMISSON, ilitek_proc_entry, &ilitek_proc_fops_setmode_1)) {
		tp_err("Error, failed to creat procfs setmode_1.\n");
		remove_proc_entry("setmode_1", ilitek_proc_entry);
	}
	if (!proc_create("setmode_2", ILITEK_DEVICE_NODE_PERMISSON, ilitek_proc_entry, &ilitek_proc_fops_setmode_2)) {
		tp_err("Error, failed to creat procfs setmode_1.\n");
		remove_proc_entry("setmode_2", ilitek_proc_entry);
	}

	return 0;
}

void ilitek_remove_sys_node(void)
{
	if (ts->ilitek_func_kobj) {
		sysfs_remove_group(ts->ilitek_func_kobj, ilitek_attribute_group);
		kobject_put(ts->ilitek_func_kobj);
		ts->ilitek_func_kobj = NULL;
	}
	if (&ts->client->dev.kobj) {
		sysfs_remove_group(&ts->client->dev.kobj, ilitek_attribute_group);
		kobject_put(&ts->client->dev.kobj);
		//&ts->client->dev.kobj = NULL;
	}
}

int32_t ilitek_remove_tool_node(void)
{
	cdev_del(&ilitek_dev.cdev);
	unregister_chrdev_region(ilitek_dev.devno, 1);
	device_destroy(ilitek_dev.class, ilitek_dev.devno);
	class_destroy(ilitek_dev.class);
	if (ilitek_proc) {
		tp_msg("remove procfs ilitek_ctrl.\n");
		remove_proc_entry("ilitek_ctrl", NULL);
		ilitek_proc = NULL;
	}
	if (ilitek_proc_irq_enable) {
		tp_msg("remove procfs ilitek_irq_enable.\n");
		remove_proc_entry("ilitek_irq_enable", NULL);
		ilitek_proc_irq_enable = NULL;
	}

	if (ilitek_proc_entry) {
		tp_msg("remove procfs inode\n");
		remove_proc_entry("firmware_version", ilitek_proc_entry);
		remove_proc_entry("update_fw", ilitek_proc_entry);
		remove_proc_entry("setmode_2", ilitek_proc_entry);
		remove_proc_entry("setmode_1", ilitek_proc_entry);
		remove_proc_entry("setmode_0", ilitek_proc_entry);
		remove_proc_entry("ilitek", NULL);
		ilitek_proc_entry = NULL;
	}
	return 0;
}
#endif
