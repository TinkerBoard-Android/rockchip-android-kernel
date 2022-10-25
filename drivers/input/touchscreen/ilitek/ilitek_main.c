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
struct ilitek_ts_data *ts;

char ilitek_driver_information[] = { DERVER_VERSION_0, DERVER_VERSION_1, DERVER_VERSION_2, DERVER_VERSION_3, CUSTOMER_H_ID, CUSTOMER_L_ID, TEST_VERSION};
#if ILITEK_PLAT == ILITEK_PLAT_MTK
extern struct tpd_device *tpd;
#ifdef ILITEK_ENABLE_DMA
static uint8_t *I2CDMABuf_va;
static dma_addr_t I2CDMABuf_pa;
#endif
#endif

#if defined(ILITEK_WAKELOCK_SUPPORT)
struct wake_lock ilitek_wake_lock;
#endif

#ifdef ILITEK_TUNING_MESSAGE
static struct sock *ilitek_netlink_sock;
bool ilitek_debug_flag;
static u_int ilitek_pid = 100, ilitek_seq = 23;
#endif

static void __maybe_unused ilitek_udp_reply(void *payload, int size)
{
#ifdef ILITEK_TUNING_MESSAGE
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	int len = NLMSG_SPACE(size);
	int ret;
	int pid = ilitek_pid, seq = ilitek_seq;

	tp_dbg("[%s] ilitek_debug_flag: %d\n", __func__, ilitek_debug_flag);
	if (!ilitek_debug_flag)
		return;

	skb = alloc_skb(len, GFP_ATOMIC);
	if (!skb) {
		tp_msg("alloc skb error\n");
		return;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	nlh = nlmsg_put(skb, pid, seq, 0, size, 0);
#else
	nlh = NLMSG_PUT(skb, pid, seq, 0, size);
#endif

	nlh->nlmsg_flags = 0;
	memcpy(NLMSG_DATA(nlh), payload, size);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	NETLINK_CB(skb).portid = 0;	/* from kernel */
#else
	NETLINK_CB(skb).pid = 0;	/* from kernel */
#endif

	NETLINK_CB(skb).dst_group = 0;	/* unicast */

	ret = netlink_unicast(ilitek_netlink_sock, skb, pid, MSG_DONTWAIT);
	if (ret < 0) {
		tp_err("ilitek send failed, ret: %d\n", ret);
		return;
	}
	return;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
nlmsg_failure:			/* Used by NLMSG_PUT */
	if (skb)
		kfree_skb(skb);
#endif
#endif /* ILITEK_TUNING_MESSAGE */
}

static void __maybe_unused udp_receive(struct sk_buff *skb)
{
#ifdef ILITEK_TUNING_MESSAGE
	int count = 0, ret = 0, i = 0;
	uint8_t *data;
	struct nlmsghdr *nlh;

	nlh = (struct nlmsghdr *)skb->data;
	ilitek_pid = NETLINK_CREDS(skb)->pid;
	ilitek_seq = nlh->nlmsg_seq;

	tp_dbg("netlink received, pid: %d, seq: %d\n",
		     ilitek_pid, ilitek_seq);

	data = (uint8_t *) NLMSG_DATA(nlh);
	count = nlmsg_len(nlh);
	if (!strcmp(data, "Open!")) {
		tp_msg("data is :%s\n", (char *)data);
		ts->operation_protection = true;
		ilitek_udp_reply(data, sizeof("Open!"));
	} else if (!strcmp(data, "Close!")) {
		tp_msg("data is :%s\n", (char *)data);
		ts->operation_protection = false;
	} else if (!strcmp(data, "Wifi_Paint_Start") ||
		   !strcmp(data, "Daemon_Debug_Start")) {
		ilitek_debug_flag = true;
	} else if (!strcmp(data, "Wifi_Paint_End") ||
		   !strcmp(data, "Daemon_Debug_End")) {
		ilitek_debug_flag = false;
	}


	tp_dbg("count = %d  data[count -3] = %d data[count -2] = %c\n", count, data[count - 3], data[count - 2]);
	for (i = 0; i < count; i++) {
		//tp_msg("data[%d] = 0x%x\n", i, data[i]);
	}
	if (data[count - 2] == 'I' && (count == 20 || count == 52) &&
	    data[0] == 0x77 && data[1] == 0x77) {
		tp_dbg("IOCTL_WRITE CMD = %d\n", data[2]);
		switch (data[2]) {
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
		case 21:
			tp_msg("ilitek The ilitek_debug_flag = %d.\n", data[3]);
			if (data[3] == 0)
				ilitek_debug_flag = false;
			else if (data[3] == 1)
				ilitek_debug_flag = true;
			break;
		case 15:
			if (data[3] == 0) {
				ilitek_irq_disable();
				tp_dbg("ilitek_irq_disable.\n");
			} else {
				ilitek_irq_enable();
				tp_dbg("ilitek_irq_enable.\n");
			}
			break;
		case 16:
			ts->operation_protection = data[3];
			tp_msg("ts->operation_protection = %d\n", ts->operation_protection);
			break;
		case 8:
			tp_msg("get driver version\n");
			ilitek_udp_reply(ilitek_driver_information, 7);
			break;
		case 18:
			tp_dbg("firmware update write 33 bytes data\n");
			ret = ilitek_i2c_write(&data[3], 33);
			if (ret < 0)
				tp_err("i2c write error, ret %d, addr %x\n", ret, ts->client->addr);
			if (ret < 0) {
				data[0] = 1;
			} else {
				data[0] = 0;
			}
			ilitek_udp_reply(data, 1);
			return;
			break;
		default:
			return;
		}
	} else if (data[count - 2] == 'W') {
		ret = ilitek_i2c_write(data, count - 2);
		if (ret < 0)
			tp_err("i2c write error, ret %d, addr %x\n", ret, ts->client->addr);
		if (ret < 0) {
			data[0] = 1;
		} else {
			data[0] = 0;
		}
		ilitek_udp_reply(data, 1);
	} else if (data[count - 2] == 'R') {
		ret = ilitek_i2c_read(data, count - 2);
		if (ret < 0)
			tp_err("i2c read error, ret %d, addr %x\n", ret, ts->client->addr);
		if (ret < 0) {
			data[count - 2] = 1;
		} else {
			data[count - 2] = 0;
		}
		ilitek_udp_reply(data, count - 1);
	}
#endif /* ILITEK_TUNING_MESSAGE */
}

#ifdef ILITEK_ESD_PROTECTION
static void ilitek_esd_check(struct work_struct *work)
{
	int i = 0;
	uint8_t buf[64];


	if (ts->operation_protection) {
		tp_msg("ilitek esd ts->operation_protection is true so not check\n");
		goto ilitek_esd_check_out;
	}

	mutex_lock(&ts->ilitek_mutex);

	if (!ts->esd_check) {
		tp_msg("esd not need check ts->esd_check is false!!!\n");
		goto ilitek_esd_check_out;
	}

	for (i = 0; i < 3; i++) {
		if (api_ptl_set_cmd(GET_PTL_VER, NULL, buf) < 0) {
			tp_err("ilitek esd  i2c communication error\n");
			continue;
		}

		if (buf[0] != ts->ptl.ver_major) {
			tp_err("unexpected ptl ver %x vs. %x\n", buf[0], ts->ptl.ver);
			continue;
		}

		tp_msg("[%s] pass\n", __func__);
		goto ilitek_esd_check_out;
	}

	ilitek_reset(ts->reset_time);

ilitek_esd_check_out:
	mutex_unlock(&ts->ilitek_mutex);
	ts->esd_check = true;
	queue_delayed_work(ts->esd_wq, &ts->esd_work, ts->esd_delay);
}
#endif

void ilitek_irq_enable(void)
{
	if (!ts->irq_registerred)
		return;

	if (atomic_read(&ts->irq_enabled))
		return;

#ifdef MTK_UNDTS
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#else
	enable_irq(ts->client->irq);
#endif

	atomic_set(&ts->irq_enabled, 1);
	tp_dbg("\n");
}

void ilitek_irq_disable(void)
{
	if (!ts->irq_registerred)
		return;

	if (!atomic_read(&ts->irq_enabled))
		return;

#ifdef MTK_UNDTS
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#else
	disable_irq_nosync(ts->client->irq);
#endif

	atomic_set(&ts->irq_enabled, 0);
	tp_dbg("\n");
}

#ifdef ILITEK_ENABLE_DMA
static int ilitek_dma_i2c_read(struct i2c_client *client, uint8_t *buf, int len)
{
	int i = 0, err = 0;

	if (len < 8) {
		client->ext_flag = client->ext_flag & (~I2C_DMA_FLAG);
		//client->addr = client->addr & I2C_MASK_FLAG;
		return i2c_master_recv(client, buf, len);
	}
	client->ext_flag = client->ext_flag | I2C_DMA_FLAG;
	//client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;
	err = i2c_master_recv(client, (uint8_t *)I2CDMABuf_pa, len);
	if (err < 0)
		return err;
	for (i = 0; i < len; i++)
		buf[i] = I2CDMABuf_va[i];
	return err;
}

static int ilitek_dma_i2c_write(struct i2c_client *client, uint8_t *pbt_buf, int dw_len)
{
	int i = 0;

	if (dw_len <= 8) {
		client->ext_flag = client->ext_flag & (~I2C_DMA_FLAG);
		//client->addr = client->addr & I2C_MASK_FLAG;
		return i2c_master_send(client, pbt_buf, dw_len);
	}
	for (i = 0; i < dw_len; i++)
		I2CDMABuf_va[i] = pbt_buf[i];
	client->ext_flag = client->ext_flag | I2C_DMA_FLAG;
	//client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;
	return i2c_master_send(client, (uint8_t *)I2CDMABuf_pa, dw_len);
}
#endif

static int ilitek_i2c_transfer(struct i2c_msg *msgs, int cnt)
{
	int ret = 0;
	struct i2c_client *client = ts->client;
	int count = ILITEK_I2C_RETRY_COUNT;
#ifdef ILITEK_ENABLE_DMA
	int i = 0;

	for (i = 0; i < cnt; i++) {
		while (count >= 0) {
			count -= 1;
			msgs[i].ext_flag = 0;
			if (msgs[i].flags == I2C_M_RD)
				ret = ilitek_dma_i2c_read(client, msgs[i].buf, msgs[i].len);
			else if (msgs[i].flags == 0)
				ret = ilitek_dma_i2c_write(client, msgs[i].buf, msgs[i].len);

			if (ret < 0) {
				tp_err("ilitek i2c transfer err\n");
				mdelay(20);
				continue;
			}
			break;
		}
	}
#else
#if ILITEK_PLAT == ILITEK_PLAT_ROCKCHIP
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 0, 0)
	int i = 0;

	for (i = 0; i < cnt; i++)
		msgs[i].scl_rate = 400000;
#endif
#endif
	while (count >= 0) {
		count -= 1;
		ret = i2c_transfer(client->adapter, msgs, cnt);
		if (ret < 0) {
			tp_err("ilitek_i2c_transfer cmd: %x, err: %d\n",
				(msgs[0].buf != NULL) ? msgs[0].buf[0] : 0, ret);
			mdelay(20);
			continue;
		}
		break;
	}
#endif
	return ret;
}

int ilitek_i2c_write(uint8_t *cmd, int length)
{
	int ret = 0;
	struct i2c_client *client = ts->client;
	struct i2c_msg msgs[] = {
		{.addr = client->addr, .flags = 0, .len = length, .buf = cmd,}
	};

	ret = ilitek_i2c_transfer(msgs, 1);
	if (ret < 0)
		tp_err("%s, i2c write error, ret %d\n", __func__, ret);
	return ret;
}

int ilitek_i2c_read(uint8_t *data, int length)
{
	int ret = 0;
	struct i2c_client *client = ts->client;
	struct i2c_msg msgs_ret[] = {
		{.addr = client->addr, .flags = I2C_M_RD, .len = length, .buf = data,}
	};

	ret = ilitek_i2c_transfer(msgs_ret, 1);
	if (ret < 0)
		tp_err("%s, i2c read error, ret %d\n", __func__, ret);

	return ret;
}

int ilitek_i2c_write_and_read(uint8_t *cmd, int write_len, int delay, uint8_t *data, int read_len)
{
	int ret = 0;
	struct i2c_client *client = ts->client;
	struct i2c_msg msgs[2] = {
		{.addr = client->addr, .flags = 0, .len = write_len, .buf = cmd,},
		{.addr = client->addr, .flags = I2C_M_RD, .len = read_len, .buf = data,}
	};

	if (ts->ilitek_repeat_start && delay == 0 &&
		write_len > 0 && read_len > 0) {
		ret = ilitek_i2c_transfer(msgs, 2);
		if (ret < 0)
			return ret;
	} else {
		if (write_len > 0) {
			ret = ilitek_i2c_transfer(msgs, 1);
			if (ret < 0)
				return ret;
		}
		if (delay > 0)
			mdelay(delay);
		if (read_len > 0) {
			ret = ilitek_i2c_transfer(msgs + 1, 1);
			if (ret < 0)
				return ret;
		}
	}

	return ret;
}

void __maybe_unused ilitek_gpio_dbg(void)
{
#if defined(ILITEK_GPIO_DEBUG)
	gpio_direction_output(ts->test_gpio, 0);
	mdelay(1);
	gpio_direction_output(ts->test_gpio, 1);
#endif
}

void ilitek_reset(int delay)
{
	tp_msg("delay: %d\n", delay);

	ilitek_irq_disable();

#ifdef MTK_UNDTS
	mt_set_gpio_mode(ILITEK_RESET_GPIO, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(ILITEK_RESET_GPIO, GPIO_DIR_OUT);
	mt_set_gpio_out(ILITEK_RESET_GPIO, GPIO_OUT_ONE);
	mdelay(10);

	mt_set_gpio_mode(ILITEK_RESET_GPIO, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(ILITEK_RESET_GPIO, GPIO_DIR_OUT);
	mt_set_gpio_out(ILITEK_RESET_GPIO, GPIO_OUT_ZERO);
	mdelay(10);

	mt_set_gpio_mode(ILITEK_RESET_GPIO, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(ILITEK_RESET_GPIO, GPIO_DIR_OUT);
	mt_set_gpio_out(ILITEK_RESET_GPIO, GPIO_OUT_ONE);
	mdelay(delay);

#else

	if (ts->reset_gpio >= 0) {
#if ILITEK_PLAT != ILITEK_PLAT_MTK
		gpio_direction_output(ts->reset_gpio, 1);
		mdelay(10);
		gpio_direction_output(ts->reset_gpio, 0);
		mdelay(10);
		gpio_direction_output(ts->reset_gpio, 1);
		mdelay(delay);
#else
		tpd_gpio_output(ts->reset_gpio, 1);
		mdelay(10);
		tpd_gpio_output(ts->reset_gpio, 0);
		mdelay(10);
		tpd_gpio_output(ts->reset_gpio, 1);
		mdelay(delay);
#endif
	} else {
		tp_err("reset pin is invalid\n");
	}
#endif

	ilitek_irq_enable();
}

static int ilitek_free_gpio(void)
{

#ifndef MTK_UNDTS
	if (gpio_is_valid(ts->reset_gpio)) {
		tp_msg("reset_gpio is valid so free\n");
		gpio_free(ts->reset_gpio);
	}
	if (gpio_is_valid(ts->irq_gpio)) {
		tp_msg("irq_gpio is valid so free\n");
		gpio_free(ts->irq_gpio);
	}
#endif

#if defined(ILITEK_GPIO_DEBUG)
	if (gpio_is_valid(ts->test_gpio)) {
		tp_msg("test_gpio is valid so free\n");
		gpio_free(ts->test_gpio);
	}
#endif

	return 0;
}

static int ilitek_request_input_dev(void)
{
	int error = 0;
	int i = 0;
	struct input_dev *input;

#ifdef ILITEK_USE_MTK_INPUT_DEV
	ts->input_dev = tpd->dev;
#else
	ts->input_dev = input_allocate_device();
#endif
	if (!ts->input_dev) {
		tp_err("allocate input device, error\n");
		return -EFAULT;
	}

	input = ts->input_dev;

	tp_dbg("[%s] start\n", __func__);

#ifndef ILITEK_USE_MTK_INPUT_DEV
	__set_bit(INPUT_PROP_DIRECT, input->propbit);
	input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

#if !ILITEK_ROTATE_FLAG
#ifdef ILITEK_USE_LCM_RESOLUTION
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, TOUCH_SCREEN_X_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, TOUCH_SCREEN_Y_MAX, 0, 0);
#else
	input_set_abs_params(input, ABS_MT_POSITION_X, ts->screen_min_x, ts->screen_max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, ts->screen_min_y, ts->screen_max_y, 0, 0);
#endif
#else
#ifdef ILITEK_USE_LCM_RESOLUTION
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, TOUCH_SCREEN_Y_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, TOUCH_SCREEN_X_MAX, 0, 0);
#else
	input_set_abs_params(input, ABS_MT_POSITION_X, ts->screen_min_y, ts->screen_max_y, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, ts->screen_min_x, ts->screen_max_x, 0, 0);
#endif
#endif
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	input->name = ILITEK_TS_NAME;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &(ts->client)->dev;
#endif

#if defined(ILITEK_USE_MTK_INPUT_DEV) && defined(MTK_UNDTS)
	if (tpd_dts_data.use_tpd_button) {
		for (i = 0; i < tpd_dts_data.tpd_key_num; i++)
			input_set_capability(input, EV_KEY, tpd_dts_data.tpd_key_local[i]);
	}
#endif

#ifdef ILITEK_TOUCH_PROTOCOL_B
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0)
	input_mt_init_slots(input, ts->max_tp, INPUT_MT_DIRECT);
#else
	input_mt_init_slots(input, ts->max_tp);
#endif
#else
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, ts->max_tp, 0, 0);
#endif

#ifdef ILITEK_REPORT_PRESSURE
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, 255, 0, 0);
#endif

	for (i = 0; i < ts->keycount; i++)
		set_bit(ts->keyinfo[i].id & KEY_MAX, input->keybit);

	input_set_capability(input, EV_KEY, KEY_POWER);

#ifndef ILITEK_USE_MTK_INPUT_DEV
	error = input_register_device(ts->input_dev);
	if (error) {
		tp_err("input_register_device failed, err: %d\n", error);
		input_free_device(ts->input_dev);
	}
#endif

	return error;
}

static int ilitek_touch_down(int id, int x, int y, int p, int h, int w)
{
	struct input_dev *input = ts->input_dev;
	if (y != ILITEK_RESOLUTION_MAX) {

#if defined(ILITEK_USE_MTK_INPUT_DEV) || defined(ILITEK_USE_LCM_RESOLUTION)
		x = (x - ts->screen_min_x) * TOUCH_SCREEN_X_MAX / (ts->screen_max_x - ts->screen_min_x);
		y = (y - ts->screen_min_y) * TOUCH_SCREEN_Y_MAX / (ts->screen_max_y - ts->screen_min_y);
#endif
	}
	input_report_key(input, BTN_TOUCH, 1);
#ifdef ILITEK_TOUCH_PROTOCOL_B
	input_mt_slot(input, id);
	input_mt_report_slot_state(input, MT_TOOL_FINGER, true);
#endif
#if !ILITEK_ROTATE_FLAG
	input_event(input, EV_ABS, ABS_MT_POSITION_X, x);
	input_event(input, EV_ABS, ABS_MT_POSITION_Y, y);
#else
	input_event(input, EV_ABS, ABS_MT_POSITION_X, y);
	input_event(input, EV_ABS, ABS_MT_POSITION_Y, x);
#endif
	input_event(input, EV_ABS, ABS_MT_TOUCH_MAJOR, h);
	input_event(input, EV_ABS, ABS_MT_WIDTH_MAJOR, w);
#ifdef ILITEK_REPORT_PRESSURE
	input_event(input, EV_ABS, ABS_MT_PRESSURE, p);
#endif
#ifndef ILITEK_TOUCH_PROTOCOL_B
	input_event(input, EV_ABS, ABS_MT_TRACKING_ID, id);
	input_mt_sync(input);
#endif

#if ILITEK_PLAT == ILITEK_PLAT_MTK
#ifdef CONFIG_MTK_BOOT
#ifndef MTK_UNDTS
	if (tpd_dts_data.use_tpd_button) {
		if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode()) {
			tpd_button(x, y, 1);
			tp_dbg("tpd_button(x, y, 1) = tpd_button(%d, %d, 1)\n", x, y);
		}
	}
#endif
#endif
#endif
	return 0;
}

static int ilitek_touch_release(int id)
{
	struct input_dev *input = ts->input_dev;

#ifdef ILITEK_TOUCH_PROTOCOL_B
	if (ts->touch_flag[id] == 1) {
		tp_dbg("release point id = %d\n", id);
		input_mt_slot(input, id);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, false);
	}
#else
	input_report_key(input, BTN_TOUCH, 0);
	input_mt_sync(input);
#endif
	set_arr(ts->touch_flag, id, 0);

#if ILITEK_PLAT == ILITEK_PLAT_MTK
#ifdef CONFIG_MTK_BOOT
#ifndef MTK_UNDTS
	if (tpd_dts_data.use_tpd_button) {
		if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode()) {
			tpd_button(0, 0, 0);
			tp_dbg("tpd_button(x, y, 0) = tpd_button(%d, %d, 0)\n", 0, 0);
		}
	}
#endif
#endif
#endif

	return 0;
}

static int ilitek_touch_release_all_point(void)
{
	struct input_dev *input = ts->input_dev;
	int i = 0;

#ifdef ILITEK_TOUCH_PROTOCOL_B
	input_report_key(input, BTN_TOUCH, 0);
	for (i = 0; i < ts->max_tp; i++)
		ilitek_touch_release(i);
#else
	for (i = 0; i < ts->max_tp; i++)
		set_arr(ts->touch_flag, i, 0);
	ilitek_touch_release(0);
#endif
	ts->is_touched = false;
	input_sync(input);
	return 0;
}

static int ilitek_check_key_down(int x, int y)
{
	int j = 0;
	for (j = 0; j < ts->keycount; j++) {
		if ((x >= ts->keyinfo[j].x && x <= ts->keyinfo[j].x + ts->key_xlen) &&
		    (y >= ts->keyinfo[j].y && y <= ts->keyinfo[j].y + ts->key_ylen)) {
#if ILITEK_PLAT != ILITEK_PLAT_MTK
			input_report_key(ts->input_dev, ts->keyinfo[j].id, 1);
#else
#ifndef MTK_UNDTS
			if (tpd_dts_data.use_tpd_button) {
				x = tpd_dts_data.tpd_key_dim_local[j].key_x;
				y = tpd_dts_data.tpd_key_dim_local[j].key_y;
				tp_dbg("key index=%x, tpd_dts_data.tpd_key_local[%d]=%d key down\n", j, j, tpd_dts_data.tpd_key_local[j]);
				ilitek_touch_down(0, x, y, 10, 128, 1);
			}
#else
			x = touch_key_point_maping_array[j].point_x;
			y = touch_key_point_maping_array[j].point_y;
			ilitek_touch_down(0, x, y, 10, 128, 1);
#endif
#endif
			ts->keyinfo[j].status = 1;
			ts->touch_key_hold_press = true;
			ts->is_touched = true;
			tp_dbg("Key, Keydown ID=%d, X=%d, Y=%d, key_status=%d\n", ts->keyinfo[j].id, x, y, ts->keyinfo[j].status);
			break;
		}
	}
	return 0;
}

static int ilitek_check_key_release(int x, int y, int check_point)
{
	int j = 0;

	for (j = 0; j < ts->keycount; j++) {
		if (check_point) {
			if ((ts->keyinfo[j].status == 1) && (x < ts->keyinfo[j].x ||
			    x > ts->keyinfo[j].x + ts->key_xlen || y < ts->keyinfo[j].y ||
			    y > ts->keyinfo[j].y + ts->key_ylen)) {
#if ILITEK_PLAT != ILITEK_PLAT_MTK
				input_report_key(ts->input_dev, ts->keyinfo[j].id, 0);
#else
#ifndef MTK_UNDTS
				if (tpd_dts_data.use_tpd_button) {
					tp_dbg("key index=%x, tpd_dts_data.tpd_key_local[%d]=%d key up\n", j, j, tpd_dts_data.tpd_key_local[j]);
					ilitek_touch_release(0);
				}
#else
				ilitek_touch_release(0);
#endif
#endif
				ts->keyinfo[j].status = 0;
				ts->touch_key_hold_press = false;
				tp_dbg("Key, Keyout ID=%d, X=%d, Y=%d, key_status=%d\n",
						ts->keyinfo[j].id, x, y, ts->keyinfo[j].status);
				break;
			}
		} else {
			if (ts->keyinfo[j].status == 1) {
#if ILITEK_PLAT != ILITEK_PLAT_MTK
				input_report_key(ts->input_dev, ts->keyinfo[j].id, 0);
#else
#ifndef MTK_UNDTS
				if (tpd_dts_data.use_tpd_button) {
					tp_dbg("key index=%x, tpd_dts_data.tpd_key_local[%d]=%d key up\n", j, j, tpd_dts_data.tpd_key_local[j]);
					ilitek_touch_release(0);
				}
#else
				ilitek_touch_release(0);
#endif
#endif
				ts->keyinfo[j].status = 0;
				ts->touch_key_hold_press = false;
				tp_dbg("Key, Keyout ID=%d, X=%d, Y=%d, key_status=%d\n",
						ts->keyinfo[j].id, x, y, ts->keyinfo[j].status);
				break;
			}
		}
	}
	return 0;
}

int event_spacing;
static uint8_t finger_state;
static int start_x;
static int start_y;
static int current_x;
static int current_y;

#if ILITEK_GET_TIME_FUNC == ILITEK_GET_TIME_FUNC_WITH_TIME
static struct timeval start_event_time;
#else
unsigned long start_event_time_jiffies;
#endif

static int ilitek_get_time_diff(void)
{
	int diff_milliseconds = 0;
#if ILITEK_GET_TIME_FUNC == ILITEK_GET_TIME_FUNC_WITH_TIME
	struct timeval time_now;

	do_gettimeofday(&time_now);
	diff_milliseconds += (time_now.tv_sec - start_event_time.tv_sec) * 1000;

	if (time_now.tv_usec < start_event_time.tv_usec) {
		diff_milliseconds -= 1000;
		diff_milliseconds += (1000 * 1000 + time_now.tv_usec - start_event_time.tv_usec) / 1000;
	} else
		diff_milliseconds += (time_now.tv_usec - start_event_time.tv_usec) / 1000;

	if (diff_milliseconds < (-10000))
		diff_milliseconds = 10000;
	tp_msg("time_now.tv_sec = %d start_event_time.tv_sec = %d time_now.tv_usec = %d start_event_time.tv_usec = %d diff_milliseconds = %d\n",
			(int)time_now.tv_sec, (int)start_event_time.tv_sec, (int)time_now.tv_usec, (int)start_event_time.tv_usec, diff_milliseconds);
#else
	diff_milliseconds = jiffies_to_msecs(jiffies) - jiffies_to_msecs(start_event_time_jiffies);
	tp_msg("jiffies_to_msecs(jiffies) = %u jiffies_to_msecs(start_event_time_jiffies) = %u diff_milliseconds = %d\n", jiffies_to_msecs(jiffies),
			jiffies_to_msecs(start_event_time_jiffies), diff_milliseconds);
#endif
	return diff_milliseconds;
}

static uint8_t ilitek_double_click_touch(int finger_id, int x, int y,
					 uint8_t finger_state)
{
	tp_msg("start finger_state = %d\n", finger_state);
	if (finger_id > 0) {
		finger_state = 0;
		goto out;
	}
	if (finger_state == 0 || finger_state == 5) {

		finger_state = 1;
		start_x = x;
		start_y = y;
		current_x = 0;
		current_y = 0;
		event_spacing = 0;
#if ILITEK_GET_TIME_FUNC == ILITEK_GET_TIME_FUNC_WITH_TIME
		do_gettimeofday(&start_event_time);
#else
		start_event_time_jiffies = jiffies;
#endif
	} else if (finger_state == 1) {
		event_spacing = ilitek_get_time_diff();
		if (event_spacing > DOUBLE_CLICK_ONE_CLICK_USED_TIME)
			finger_state = 4;
	} else if (finger_state == 2) {
		finger_state = 3;
		current_x = x;
		current_y = y;
		event_spacing = ilitek_get_time_diff();
		if (event_spacing > (DOUBLE_CLICK_ONE_CLICK_USED_TIME + DOUBLE_CLICK_NO_TOUCH_TIME))
			finger_state = 0;
	} else if (finger_state == 3) {
		current_x = x;
		current_y = y;
		event_spacing = ilitek_get_time_diff();
		if (event_spacing > DOUBLE_CLICK_TOTAL_USED_TIME) {
			start_x = current_x;
			start_y = current_y;
			finger_state = 4;
		}
	}
out:
	tp_msg("finger_state = %d event_spacing = %d\n", finger_state, event_spacing);
	return finger_state;
}

static uint8_t ilitek_double_click_release(uint8_t finger_state)
{
	tp_msg("start finger_state = %d\n", finger_state);
	if (finger_state == 1) {
		finger_state = 2;
		event_spacing = ilitek_get_time_diff();
		if (event_spacing > DOUBLE_CLICK_ONE_CLICK_USED_TIME)
			finger_state = 0;
	}
	if (finger_state == 3) {
		event_spacing = ilitek_get_time_diff();
		if ((event_spacing < DOUBLE_CLICK_TOTAL_USED_TIME && event_spacing > 50) && (ABSSUB(current_x, start_x) < DOUBLE_CLICK_DISTANCE)
				&& ((ABSSUB(current_y, start_y) < DOUBLE_CLICK_DISTANCE))) {
			finger_state = 5;
			goto out;
		} else
			finger_state = 0;
	} else if (finger_state == 4)
		finger_state = 0;
out:
	tp_msg("finger_state = %d event_spacing = %d\n", finger_state, event_spacing);
	return finger_state;
}

void __maybe_unused ilitek_gesture_handle(bool touch, int idx, int x, int y)
{
	struct input_dev *input = ts->input_dev;

	if (ts->gesture_status == Gesture_Double_Click) {
		if (touch) {
			finger_state = ilitek_double_click_touch(idx, x, y, finger_state);
			return;
		}
		finger_state = ilitek_double_click_release(finger_state);

		if (finger_state != 5)
			return;
	}

#ifdef ILITEK_WAKELOCK_SUPPORT
	wake_lock_timeout(&ilitek_wake_lock, 5 * HZ);
#endif

	input_report_key(input, KEY_POWER, 1);
	input_sync(input);
	input_report_key(input, KEY_POWER, 0);
	input_sync(input);
}

static void ilitek_check_algo(uint8_t mode_status)
{
#ifdef ILITEK_CHECK_FUNCMODE
	tp_dbg("mode_status = 0x%X\n", mode_status);
	if (mode_status & 0x80)
		tp_dbg("Palm reject mode enable\n");
	if (mode_status & 0x40)
		tp_dbg("Thumb mdoe enable\n");
	if (mode_status & 0x04)
		tp_dbg("Water mode enable\n");
	if (mode_status & 0x02)
		tp_dbg("Mist mode enable\n");
	if (mode_status & 0x01)
		tp_dbg("Normal mode\n");
#endif
}

int ilitek_read_data_and_report_3XX(void)
{
	int ret = 0;
	int packet = 0;
	int report_max_point = 6;
	int release_point = 0;
	int tp_status = 0;
	int i = 0;
	int x = 0;
	int y = 0;
	struct input_dev *input = ts->input_dev;
	uint8_t buf[64];
	uint8_t mode_status;

	memset(buf, 0, sizeof(buf));

	buf[0] = CMD_GET_TOUCH_INFO;
	ret = ilitek_i2c_write_and_read(buf, 1, 0, buf, 32);
	if (ret < 0) {
		tp_err("get touch information err\n");
		if (ts->is_touched) {
			ilitek_touch_release_all_point();
			ilitek_check_key_release(x, y, 0);
		}
		return ret;
	}

	mode_status = buf[31];
	buf[31] = 0;
	ilitek_check_algo(mode_status);

	packet = buf[0];
	if (packet == 2) {
		ret = ilitek_i2c_read(buf + 31, 20);
		if (ret < 0) {
			tp_err("get touch information packet 2 err\n");
			if (ts->is_touched) {
				ilitek_touch_release_all_point();
				ilitek_check_key_release(x, y, 0);
			}
			return ret;
		}
		report_max_point = 10;
	}
	buf[62] = mode_status;

	ilitek_udp_reply(buf, 64);

	if (buf[1] == 0x5F || buf[0] == 0xDB) {
		tp_dbg("debug message return\n");
		return 0;
	}

	for (i = 0; i < report_max_point; i++) {
		tp_status = buf[i * 5 + 1] >> 7;

		tp_dbg("buf: 0x%x, tp_status: %d\n", buf[i * 5 + 1], tp_status);
		if (!tp_status) {
			release_point++;
#ifdef ILITEK_TOUCH_PROTOCOL_B
			ilitek_touch_release(i);
#endif
			continue;
		}

		set_arr(ts->touch_flag, i, 1);

		x = ((buf[i * 5 + 1] & 0x3F) << 8) + buf[i * 5 + 2];
		y = (buf[i * 5 + 3] << 8) + buf[i * 5 + 4];

		if (ts->system_suspend) {
			tp_msg("system is suspend not report point\n");
			ilitek_gesture_handle(true, i, x, y);
			continue;
		}
		if (!(ts->is_touched))
			ilitek_check_key_down(x, y);
		if (!(ts->touch_key_hold_press)) {
			if (x > ts->screen_max_x || y > ts->screen_max_y ||
			    x < ts->screen_min_x || y < ts->screen_min_y) {
				tp_err("Point[%d]: (%d, %d), Limit: (%d:%d, %d:%d) OOB\n",
					i, x, y, ts->screen_min_x, ts->screen_max_x,
					ts->screen_min_y, ts->screen_max_y);
				tp_err("Raw data: %x-%x-%x-%x-%x\n",
					buf[i * 5 + 1], buf[i * 5 + 2],
					buf[i * 5 + 3], buf[i * 5 + 4],
					buf[i * 5 + 5]);
				continue;
			}
			ts->is_touched = true;
			if (ILITEK_REVERT_X)
				x = ts->screen_max_x - x + ts->screen_min_x;
			if (ILITEK_REVERT_Y)
				y = ts->screen_max_y - y + ts->screen_min_y;
			tp_dbg("Touch id=%02X, x: %04d, y: %04d\n", i, x, y);
			ilitek_touch_down(i, x, y, 10, 128, 1);
		}
	}
	tp_dbg("release point:  %d, packet: %d\n", release_point, packet);
	if (packet == 0 || release_point == report_max_point) {
		if (ts->is_touched)
			ilitek_touch_release_all_point();

		ilitek_check_key_release(x, y, 0);
		ts->is_touched = false;

		if (ts->system_suspend)
			ilitek_gesture_handle(false, 0, 0, 0);
	}
	input_sync(input);
	return 0;
}

int ilitek_read_data_and_report_6XX(void)
{
	int ret = 0;
	int count = 0;
	int report_max_point = 6;
	int release_point = 0;
	int i = 0;
	struct input_dev *input = ts->input_dev;
	unsigned char buf[512];
	unsigned char tmp[64];
	int packet_len = 0;
	int packet_max_point = 0;
	struct touch_info_v6 *info;

	tp_dbg("[%s] format:%d\n", __func__, ts->format);

	switch (ts->format) {
	case 0:
		packet_len = REPORT_FORMAT0_PACKET_LENGTH;
		packet_max_point = REPORT_FORMAT0_PACKET_MAX_POINT;
		break;
	case 1:
		packet_len = REPORT_FORMAT1_PACKET_LENGTH;
		packet_max_point = REPORT_FORMAT1_PACKET_MAX_POINT;
		break;
	case 2:
		packet_len = REPORT_FORMAT2_PACKET_LENGTH;
		packet_max_point = REPORT_FORMAT2_PACKET_MAX_POINT;
		break;
	case 3:
		packet_len = REPORT_FORMAT3_PACKET_LENGTH;
		packet_max_point = REPORT_FORMAT3_PACKET_MAX_POINT;
		break;
	default:
		packet_len = REPORT_FORMAT0_PACKET_LENGTH;
		packet_max_point = REPORT_FORMAT0_PACKET_MAX_POINT;
		break;
	}

	for(i = 0; i < ts->max_tp; i++)
		ts->tp[i].status = false;

	ret = ilitek_i2c_read(buf, 64);
	if (ret < 0) {
		tp_err("get touch information err\n");
		if (ts->is_touched) {
			ilitek_touch_release_all_point();
			ilitek_check_key_release(0, 0, 0);
		}
		return ret;
	}

	if (ts->ilitek_log_level_value == ILITEK_DEBUG_LOG_LEVEL) {
		tp_msg_arr("ilitek_i2c_read", buf, 64);
	}
	/*
	 * Check checksum for I2C comms. debug.
	 */
	if (!checksum(buf, 64, buf[63])) {
		tp_err("checksum failed, just skip...\n");
		return 0;
	}

	ilitek_udp_reply(buf, 64);


	if (buf[0] == 0xDB) {
		tp_dbg("debug message return\n");
		return 0;
	}

	report_max_point = buf[REPORT_ADDRESS_COUNT];
	if (report_max_point > ts->max_tp) {
		tp_err("FW report max point:%d > panel information max point:%d\n",
			report_max_point, ts->max_tp);
		return ILITEK_FAIL;
	}
	count = CEIL(report_max_point, packet_max_point);
	for (i = 1; i < count; i++) {
		ret = ilitek_i2c_read(tmp, 64);
		if (ret < 0) {
			tp_err("get touch information err, count=%d\n", count);
			if (ts->is_touched) {
				ilitek_touch_release_all_point();
				ilitek_check_key_release(0, 0, 0);
			}
			return ret;
		}

		if (!checksum(tmp, 64, tmp[63])) {
			tp_err("checksum failed, just skip...\n");
			return 0;
		}

		memcpy(buf + 1 + i * packet_len * packet_max_point,
		       tmp + 1, 63);

		ilitek_udp_reply(tmp, 64);

	}

	for (i = 0; i < report_max_point; i++) {
		info = (struct touch_info_v6 *)(buf + i * packet_len + 1);

		ts->tp[i].status = info->status;
		ts->tp[i].id = info->id;

		tp_dbg("buf: 0x%x, tp_status: %d, id: %d\n", buf[i * packet_len + 1],
			ts->tp[i].status, ts->tp[i].id);

		if (ts->tp[i].id >= ts->max_tp) {
			tp_err("id: %d, limit: %d OOB\n", ts->tp[i].id, ts->max_tp);
			continue;
		}

		if (!ts->tp[i].status) {
			release_point++;
#ifdef ILITEK_TOUCH_PROTOCOL_B
			ilitek_touch_release(ts->tp[i].id);
#endif
			continue;
		}

		set_arr(ts->touch_flag, ts->tp[i].id, 1);

		ts->tp[i].x = info->x;
		ts->tp[i].y = info->y;

		ts->tp[i].p = 10;
		ts->tp[i].w = 10;
		ts->tp[i].h = 10;

		if (ts->format == 1 || ts->format == 3)
			ts->tp[i].p = info->p;

		if (ts->format == 2 || ts->format == 3) {
			ts->tp[i].w = info->w;
			ts->tp[i].h = info->h;
		}

		tp_dbg("id: %d, x: %d, y: %d, p: %d, w: %d, h: %d\n",
			ts->tp[i].id, ts->tp[i].x, ts->tp[i].y,
			ts->tp[i].p, ts->tp[i].w, ts->tp[i].h);
		if (ts->system_suspend) {
			tp_msg("system is suspend not report point\n");
			ilitek_gesture_handle(true, i, ts->tp[i].x, ts->tp[i].y);
			continue;
		}

		if (!(ts->is_touched))
			ilitek_check_key_down(ts->tp[i].x, ts->tp[i].y);

		if (!(ts->touch_key_hold_press)) {
			if (ts->tp[i].x > ts->screen_max_x ||
			    ts->tp[i].y > ts->screen_max_y ||
			    ts->tp[i].x < ts->screen_min_x ||
			    ts->tp[i].y < ts->screen_min_y) {
				tp_err("Point[%d]: (%d, %d), Limit: (%d:%d, %d:%d) OOB\n",
					ts->tp[i].id, ts->tp[i].x, ts->tp[i].y,
					ts->screen_min_x, ts->screen_max_x,
					ts->screen_min_y, ts->screen_max_y);
			} else {
				ts->is_touched = true;
				if (ILITEK_REVERT_X)
					ts->tp[i].x = ts->screen_max_x - ts->tp[i].x + ts->screen_min_x;

				if (ILITEK_REVERT_Y)
					ts->tp[i].y = ts->screen_max_y - ts->tp[i].y + ts->screen_min_y;

				tp_dbg("Point[%02X]: X:%04d, Y:%04d, P:%d, H:%d, W:%d\n",
						ts->tp[i].id, ts->tp[i].x,ts->tp[i].y,
						ts->tp[i].p, ts->tp[i].h, ts->tp[i].w);
				ilitek_touch_down(ts->tp[i].id, ts->tp[i].x,
						  ts->tp[i].y, ts->tp[i].p,
						  ts->tp[i].h, ts->tp[i].w);
			}
		}
		if ((ts->touch_key_hold_press))
			ilitek_check_key_release(ts->tp[i].x, ts->tp[i].y, 1);
	}

	tp_dbg("release point counter =  %d , report max point = %d\n", release_point, report_max_point);
	if (release_point == report_max_point) {
		if (ts->is_touched)
			ilitek_touch_release_all_point();

		ilitek_check_key_release(0, 0, 0);
		ts->is_touched = false;

		if (ts->system_suspend)
			ilitek_gesture_handle(false, 0, 0, 0);
	}
	input_sync(input);
	return 0;
}

static int ilitek_i2c_process_and_report(void)
{
	int ret = 0;
	mutex_lock(&ts->ilitek_mutex);
	if (!ts->unhandle_irq)
		ret = ts->process_and_report();
	mutex_unlock(&ts->ilitek_mutex);
	return ret;
}

#ifdef MTK_UNDTS
static void ilitek_i2c_isr(void)
#else
static irqreturn_t ilitek_i2c_isr(int irq, void *dev_id)
#endif
{
	tp_dbg("\n");
#ifdef ILITEK_ESD_PROTECTION
	ts->esd_check = false;
#endif
	if (atomic_read(&ts->firmware_updating)) {
		tp_dbg("firmware_updating return\n");
#ifdef MTK_UNDTS
		return;
#else
		return IRQ_HANDLED;
#endif
	}

#ifdef ILITEK_ISR_PROTECT
	ilitek_irq_disable();
#endif

	atomic_set(&ts->get_INT, 1);
	ilitek_gpio_dbg();

	if (ilitek_i2c_process_and_report() < 0)
		tp_err("process error\n");

#ifdef ILITEK_ISR_PROTECT
	ilitek_irq_enable();
#endif

#ifndef MTK_UNDTS
	return IRQ_HANDLED;
#endif
}

static int ilitek_request_irq(void)
{
	int ret = 0;
#if ILITEK_PLAT == ILITEK_PLAT_MTK
#ifndef MTK_UNDTS
	struct device_node *node;
#endif
#endif

#if ILITEK_PLAT != ILITEK_PLAT_MTK
	if ((ts->client->irq == 0) && (ts->irq_gpio > 0)) {
		ts->client->irq = gpio_to_irq(ts->irq_gpio);
		printk ("IRQ : gpio %d => %d\n", ts->irq_gpio, ts->client->irq);
	} else {
		printk ("IRQ : already specified => %d\n", ts->client->irq);
	}
#else
#ifndef MTK_UNDTS
	node = of_find_matching_node(NULL, touch_of_match);
	if (node)
		ts->client->irq = irq_of_parse_and_map(node, 0);
#endif
#endif

#ifdef MTK_UNDTS
	mt_set_gpio_mode(ILITEK_IRQ_GPIO, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(ILITEK_IRQ_GPIO, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(ILITEK_IRQ_GPIO, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(ILITEK_IRQ_GPIO, GPIO_PULL_UP);

	mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE, ilitek_i2c_isr, 1);
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#else
	tp_msg("ts->client->irq: %d\n", ts->client->irq);
	if (ts->client->irq > 0) {
		ret = request_threaded_irq(ts->client->irq, NULL, ilitek_i2c_isr,
					   ts->irq_tri_type | IRQF_ONESHOT,
					   "ilitek_touch_irq", ts);
		if (ret)
			tp_err("ilitek_request_irq, error\n");
	} else {
		ret = -EINVAL;
	}
#endif

	ts->irq_registerred = true;
	atomic_set(&ts->irq_enabled, 1);

	return ret;
}

static int __maybe_unused ilitek_update_thread(void *arg)
{
	int ret = 0;

#ifdef ILITEK_UPDATE_FW
	tp_msg("\n");

	if (kthread_should_stop()) {
		tp_msg("ilitek_update_thread, stop\n");
		return -1;
	}

	mdelay(100);
	atomic_set(&ts->firmware_updating, 1);
	ts->operation_protection = true;
	ret = ilitek_upgrade_firmware();
	ret = ilitek_read_tp_info(true);
	ts->operation_protection = false;
	atomic_set(&ts->firmware_updating, 0);

	ret = ilitek_request_input_dev();
	if (ret)
		tp_err("register input device, error\n");
	ret = ilitek_request_irq();
	if (ret)
		tp_err("ilitek_request_irq, error\n");
#endif

	return ret;
}

#if 0
static int ilitek_irq_handle_thread(void *arg)
{
	int ret = 0;
	struct sched_param param = {.sched_priority = 4 };

	sched_setscheduler(current, SCHED_RR, &param);
	tp_msg("%s, enter\n", __func__);

	// mainloop
	while (!kthread_should_stop() && !ts->ilitek_exit_report) {
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, ts->irq_trigger);
		ts->irq_trigger = false;
		ts->irq_handle_count++;
		set_current_state(TASK_RUNNING);
		if (ilitek_i2c_process_and_report() < 0)
			tp_err("process error\n");
		ilitek_irq_enable();
	}
	tp_err("%s, exit\n", __func__);
	tp_err("%s, exit\n", __func__);
	tp_err("%s, exit\n", __func__);
	return ret;
}
#endif

void ilitek_suspend(void)
{
	tp_msg("\n");

#ifdef ILITEK_ESD_PROTECTION
	ts->esd_check = false;
	cancel_delayed_work_sync(&ts->esd_work);
#endif

	if (ts->operation_protection || atomic_read(&ts->firmware_updating)) {
		tp_msg("operation_protection or firmware_updating return\n");
		return;
	}

	if (ts->gesture_status) {
		ts->wake_irq_enabled = (enable_irq_wake(ts->client->irq) == 0);
		if (api_set_idlemode(ENABLE_MODE) < 0)
			tp_err("enable Idle mode err\n");
	} else {
#if ILITEK_LOW_POWER == ILITEK_SLEEP
		mutex_lock(&ts->ilitek_mutex);
		if (api_ptl_set_cmd(SET_IC_SLEEP, NULL, NULL) < 0)
			tp_err("0x30 set tp sleep err\n");
		mutex_unlock(&ts->ilitek_mutex);
#endif
		ilitek_irq_disable();
	}

	ts->system_suspend = true;
}

void ilitek_resume(void)
{
	tp_msg("\n");

	if (ts->operation_protection || atomic_read(&ts->firmware_updating)) {
		tp_msg("operation_protection or firmware_updating return\n");
		return;
	}

	if (ts->gesture_status) {
		ilitek_irq_disable();
		if (api_set_idlemode(DISABLE_MODE) < 0)
			tp_err("disable Idle mode err\n");

		if (ts->gesture_status == Gesture_Double_Click)
			finger_state = 0;

		if (ts->wake_irq_enabled) {
			disable_irq_wake(ts->client->irq);
			ts->wake_irq_enabled = false;
		}
	} else {
		/*
		 * If ILITEK_SLEEP is defined and FW support wakeup command,
		 * the reset can be mark.
		 */
		 ilitek_reset(ts->reset_time);

#if ILITEK_LOW_POWER == ILITEK_SLEEP
		mutex_lock(&ts->ilitek_mutex);
		if (api_ptl_set_cmd(SET_IC_WAKE, NULL, NULL) < 0)
			tp_err("0x31 set wake up err\n");
		mutex_unlock(&ts->ilitek_mutex);
#endif
	}

#ifdef ILITEK_ESD_PROTECTION
	ts->esd_check = true;
	if (ts->esd_wq)
		queue_delayed_work(ts->esd_wq, &ts->esd_work, ts->esd_delay);
#endif

	ilitek_touch_release_all_point();
	ilitek_check_key_release(0, 0, 0);

	ts->system_suspend = false;

	ilitek_irq_enable();
}

#if ILITEK_PLAT == ILITEK_PLAT_ALLWIN
int ilitek_suspend_allwin(struct i2c_client *client, pm_message_t mesg)
{
	ilitek_suspend();
	return 0;
}

int ilitek_resume_allwin(struct i2c_client *client)
{
	ilitek_resume();
	return 0;
}
#endif

#if ILITEK_PLAT != ILITEK_PLAT_MTK
#if defined(CONFIG_FB) || defined(CONFIG_QCOM_DRM)
static int __maybe_unused ilitek_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data) {
#ifdef CONFIG_QCOM_DRM
	struct msm_drm_notifier *ev_data = data;
#else
	struct fb_event *ev_data = data;
#endif
	int *blank;
	tp_msg("FB EVENT event: %lu\n", event);

#ifdef CONFIG_QCOM_DRM
	if (!ev_data || (ev_data->id != 0))
		return 0;
#endif
	if (ev_data && ev_data->data && event == ILITEK_EVENT_BLANK) {
		blank = ev_data->data;
		tp_msg("blank: %d\n", *blank);
		if (*blank == ILITEK_BLANK_POWERDOWN) {
			ilitek_suspend();
		}
		else if (*blank == ILITEK_BLANK_UNBLANK || *blank == ILITEK_BLANK_NORMAL) {
			ilitek_resume();
		}
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void __maybe_unused ilitek_early_suspend(struct early_suspend *h)
{
	ilitek_suspend();
}

static void __maybe_unused ilitek_late_resume(struct early_suspend *h)
{
	ilitek_resume();
}
#endif
#endif

static int ilitek_get_gpio_num(void)
{
	int ret = 0;
#ifdef ILITEK_GET_GPIO_NUM
#if ILITEK_PLAT == ILITEK_PLAT_ALLWIN
	tp_msg("(config_info.wakeup_gpio.gpio) = %d (config_info.int_number) = %d\n", (config_info.wakeup_gpio.gpio), (config_info.int_number));
	ts->reset_gpio = (config_info.wakeup_gpio.gpio);
	ts->irq_gpio = (config_info.int_number);
#else
#ifdef CONFIG_OF
	struct device *dev = &(ts->client->dev);
	struct device_node *np = dev->of_node;

	ts->reset_gpio = of_get_named_gpio(np, "ilitek,reset-gpio", 0);
	if (ts->reset_gpio < 0)
		tp_err("reset_gpio = %d\n", ts->reset_gpio);
	ts->irq_gpio = of_get_named_gpio(np, "ilitek,irq-gpio", 0);
	if (ts->irq_gpio < 0)
		tp_err("irq_gpio = %d\n", ts->irq_gpio);
#endif
#endif
#else
	ts->reset_gpio = ILITEK_RESET_GPIO;
	ts->irq_gpio = ILITEK_IRQ_GPIO;
#endif
	tp_msg("reset_gpio = %d irq_gpio = %d\n", ts->reset_gpio, ts->irq_gpio);


#if defined(ILITEK_GPIO_DEBUG)
	do {
		ts->test_gpio = of_get_named_gpio(np, "ilitek,test-gpio", 0);
		if (ts->test_gpio < 0) {
			tp_err("test_gpio: %d\n", ts->test_gpio);
			break;
		}

		tp_msg("test_gpio: %d\n", ts->test_gpio);

		if (gpio_request(ts->test_gpio, "ilitek-test-gpio")) {
			tp_err("request test_gpio failed\n");
			break;
		}

		gpio_direction_output(ts->test_gpio, 1);

	} while (0);
#endif

	return ret;
}

static int ilitek_request_gpio(void)
{
	int ret = 0;

	ilitek_get_gpio_num();

#if ILITEK_PLAT != ILITEK_PLAT_MTK
	if (ts->reset_gpio > 0) {
		ret = gpio_request(ts->reset_gpio, "ilitek-reset-gpio");
		if (ret) {
			tp_err("Failed to request reset_gpio so free retry\n");
			gpio_free(ts->reset_gpio);
			ret = gpio_request(ts->reset_gpio, "ilitek-reset-gpio");
			if (ret)
				tp_err("Failed to request reset_gpio\n");
		}
		if (ret) {
			tp_err("Failed to request reset_gpio\n");
		} else {
			ret = gpio_direction_output(ts->reset_gpio, 1);
			if (ret)
				tp_err("Failed to direction output rest gpio err\n");
		}
	}
	if (ts->irq_gpio > 0) {
		ret = gpio_request(ts->irq_gpio, "ilitek-irq-gpio");
		if (ret) {
			tp_err("Failed to request irq_gpio so free retry\n");
			gpio_free(ts->irq_gpio);
			ret = gpio_request(ts->irq_gpio, "ilitek-irq-gpio");
			if (ret)
				tp_err("Failed to request irq_gpio\n");
		}
		if (ret) {
			tp_err("Failed to request irq_gpio\n");
		} else {
			ret = gpio_direction_input(ts->irq_gpio);
			if (ret)
				tp_err("Failed to direction input irq gpio err\n");
		}
	}
#endif
	return ret;
}

static int ilitek_create_esdandcharge_workqueue(void)
{
#ifdef ILITEK_ESD_PROTECTION
	INIT_DELAYED_WORK(&ts->esd_work, ilitek_esd_check);
	ts->esd_wq = create_singlethread_workqueue("ilitek_esd_wq");
	if (!ts->esd_wq) {
		tp_err("create workqueue esd work err\n");
	} else {
		ts->esd_check = true;
		ts->esd_delay = 2 * HZ;
		queue_delayed_work(ts->esd_wq, &ts->esd_work, ts->esd_delay);
	}
#endif
	return 0;
}

static int __maybe_unused ilitek_register_resume_suspend(void)
{
	int ret = 0;

#ifdef ILITEK_REGISTER_SUSPEND_RESUME

#if ILITEK_PLAT != ILITEK_PLAT_MTK
#if defined(CONFIG_FB) || defined(CONFIG_QCOM_DRM)
	ts->fb_notif.notifier_call = ilitek_notifier_callback;
#ifdef CONFIG_QCOM_DRM
	ret = msm_drm_register_client(&ts->fb_notif);
#else
	ret = fb_register_client(&ts->fb_notif);
#endif
	if (ret)
		tp_err("Unable to register fb_notifier: %d\n", ret);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = ilitek_early_suspend;
	ts->early_suspend.resume = ilitek_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
#endif /* ILITEK_PLAT != ILITEK_PLAT_MTK */

#if ILITEK_PLAT == ILITEK_PLAT_ALLWIN
	device_enable_async_suspend(&ts->client->dev);
	pm_runtime_set_active(&ts->client->dev);
	pm_runtime_get(&ts->client->dev);
	pm_runtime_enable(&ts->client->dev);
#endif

#endif /* ILITEK_REGISTER_SUSPEND_RESUME */
	return ret;
}

static void __maybe_unused ilitek_release_resume_suspend(void)
{
#ifdef ILITEK_REGISTER_SUSPEND_RESUME

#if defined(CONFIG_FB) || defined(CONFIG_QCOM_DRM)
#ifdef CONFIG_QCOM_DRM
	msm_drm_unregister_client(&ts->fb_notif);
#else
	fb_unregister_client(&ts->fb_notif);
#endif
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif

#endif /* ILITEK_REGISTER_SUSPEND_RESUME */
}

static int __maybe_unused ilitek_init_netlink(void)
{
#ifdef ILITEK_TUNING_MESSAGE
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
	struct netlink_kernel_cfg cfg = {
		.groups = 0,
		.input = udp_receive,
	};
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0)
	ilitek_netlink_sock = netlink_kernel_create(&init_net, NETLINK_USERSOCK, &cfg);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 0)
	ilitek_netlink_sock = netlink_kernel_create(&init_net, NETLINK_CRYPTO, 0, udp_receive, NULL, THIS_MODULE);
#else
	ilitek_netlink_sock = netlink_kernel_create(&init_net, NETLINK_CRYPTO, THIS_MODULE, &cfg);
#endif

	if (!ilitek_netlink_sock)
		tp_err("netlink_kernel_create failed\n");

	ilitek_debug_flag = false;
#endif
	return 0;
}

int ilitek_read_tp_info(bool boot)
{
	int ret = 0;
	uint8_t outbuf[64] = {0};

	tp_msg("driver version %d.%d.%d.%d.%d.%d.%d\n",
		ilitek_driver_information[0], ilitek_driver_information[1],
		ilitek_driver_information[2], ilitek_driver_information[3],
		ilitek_driver_information[4], ilitek_driver_information[5],
		ilitek_driver_information[6]);
	if (api_set_testmode(true) < ILITEK_SUCCESS)
		goto transfer_err;
	if (api_ptl_set_cmd(GET_MCU_MOD, NULL, outbuf) < ILITEK_SUCCESS)
		goto transfer_err;
	if (api_ptl_set_cmd(GET_PTL_VER, NULL, outbuf) < ILITEK_SUCCESS)
		goto transfer_err;
	if (api_ptl_set_cmd(GET_MCU_VER, NULL, outbuf) < ILITEK_SUCCESS)
		goto transfer_err;
	if (api_ptl_set_cmd(GET_FW_VER, NULL, outbuf) < ILITEK_SUCCESS)
		goto transfer_err;
	if (boot && api_ptl_set_cmd(GET_SCRN_RES, NULL, outbuf) < ILITEK_SUCCESS)
			goto transfer_err;
	if (api_ptl_set_cmd(GET_TP_RES, NULL, outbuf) < ILITEK_SUCCESS)
		goto transfer_err;
	if (api_set_testmode(false) < ILITEK_SUCCESS)
		goto transfer_err;

	return ret;
transfer_err:
	tp_err("failed\n");
	return ILITEK_FAIL;
}

static int __maybe_unused ilitek_alloc_dma(void)
{
#ifdef ILITEK_ENABLE_DMA
	tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	I2CDMABuf_va = (u8 *) dma_alloc_coherent(&tpd->dev->dev, ILITEK_DMA_SIZE, &I2CDMABuf_pa, GFP_KERNEL);
	if (!I2CDMABuf_va) {
		tp_err("ilitek [TPD] tpd->dev->dev dma_alloc_coherent error\n");
		I2CDMABuf_va = (u8 *) dma_alloc_coherent(NULL, ILITEK_DMA_SIZE, &I2CDMABuf_pa, GFP_KERNEL);
		if (!I2CDMABuf_va) {
			tp_err("ilitek [TPD] NULL dma_alloc_coherent error\n");
			return -ENOMEM;
		}
	}
	memset(I2CDMABuf_va, 0, ILITEK_DMA_SIZE);
	//ts->client->ext_flag |= I2C_DMA_FLAG;
#endif

	return 0;
}

static int __maybe_unused ilitek_free_dma(void)
{
#ifdef ILITEK_ENABLE_DMA
	if (I2CDMABuf_va) {
		dma_free_coherent(&tpd->dev->dev, ILITEK_DMA_SIZE,
				  I2CDMABuf_va, I2CDMABuf_pa);

		I2CDMABuf_va = NULL;
		I2CDMABuf_pa = 0;

	}
#endif
	return 0;
}

static int __maybe_unused ilitek_power_on(bool status)
{
	int ret = 0;

	tp_msg("%s\n", status ? "POWER ON" : "POWER OFF");

#ifdef ILITEK_ENABLE_REGULATOR_POWER_ON
#if ILITEK_PLAT == ILITEK_PLAT_ALLWIN
	input_set_power_enable(&(config_info.input_type), status);
#else

	if (status) {
		if (ts->vdd) {
			ret = regulator_enable(ts->vdd);
			if (ret < 0) {
				tp_err("regulator_enable vdd fail\n");
				return -EINVAL;
			}
		}
		if (ts->vdd_i2c) {
			ret = regulator_enable(ts->vdd_i2c);
			if (ret < 0) {
				tp_err("regulator_enable vdd_i2c fail\n");
				return -EINVAL;
			}
		}
	} else {
		if (ts->vdd) {
			ret = regulator_disable(ts->vdd);
			if (ret < 0)
				tp_err("regulator_enable vdd fail\n");
		}
		if (ts->vdd_i2c) {
			ret = regulator_disable(ts->vdd_i2c);
			if (ret < 0)
				tp_err("regulator_enable vdd_i2c fail\n");
		}
	}

#ifdef MTK_UNDTS
	if (status)
		hwPowerOn(PMIC_APP_CAP_TOUCH_VDD, VOL_3300, "TP");
#endif
#endif
#endif

	return ret;
}

static int __maybe_unused ilitek_request_regulator(struct ilitek_ts_data *ts)
{
#ifdef ILITEK_ENABLE_REGULATOR_POWER_ON
	int ret = 0;
	char *vdd_name = "vdd";
	char *vcc_i2c_name = "vcc_i2c";

#if ILITEK_PLAT == ILITEK_PLAT_MTK
	vdd_name = "vtouch";
	ts->vdd = regulator_get(tpd->tpd_dev, vdd_name);
	tpd->reg = ts->vdd;
	if (IS_ERR(ts->vdd)) {
		tp_err("regulator_get vdd fail\n");
		ts->vdd = NULL;
	} else {
		ret = regulator_set_voltage(ts->vdd, 3000000, 3300000);
		if (ret)
			tp_err("Could not set vdd to 3000~3300mv.\n");
	}
#elif ILITEK_PLAT != ILITEK_PLAT_ALLWIN
	ts->vdd = regulator_get(&ts->client->dev, vdd_name);
	if (IS_ERR(ts->vdd)) {
		tp_err("regulator_get vdd fail\n");
		ts->vdd = NULL;
	} else {
		ret = regulator_set_voltage(ts->vdd, 3000000, 3300000);
		if (ret)
			tp_err("Could not set vdd to 3000~3300mv.\n");

	}

	ts->vdd_i2c = regulator_get(&ts->client->dev, vcc_i2c_name);
	if (IS_ERR(ts->vdd_i2c)) {
		tp_err("regulator_get vdd_i2c fail\n");
		ts->vdd_i2c = NULL;
	} else {
		ret = regulator_set_voltage(ts->vdd_i2c, 3000000, 3300000);
		if (ret)
			tp_err("Could not set i2c to 3000~3300mv.\n");
	}
#endif /* ILITEK_PLAT == ILITEK_PLAT_MTK */
#endif /* ILITEK_ENABLE_REGULATOR_POWER_ON */

	return 0;
}

static void __maybe_unused ilitek_release_regulator(void)
{
#if defined(ILITEK_ENABLE_REGULATOR_POWER_ON) && ILITEK_PLAT != ILITEK_PLAT_ALLWIN
	if (ts->vdd)
		regulator_put(ts->vdd);
	if (ts->vdd_i2c)
		regulator_put(ts->vdd_i2c);
#endif
}

void ilitek_register_gesture(struct ilitek_ts_data *ts, bool init)
{
	if (init) {
		device_init_wakeup(&ts->client->dev, 1);

#ifdef ILITEK_WAKELOCK_SUPPORT
		wake_lock_init(&ilitek_wake_lock, WAKE_LOCK_SUSPEND, "ilitek wakelock");
#endif
		return;
	}

	device_init_wakeup(&ts->client->dev, 0);

#ifdef ILITEK_WAKELOCK_SUPPORT
	wake_lock_destroy(&ilitek_wake_lock);
#endif
}


int ilitek_main_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	if (!client) {
		tp_err("i2c client is NULL\n");
		return -1;
	}

	tp_msg("ILITEK client->addr = 0x%x client->irq = %d\n",
		    client->addr, client->irq);

	if (client->addr != 0x41)
		client->addr = 0x41;

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		tp_err("Alloc GFP_KERNEL memory failed\n");
		return -ENOMEM;
	}
	ts->irq_registerred = false;

	ts->client = client;
	ts->ilitek_log_level_value = ILITEK_DEFAULT_LOG_LEVEL;
	tp_msg("ilitek log level: %d\n", ts->ilitek_log_level_value);
	ts->ilitek_repeat_start = true;
	mutex_init(&ts->ilitek_mutex);
	ts->unhandle_irq = false;

	ilitek_alloc_dma();

	ilitek_request_regulator(ts);
	ilitek_power_on(true);

	ilitek_request_gpio();

	ilitek_reset(1000);
	ret = api_init_func();
	if (ret < 0) {
		tp_err("init read tp protocol error so exit\n");
		goto err_free_gpio;
	}
	ilitek_read_tp_info(true);

#ifdef ILITEK_UPDATE_FW
	ts->update_thread = kthread_run(ilitek_update_thread, NULL, "ilitek_update_thread");
	if (ts->update_thread == (struct task_struct *)ERR_PTR) {
		ts->update_thread = NULL;
		tp_err("kthread create ilitek_update_thread, error\n");
	}
#else
	ret = ilitek_request_input_dev();
	if (ret) {
		tp_err("ilitek_request_input_dev failed, err: %d\n", ret);
		goto err_free_gpio;
	}

	ret = ilitek_request_irq();
	if (ret) {
		tp_err("ilitek_request_irq failed, err: %d\n", ret);
		input_free_device(ts->input_dev);
		goto err_free_gpio;
	}
#endif

#ifdef ILITEK_ISR_PROTECT
	tp_msg("ILITEK_ISR_PROTECT is enabled\n");
#endif

	ilitek_register_resume_suspend();

	ilitek_create_sysfsnode();

#ifdef ILITEK_TOOL
	ilitek_create_tool_node();
#endif
	ilitek_init_netlink();

	ilitek_create_esdandcharge_workqueue();

	ts->gesture_status = ILITEK_GESTURE_DEFAULT;
	if (ts->gesture_status)
		ilitek_register_gesture(ts, true);

	return 0;

err_free_gpio:
	ilitek_free_gpio();
	ilitek_power_on(false);
	ilitek_release_regulator();

	tp_err("return -ENODEV\n");
	kfree(ts);

	return -ENODEV;
}

int ilitek_main_remove(struct i2c_client *client)
{
	tp_msg("\n");

	if (!ts)
		return 0;

	if (ts->gesture_status)
		ilitek_register_gesture(ts, false);

#ifndef MTK_UNDTS
	free_irq(ts->client->irq, ts);
#endif

#ifdef ILITEK_TUNING_MESSAGE
	if (ilitek_netlink_sock != NULL) {
		netlink_kernel_release(ilitek_netlink_sock);
		ilitek_netlink_sock = NULL;
	}
#endif

	ilitek_release_resume_suspend();

	if (ts->input_dev) {
		input_unregister_device(ts->input_dev);
		ts->input_dev = NULL;
	}

#ifdef ILITEK_TOOL
	ilitek_remove_tool_node();
#endif

	ilitek_remove_sys_node();

#ifdef ILITEK_ESD_PROTECTION
	if (ts->esd_wq) {
		cancel_delayed_work(&ts->esd_work);
		flush_workqueue(ts->esd_wq);
		destroy_workqueue(ts->esd_wq);
		ts->esd_wq = NULL;
	}
#endif

	ilitek_power_on(false);
	ilitek_release_regulator();

	ilitek_free_gpio();

	ilitek_free_dma();

	kfree(ts);
	ts = NULL;

	return 0;
}
