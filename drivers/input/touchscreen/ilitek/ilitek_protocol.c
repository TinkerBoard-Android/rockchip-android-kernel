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
#include "ilitek_common.h"
#include <linux/slab.h>
#include <linux/delay.h>
#include "ilitek_protocol.h"

typedef int protocol_func(uint8_t *inbuf, uint8_t *outbuf);

typedef struct
{
	uint16_t cmd;
	protocol_func *func;
	uint8_t *name;
	uint8_t flag;
} PROTOCOL_MAP;


static int api_ptl_set_func_mode(uint8_t *inbuf, uint8_t *outbuf)
{
	int ret = ILITEK_SUCCESS;

	inbuf[0] = CMD_SET_FUNC_MOD;
	ret = ilitek_i2c_write_and_read(inbuf, 4, 10, outbuf, 0);
	return ret;
}
static int api_ptl_system_busy(uint8_t *inbuf, uint8_t *outbuf)
{
	int ret = ILITEK_SUCCESS;

	inbuf[0] = CMD_GET_SYS_BUSY;
	ret = ilitek_i2c_write_and_read(inbuf, 1, 10, outbuf, 1);
	return ret;
}

static int api_ptl_set_test_mode(uint8_t *inbuf, uint8_t *outbuf)
{
	int ret = ILITEK_SUCCESS;

	inbuf[0] = CMD_SET_TEST_MOD;
	ret = ilitek_i2c_write_and_read(inbuf, 2, 0, outbuf, 0);
	return ret;
}

static int api_ptl_write_enable(uint8_t *inbuf, uint8_t *outbuf)
{
	int ret = ILITEK_SUCCESS;
	uint8_t buf[64] = {0};

	buf[0] = CMD_WRITE_ENABLE;
	buf[1] = 0x5A;
	buf[2] = 0xA5;
	ret = ilitek_i2c_write_and_read(buf, 3, 0, outbuf, 0);
	return ret;
}

static int api_ptl_set_apmode(uint8_t *inbuf, uint8_t *outbuf)
{
	int ret = ILITEK_SUCCESS;
	uint8_t buf[64] = {0};

	buf[0] = CMD_SET_AP_MODE;
	ret = ilitek_i2c_write_and_read(buf, 1, 0, outbuf, 0);
	return ret;
}

static int api_ptl_set_blmode(uint8_t *inbuf, uint8_t *outbuf)
{
	int ret = ILITEK_SUCCESS;
	uint8_t buf[64] = {0};

	buf[0] = CMD_SET_BL_MODE;
	buf[1] = 0x5A;
	buf[2] = 0xA5;
	ret = ilitek_i2c_write_and_read(buf, 1, 0, outbuf, 0);
	return ret;
}

static int api_ptl_get_mcu_ver(uint8_t *inbuf, uint8_t *outbuf)
{
	uint8_t buf[64] = {0};

	ts->ic_type = 0;

	buf[0] = CMD_GET_MCU_VER;
	if (ilitek_i2c_write_and_read(buf, 1, 5, outbuf, 32) < 0)
		return ILITEK_FAIL;

	memcpy(ts->mcu_ver, outbuf, 5);
	memset(ts->product_id, 0, sizeof(ts->product_id));
	memcpy(ts->product_id, outbuf + 6, 26);

	ts->ic_type = ts->mcu_ver[0] + (ts->mcu_ver[1] << 8);
	tp_msg("MCU KERNEL version:0x%x\n", ts->ic_type);
	tp_msg("Module name:%s\n", ts->product_id);

	if (ts->ptl.ver >= PROTOCOL_V6 || ts->ptl.ver == BL_V1_8)
		ts->upg.ic_df_start_addr = (outbuf[4] << 16) + (outbuf[3] << 8) + outbuf[2];
	else
		ts->upg.ic_df_start_addr = (outbuf[2] << 16) + (outbuf[3] << 8) + outbuf[4];

	tp_msg("IC DF start addr:0x%x\n", ts->upg.ic_df_start_addr);

	return ILITEK_SUCCESS;
}

static int api_ptl_check_mode(uint8_t *inbuf, uint8_t *outbuf)
{
	int ret = ILITEK_SUCCESS;
	uint8_t buf[64] = {0};

	buf[0] = CMD_GET_MCU_MOD;
	ret = ilitek_i2c_write_and_read(buf, 1, 5, outbuf, 2);
	tp_msg("ilitek ic. mode =%d , it's %s\n", outbuf[0], ((outbuf[0] == 0x5A) ? "AP MODE" : "BL MODE"));

	ts->force_update = false;

#ifdef ILITEK_UPDATE_FW
	if (outbuf[0] == 0x55)
		ts->force_update = true;
#endif
	return ret;
}

static int api_ptl_get_fw_ver(uint8_t *inbuf, uint8_t *outbuf)
{
	int ret = ILITEK_SUCCESS;
	uint8_t buf[64] = {0};

	buf[0] = CMD_GET_FW_VER;
	ret = ilitek_i2c_write_and_read(buf, 1, 5, outbuf, 8);

	memcpy(ts->fw_ver, outbuf, 8);

	tp_msg_arr("firmware version:", ts->fw_ver, 8);

	return ret;
}

static int api_ptl_get_ptl_ver(uint8_t *inbuf, uint8_t *outbuf)
{
	int ret = ILITEK_SUCCESS;
	uint8_t buf[64] = {0};

	buf[0] = CMD_GET_PTL_VER;
	if (ilitek_i2c_write_and_read(buf, 1, 5, outbuf, 3) < ILITEK_SUCCESS)
		return ILITEK_FAIL;
	ts->ptl.ver = (((int)outbuf[0]) << 16) + (((int)outbuf[1]) << 8);
	ts->ptl.ver_major = outbuf[0];
	ts->ptl.ver_mid = outbuf[1];
	ts->ptl.ver_minor = outbuf[2];

	ts->bl_ver = (outbuf[0] << 8) + outbuf[1];

	tp_msg("protocol version: %d.%d.%d  ts->ptl.ver = 0x%x\n",
		ts->ptl.ver_major, ts->ptl.ver_mid, ts->ptl.ver_minor,
		ts->ptl.ver);
	return ret;
}

static int api_ptl_get_sc_res(uint8_t *inbuf, uint8_t *outbuf)
{
	int ret = ILITEK_SUCCESS;
	uint8_t buf[64] = {0};

	buf[0] = CMD_GET_SCRN_RES;
	ret = ilitek_i2c_write_and_read(buf, 1, 5, outbuf, 8);
	ts->screen_min_x = outbuf[0];
	ts->screen_min_x += ((int)outbuf[1]) * 256;
	ts->screen_min_y = outbuf[2];
	ts->screen_min_y += ((int)outbuf[3]) * 256;
	ts->screen_max_x = outbuf[4];
	ts->screen_max_x += ((int)outbuf[5]) * 256;
	ts->screen_max_y = outbuf[6];
	ts->screen_max_y += ((int)outbuf[7]) * 256;
	tp_msg("screen_min_x: %d, screen_min_y: %d screen_max_x: %d, screen_max_y: %d\n",
		    ts->screen_min_x, ts->screen_min_y, ts->screen_max_x, ts->screen_max_y);
	return ret;
}

static int api_ptl_set_sleep(uint8_t *inbuf, uint8_t *outbuf)
{
	int ret = ILITEK_SUCCESS;
	uint8_t buf[64] = {0};

	buf[0] = CMD_SET_IC_SLEEP;
	ret = ilitek_i2c_write_and_read(buf, 1, 0, outbuf, 0);
	return ret;
}

static int api_ptl_set_wakeup(uint8_t *inbuf, uint8_t *outbuf)
{
	int ret = ILITEK_SUCCESS;
	uint8_t buf[64] = {0};

	buf[0] = CMD_SET_IC_WAKE;
	ret = ilitek_i2c_write_and_read(buf, 1, 0, outbuf, 0);
	return ret;
}

static int api_ptl_get_key_info_v3(uint8_t *inbuf, uint8_t *outbuf)
{
	int ret = ILITEK_SUCCESS, i = 0;
	uint8_t buf[64] = {0};
	int key_num = ts->keycount;

	buf[0] = CMD_GET_KEY_INFO;
	ret = ilitek_i2c_write_and_read(buf, 1, 10, outbuf, ILITEK_KEYINFO_FIRST_PACKET);
	if (ret < 0)
		return ret;
	if (key_num > 5) {
		for (i = 0; i < CEIL(key_num, ILITEK_KEYINFO_FORMAT_LENGTH) - 1; i++) {
			tp_msg("read keyinfo times i = %d\n", i);
			ret = ilitek_i2c_write_and_read(buf, 0, 10,
					outbuf + ILITEK_KEYINFO_FIRST_PACKET + ILITEK_KEYINFO_OTHER_PACKET * i,
					ILITEK_KEYINFO_OTHER_PACKET);
			if (ret < 0)
				return ret;
		}
	}

	ts->key_xlen = (outbuf[0] << 8) + outbuf[1];
	ts->key_ylen = (outbuf[2] << 8) + outbuf[3];
	tp_msg("key_xlen: %d, key_ylen: %d\n", ts->key_xlen, ts->key_ylen);

	for (i = 0; i < key_num; i++) {
		ts->keyinfo[i].id = outbuf[i * ILITEK_KEYINFO_FORMAT_LENGTH + ILITEK_KEYINFO_V3_HEADER + ILITEK_KEYINFO_FORMAT_ID];
		ts->keyinfo[i].x = (outbuf[i * ILITEK_KEYINFO_FORMAT_LENGTH + ILITEK_KEYINFO_V3_HEADER + ILITEK_KEYINFO_FORMAT_X_MSB] << 8)
			+ outbuf[i * ILITEK_KEYINFO_FORMAT_LENGTH + ILITEK_KEYINFO_V3_HEADER + ILITEK_KEYINFO_FORMAT_X_LSB];
		ts->keyinfo[i].y = (outbuf[i * ILITEK_KEYINFO_FORMAT_LENGTH + ILITEK_KEYINFO_V3_HEADER + ILITEK_KEYINFO_FORMAT_Y_MSB] << 8)
			+ outbuf[i * ILITEK_KEYINFO_FORMAT_LENGTH + ILITEK_KEYINFO_V3_HEADER + ILITEK_KEYINFO_FORMAT_Y_LSB];
		ts->keyinfo[i].status = 0;
		tp_msg("key_id: %d, key_x: %d, key_y: %d, key_status: %d\n",
				ts->keyinfo[i].id, ts->keyinfo[i].x, ts->keyinfo[i].y, ts->keyinfo[i].status);
	}
	return ret;
}

static int api_ptl_get_tp_res_v3(uint8_t *inbuf, uint8_t *outbuf)
{
	int ret = ILITEK_SUCCESS;
	uint8_t buf[64] = {0};

	buf[0] = CMD_GET_TP_RES;
	ret = ilitek_i2c_write_and_read(buf, 1, 5, outbuf, 10);
	if (ret < 0)
		return ret;
	ts->max_tp = outbuf[6];
	ts->max_btn = outbuf[7];
	ts->keycount = outbuf[8];
	if (ts->keycount > 20) {
		tp_err("exception keycount > 20 is %d set keycount = 0\n", ts->keycount);
		ts->keycount = 0;
	}
	ts->tp_max_x = outbuf[0];
	ts->tp_max_x += ((int)outbuf[1]) << 8;
	ts->tp_max_y = outbuf[2];
	ts->tp_max_y += ((int)outbuf[3]) << 8;
	ts->x_ch = outbuf[4];
	ts->y_ch = outbuf[5];

	if (ts->keycount > ILITEK_SUPPORT_MAX_KEY_CNT) {
		tp_err("exception keycount %d > %d\n", ts->keycount, ILITEK_SUPPORT_MAX_KEY_CNT);
		return -EINVAL;
	} else if (ts->keycount > 0 && ts->ptl.ver_major > 0x1) {
		ret = api_ptl_get_key_info_v3(NULL, outbuf);
		if (ret < 0)
			return ret;
	}

	if (ts->max_tp > 40) {
		tp_err("exception max_tp > 40 is %d set max_tp = 10\n", ts->max_tp);
		ts->max_tp = 10;
	}
	tp_msg("tp_min_x: %d, tp_max_x: %d, tp_min_y: %d, tp_max_y: %d, ch_x: %d, ch_y: %d, max_tp: %d, key_count: %d\n",
			ts->tp_min_x, ts->tp_max_x, ts->tp_min_y, ts->tp_max_y, ts->x_ch,
			ts->y_ch, ts->max_tp, ts->keycount);
	return ret;
}

static int api_ptl_get_key_info_v6(uint8_t *inbuf, uint8_t *outbuf)
{
	int ret = ILITEK_SUCCESS, i = 0;
	uint8_t buf[64] = {0};
	int key_num = ts->keycount;

	buf[0] = CMD_GET_KEY_INFO;
	ret = ilitek_i2c_write_and_read(buf, 1, 10, outbuf, ILITEK_KEYINFO_FIRST_PACKET);
	if (ret < 0)
		return ret;

	ts->key_xlen = (outbuf[2] << 8) + outbuf[1];
	ts->key_ylen = (outbuf[4] << 8) + outbuf[3];
	tp_msg("v6 key_xlen: %d, key_ylen: %d\n", ts->key_xlen, ts->key_ylen);

	for (i = 0; i < key_num; i++) {
		ts->keyinfo[i].id = outbuf[i * ILITEK_KEYINFO_FORMAT_LENGTH + ILITEK_KEYINFO_V6_HEADER + ILITEK_KEYINFO_FORMAT_ID];
		ts->keyinfo[i].x = (outbuf[i * ILITEK_KEYINFO_FORMAT_LENGTH + ILITEK_KEYINFO_V6_HEADER + ILITEK_KEYINFO_FORMAT_X_LSB] << 8)
			+ outbuf[i * ILITEK_KEYINFO_FORMAT_LENGTH + ILITEK_KEYINFO_V6_HEADER + ILITEK_KEYINFO_FORMAT_X_MSB];
		ts->keyinfo[i].y = (outbuf[i * ILITEK_KEYINFO_FORMAT_LENGTH + ILITEK_KEYINFO_V6_HEADER + ILITEK_KEYINFO_FORMAT_Y_LSB] << 8)
			+ outbuf[i * ILITEK_KEYINFO_FORMAT_LENGTH + ILITEK_KEYINFO_V6_HEADER + ILITEK_KEYINFO_FORMAT_Y_MSB];
		ts->keyinfo[i].status = 0;
		tp_msg("key_id: %d, key_x: %d, key_y: %d, key_status: %d\n",
				ts->keyinfo[i].id, ts->keyinfo[i].x, ts->keyinfo[i].y, ts->keyinfo[i].status);
	}
	return ret;
}

static int api_ptl_get_tp_res_v6(uint8_t *inbuf, uint8_t *outbuf)
{
	int ret = ILITEK_SUCCESS;
	uint8_t buf[64] = {0};

	buf[0] = CMD_GET_TP_RES;
	ret = ilitek_i2c_write_and_read(buf, 1, 5, outbuf, 15);
	if (ret < 0)
		return ret;
	ts->tp_max_x = outbuf[0];
	ts->tp_max_x += ((int)outbuf[1]) << 8;
	ts->tp_max_y = outbuf[2];
	ts->tp_max_y += ((int)outbuf[3]) << 8;
	ts->x_ch = outbuf[4] + ((int)outbuf[5] << 8);
	ts->y_ch = outbuf[6] + ((int)outbuf[7] << 8);
	ts->max_tp = outbuf[8];
	ts->keycount = outbuf[9];
	ts->ic_num = outbuf[10];
	ts->format = outbuf[12];

	if (ts->keycount > ILITEK_SUPPORT_MAX_KEY_CNT) {
		tp_err("exception keycount %d > %d\n", ts->keycount, ILITEK_SUPPORT_MAX_KEY_CNT);
		return -EINVAL;
	} else if (ts->keycount > 0 && ts->ptl.ver_major > 0x1) {
		ret = api_ptl_get_key_info_v6(NULL, outbuf);
		if (ret < 0)
			return ret;
	}
	if (ts->max_tp > 40) {
		tp_err("exception max_tp > 40 is %d set max_tp = 40\n", ts->max_tp);
		ts->max_tp = 40;
	}
	tp_msg("tp_min_x: %d, tp_max_x: %d, tp_min_y: %d, tp_max_y: %d, ch_x: %d, ch_y: %d, max_tp: %d, key_count: %d, ic_num: %d report format: %d\n",
			ts->tp_min_x, ts->tp_max_x, ts->tp_min_y, ts->tp_max_y, ts->x_ch,
			ts->y_ch, ts->max_tp, ts->keycount, ts->ic_num, ts->format);
	return ret;
}

static int api_ptl_get_key_info(uint8_t *inbuf, uint8_t *outbuf)
{
	if (ts->ptl.flag == PTL_V3)
		return api_ptl_get_key_info_v3(inbuf, outbuf);
	else if (ts->ptl.flag == PTL_V6)
		return api_ptl_get_key_info_v6(inbuf, outbuf);
	else
		return -EINVAL;
}

static int api_ptl_get_tp_res(uint8_t *inbuf, uint8_t *outbuf)
{
	if (ts->ptl.flag == PTL_V3)
		return api_ptl_get_tp_res_v3(inbuf, outbuf);
	else if (ts->ptl.flag == PTL_V6)
		return api_ptl_get_tp_res_v6(inbuf, outbuf);
	else
		return -EINVAL;
}


static int api_ptl_write_flash_enable(uint8_t *inbuf, uint8_t *outbuf)
{
	int ret = ILITEK_SUCCESS;
	uint8_t buf[64] = {0};

	buf[0] = CMD_SET_W_FLASH;
	buf[1] = 0x5A;
	buf[2] = 0xA5;
	ret = ilitek_i2c_write_and_read(buf, 3, 0, outbuf, 0);
	return ret;
}

static int api_ptl_get_block_crc(uint8_t *inbuf, uint8_t *outbuf)
{
	int ret = ILITEK_SUCCESS;
	uint16_t crc=0;

	inbuf[0] = CMD_GET_BLK_CRC;
	if (inbuf[1] == 0) {
		ret = ilitek_i2c_write_and_read(inbuf, 8, 10, outbuf, 0);
		ret = ilitek_check_busy(50, 50, ILITEK_TP_SYSTEM_BUSY);
	}
	inbuf[1] = 1;
	ret = ilitek_i2c_write_and_read(inbuf, 2, 1, outbuf, 2);
	crc = outbuf[0]+(outbuf[1] << 8);
	return crc;
}

static int api_ptl_set_mode(uint8_t *inbuf, uint8_t *outbuf)
{
	int ret = ILITEK_SUCCESS;
	inbuf[0] = CMD_SET_MOD_CTRL;
	inbuf[2] = 0;
	ret = ilitek_i2c_write_and_read(inbuf, 3, 100, outbuf, 0);
	return ret;
}

static int api_ptl_set_cdc_init(uint8_t *inbuf, uint8_t *outbuf)
{
	int ret = ILITEK_SUCCESS;

	inbuf[0] = CMD_SET_CDC_INIT;
	ret = ilitek_i2c_write_and_read(inbuf, 2, 10, outbuf, 0);
	return ret;
}

/* Private define ------------------------------------------------------------*/
PROTOCOL_MAP ptl_map[] =
{
	/* common cmds */
	[GET_PTL_VER] = {CMD_GET_PTL_VER, api_ptl_get_ptl_ver, "GET_PTL_VER", PTL_V3 | PTL_V6},
	[GET_FW_VER] = {CMD_GET_FW_VER, api_ptl_get_fw_ver, "GET_FW_VER", PTL_V3 | PTL_V6},
	[GET_SCRN_RES] = {CMD_GET_SCRN_RES, api_ptl_get_sc_res, "GET_SCRN_RES", PTL_V3 | PTL_V6},
	[GET_TP_RES] = {CMD_GET_TP_RES, api_ptl_get_tp_res ,"GET_TP_RES", PTL_V3 | PTL_V6},
	[GET_MCU_MOD] = {CMD_GET_MCU_MOD, api_ptl_check_mode, "GET_MCU_MODE", PTL_V3 | PTL_V6},
	[GET_MCU_VER] = {CMD_GET_MCU_VER, api_ptl_get_mcu_ver, "GET_MCU_VER", PTL_V3 | PTL_V6},
	[GET_KEY_INFO] = {CMD_GET_KEY_INFO, api_ptl_get_key_info, "GET_KEY_INFO", PTL_V3 | PTL_V6},
	[SET_IC_SLEEP] = {CMD_SET_IC_SLEEP, api_ptl_set_sleep, "SET_IC_SLEEP", PTL_V3 | PTL_V6},
	[SET_IC_WAKE] = {CMD_SET_IC_WAKE, api_ptl_set_wakeup, "SET_IC_WAKE", PTL_V3 | PTL_V6},
	[SET_FUNC_MOD] = {CMD_SET_FUNC_MOD, api_ptl_set_func_mode, "SET_FUNC_MOD", PTL_V3 | PTL_V6},
	[GET_SYS_BUSY] = {CMD_GET_SYS_BUSY, api_ptl_system_busy, "GET_SYS_BUSY", PTL_V3 | PTL_V6},
	[SET_AP_MODE] = {CMD_SET_AP_MODE, api_ptl_set_apmode, "SET_AP_MODE", PTL_V3 | PTL_V6},
	[SET_BL_MODE] = {CMD_SET_BL_MODE, api_ptl_set_blmode, "SET_BL_MODE", PTL_V3 | PTL_V6},

	/* v3 only cmds */
	[SET_WRITE_EN] = {CMD_WRITE_ENABLE, api_ptl_write_enable, "CMD_WRITE_DATA", PTL_V3},
	[SET_TEST_MOD] = {CMD_SET_TEST_MOD, api_ptl_set_test_mode, "SET_TEST_MODE", PTL_V3},

	/* v6 only cmds */
	[SET_MOD_CTRL] = {CMD_SET_MOD_CTRL, api_ptl_set_mode, "SET_CTRL_MODE", PTL_V6},
	[SET_W_FLASH] = {CMD_SET_W_FLASH, api_ptl_write_flash_enable, "SET_WRITE_FLASH", PTL_V6},
	[GET_BLK_CRC] = {CMD_GET_BLK_CRC, api_ptl_get_block_crc, "GET_BLK_CRC", PTL_V6},
	[SET_CDC_INIT] = {CMD_SET_CDC_INIT, api_ptl_set_cdc_init, "SET_CRC_INIT", PTL_V6},
};

//-----------------------extern api function------------------------//
int api_ptl_set_cmd(uint16_t idx, uint8_t *inbuf, uint8_t *outbuf)
{
	int ret = ILITEK_SUCCESS;

	if (idx >= MAX_CMD_CNT)
		return -EINVAL;

	if (!(ts->ptl.flag & ptl_map[idx].flag)) {
		tp_err("Unexpected cmd: %s for %x only, now is %x\n",
			ptl_map[idx].name, ptl_map[idx].flag, ts->ptl.flag);
		return -EINVAL;
	}

	ret = ptl_map[idx].func(inbuf, outbuf);
	if (ret < 0)
		return ret;

	return ILITEK_SUCCESS;
}

int api_set_data_length(uint32_t data_len)
{
	int ret = ILITEK_SUCCESS;
	uint8_t buf[64] = {0};

	buf[0] = CMD_SET_DATA_LEN;
	buf[1] = (uint8_t)(data_len & 0xFF);
	buf[2] = (uint8_t)(data_len >> 8);
	ret = ilitek_i2c_write_and_read(buf, 3, 1, NULL, 0);
	return ret;
}

int api_write_flash_enable_BL1_8(uint32_t start,uint32_t end)
{
	uint8_t buf[64] = {0};

	buf[0] = (uint8_t)CMD_SET_W_FLASH;
	buf[1] = 0x5A;
	buf[2] = 0xA5;
	buf[3] = start & 0xFF;
	buf[4] = (start >> 8) & 0xFF;
	buf[5] = start >> 16;
	buf[6] = end & 0xFF;
	buf[7] = (end >> 8) & 0xFF;
	buf[8] = end >> 16;

	return ilitek_i2c_write_and_read(buf, 9, 1, NULL, 0);
}

uint16_t api_get_block_crc(uint32_t start, uint32_t end, uint32_t type)
{
	uint8_t inbuf[8] = {0}, outbuf[2] = {0};

	inbuf[1] = type;
	inbuf[2] = start;
	inbuf[3] = (start >> 8) & 0xFF;
	inbuf[4] = (start >> 16) & 0xFF;
	inbuf[5] = end & 0xFF;
	inbuf[6] = (end >> 8) & 0xFF;
	inbuf[7] = (end >> 16) & 0xFF;
	if (api_ptl_set_cmd(GET_BLK_CRC, inbuf, outbuf) < 0)
		return ILITEK_FAIL;

	return 	outbuf[0]+(outbuf[1] << 8);
}


int api_get_slave_mode(int number)
{
	int i = 0;
	uint8_t inbuf[64] = {0}, outbuf[64] = {0};

	inbuf[0] = (uint8_t)CMD_GET_MCU_MOD;
	if (ilitek_i2c_write_and_read(inbuf, 1, 5, outbuf, number*2) < 0)
		return ILITEK_FAIL;
	for(i = 0; i < number && i < ARRAY_SIZE(ts->ic); i++){
		ts->ic[i].mode = outbuf[i * 2];
		tp_msg("IC[%d] mode: 0x%x\n", i, ts->ic[i].mode);
	}
	return ILITEK_SUCCESS;
}

uint32_t api_get_ap_crc(int number)
{
	int i = 0;
	uint8_t inbuf[64] = {0}, outbuf[64] = {0};

	inbuf[0] = CMD_GET_AP_CRC;
	if (ilitek_i2c_write_and_read(inbuf, 1, 1, outbuf, number * 2) < 0)
		return ILITEK_FAIL;

	for (i = 0; i < number && i < ARRAY_SIZE(ts->ic); i++){
		ts->ic[i].crc = outbuf[i * 2] + (outbuf[i * 2 + 1] << 8);
		tp_msg("IC[%d] CRC: 0x%x\n", i, ts->ic[i].crc);
	}
	return ILITEK_SUCCESS;
}

int api_set_access_slave(uint8_t type)
{
	int error;
	uint8_t inbuf[64];

	inbuf[0] = (uint8_t)CMD_ACCESS_SLAVE;
	inbuf[1] = 0x3;
	inbuf[2] = type;

	if ((error = ilitek_i2c_write_and_read(inbuf, 3, 1, NULL, 0)) < 0)
		return error;

	mdelay(2000);

	if (ilitek_check_busy(10, 100, ILITEK_TP_SYSTEM_BUSY) < 0) {
		tp_err("%s, Last: CheckBusy Failed\n", __func__);
		return ILITEK_FAIL;
	}
	return ILITEK_SUCCESS;
}

int api_set_testmode(bool testmode)
{
	uint8_t inbuf[3];
	if (ts->ptl.flag == PTL_V3) {
		if (testmode)
			inbuf[1] = 0x01;
		else
			inbuf[1] = 0x00;

		if (api_ptl_set_cmd(SET_TEST_MOD, inbuf, NULL) < 0)
			return ILITEK_FAIL;

		mdelay(10);
	} else if(ts->ptl.flag == PTL_V6) {
		if (testmode)
			inbuf[1] = ENTER_SUSPEND_MODE;
		else
			inbuf[1] = ENTER_NORMAL_MODE;

		inbuf[2] = 0;
		if (api_ptl_set_cmd(SET_MOD_CTRL, inbuf, NULL) < 0)
			return ILITEK_FAIL;

		mdelay(100);
	}
	return ILITEK_SUCCESS;
}

int api_init_func(void)
{
	int ret = ILITEK_SUCCESS;
	uint8_t outbuf[64] = {0};

	tp_msg("\n");
	ts->ptl.flag = PTL_V6;

	//Prevent interrupt will interfere with i2c transfer.
	if (api_set_testmode(true) < ILITEK_SUCCESS)
		return ILITEK_FAIL;

	if (api_ptl_set_cmd(GET_PTL_VER, NULL, outbuf) < ILITEK_SUCCESS)
		return ILITEK_FAIL;

	if (api_set_testmode(false) < ILITEK_SUCCESS)
		return ILITEK_FAIL;

	ts->ptl.flag = PTL_V3;
	ts->process_and_report = ilitek_read_data_and_report_3XX;
	ts->reset_time = 200;
	ts->irq_tri_type = IRQF_TRIGGER_FALLING;

	if (ts->ptl.ver_major == 0x3 || ts->ptl.ver == BL_V1_6 || ts->ptl.ver == BL_V1_7) {
		ts->ptl.flag = PTL_V3;
		tp_msg("Protocol: V3, set ISR falling trigger\n");
		ts->process_and_report = ilitek_read_data_and_report_3XX;
		ts->reset_time = 200;
		ts->irq_tri_type = IRQF_TRIGGER_FALLING;
	} else if (ts->ptl.ver_major == 0x6 || ts->ptl.ver == BL_V1_8) {
		ts->ptl.flag = PTL_V6;
		ts->process_and_report = ilitek_read_data_and_report_6XX;
		ts->reset_time = 600;
#if 0
		tp_msg("Protocol: V6, set ISR rising trigger\n");
		ts->irq_tri_type = IRQF_TRIGGER_RISING;
#else
		tp_msg("Protocol: V6, set ISR falling trigger\n");
		ts->irq_tri_type = IRQF_TRIGGER_FALLING;
#endif		
	} else {
		tp_err("Unknown protocl: %x, set as V3\n", ts->ptl.ver);
	}
	return ret;
}

int api_get_funcmode(int mode)
{
	int ret = ILITEK_SUCCESS;
	uint8_t inbuf[2], outbuf[64] = {0};

	inbuf[0] = CMD_SET_FUNC_MOD;
	ret = ilitek_i2c_write_and_read(inbuf, 1, 10, outbuf, 3);
	tp_msg("mode:%d\n", outbuf[2]);
	if (mode != outbuf[2])
		tp_msg("Set mode fail, set:%d, read:%d\n", mode, outbuf[2]);

	return ret;
}

int api_set_funcmode(int mode)
{
	int ret = ILITEK_SUCCESS, i = 0;
	uint8_t inbuf[4] = {CMD_SET_FUNC_MOD, 0x55, 0xAA, mode}, outbuf[64] = {0};;

	tp_msg("ts->ptl.ver:0x%x\n", ts->ptl.ver);
	if (ts->ptl.ver >= 0x30400) {
		ret = api_ptl_set_func_mode(inbuf, NULL);
		for (i = 0; i < 20; i++) {
			mdelay(100);
			ret = api_ptl_system_busy(inbuf, outbuf);
			if (ret < ILITEK_SUCCESS)
				return ret;
			if (outbuf[0] == ILITEK_TP_SYSTEM_READY) {
				tp_msg("system is ready\n");
				ret = api_get_funcmode(mode);
				return ret;
			}
		}
		tp_msg("system is busy\n");
	} else {
		tp_msg("It is protocol not support\n");
	}
	return ret;
}

int api_set_idlemode(int mode)
{
	int ret = 0;
	int8_t inbuf[2];

	inbuf[0] = CMD_SET_IDLE;
	inbuf[1] = mode;
	tp_msg("mode:%d\n", mode);
	ret = ilitek_i2c_write_and_read(inbuf, 2, 10, NULL, 0);
	return ret;
}
