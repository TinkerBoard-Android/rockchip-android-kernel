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
#include <linux/vmalloc.h>

#ifdef ILITEK_UPDATE_FW
#include "ilitek_fw.h"
#endif

static uint16_t UpdateCRC(uint16_t crc, uint8_t newbyte)
{
	char i;			// loop counter
#define CRC_POLY 0x8408		// CRC16-CCITT FCS (X^16+X^12+X^5+1)

	crc = crc ^ newbyte;

	for (i = 0; i < 8; i++) {
		if (crc & 0x01) {
			crc = crc >> 1;
			crc ^= CRC_POLY;
		} else {
			crc = crc >> 1;
		}
	}
	return crc;
}

uint16_t get_dri_crc(uint32_t startAddr, uint32_t endAddr,
		     const uint8_t input[])
{
	uint16_t CRC = 0;
	uint32_t i = 0;

	// 2 is CRC
	for (i = startAddr; i <= endAddr - 2; i++)
		CRC = UpdateCRC (CRC, input[i]);

	return CRC;
}

uint32_t get_dri_checksum(uint32_t startAddr, uint32_t endAddr,
			  const uint8_t input[])
{
	uint32_t sum = 0;
	uint32_t i = 0;

	for (i = startAddr; i < endAddr; i++)
		sum += input[i];

	return sum;
}

static uint32_t hex_2_dec(const uint8_t *hex, int32_t len)
{
	uint32_t ret = 0, temp = 0;
	int32_t i, shift = (len - 1) * 4;
	for (i = 0; i < len; shift -= 4, i++) {
		if ((hex[i] >= '0') && (hex[i] <= '9')) {
			temp = hex[i] - '0';
		} else if ((hex[i] >= 'a') && (hex[i] <= 'z')) {
			temp = (hex[i] - 'a') + 10;
		} else {
			temp = (hex[i] - 'A') + 10;
		}
		ret |= (temp << shift);
	}
	return ret;
}

bool checksum(unsigned char *buf, int len, unsigned char fw_checksum)
{
	unsigned char chk;

	chk = get_dri_checksum(0, len - 1, buf);
	chk = ~(chk) + 1;

	if (chk == fw_checksum)
		return true;

	tp_err("checksum not match, IC: %x vs. Driver: %x\n", fw_checksum, chk);

	return false;
}

static uint32_t findstr(int start, int end, const uint8_t *data,
			unsigned int buf_size, const uint8_t *tag, int tag_len)
{
	unsigned int i;

	for (i = start; i <= (end - tag_len) && i < (buf_size - tag_len); i++) {
		if (!strncmp((char *)(data + i), (char *)tag, tag_len))
			return i + 1 + 32;
	}

	return end;
}

uint32_t get_endaddr(uint32_t startAddr, uint32_t endAddr,
		     const uint8_t buf[], unsigned int buf_size, bool is_AP)
{
	uint32_t addr;
	uint8_t tag[32];
	const uint8_t ap_tag[] = "ILITek AP CRC   ";
	const uint8_t blk_tag[] = "ILITek END TAG  ";

	memset(tag, 0xFF, sizeof(tag));

	if (is_AP)
		memcpy(tag + 16, ap_tag, 16);
	else
		memcpy(tag + 16, blk_tag, 16);

	addr = findstr(startAddr, endAddr, buf, buf_size, tag, 32);
	tp_dbg("find tag in start/end: 0x%x/0x%x, tag addr: 0x%x\n",
		startAddr, endAddr, addr);
	return addr;
}

static int32_t check_busy(int32_t delay)
{
	int32_t i;
	uint8_t inbuf[2], outbuf[2];
	for (i = 0; i < 1000; i++) {
		inbuf[0] = CMD_GET_SYS_BUSY;
		if (ilitek_i2c_write_and_read(inbuf, 1, delay, outbuf, 1) < 0)
			return ILITEK_I2C_TRANSFER_ERR;

		if (outbuf[0] == ILITEK_TP_SYSTEM_READY)
			return 0;
	}
	tp_msg("check_busy error\n");
	return -1;
}

int32_t ilitek_changetoblmode(bool mode)
{
	int32_t i = 0;
	uint8_t outbuf[64];

	if (api_ptl_set_cmd(GET_MCU_MOD, NULL, outbuf) < 0)
		return ILITEK_I2C_TRANSFER_ERR;

	msleep(30);
	if (outbuf[0] == ILITEK_TP_MODE_APPLICATION)
		tp_msg("Touch IC mode: %x as AP MODE\n", outbuf[0]);
	else if (outbuf[0] == ILITEK_TP_MODE_BOOTLOADER)
		tp_msg("Touch IC mode: %x as BL MODE\n", outbuf[0]);
	else
		tp_err("Touch IC mode: %x as UNKNOWN MODE\n", outbuf[0]);

	if ((outbuf[0] == ILITEK_TP_MODE_APPLICATION && !mode) ||
	    (outbuf[0] == ILITEK_TP_MODE_BOOTLOADER && mode)) {
		if (mode)
			tp_msg("ilitek change to BL mode ok already BL mode\n");
		else
			tp_msg("ilitek change to AP mode ok already AP mode\n");

		return ILITEK_SUCCESS;

	}

	for (i = 0; i < 5; i++) {
		if (ts->bl_ver == 0x0108 || ts->ptl.ver_major == 0x6) {
			if (api_ptl_set_cmd(SET_W_FLASH, NULL, outbuf) < 0)
				return ILITEK_I2C_TRANSFER_ERR;

		} else {
			if (api_ptl_set_cmd(SET_WRITE_EN, NULL, outbuf) < 0)
				return ILITEK_I2C_TRANSFER_ERR;
		}

		msleep(20);
		if (mode) {
			if (api_ptl_set_cmd(SET_BL_MODE, NULL, outbuf) < 0)
				return ILITEK_I2C_TRANSFER_ERR;
		} else {
			if (api_ptl_set_cmd(SET_AP_MODE, NULL, outbuf) < 0)
				return ILITEK_I2C_TRANSFER_ERR;
		}

		msleep(500 + i * 100);
		if (api_ptl_set_cmd(GET_MCU_MOD, NULL, outbuf) < 0)
			return ILITEK_I2C_TRANSFER_ERR;

		msleep(30);
		if (outbuf[0] == ILITEK_TP_MODE_APPLICATION)
			tp_msg("Touch IC mode: %x as AP MODE\n", outbuf[0]);
		else if (outbuf[0] == ILITEK_TP_MODE_BOOTLOADER)
			tp_msg("Touch IC mode: %x as BL MODE\n", outbuf[0]);
		else
			tp_err("Touch IC mode: %x as UNKNOWN MODE\n", outbuf[0]);

		if ((outbuf[0] == ILITEK_TP_MODE_APPLICATION && !mode) ||
		    (outbuf[0] == ILITEK_TP_MODE_BOOTLOADER && mode)) {
			if (mode)
				tp_msg("ilitek change to BL mode ok\n");
			else
				tp_msg("ilitek change to AP mode ok\n");

			return ILITEK_SUCCESS;
		}
	}

	if (mode) {
		tp_err("change to bl mode err, 0x%X\n", outbuf[0]);
		return ILITEK_TP_CHANGETOBL_ERR;
	} else {
		tp_err("change to ap mode err, 0x%X\n", outbuf[0]);
		return ILITEK_TP_CHANGETOAP_ERR;
	}
}

static int32_t ilitek_upgrade_BL1_6(uint8_t *CTPM_FW)
{
	int32_t ret = 0, upgrade_status = 0, i = 0, j = 0, k = 0;
	uint8_t buf[64] = { 0 };

	uint32_t df_startaddr = ts->upg.df_start_addr;
	uint32_t df_endaddr = ts->upg.df_end_addr;
	uint32_t df_checksum = ts->upg.df_check;
	uint32_t ap_startaddr = ts->upg.ap_start_addr;
	uint32_t ap_endaddr = ts->upg.ap_end_addr;
	uint32_t ap_checksum = ts->upg.ap_check;

	uint8_t total_checksum = 0;
	uint32_t df_write_checksum = 0, ap_write_checksum = 0;

	if (df_startaddr < df_endaddr)
		ts->has_df = true;
	else
		ts->has_df = false;

	buf[0] = (uint8_t)CMD_WRITE_ENABLE;
	buf[1] = 0x5A;
	buf[2] = 0xA5;
	buf[3] = 0x01;
	if (!ts->has_df) {
		tp_msg("ilitek no df data set df_endaddr 0x1ffff\n");
		df_endaddr = 0x1ffff;
		df_checksum = 0x1000 * 0xff;
		buf[4] = df_endaddr >> 16;
		buf[5] = (df_endaddr >> 8) & 0xFF;
		buf[6] = (df_endaddr) & 0xFF;
		buf[7] = df_checksum >> 16;
		buf[8] = (df_checksum >> 8) & 0xFF;
		buf[9] = df_checksum & 0xFF;
	} else {
		buf[4] = df_endaddr >> 16;
		buf[5] = (df_endaddr >> 8) & 0xFF;
		buf[6] = (df_endaddr) & 0xFF;
		buf[7] = df_checksum >> 16;
		buf[8] = (df_checksum >> 8) & 0xFF;
		buf[9] = df_checksum & 0xFF;
	}
	tp_msg("df_startaddr=0x%X, df_endaddr=0x%X, df_checksum=0x%X\n",
		df_startaddr, df_endaddr, df_checksum);
	ret = ilitek_i2c_write(buf, 10);
	if (ret < 0)
		tp_err("ilitek_i2c_write\n");

	msleep(20);
	tp_msg("Start to write DF data...\n");
	for (i = df_startaddr, j = 0; i < df_endaddr; i += 32) {
		buf[0] = CMD_WRITE_DATA;
		for (k = 0; k < 32; k++) {
			if (!ts->has_df)
				buf[1 + k] = 0xff;
			else
				buf[1 + k] = CTPM_FW[i + k];

			df_write_checksum += buf[1 + k];
			total_checksum += buf[1 + k];
		}

		ret = ilitek_i2c_write(buf, 33);
		if (ret < 0) {
			tp_err("ilitek_i2c_write_and_read\n");
			return ILITEK_I2C_TRANSFER_ERR;
		}

		if (!(++j % (ts->page_number)))
			mdelay(50);
		else
			mdelay(3);

		upgrade_status = ((i - df_startaddr) * 100) / (df_endaddr - df_startaddr);
		if (!(upgrade_status % 10))
			tp_dbg("Firmware Upgrade(Data flash) status %02d%%\n", upgrade_status);
	}

	buf[0] = (uint8_t)CMD_WRITE_ENABLE;
	buf[1] = 0x5A;
	buf[2] = 0xA5;
	buf[3] = 0x00;

	if ((ap_endaddr + 1) % 32) {
		ap_checksum = ap_checksum + (32 + 32 - ((ap_endaddr + 1) % 32)) * 0xff;
		ap_endaddr = ap_endaddr + 32 + 32 - ((ap_endaddr + 1) % 32);
	} else {
		ap_checksum = (ap_checksum + 32 * 0xff) & 0xFF;
		ap_endaddr = ((ap_endaddr + 32)) & 0xFF;
	}
	buf[4] = ap_endaddr >> 16;
	buf[5] = (ap_endaddr >> 8) & 0xFF;
	buf[6] = ap_endaddr & 0xFF;
	buf[7] = ap_checksum >> 16;
	buf[8] = (ap_checksum >> 8) & 0xFF;
	buf[9] = ap_checksum & 0xFF;
	ret = ilitek_i2c_write(buf, 10);
	tp_msg("ap_startaddr=0x%X, ap_endaddr=0x%X, ap_checksum=0x%X\n",
		ap_startaddr, ap_endaddr, ap_checksum);

	msleep(20);
	tp_msg("Start to write AP data...\n");
	for (i = ap_startaddr, j = 0; i < ap_endaddr; i += 32) {
		buf[0] = CMD_WRITE_DATA;
		for (k = 0; k < 32; k++) {
			if (k == 31 && i + 32 > ap_endaddr)
				buf[1 + k] = 0;
			else
				buf[1 + k] = CTPM_FW[i + k];

			ap_write_checksum += buf[1 + k];
			total_checksum += buf[1 + k];
		}

		ret = ilitek_i2c_write(buf, 33);
		if (ret < 0) {
			tp_err("%s, ilitek_i2c_write_and_read\n", __func__);
			return ILITEK_I2C_TRANSFER_ERR;
		}

		if (!(++j % (ts->page_number)))
			mdelay(50);
		else
			mdelay(3);

		upgrade_status = ((i - ap_startaddr) * 100) / (ap_endaddr - ap_startaddr);
		if (!(upgrade_status % 10))
			tp_dbg("Firmware Upgrade(AP) status %02d%%\n", upgrade_status);
	}

	tp_msg("AP checksum (Hex/Write): (0x%x/0x%x), DF checksum (Hex/Write): (0x%x/0x%x)\n",
		ap_checksum, ap_write_checksum,
		df_checksum, df_write_checksum);

	buf[0] = 0xC7;
	ilitek_i2c_write_and_read(buf, 1, 10, buf, 1);
	tp_msg("Compare 1byte checksum (IC:%x/FW:%x)\n",
		buf[0], total_checksum);

	/* for upgrade via .ili especially */
	ilitek_reset(ts->reset_time);

	return 0;
}

static int32_t ilitek_upgrade_BL1_7(uint8_t *CTPM_FW)
{
	int32_t ret = 0, upgrade_status = 0, i = 0, k = 0, CRC_DF = 0, CRC_AP = 0;
	uint8_t buf[64] = { 0 };
	uint32_t df_startaddr = ts->upg.df_start_addr;
	uint32_t df_endaddr = ts->upg.df_end_addr;
	uint32_t ap_startaddr = ts->upg.ap_start_addr;
	uint32_t ap_endaddr = ts->upg.ap_end_addr;

	for (i = df_startaddr + 2; i < df_endaddr; i++)
		CRC_DF = UpdateCRC(CRC_DF, CTPM_FW[i]);

	tp_msg("CRC_DF = 0x%X\n", CRC_DF);

	CRC_AP = get_dri_crc(ap_startaddr, ap_endaddr, CTPM_FW);

	tp_msg("CRC_AP = 0x%x\n", CRC_AP);

	buf[0] = (uint8_t)CMD_WRITE_ENABLE;	//0xc4
	buf[1] = 0x5A;
	buf[2] = 0xA5;
	buf[3] = 0x01;
	buf[4] = df_endaddr >> 16;
	buf[5] = (df_endaddr >> 8) & 0xFF;
	buf[6] = (df_endaddr) & 0xFF;
	buf[7] = CRC_DF >> 16;
	buf[8] = (CRC_DF >> 8) & 0xFF;
	buf[9] = CRC_DF & 0xFF;
	ret = ilitek_i2c_write(buf, 10);
	if (ret < 0) {
		tp_err("ilitek_i2c_write\n");
		return ILITEK_I2C_TRANSFER_ERR;
	}
	check_busy(1);
	if (((df_endaddr) % 32) != 0)
		df_endaddr += 32;

	tp_msg("Start to write DF data...\n");
	for (i = df_startaddr; i < df_endaddr; i += 32) {
		// we should do delay when the size is equal to 512 bytes
		buf[0] = (uint8_t)CMD_WRITE_DATA;
		for (k = 0; k < 32; k++)
			buf[1 + k] = CTPM_FW[i + k];

		if (ilitek_i2c_write(buf, 33) < 0) {
			tp_err("%s, ilitek_i2c_write_and_read\n", __func__);
			return ILITEK_I2C_TRANSFER_ERR;
		}
		check_busy(1);
		upgrade_status = ((i - df_startaddr) * 100) / (df_endaddr - df_startaddr);
		tp_dbg("Firmware Upgrade(Data flash), %02d%%\n", upgrade_status);
	}

	buf[0] = (uint8_t)0xC7;
	if (ilitek_i2c_write(buf, 1) < 0) {
		tp_err("ilitek_i2c_write\n");
		return ILITEK_I2C_TRANSFER_ERR;
	}
	check_busy(1);
	buf[0] = (uint8_t)0xC7;
	ilitek_i2c_write_and_read(buf, 1, 10, buf, 4);
	tp_msg("upgrade end write c7 read 0x%X, 0x%X, 0x%X, 0x%X\n", buf[0], buf[1], buf[2], buf[3]);
	if (CRC_DF != (buf[1] << 8) + buf[0]) {
		tp_err("CRC DF compare error\n");
		//return ILITEK_UPDATE_FAIL;
	} else {
		tp_msg("CRC DF compare Right\n");
	}
	buf[0] = (uint8_t)CMD_WRITE_ENABLE;	//0xc4
	buf[1] = 0x5A;
	buf[2] = 0xA5;
	buf[3] = 0x00;
	buf[4] = (ap_endaddr + 1) >> 16;
	buf[5] = ((ap_endaddr + 1) >> 8) & 0xFF;
	buf[6] = ((ap_endaddr + 1)) & 0xFF;
	buf[7] = 0;
	buf[8] = (CRC_AP & 0xFFFF) >> 8;
	buf[9] = CRC_AP & 0xFFFF;
	ret = ilitek_i2c_write(buf, 10);
	if (ret < 0) {
		tp_err("ilitek_i2c_write\n");
		return ILITEK_I2C_TRANSFER_ERR;
	}
	check_busy(1);
	if (((ap_endaddr + 1) % 32) != 0) {
		tp_msg("ap_endaddr += 32\n");
		ap_endaddr += 32;
	}
	tp_msg("Start to write AP data...\n");
	for (i = ap_startaddr; i < ap_endaddr; i += 32) {
		buf[0] = (uint8_t)CMD_WRITE_DATA;
		for (k = 0; k < 32; k++) {
			buf[1 + k] = CTPM_FW[i + k];
		}
		if (ilitek_i2c_write(buf, 33) < 0) {
			tp_err("ilitek_i2c_write\n");
			return ILITEK_I2C_TRANSFER_ERR;
		}
		check_busy(1);
		upgrade_status = ((i - ap_startaddr) * 100) / (ap_endaddr - ap_startaddr);
		tp_dbg("Firmware Upgrade(AP) status: %02d%%\n", upgrade_status);
	}
	for (i = 0; i < 20; i++) {
		buf[0] = (uint8_t)0xC7;
		if (ilitek_i2c_write(buf, 1) < 0) {
			tp_err("ilitek_i2c_write\n");
			return ILITEK_I2C_TRANSFER_ERR;
		}
		check_busy(1);
		buf[0] = (uint8_t)0xC7;
		ret = ilitek_i2c_write_and_read(buf, 1, 10, buf, 4);
		if (ret < 0) {
			tp_err("ilitek_i2c_write_and_read 0xc7\n");
			return ILITEK_I2C_TRANSFER_ERR;
		}
		tp_msg("upgrade end write c7 read 0x%X, 0x%X, 0x%X, 0x%X\n", buf[0], buf[1], buf[2], buf[3]);
		if (CRC_AP != (buf[1] << 8) + buf[0]) {
			tp_err("CRC compare error retry\n");
			//return ILITEK_TP_UPGRADE_FAIL;
		} else {
			tp_msg("CRC compare Right\n");
			break;
		}
	}
	if (i >= 20) {
		tp_err("CRC compare error\n");
		return ILITEK_TP_UPGRADE_FAIL;
	}
	return 0;
}

int program_block_BL1_8(uint8_t *buffer, int block, uint32_t len)
{
	int ret = ILITEK_SUCCESS, i;
	int upgrade_status;
	uint16_t dri_crc = 0, ic_crc = 0;
	static uint8_t *buff = NULL;

	//buff = vmalloc(sizeof(uint8_t) * (len+1));
	buff = kmalloc(len+1, GFP_KERNEL);
	if (NULL == buff) {
		tp_err("buff NULL\n");
		return -ENOMEM;
	}
	memset(buff, 0xff, len+1);
	dri_crc = get_dri_crc(ts->blk[block].start, ts->blk[block].end, buffer);
	ret = api_write_flash_enable_BL1_8(ts->blk[block].start, ts->blk[block].end);
	tp_msg("Upgrade block %d\n", block);

	tp_msg("Start to write block[%d] data...\n", block);
	for (i = ts->blk[block].start;
	     i < ts->blk[block].end; i += len) {
		buff[0] = (uint8_t) CMD_WRITE_DATA;
		memcpy(buff + 1, buffer + i, len);

		ret = ilitek_i2c_write_and_read(buff, len + 1, 1, NULL, 0);
		if (ilitek_check_busy(40, 50, ILITEK_TP_SYSTEM_BUSY) < 0) {
			tp_err("%s, Write Datas: CheckBusy Failed\n", __func__);
			goto error_free;
		}
		upgrade_status = ((i - ts->blk[block].start + 1) * 100) / (ts->blk[block].end - ts->blk[block].start);
		tp_dbg("Firmware Upgrade(block: %d) status %02d%%\n", block, upgrade_status);
	}

	ic_crc = api_get_block_crc(ts->blk[block].start, ts->blk[block].end, CRC_GET_FROM_FLASH);
	tp_msg("Block:%d start:0x%x end:0x%x Real=0x%x,Get=0x%x\n",
		block, ts->blk[block].start, ts->blk[block].end,
		dri_crc, ic_crc);

	if (dri_crc != ic_crc) {
		tp_err("WriteAPData: CheckSum Failed! Real=0x%x,Get=0x%x\n",
			dri_crc, ic_crc);
		goto error_free;
	}

	kfree(buff);
	return ILITEK_SUCCESS;

error_free:
	kfree(buff);
	return ILITEK_FAIL;
}

static int program_slave_BL1_8(void)
{
	int ret = ILITEK_SUCCESS, i;
	int retry;

	if (ts->ptl.ver < PROTOCOL_V6) {
		tp_err("protocol: 0x%x not support\n", ts->ptl.ver);
		return -EINVAL;
	}

	ret = api_set_testmode(true);

	retry = 1;
	do {
		api_get_ap_crc(ts->ic_num);

		for (i = 0; i < ts->ic_num && i < ARRAY_SIZE(ts->ic); i++) {
			if (ts->ic[0].crc == ts->ic[i].crc)
				continue;

			tp_err("Master CRC:0x%x, IC[%d] CRC:0x%x not match or force upgrade, retry: %d\n",
				ts->ic[0].crc, i, ts->ic[i].crc, retry);

			if (retry <= 0)
				return -EFAULT;

			api_set_access_slave(PROTOCOL_V6_ACCESS_SLAVE_PROGRAM);
			api_write_flash_enable_BL1_8(ts->blk[0].start,
						     ts->blk[0].end);
			tp_msg("Please wait updating...\n");
			msleep(20000);
			break;
		}

	} while (--retry > 0);

	retry = 1;
	do {
		api_get_slave_mode(ts->ic_num);

		for (i = 0; i < ts->ic_num && i < ARRAY_SIZE(ts->ic); i++) {
			if (ts->ic[i].mode == ILITEK_TP_MODE_APPLICATION)
				continue;

			tp_err("IC[%d] Mode:0x%x not in AP, retry: %d\n",
				i, ts->ic[i].mode, retry);

			if (retry <= 0)
				return -EFAULT;

			api_set_access_slave(PROTOCOL_V6_ACCESS_SLAVE_SET_APMODE);
			break;
		}
	} while (--retry > 0);

	if (ilitek_read_tp_info(false) < 0) {
		tp_err("init read tp info error so exit\n");
		return ILITEK_FAIL;
	}

	ret = api_set_testmode(false);
	msleep(2000);

	return ILITEK_SUCCESS;
}

static int32_t ilitek_upgrade_BL1_8(uint8_t *fw_buf)
{
	int ret = ILITEK_SUCCESS, count = 0;
	uint32_t update_len = set_update_len;

	ret = api_set_data_length(update_len);
	for (count = 0; count < ts->upg.blk_num; count++) {
		if (!ts->blk[count].crc_match) {
			if (program_block_BL1_8(fw_buf, count, update_len) < 0) {
				tp_err("Upgrade Block:%d Fail\n", count);
				return ILITEK_FAIL;
			}
		}
	}

	if (ilitek_changetoblmode(false) < 0) {
		tp_err("Change to ap mode failed\n");
		return ILITEK_FAIL;
	}

	msleep(500);
	if (ilitek_read_tp_info(false) < 0) {
		tp_err("init read tp info error so exit\n");
		return ILITEK_FAIL;
	}

	if (ts->ic_type == 0x2326) {
		tp_msg("Firmware Upgrade on Slave\n");
		ret = program_slave_BL1_8();
	}

	ilitek_reset(ts->reset_time);

	return ret;
}

bool check_FW_upgrade(unsigned char *buffer)
{
	uint16_t dri_crc = 0, ic_crc = 0;
	int count = 0;

	if (ts->ptl.ver >= PROTOCOL_V6 || ts->ptl.ver == BL_V1_8) {
		int update = false;

		for (count = 0; count < ts->upg.blk_num; count++) {
			ts->blk[count].crc_match = true;
			ic_crc = api_get_block_crc(ts->blk[count].start, ts->blk[count].end, CRC_CALCULATION_FROM_IC);
			dri_crc = get_dri_crc(ts->blk[count].start, ts->blk[count].end, buffer);

			if (ic_crc != dri_crc) {
				ts->blk[count].crc_match = false;
				update = true;
			}
			tp_msg("Block[%d], Start/End Addr: 0x%x/0x%x, IC/Hex CRC: 0x%x/0x%x Matched:%d\n",
				count, ts->blk[count].start, ts->blk[count].end,
				ic_crc, dri_crc, ts->blk[count].crc_match);
		}
		return update;
	}
	return true;
}

int32_t _ilitek_upgrade_firmware(uint8_t *buffer)
{
	int32_t ret = 0, retry = 0;
	uint8_t buf[64] = { 0 };
	bool update;
	unsigned int i;

	tp_msg("ap_startaddr:0x%X, ap_endaddr:0x%X, ap_checksum:0x%X, ap_len: %d\n",
		ts->upg.ap_start_addr, ts->upg.ap_end_addr, ts->upg.ap_check, ts->upg.ap_len);
	tp_msg("df_startaddr:0x%X, df_endaddr:0x%X, df_checksum:0x%X, df_len: %d\n",
		ts->upg.df_start_addr, ts->upg.df_end_addr, ts->upg.df_check, ts->upg.df_len);

	tp_msg("------------Daemon Block information------------\n");
	for (i = 0; i < ts->upg.blk_num && i < ARRAY_SIZE(ts->blk); i++) {
		tp_msg("Block[%u]: start/end: 0x%x/0x%x\n",
			i, ts->blk[i].start, ts->blk[i].end);
	}

Retry:
	if (retry)
		ilitek_reset(ts->reset_time);

	update = true;
	if (retry++ > 2) {
		tp_err("upgrade retry 2 times failed\n");
		return ret;
	}

	if (api_set_testmode(true) < ILITEK_SUCCESS)
		goto Retry;

	if (api_ptl_set_cmd(GET_MCU_VER, NULL, buf) < ILITEK_SUCCESS)
		goto Retry;

	update = check_FW_upgrade(buffer);
	if (update) {
		ret = ilitek_changetoblmode(true);
		if (ret) {
			tp_err("change to bl mode err ret = %d\n", ret);
			goto Retry;
		}

		if (api_ptl_set_cmd(GET_PTL_VER, NULL, buf) < ILITEK_SUCCESS)
			goto Retry;

		tp_msg("bl protocol version %d.%d\n", buf[0], buf[1]);
		ts->page_number = 16;
		buf[0] = 0xc7;
		ret = ilitek_i2c_write_and_read(buf, 1, 10, buf, 1);
		tp_msg("0xc7 read= %x\n", buf[0]);
	}

	if (((ts->ptl.ver & 0xFFFF00) == BL_V1_8) || ts->ptl.ver >= PROTOCOL_V6) {
		ret = ilitek_upgrade_BL1_8(buffer);
	} else if((ts->ptl.ver & 0xFFFF00) == BL_V1_7) {
		ret = ilitek_upgrade_BL1_7(buffer);
	} else if((ts->ptl.ver & 0xFFFF00) == BL_V1_6) {
		ret = ilitek_upgrade_BL1_6(buffer);
	} else {
		tp_err("not support BL protocol ver: %x\n", ts->bl_ver);
		goto Retry;
	}

	if (ret < 0) {
		tp_err("FW upgrade failed, err: %d\n", ret);
		goto Retry;
	}

	tp_msg("upgrade firmware completed!\n");

	if (ilitek_changetoblmode(false)) {
		tp_err("change to ap mode failed\n");
		goto Retry;
	}

	ilitek_reset(ts->reset_time);

	ret = api_init_func();

	return 0;
}

int hex_mapping_convert(uint32_t addr, uint8_t *buffer)
{
	uint32_t start = 0, count = 0, index = 0;

	ts->upg.blk_num = 0;

	tp_msg("info addr: 0x%x\n", addr);

	start = addr + HEX_FWVERSION_ADDRESS;
	for (count = start, index = HEX_FWVERSION_SIZE - 1;
	     count < start + HEX_FWVERSION_SIZE; count++, --index)
		ts->upg.hex_fw_ver[index] = buffer[count];

	tp_msg_arr("Hex FW version:", ts->upg.hex_fw_ver, HEX_FWVERSION_SIZE);
	tp_msg_arr("Hex DF start addr:", buffer + addr + HEX_DATA_FLASH_ADDRESS,
		   HEX_DATA_FLASH_SIZE);

	tp_msg_arr("Hex IC type:", buffer + addr + HEX_KERNEL_VERSION_ADDRESS,
		   HEX_KERNEL_VERSION_SIZE);
	ts->upg.hex_ic_type = buffer[addr + HEX_KERNEL_VERSION_ADDRESS];
	ts->upg.hex_ic_type += (buffer[addr + HEX_KERNEL_VERSION_ADDRESS + 1] << 8);

	start = addr + HEX_MEMONY_MAPPING_VERSION_ADDRESS;
	for (count = start, index = 0;
	     count < start + HEX_MEMONY_MAPPING_VERSION_SIZE; count++, index++)
		ts->upg.map_ver += buffer[count] << (index*8);

	tp_msg("Hex Mapping Version: 0x%x\n", ts->upg.map_ver);

	if (ts->upg.map_ver < 0x10000)
		return ILITEK_SUCCESS;

	ts->upg.blk_num = buffer[addr + HEX_FLASH_BLOCK_NUMMER_ADDRESS];
	if (ts->upg.blk_num > ARRAY_SIZE(ts->blk)) {
		tp_err("Unexpected block num: %u\n", ts->upg.blk_num);
		ts->upg.blk_num = ARRAY_SIZE(ts->blk);
	}

	tp_msg("------------Hex Block information------------\n");
	tp_msg("Hex flash block number: %d\n", ts->upg.blk_num);
	memset(ts->blk, 0, sizeof(ts->blk));
	for (count = 0; count < ts->upg.blk_num; count++) {
		start = addr + HEX_FLASH_BLOCK_INFO_ADDRESS + HEX_FLASH_BLOCK_INFO_SIZE * count;
		ts->blk[count].start = buffer[start] + (buffer[start+1] << 8) + (buffer[start+2] << 16);
		if (count == ts->upg.blk_num - 1) {
			addr = addr + HEX_FLASH_BLOCK_END_ADDRESS;
			ts->blk[count].end = buffer[addr] + (buffer[addr+1] << 8) + (buffer[addr+2] << 16);
		} else {
			ts->blk[count].end = buffer[start+3] + (buffer[start+4] << 8) + (buffer[start+5] << 16) - 1;
		}
		tp_msg("Hex Block:%d, start:0x%x end:0x%x\n", count, ts->blk[count].start,  ts->blk[count].end);
	}

	return ILITEK_SUCCESS;
}

int hex_file_convert(struct ilitek_ts_data *ts, unsigned char *pbuf,
		     unsigned char *buffer, unsigned int hexfilesize)
{
	unsigned int exaddr = 0;
	unsigned int i = 0, j = 0, k = 0;
	unsigned int start_addr = 0xFFFF, hex_info_addr = 0;
	unsigned int count = 0;
	bool read_mapping = false;
	unsigned int len = 0, addr = 0, type = 0;

	for (i = 0; i < hexfilesize;) {
		int offset;

		len = hex_2_dec(&pbuf[i + 1], HEX_BYTE_CNT_LEN);
		addr = hex_2_dec(&pbuf[i + 3], HEX_ADDR_LEN);
		type = hex_2_dec(&pbuf[i + 7], HEX_RECORD_TYPE_LEN);

		if (type == HEX_TYPE_ELA)
			exaddr = hex_2_dec(&pbuf[i + HEX_DATA_POS_HEAD], (len * 2)) << 16;
		if (type == HEX_TYPE_ESA)
			exaddr = hex_2_dec(&pbuf[i + HEX_DATA_POS_HEAD], (len * 2)) << 4;
		addr = addr + exaddr;

		if (type == HEX_TYPE_ILI_MEM_MAP) {
			hex_info_addr = hex_2_dec(&pbuf[i + HEX_DATA_POS_HEAD], (len * 2));
			tp_msg("%s, hex_info_addr = 0x%x\n", __func__, hex_info_addr);
			ts->upg.hex_info_flag = true;
		}

		if (addr >= hex_info_addr + HEX_MEMONY_MAPPING_FLASH_SIZE &&
		    ts->upg.hex_info_flag && read_mapping == false) {
			read_mapping = true;
			hex_mapping_convert(hex_info_addr, buffer);
		}

		/* calculate checksum */
		for (j = HEX_DATA_POS_HEAD; j < (HEX_DATA_POS_HEAD + (len * 2)); j += 2) {
			if (type == HEX_TYPE_DATA) {
				if (addr + (j - HEX_DATA_POS_HEAD) / 2 < ts->upg.df_start_addr)
					ts->upg.ap_check += hex_2_dec(&pbuf[i + j], 2);
 				else
					ts->upg.df_check += hex_2_dec(&pbuf[i + j], 2);
			}
		}

		if (pbuf[i + j + 2 + 1] == 0x0A) // CR+LF (0x0D 0x0A)
			offset = 2;
		else	// CR  (0x0D)
			offset = 1;

		if (type == HEX_TYPE_DATA) {
			if (addr < start_addr)
				start_addr = addr;

			if (addr < ts->upg.ap_start_addr)
				ts->upg.ap_start_addr = addr;

			for (count = 0; count < ts->upg.blk_num; count++) {
				if (addr + len - 1 > ts->blk[count].start &&
				    count == ts->upg.blk_num - 1)
					ts->blk[count].end = addr + len - 1;
				else if (addr + len - 1 > ts->blk[count].start &&
				    addr + len - 1 < ts->blk[count + 1].start)
					ts->blk[count].end = addr + len - 1;

			}

			if ((addr + len) > ts->upg.ap_end_addr && (addr < ts->upg.df_start_addr))
				ts->upg.ap_end_addr = addr + len;

			if ((addr < ts->upg.ap_start_addr) && (addr >= ts->upg.df_start_addr))
				ts->upg.df_start_addr = addr;

			if ((addr + len) > ts->upg.df_end_addr && (addr >= ts->upg.df_start_addr))
				ts->upg.df_end_addr = addr + len;

			for (j = 0, k = 0; j < (len * 2); j += 2, k++)
				buffer[addr + k] = hex_2_dec(&pbuf[i + HEX_DATA_POS_HEAD + j], 2);
		}

		if (type == HEX_TYPE_ILI_SDA) {
			ts->upg.df_tag_exist = true;
			ts->upg.df_start_addr = hex_2_dec(&pbuf[i + HEX_DATA_POS_HEAD], len * 2);
			tp_msg("-----------Data Flash Start address:0x%x\n", ts->upg.df_start_addr);
		}
		i += HEX_DATA_POS_HEAD + (len * 2) + HEX_CHKSUM_LEN + offset;
	}

	return 0;
}

static int decode_hex(struct ilitek_ts_data *ts, uint8_t *fw_buf,
		      const char *filename)
{
	int error;
	const struct firmware *fw;
	struct device *dev = &ts->client->dev;

	tp_msg("\n");

	if ((error = request_firmware(&fw, filename, dev))) {
		tp_err("request fw: %s failed, err:%d\n", filename, error);
		return error;
	}

	hex_file_convert(ts, (unsigned char *)fw->data, fw_buf, fw->size);

	release_firmware(fw);

	return 0;
}

static int _decode_bin(struct ilitek_ts_data *ts, uint8_t *fw_buf,
		       unsigned int fw_size, uint32_t info_addr)
{
	uint8_t buf[64];
	unsigned int i;

	if (ts->ptl.ver >= PROTOCOL_V6 || ts->ptl.ver == BL_V1_8) {
		api_ptl_set_cmd(GET_MCU_VER, NULL, buf);

		/* Needs to check Kernel Version first */
		if (info_addr)
			hex_mapping_convert(info_addr, fw_buf);
		else if (ts->upg.ic_df_start_addr == 0x2C000)
			hex_mapping_convert(0x4020, fw_buf);
		else
			hex_mapping_convert(0x3020, fw_buf);

		for (i = 0; i < ts->upg.blk_num; i++) {
			if (!i)
				ts->blk[i].end = get_endaddr(
					ts->blk[i].start, ts->blk[i].end,
					fw_buf, fw_size, true);
            		else
                		ts->blk[i].end = get_endaddr(
                			ts->blk[i].start, ts->blk[i].end,
                			fw_buf, fw_size, false);
        	}
	} else {
		ilitek_changetoblmode(true);
		api_ptl_set_cmd(GET_PTL_VER, NULL, buf);
		api_ptl_set_cmd(GET_MCU_VER, NULL, buf);

		ts->upg.df_start_addr = ts->upg.ic_df_start_addr;

		if (ts->ptl.ver == BL_V1_7) {
			hex_mapping_convert(0x2020, fw_buf);
			ts->upg.ap_start_addr = 0x2000;
			ts->upg.ap_end_addr = get_endaddr(ts->upg.ap_start_addr,
							  ts->upg.df_start_addr,
							  fw_buf, fw_size,
							  true) + 1;
			ts->upg.ap_check = get_dri_checksum(ts->upg.ap_start_addr,
							    ts->upg.ap_end_addr,
							    fw_buf);
		} else if (ts->ptl.ver == BL_V1_6) {
			hex_mapping_convert(0x500, fw_buf);
			ts->upg.ap_start_addr = 0x0;
			ts->upg.ap_end_addr = get_endaddr(ts->upg.ap_start_addr,
							  ts->upg.df_start_addr,
							  fw_buf, fw_size,
							  true) + 3;
			ts->upg.ap_check = get_dri_checksum(ts->upg.ap_start_addr,
							    ts->upg.ap_end_addr,
							    fw_buf);
		} else {
			tp_err("unexpected BL version: 0x%04X\n", ts->ptl.ver);
			return -EINVAL;
		}

		ts->upg.df_end_addr = fw_size;
		ts->upg.df_check = get_dri_checksum(ts->upg.df_start_addr,
						    ts->upg.df_end_addr,
						    fw_buf);
	}

	ts->upg.hex_info_flag = true;

	return 0;
}

static int decode_bin(struct ilitek_ts_data *ts, uint8_t *fw_buf,
		      const char *filename)
{
	int error;
	const struct firmware *fw;
	struct device *dev = &ts->client->dev;
	unsigned int fw_size;

	tp_msg("\n");

	if ((error = request_firmware(&fw, filename, dev))) {
		tp_err("request fw: %s failed, err:%d\n", filename, error);
		return error;
	}

	tp_msg("fw size: %lu bytes\n", fw->size);
	memcpy(fw_buf, fw->data, fw->size);
	fw_size = fw->size;
	release_firmware(fw);

	return _decode_bin(ts, fw_buf, fw_size, 0);
}

static int __maybe_unused decode_ili(struct ilitek_ts_data *ts, uint8_t *fw_buf)
{
#ifdef ILITEK_UPDATE_FW
	int error;
	uint32_t info_addr, i;
	bool need_update = false;
	uint8_t hex_fw_ver[8];

	tp_msg("sizeof ili file: %zu bytes\n", sizeof(CTPM_FW));

	memcpy(hex_fw_ver, CTPM_FW + 18, 8);

	tp_msg_arr("IC  fw ver:", ts->fw_ver, 8);
	tp_msg_arr("Hex fw ver:", hex_fw_ver, 8);

	if (!(ts->force_update)) {
		for (i = 0; i < 8; i++) {
			if (hex_fw_ver[i] > ts->fw_ver[i]) {
				need_update = true;
				break;
			}

			if (hex_fw_ver[i] < ts->fw_ver[i])
				break;
		}

		if (!need_update) {
			tp_msg("Hex FW version is older so not upgrade\n");
			return -EEXIST;
		}
	}

	info_addr = 0;
	if (ts->ptl.ver_major == 0x6 || ts->ptl.ver == BL_V1_8) {
		info_addr = (CTPM_FW[10] << 8) + CTPM_FW[11];
		tp_msg("Hex info addr: 0x%x\n", info_addr);
	}

	tp_msg("Hex info addr: 0x%x\n", info_addr);

	memcpy(fw_buf, CTPM_FW + 32, sizeof(CTPM_FW) - 32);
	if ((error = _decode_bin(ts, fw_buf, sizeof(CTPM_FW) - 32, info_addr)))
		return error;

	if (ts->ptl.ver_major != 0x6 && ts->ptl.ver != BL_V1_8) {
		ts->upg.ap_start_addr = (CTPM_FW[0] << 16) + (CTPM_FW[1] << 8) + CTPM_FW[2];
		ts->upg.ap_end_addr = (CTPM_FW[3] << 16) + (CTPM_FW[4] << 8) + CTPM_FW[5];
		ts->upg.ap_check = (CTPM_FW[6] << 16) + (CTPM_FW[7] << 8) + CTPM_FW[8];
		ts->upg.df_end_addr = (CTPM_FW[12] << 16) + (CTPM_FW[13] << 8) + CTPM_FW[14];
		ts->upg.df_check = (CTPM_FW[15] << 16) + (CTPM_FW[16] << 8) + CTPM_FW[17];
		ts->upg.df_len = (CTPM_FW[26] << 16) + (CTPM_FW[27] << 8) + CTPM_FW[28];
		ts->upg.ap_len = (CTPM_FW[29] << 16) + (CTPM_FW[30] << 8) + CTPM_FW[31];
		ts->upg.hex_ic_type = CTPM_FW[10] + (CTPM_FW[11] << 8);
		tp_msg("ILI IC type: 0x%x\n", ts->upg.hex_ic_type);
	}
#endif

	return 0;
}

static int decode_firmware(struct ilitek_ts_data *ts, uint8_t *fw_buf)
{
	int error;

	ts->upg.ap_start_addr = 0xFFFF;
	ts->upg.ap_end_addr = 0x0;
	ts->upg.ap_check = 0x0;
	ts->upg.df_start_addr = 0xFFFF;
	ts->upg.df_end_addr = 0x0;
	ts->upg.df_check = 0x0;
	ts->upg.hex_info_flag = false;
	ts->upg.df_tag_exist = false;

	if ((error = decode_hex(ts, fw_buf, "ilitek.hex")) &&
	    (error = decode_bin(ts, fw_buf, "ilitek.bin"))) {

#ifdef ILITEK_UPDATE_FW
		if (!(error = decode_ili(ts, fw_buf)))
			return 0;
#endif

		return error;
	}

	return 0;
}

int ilitek_upgrade_firmware()
{
	int error;
	uint8_t *fw_buf;
	uint8_t outbuf[64];

	memset(outbuf, 0, sizeof(outbuf));

	fw_buf = (uint8_t *)vmalloc(ILITEK_HEX_UPGRADE_SIZE);
	if (!fw_buf) {
		tp_err("allocte fw_buf memory failed\n");
		return -ENOMEM;
	}

	ilitek_irq_disable();
	mutex_lock(&ts->ilitek_mutex);

	if ((error = api_ptl_set_cmd(GET_MCU_VER, NULL, outbuf)) < 0 ||
	    (error = api_ptl_set_cmd(GET_PTL_VER, NULL, outbuf)) < 0)
		goto unlock_and_irq_enable;

	memset(fw_buf, 0xFF, ILITEK_HEX_UPGRADE_SIZE);
	if (ts->ic_type == 0x2312 || ts->ic_type == 0x2315)
		memset(fw_buf + 0x1F000, 0, 0x1000);

	if (decode_firmware(ts, fw_buf)) {
		tp_err("decode firmware failed\n");
		error = -EFAULT;
		goto unlock_and_irq_enable;
	}

	tp_msg("MCU version: 0x%04x, Hex IC type: 0x%04x\n",
		ts->ic_type, ts->upg.hex_ic_type);
	if (ts->upg.hex_info_flag && ts->upg.hex_ic_type &&
	    ts->ic_type != ts->upg.hex_ic_type) {
		tp_err("ic: ILI%04x, hex: ILI%04x not match\n",
			ts->ic_type, ts->upg.hex_ic_type);

		error = -EINVAL;
		goto unlock_and_irq_enable;
	} else if ((!ts->upg.hex_info_flag || !ts->upg.hex_ic_type) &&
		   ts->mcu_ver[0] != 0x03 && ts->mcu_ver[0] != 0x09) {
		tp_err("ic: ILI%04x, hex: ILI230X not match\n", ts->ic_type);

		error = -EINVAL;
		goto unlock_and_irq_enable;
	}

	atomic_set(&ts->firmware_updating, 1);
	ts->operation_protection = true;
	error = _ilitek_upgrade_firmware(fw_buf);
	ts->operation_protection = false;
	atomic_set(&ts->firmware_updating, 0);
	if (error < 0) {
		tp_err("upgrade failed, err: %d\n", error);
		goto unlock_and_irq_enable;
	}

	ilitek_read_tp_info(false);

unlock_and_irq_enable:
	mutex_unlock(&ts->ilitek_mutex);
	ilitek_irq_enable();

	vfree(fw_buf);

	return error;
}
