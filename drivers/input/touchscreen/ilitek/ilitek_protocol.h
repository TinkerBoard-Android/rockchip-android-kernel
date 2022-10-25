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
#ifndef __ILITEK_PROTOCOL_H__
#define __ILITEK_PROTOCOL_H__

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define CMD_GET_TOUCH_INFO			0x10
#define CMD_GET_TP_RES				0x20
#define CMD_GET_SCRN_RES			0x21
#define CMD_GET_KEY_INFO			0x22
#define CMD_SET_IC_SLEEP			0x30
#define CMD_SET_IC_WAKE				0x31
#define CMD_SET_IDLE				0x34
#define CMD_GET_FW_VER				0x40
#define CMD_GET_PTL_VER				0x42
#define CMD_GET_MCU_VER				0x61
#define CMD_SET_FUNC_MOD			0x68

#define CMD_GET_SYS_BUSY			0x80
#define CMD_GET_MCU_MOD				0xC0
#define CMD_SET_AP_MODE				0xC1
#define CMD_SET_BL_MODE				0xC2
#define CMD_WRITE_DATA				0xC3
#define CMD_WRITE_ENABLE			0xC4
#define CMD_GET_AP_CRC				0xC7
#define CMD_SET_DATA_LEN			0xC9	/* v6 only */
#define CMD_ACCESS_SLAVE			0xCB	/* v6 only */
#define CMD_SET_W_FLASH				0xCC	/* v6 only */
#define CMD_GET_BLK_CRC				0xCD	/* v6 only */
#define CMD_SET_MOD_CTRL			0xF0	/* v6 only */
#define CMD_SET_CDC_INIT			0xF1	/* v6 only */
#define CMD_GET_CDC_DATA			0xF2	/* v6 only */
#define CMD_SET_TEST_MOD			0xF2	/* v6 only */

//Access Slave
#define PROTOCOL_V6_ACCESS_SLAVE_SET_APMODE		0xC1
#define PROTOCOL_V6_ACCESS_SLAVE_SET_BLMODE		0xC2
#define PROTOCOL_V6_ACCESS_SLAVE_PROGRAM		0xC3

//TP system status
#define ILITEK_TP_SYSTEM_READY				0x50
#define ILITEK_TP_NO_NEED				0
#define ILITEK_TP_SYSTEM_BUSY				0x1
#define ILITEK_TP_INITIAL_BUSY				(0x1 << 1)
#define ILITEK_TP_SET_MODE_0				0x0
#define ILITEK_TP_SET_MODE_1				0x1
#define ILITEK_TP_SET_MODE_2				0x2

//define Mode Control value
#define ENTER_NORMAL_MODE				0
#define ENTER_TEST_MODE					1
#define ENTER_DEBUG_MODE				2
#define ENTER_SUSPEND_MODE				3  //for 2326 Suspend Scan

#define CRC_CALCULATION_FROM_IC				0
#define CRC_GET_FROM_FLASH				1


/* Extern variables ----------------------------------------------------------*/
/* Extern functions ---------------------------------------------------------*/
extern int ilitek_read_data_and_report_3XX(void);
extern int ilitek_read_data_and_report_6XX(void);
extern int32_t ilitek_boot_upgrade_3XX(void);
extern int32_t ilitek_boot_upgrade_6XX(void);

extern int api_ptl_set_cmd(uint16_t idx, uint8_t *inbuf, uint8_t *outbuf);
extern int api_get_funcmode(int mode);
extern int api_set_funcmode(int mode);
extern int api_set_testmode(bool testmode);
extern int api_init_func(void);
extern uint16_t api_get_block_crc(uint32_t start, uint32_t end, uint32_t type);
extern int api_write_flash_enable_BL1_8(uint32_t start,uint32_t end);
extern int api_set_data_length(uint32_t data_len);
extern int api_get_slave_mode(int number);
extern int api_set_access_slave(uint8_t type);
extern uint32_t api_get_ap_crc(int number);

#endif