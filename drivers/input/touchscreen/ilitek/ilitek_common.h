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
#ifndef _ILITEK_COMMON_H_
#define _ILITEK_COMMON_H_
/* Includes of headers ------------------------------------------------------*/
#include <linux/sched.h>
#include "ilitek_ts.h"
/* Extern define ------------------------------------------------------------*/
//driver information
#define DERVER_VERSION_0 				5
#define DERVER_VERSION_1 				9
#define DERVER_VERSION_2 				0
#define DERVER_VERSION_3				5
#define CUSTOMER_H_ID					0
#define CUSTOMER_L_ID					0
#define TEST_VERSION					0
//
#define ILITEK_TP_MODE_APPLICATION			0x5A
#define ILITEK_TP_MODE_BOOTLOADER			0x55

//error code
#define ILITEK_TP_UPGRADE_FAIL				-5
#define ILITEK_I2C_TRANSFER_ERR				-4
#define ILITEK_TP_CHANGETOBL_ERR			-3
#define ILITEK_TP_CHANGETOAP_ERR			-2
#define ILITEK_FAIL					-1
#define ILITEK_SUCCESS					0

#define ILITEK_ERR_LOG_LEVEL				1
#define ILITEK_INFO_LOG_LEVEL				3
#define ILITEK_DEBUG_LOG_LEVEL				4
#define ILITEK_DEFAULT_LOG_LEVEL			ILITEK_INFO_LOG_LEVEL
//#define ILITEK_DEFAULT_LOG_LEVEL			ILITEK_DEBUG_LOG_LEVEL

#define ILITEK_KEYINFO_V3_HEADER			4
#define ILITEK_KEYINFO_V6_HEADER			5
#define ILITEK_KEYINFO_FORMAT_LENGTH			5
#define ILITEK_KEYINFO_FIRST_PACKET			29
#define ILITEK_KEYINFO_OTHER_PACKET			25
#define ILITEK_KEYINFO_FORMAT_ID			0
#define ILITEK_KEYINFO_FORMAT_X_MSB			1
#define ILITEK_KEYINFO_FORMAT_X_LSB			2
#define ILITEK_KEYINFO_FORMAT_Y_MSB			3
#define ILITEK_KEYINFO_FORMAT_Y_LSB			4

//hex parse define
#define HEX_FWVERSION_ADDRESS				0x0C
#define HEX_FWVERSION_SIZE				8
#define HEX_DATA_FLASH_ADDRESS				0x22
#define HEX_DATA_FLASH_SIZE				3
#define HEX_KERNEL_VERSION_ADDRESS			0x6
#define HEX_KERNEL_VERSION_SIZE				6
#define HEX_MEMONY_MAPPING_VERSION_SIZE			3
#define HEX_MEMONY_MAPPING_VERSION_ADDRESS		0x0
#define HEX_FLASH_BLOCK_NUMMER_ADDRESS			80
#define HEX_FLASH_BLOCK_NUMMER_SIZE			1
#define HEX_FLASH_BLOCK_INFO_ADDRESS			84
#define HEX_FLASH_BLOCK_INFO_SIZE			3
#define HEX_FLASH_BLOCK_END_ADDRESS			123
#define HEX_MEMONY_MAPPING_FLASH_SIZE			128

#define HEX_TYPE_DATA			0x00
#define HEX_TYPE_EOF			0x01
#define HEX_TYPE_ESA			0x02
#define HEX_TYPE_SSA			0x03
#define HEX_TYPE_ELA			0x04
#define HEX_TYPE_SLA			0x05
#define HEX_TYPE_ILI_MEM_MAP		0xAC
#define HEX_TYPE_ILI_SDA		0xAD
#define HEX_START_CODE_LEN		1
#define HEX_BYTE_CNT_LEN		2
#define HEX_ADDR_LEN			4
#define HEX_RECORD_TYPE_LEN		2
#define HEX_DATA_POS_HEAD		9
#define HEX_CHKSUM_LEN			2

#define MEMONY_MAPPING_ADDRESS_V6			0x3020
#define LEGO_AP_START_ADDRESS				0x3000
#define ILI25XX_AP_START_ADDRESS			0x2000
#define ILI23XX_AP_START_ADDRESS			0x0
#define ILITEK_ILI_HEADER_LENGTH			0x20

#define ILITEK_IOCTL_MAX_TRANSFER			5000
#define ILITEK_HEX_UPGRADE_SIZE				(256 * 1024 + 32)
#define ILITEK_SUPPORT_MAX_POINT			40
#define REPORT_ADDRESS_COUNT				61
#define REPORT_ADDRESS_ALGO_MODE			62
#define REPORT_ADDRESS_CHECKSUM				63
#define REPORT_ADDRESS_PACKET_ID			0x0
#define REPORT_FORMAT0_PACKET_MAX_POINT			10
#define REPORT_FORMAT0_PACKET_LENGTH			5
#define REPORT_FORMAT1_PACKET_MAX_POINT			10
#define REPORT_FORMAT1_PACKET_LENGTH			6
#define REPORT_FORMAT2_PACKET_MAX_POINT			7
#define REPORT_FORMAT2_PACKET_LENGTH			8
#define REPORT_FORMAT3_PACKET_MAX_POINT			7
#define REPORT_FORMAT3_PACKET_LENGTH			8

#define PROTOCOL_V6					0x60000
#define BL_V1_8						0x10800
#define BL_V1_7						0x10700
#define BL_V1_6						0x10600
#define RESET_L						1
#define RESET_H						0

#define ENABLE_MODE					1
#define DISABLE_MODE					0

#define debug_level(level, fmt, arg...)	\
	do {				       \
		if (ts && level > ts->ilitek_log_level_value)		\
			break;						\
		if (level == ILITEK_ERR_LOG_LEVEL)			\
			printk("[ILITEK][ERR][%s:%d] "fmt, __func__, __LINE__, ##arg); \
		else if (level == ILITEK_INFO_LOG_LEVEL)		\
			printk("[ILITEK][MSG][%s:%d] "fmt, __func__, __LINE__, ##arg); \
		else if (level == ILITEK_DEBUG_LOG_LEVEL)		\
			printk("[ILITEK][DBG][%s:%d] "fmt, __func__, __LINE__, ##arg); \
	} while (0)

#define tp_err(fmt, arg...) debug_level(ILITEK_ERR_LOG_LEVEL, fmt, ##arg)
#define tp_msg(fmt, arg...) debug_level(ILITEK_INFO_LOG_LEVEL, fmt, ##arg)
#define tp_dbg(fmt, arg...) debug_level(ILITEK_DEBUG_LOG_LEVEL, fmt, ##arg)

#define PTL_V3	(0x01)
#define PTL_V6	(0x02)

#define ILITEK_SUPPORT_MAX_KEY_CNT (50)

#define set_arr(arr, idx, val)			\
	do {					\
		if (idx < ARRAY_SIZE(arr))	\
			arr[idx] = val;		\
	} while (0)

enum ilitek_cmds {
	/* common cmds */
	GET_PTL_VER = 0,
	GET_FW_VER,
	GET_SCRN_RES,
	GET_TP_RES,
	GET_MCU_MOD,
	GET_MCU_VER,
	GET_KEY_INFO,
	SET_IC_SLEEP,
	SET_IC_WAKE,
	SET_FUNC_MOD,
	GET_SYS_BUSY,
	SET_AP_MODE,
	SET_BL_MODE,

	/* v3 only cmds */
	SET_WRITE_EN,
	SET_TEST_MOD,

	/* v6 only cmds */
	SET_MOD_CTRL,
	SET_W_FLASH,
	GET_BLK_CRC,
	SET_CDC_INIT,

	/* ALWAYS keep at the end */
	MAX_CMD_CNT
};

/* Extern typedef -----------------------------------------------------------*/

/* should be removed once it fully tested */
struct __attribute__((__packed__)) touch_info_v6 {
	uint8_t id:6;
	uint8_t status:1;
	uint8_t reserve:1;
	uint16_t x;
	uint16_t y;
	uint8_t p;
	uint8_t w;
	uint8_t h;
};

struct ilitek_protocol_info {
	uint32_t ver;
	unsigned char ver_major;
	unsigned char ver_mid;
	unsigned char ver_minor;
	uint8_t flag;
};

struct ilitek_touch_info {
	uint16_t id;
	uint16_t x;
	uint16_t y;
	uint16_t p;
	uint16_t w;
	uint16_t h;
	uint16_t status;
};

struct ilitek_key_info {
	int32_t id;
	int32_t x;
	int32_t y;
	int32_t status;
	int32_t flag;
};

struct ilitek_block_info {
	uint32_t start;
	uint32_t end;
	uint16_t ic_crc;
	uint16_t dri_crc;
	bool crc_match;
};

struct ilitek_panel_info {
	int max_x;
	int min_x;
	int max_y;
	int min_y;
	int blk_num;
	int slave_num;
};

struct ilitek_node_info {
	int data;
	bool data_st;	//data status
	int max;
	bool max_st;	//max status
	int min;
	bool min_st;	//min status
	uint8_t type;
};

struct ilitek_uniformity_info {
	uint16_t max_thr;
	uint16_t min_thr;
	uint16_t up_fail;
	uint16_t low_fail;
	uint16_t win1_fail;
	uint16_t win1_thr;
	uint16_t win2_fail;
	uint16_t win2_thr;
	bool bench;
	bool bench_status;
	bool allnode_raw;
	char *allnode_raw_section;
	bool allnode_win1;
	char *allnode_win1_section;
	bool allnode_win2;
	char *allnode_win2_section;
	bool allnode_status;
	bool allnode_raw_status;
	bool allnode_win1_status;
	bool allnode_win2_status;
	struct ilitek_node_info *ben;
	struct ilitek_node_info *raw;
	struct ilitek_node_info *win1;
	struct ilitek_node_info *win2;
};

struct ilitek_ic_info {
	uint8_t mode;
	uint16_t crc;
};

struct ilitek_upgrade_info {
	uint8_t filename[256];
	uint32_t hexfilesize;

	uint32_t hex_ic_type;

	uint32_t exaddr;
	uint32_t ap_start_addr;
	uint32_t ap_end_addr;
	uint32_t ap_check;
	uint32_t ap_len;

	uint32_t ic_df_start_addr;

	uint32_t df_start_addr;
	uint32_t df_end_addr;
	uint32_t df_check;
	uint32_t df_len;

	uint32_t total_check;
	uint8_t hex_fw_ver[HEX_FWVERSION_SIZE];
	bool hex_info_flag;
	bool df_tag_exist;
	uint32_t map_ver;
	uint32_t blk_num;
};

struct ilitek_ts_data {
	int format;
	int (*process_and_report)(void);
	int ilitek_log_level_value;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct regulator *vdd;
	struct regulator *vdd_i2c;
	struct regulator *vcc_io;
	int irq_gpio;
	int reset_gpio;
	int test_gpio;
	bool system_suspend;
	uint8_t fw_ver[8];
	uint8_t mcu_ver[8];
	uint32_t ic_type;
	char product_id[30];
	unsigned int bl_ver;
	int protocol_ver;
	int tp_max_x;
	int tp_max_y;
	int tp_min_x;
	int tp_min_y;
	int screen_max_x;
	int screen_max_y;
	int screen_min_x;
	int screen_min_y;
	int max_tp;
	int max_btn;
	int x_ch;
	int y_ch;
	int keycount;
	int key_xlen;
	int key_ylen;
	struct ilitek_key_info keyinfo[ILITEK_SUPPORT_MAX_KEY_CNT];

	uint8_t irq_tri_type;

	bool is_touched;
	bool touch_key_hold_press;
	int touch_flag[ILITEK_SUPPORT_MAX_POINT];
	int ic_num;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif

	bool force_update;
	bool has_df;
	int page_number;
	struct task_struct *update_thread;

	atomic_t firmware_updating;
	bool operation_protection;
	bool unhandle_irq;

	uint8_t gesture_status;

#ifdef ILITEK_ESD_PROTECTION
	struct workqueue_struct *esd_wq;
	struct delayed_work esd_work;
	bool esd_check;
	unsigned long esd_delay;
#endif
	struct kobject *ilitek_func_kobj;
	struct mutex ilitek_mutex;
	bool ilitek_repeat_start;

	uint32_t irq_trigger_count;
	uint32_t irq_handle_count;
	struct ilitek_protocol_info ptl;
	struct ilitek_block_info blk[10];
	struct ilitek_ic_info ic[10];
	struct ilitek_upgrade_info upg;
	int reset_time;
	struct ilitek_touch_info tp[ILITEK_SUPPORT_MAX_POINT];

	atomic_t irq_enabled;
	atomic_t get_INT;

	bool wake_irq_enabled;

	bool irq_registerred;
};
/* Extern macro -------------------------------------------------------------*/
#define CEIL(n, d) ((n % d) ? (n / d) + 1 : (n / d ))
/* Extern variables ---------------------------------------------------------*/
extern uint32_t irq_handle_count;
extern char ilitek_driver_information[];
extern struct ilitek_ts_data *ts;
extern uint32_t set_update_len;
extern bool stanley_test;
#ifdef ILITEK_TUNING_MESSAGE
extern bool ilitek_debug_flag;
#endif
/* Extern function prototypes -----------------------------------------------*/
/* Extern functions ---------------------------------------------------------*/
void ilitek_resume(void);
void ilitek_suspend(void);
int ilitek_main_probe(struct i2c_client *client, const struct i2c_device_id *id);
int ilitek_main_remove(struct i2c_client *client);
void ilitek_reset(int delay);

int ilitek_i2c_write(uint8_t *cmd, int length);
int ilitek_i2c_read(uint8_t *data, int length);
int ilitek_i2c_write_and_read(uint8_t *cmd, int write_len,
				     int delay, uint8_t *data, int read_len);

void ilitek_irq_enable(void);
void ilitek_irq_disable(void);
int ilitek_read_tp_info(bool);

int ilitek_upgrade_firmware(void);
int _ilitek_upgrade_firmware(unsigned char *buffer);
int32_t ilitek_check_busy(int32_t count, int32_t delay, int32_t type);
#ifdef ILITEK_TOOL
int ilitek_create_tool_node(void);
int ilitek_remove_tool_node(void);
#endif
int hex_mapping_convert(uint32_t addr,uint8_t *buffer);
int ilitek_create_sysfsnode(void);
void ilitek_remove_sys_node(void);
uint32_t get_endaddr(uint32_t startAddr, uint32_t endAddr, const uint8_t buf[], unsigned int buf_size, bool is_AP);

bool checksum(unsigned char *buf, int len, unsigned char fw_checksum);

void ilitek_gpio_dbg(void);
int api_set_idlemode(int mode);

void tp_msg_arr(const char *tag, const uint8_t *buf, int size);

void ilitek_register_gesture(struct ilitek_ts_data *ts, bool init);

#endif
