/* include/linux/cm32183.h
 *
 * Copyright (C) 2018 Vishay Capella Microsystems Ltd.
 * Author: Frank Hsieh <pengyueh@gmail.com>
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

#ifndef __LINUX_CM32183_H
#define __LINUX_CM32183_H

#define CM32183_I2C_NAME		 "cm32183"

#define	CM32183_ADDR			        0x29 //7bit address

/* cm32183 command code */
#define	CM32183_CONF              0x00
#define CM32183_ALS_DATA          0x04
#define CM32183_W_DATA            0x05

/* cm32183 CONF command code */
#define CM32183_CONF_SD         0x0001
#define CM32183_CONF_SD_MASK    0xFFFE

#define CM32183_CONF_IT_100MS  (0 << 6)
#define CM32183_CONF_IT_200MS  (1 << 6)
#define CM32183_CONF_IT_400MS  (2 << 6)
#define CM32183_CONF_IT_800MS  (3 << 6)

#define CM32183_CONF_CH_EN     (1 << 2)


#define LS_PWR_ON					     (1 << 0)

struct cm32183_platform_data {
	int (*power)(int, uint8_t); /* power to the chip */
	uint16_t RGB_slave_address;
  uint16_t ls_cmd;
};

#endif
