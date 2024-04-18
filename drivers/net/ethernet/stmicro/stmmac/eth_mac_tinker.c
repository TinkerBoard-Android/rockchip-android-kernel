/*
 * Copyright (C) 2010 ROCKCHIP, Inc.
 * Author: roger_chen <cz@rock-chips.com>
 *
 * This program is the virtual flash device
 * used to store bd_addr or MAC
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include "eth_mac_tinker.h"

#define VERSION "0.2"
#define WLAN_MAC_FILE "/data/misc/wifi/wlan_mac"

void eth_mac_eeprom(u8 *eth_mac, int ethx)
{
	int i;
	memset(eth_mac, 0, 6);
	if (ethx == 1)
		at24_read_eeprom(eth_mac, 6, 6);
	else
		at24_read_eeprom(eth_mac, 0, 6);

	printk(KERN_CONT "GMAC: Read the Ethernet MAC address from EEPROM(%d): ", ethx);
	for(i=0; i<5; i++)
		printk(KERN_CONT "%2.2x:", eth_mac[i]);
	printk(KERN_CONT "%2.2x", eth_mac[i]);
}
