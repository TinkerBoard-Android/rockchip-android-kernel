#ifndef _ETH_MAC_TINKER_H_
#define _ETH_MAC_TINKER_H_
/*
 *  eth_mac/eth_mac.h
 *
 *  Copyright (C) 2001 Russell King.
 *
 *  This file is placed under the LGPL.
 *
 */
extern int at24_read_eeprom(char *buf, unsigned int off, size_t count);
void eth_mac_eeprom(u8 *eth_mac, int ethx);
#endif /* _ETH_MAC_TINKER_H_ */
