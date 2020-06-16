/*
 * Copyright (c) 2012  Bjørn Mork <bjorn@mork.no>
 *
 * The probing code is heavily inspired by cdc_ether, which is:
 * Copyright (C) 2003-2005 by David Brownell
 * Copyright (C) 2006 by Ole Andre Vadla Ravnas (ActiveSync)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/ethtool.h>
#include <linux/etherdevice.h>
#include <linux/time.h>
#include <linux/timekeeping.h>
#include <net/arp.h>
#include <net/ip.h>
#include <net/ipv6.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/usb/cdc.h>
#include <linux/usb/usbnet.h>
#include <linux/usb/cdc-wdm.h>

#ifndef ETH_P_MAP
#define ETH_P_MAP 0xDA1A
#endif

#if (ETH_P_MAP == 0x00F9)
#undef ETH_P_MAP
#define ETH_P_MAP 0xDA1A
#endif

#ifndef ARPHRD_RAWIP
#define ARPHRD_RAWIP ARPHRD_NONE
#endif

#ifdef CONFIG_ARCH_IPQ807x
#define CONFIG_QCA_NSS_DRV
#endif

#if 1//def CONFIG_QCA_NSS_DRV
#define _RMNET_NSS_H_
#define _RMENT_NSS_H_
struct rmnet_nss_cb {
        int (*nss_create)(struct net_device *dev);
        int (*nss_free)(struct net_device *dev);
        int (*nss_tx)(struct sk_buff *skb);
};
static struct rmnet_nss_cb *rmnet_nss_callbacks __rcu __read_mostly;
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION( 4,0,0 )) //1e9e39f4a29857a396ac7b669d109f697f66695e
#define usbnet_set_skb_tx_stats(skb, packets, bytes_delta) do { dev->net->stats.tx_packets += packets; } while(0)
#endif

/* This driver supports wwan (3G/LTE/?) devices using a vendor
 * specific management protocol called Qualcomm MSM Interface (QMI) -
 * in addition to the more common AT commands over serial interface
 * management
 *
 * QMI is wrapped in CDC, using CDC encapsulated commands on the
 * control ("master") interface of a two-interface CDC Union
 * resembling standard CDC ECM.  The devices do not use the control
 * interface for any other CDC messages.  Most likely because the
 * management protocol is used in place of the standard CDC
 * notifications NOTIFY_NETWORK_CONNECTION and NOTIFY_SPEED_CHANGE
 *
 * Alternatively, control and data functions can be combined in a
 * single USB interface.
 *
 * Handling a protocol like QMI is out of the scope for any driver.
 * It is exported as a character device using the cdc-wdm driver as
 * a subdriver, enabling userspace applications ("modem managers") to
 * handle it.
 *
 * These devices may alternatively/additionally be configured using AT
 * commands on a serial interface
 */
#define VERSION_NUMBER "V1.2.0.5"
#define QUECTEL_WWAN_VERSION "Quectel_Linux&Android_QMI_WWAN_Driver_"VERSION_NUMBER
static const char driver_name[] = "qmi_wwan_q";

/* driver specific data */
struct qmi_wwan_state {
	struct usb_driver *subdriver;
	atomic_t pmcount;
	unsigned long unused;
	struct usb_interface *control;
	struct usb_interface *data;
};

/* default ethernet address used by the modem */
static const u8 default_modem_addr[ETH_ALEN] = {0x02, 0x50, 0xf3};

#if 1 //Added by Quectel
/*
    Quectel_WCDMA&LTE_Linux_USB_Driver_User_Guide_V1.9.pdf
    5.6.	Test QMAP on GobiNet or QMI WWAN
    0 - no QMAP
    1 - QMAP (Aggregation protocol)
    X - QMAP (Multiplexing and Aggregation protocol)
*/
#define QUECTEL_WWAN_QMAP 4

#if defined(QUECTEL_WWAN_QMAP)
#define QUECTEL_QMAP_MUX_ID 0x81

static uint __read_mostly qmap_mode = 0;
module_param( qmap_mode, uint, S_IRUGO);
module_param_named( rx_qmap, qmap_mode, uint, S_IRUGO );
#endif

#if defined(CONFIG_BRIDGE) || defined(CONFIG_BRIDGE_MODULE)
//#define QUECTEL_BRIDGE_MODE
#endif

#ifdef QUECTEL_BRIDGE_MODE
static uint __read_mostly bridge_mode = BIT(0)/*|BIT(1)*/;
module_param( bridge_mode, uint, S_IRUGO );
#endif

#if defined(QUECTEL_WWAN_QMAP)
#define QUECTEL_UL_DATA_AGG 1

#if defined(QUECTEL_UL_DATA_AGG)
static long agg_time_limit __read_mostly = 10; //ms
module_param(agg_time_limit, long, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(agg_time_limit, "Maximum time packets sit in the agg buf");

struct tx_agg_ctx {
	struct hrtimer tx_timer;
	struct tasklet_struct tx_bh;
	uint tx_pending_count;
	uint tx_pending_size;
	uint tx_sending_pkts;
	uint tx_timer_active;
	uint tx_timer_pkts;
	/* QMIWDS_ADMIN_SET_DATA_FORMAT_RESP TLV_0x17 and TLV_0x18 */
	uint ul_data_aggregation_max_datagrams; //UplinkDataAggregationMaxDatagramsTlv
	uint ul_data_aggregation_max_size; //UplinkDataAggregationMaxSizeTlv
	uint dl_minimum_padding; //0x1A
	struct sk_buff *tx_pending_skb[16];

	uint tx_agg_match[8];
	uint qmap_rx_pkts[8];
	uint qmap_tx_pkts[8];
};
#endif

typedef struct {
    unsigned int size;
    unsigned int rx_urb_size;
    unsigned int ep_type;
    unsigned int iface_id;
    unsigned int qmap_mode;
    unsigned int qmap_version;
    unsigned int dl_minimum_padding;
    char ifname[8][16];
    unsigned char mux_id[8];
} RMNET_INFO;

typedef struct sQmiWwanQmap
{
	struct usbnet *mpNetDev;
	struct driver_info driver_info;
	atomic_t refcount;
	struct net_device *mpQmapNetDev[QUECTEL_WWAN_QMAP];
	uint link_state;
	uint qmap_mode;
	uint qmap_size;
	uint qmap_version;
	struct sk_buff_head skb_chain;

#if defined(QUECTEL_UL_DATA_AGG)
	struct tx_agg_ctx tx_ctx;
#endif

#ifdef QUECTEL_BRIDGE_MODE
	uint bridge_mode;
	uint bridge_ipv4;
	unsigned char bridge_mac[ETH_ALEN];
#endif
	uint use_rmnet_usb;
	RMNET_INFO rmnet_info;
} sQmiWwanQmap;

struct qmap_priv {
	struct usbnet *dev;
	struct net_device *real_dev;
	struct net_device *self_dev;
	u8 offset_id;
	u8 mux_id;
	u8 qmap_version; // 5~v1, 9~v5
	
	spinlock_t agg_lock;
	struct sk_buff *agg_skb;
	unsigned agg_count;
	struct timespec64 agg_time;
	struct hrtimer agg_hrtimer;
	struct work_struct agg_wq;
	
#ifdef QUECTEL_BRIDGE_MODE
	uint bridge_mode;
	uint bridge_ipv4;
	unsigned char bridge_mac[ETH_ALEN];
#endif
};

struct qmap_hdr {
    u8 cd_rsvd_pad;
    u8 mux_id;
    u16 pkt_len;
} __packed;

enum rmnet_map_v5_header_type {
	RMNET_MAP_HEADER_TYPE_UNKNOWN,
	RMNET_MAP_HEADER_TYPE_COALESCING = 0x1,
	RMNET_MAP_HEADER_TYPE_CSUM_OFFLOAD = 0x2,
	RMNET_MAP_HEADER_TYPE_ENUM_LENGTH
};

/* Main QMAP header */
struct rmnet_map_header {
	u8  pad_len:6;
	u8  next_hdr:1;
	u8  cd_bit:1;
	u8  mux_id;
	__be16 pkt_len;
}  __aligned(1);

/* QMAP v5 headers */
struct rmnet_map_v5_csum_header {
	u8  next_hdr:1;
	u8  header_type:7;
	u8  hw_reserved:7;
	u8  csum_valid_required:1;
	__be16 reserved;
} __aligned(1);

#ifdef QUECTEL_BRIDGE_MODE
static int is_qmap_netdev(const struct net_device *netdev);
#endif
#endif

#ifdef QUECTEL_BRIDGE_MODE
static int bridge_arp_reply(struct net_device *net, struct sk_buff *skb, uint bridge_ipv4) {
    struct arphdr *parp;
    u8 *arpptr, *sha;
    u8  sip[4], tip[4], ipv4[4];
    struct sk_buff *reply = NULL;

    ipv4[0]  = (bridge_ipv4 >> 24) & 0xFF;
    ipv4[1]  = (bridge_ipv4 >> 16) & 0xFF;
    ipv4[2]  = (bridge_ipv4 >> 8) & 0xFF;
    ipv4[3]  = (bridge_ipv4 >> 0) & 0xFF;

    parp = arp_hdr(skb);

    if (parp->ar_hrd == htons(ARPHRD_ETHER)  && parp->ar_pro == htons(ETH_P_IP)
        && parp->ar_op == htons(ARPOP_REQUEST) && parp->ar_hln == 6 && parp->ar_pln == 4) {
        arpptr = (u8 *)parp + sizeof(struct arphdr);
        sha = arpptr;
        arpptr += net->addr_len;	/* sha */
        memcpy(sip, arpptr, sizeof(sip));
        arpptr += sizeof(sip);
        arpptr += net->addr_len;	/* tha */
        memcpy(tip, arpptr, sizeof(tip));

        pr_info("%s sip = %d.%d.%d.%d, tip=%d.%d.%d.%d, ipv4=%d.%d.%d.%d\n", netdev_name(net),
            sip[0], sip[1], sip[2], sip[3], tip[0], tip[1], tip[2], tip[3], ipv4[0], ipv4[1], ipv4[2], ipv4[3]);
	//wwan0 sip = 10.151.137.255, tip=10.151.138.0, ipv4=10.151.137.255
        if (tip[0] == ipv4[0] && tip[1] == ipv4[1] && (tip[2]&0xFC) == (ipv4[2]&0xFC) && tip[3] != ipv4[3])
            reply = arp_create(ARPOP_REPLY, ETH_P_ARP, *((__be32 *)sip), net, *((__be32 *)tip), sha, default_modem_addr, sha);

        if (reply) {
            skb_reset_mac_header(reply);
            __skb_pull(reply, skb_network_offset(reply));
            reply->ip_summed = CHECKSUM_UNNECESSARY;
            reply->pkt_type = PACKET_HOST;

            netif_rx_ni(reply);
        }
        return 1;
    }

    return 0;
}

static struct sk_buff *bridge_mode_tx_fixup(struct net_device *net, struct sk_buff *skb, uint bridge_ipv4, unsigned char *bridge_mac) {
	struct ethhdr *ehdr;
	const struct iphdr *iph;

	skb_reset_mac_header(skb);
	ehdr = eth_hdr(skb);

	if (ehdr->h_proto == htons(ETH_P_ARP)) {
		if (bridge_ipv4)
			bridge_arp_reply(net, skb, bridge_ipv4);
		return NULL;
	}

	iph = ip_hdr(skb);
	//DBG("iphdr: ");
	//PrintHex((void *)iph, sizeof(struct iphdr));

// 1	0.000000000	0.0.0.0	255.255.255.255	DHCP	362	DHCP Request  - Transaction ID 0xe7643ad7
	if (ehdr->h_proto == htons(ETH_P_IP) && iph->protocol == IPPROTO_UDP && iph->saddr == 0x00000000 && iph->daddr == 0xFFFFFFFF) {
		//if (udp_hdr(skb)->dest == htons(67)) //DHCP Request
		{
			memcpy(bridge_mac, ehdr->h_source, ETH_ALEN);
			pr_info("%s PC Mac Address: %02x:%02x:%02x:%02x:%02x:%02x\n", netdev_name(net),
				bridge_mac[0], bridge_mac[1], bridge_mac[2], bridge_mac[3], bridge_mac[4], bridge_mac[5]);
		}
	}

	if (memcmp(ehdr->h_source, bridge_mac, ETH_ALEN)) {
		return NULL;
	}

	return skb;
}

static void bridge_mode_rx_fixup(sQmiWwanQmap *pQmapDev, struct net_device *net, struct sk_buff *skb) {
	uint bridge_mode = 0;
	unsigned char *bridge_mac;

	if (pQmapDev->qmap_mode > 1) {
		struct qmap_priv *priv = netdev_priv(net);
		bridge_mode = priv->bridge_mode;
		bridge_mac = priv->bridge_mac;
	}
	else {
		bridge_mode = pQmapDev->bridge_mode;
		bridge_mac = pQmapDev->bridge_mac;
	}

	if (bridge_mode)
		memcpy(eth_hdr(skb)->h_dest, bridge_mac, ETH_ALEN);
	else
		memcpy(eth_hdr(skb)->h_dest, net->dev_addr, ETH_ALEN);
}
#endif

#if defined(QUECTEL_WWAN_QMAP)
static ssize_t qmap_mode_show(struct device *dev, struct device_attribute *attr, char *buf) {
	struct net_device *netdev = to_net_dev(dev);
	struct usbnet * usbnetdev = netdev_priv( netdev );
	struct qmi_wwan_state *info = (void *)&usbnetdev->data;
	sQmiWwanQmap *pQmapDev = (sQmiWwanQmap *)info->unused;

	return snprintf(buf, PAGE_SIZE, "%d\n",  pQmapDev->qmap_mode);
}

static DEVICE_ATTR(qmap_mode, S_IRUGO, qmap_mode_show, NULL);

static ssize_t qmap_size_show(struct device *dev, struct device_attribute *attr, char *buf) {
	struct net_device *netdev = to_net_dev(dev);
	struct usbnet * usbnetdev = netdev_priv( netdev );
	struct qmi_wwan_state *info = (void *)&usbnetdev->data;
	sQmiWwanQmap *pQmapDev = (sQmiWwanQmap *)info->unused;

	return snprintf(buf, PAGE_SIZE, "%u\n",  pQmapDev->qmap_size);
}

static DEVICE_ATTR(qmap_size, S_IRUGO, qmap_size_show, NULL);

#if defined(QUECTEL_UL_DATA_AGG)
static ssize_t qmap_debug_show(struct device *dev, struct device_attribute *attr, char *buf) {
	struct net_device *netdev = to_net_dev(dev);
	struct usbnet * usbnetdev = netdev_priv( netdev );
	struct qmi_wwan_state *info = (void *)&usbnetdev->data;
	sQmiWwanQmap *pQmapDev = (sQmiWwanQmap *)info->unused;
	struct tx_agg_ctx *ctx = &pQmapDev->tx_ctx;
	uint *rx = ctx->qmap_rx_pkts;
	uint *tx = ctx->qmap_tx_pkts;
	uint *mx = ctx->tx_agg_match;

	return snprintf(buf, PAGE_SIZE, "rx_pkts: %d, %d, %d, %d, %d, %d, %d, %d\ntx_pkts: %d, %d, %d, %d, %d, %d, %d, %d\nagg_match: %d, %d, %d, %d, %d, %d, %d, %d\n",
		rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7],
		tx[0], tx[1], tx[2], tx[3], tx[4], tx[5], tx[6], tx[7],
		mx[0], mx[1], mx[2], mx[3], mx[4], mx[5], mx[6], mx[7]);
}

static DEVICE_ATTR(qmap_debug, S_IRUGO, qmap_debug_show, NULL);
#endif

static ssize_t link_state_show(struct device *dev, struct device_attribute *attr, char *buf) {
	struct net_device *netdev = to_net_dev(dev);
	struct usbnet * usbnetdev = netdev_priv( netdev );
	struct qmi_wwan_state *info = (void *)&usbnetdev->data;
	sQmiWwanQmap *pQmapDev = (sQmiWwanQmap *)info->unused;

	return snprintf(buf, PAGE_SIZE, "0x%x\n",  pQmapDev->link_state);
}

static ssize_t link_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	struct net_device *netdev = to_net_dev(dev);
	struct usbnet * usbnetdev = netdev_priv( netdev );
	struct qmi_wwan_state *info = (void *)&usbnetdev->data;
	sQmiWwanQmap *pQmapDev = (sQmiWwanQmap *)info->unused;
	unsigned link_state = 0;
	unsigned old_link = pQmapDev->link_state;
	uint offset_id = 0;

	link_state = simple_strtoul(buf, NULL, 0);
	if (pQmapDev->qmap_mode == 1)
		pQmapDev->link_state = !!link_state;
	else if (pQmapDev->qmap_mode > 1) {
		offset_id = ((link_state&0xF) - 1);

		if (0 < link_state && link_state <= pQmapDev->qmap_mode)
			pQmapDev->link_state |= (1 << offset_id);
		else if (0x80 < link_state && link_state <= (0x80 + pQmapDev->qmap_mode))
			pQmapDev->link_state &= ~(1 << offset_id);
	}

	if (old_link != pQmapDev->link_state) {
		struct net_device *qmap_net = pQmapDev->mpQmapNetDev[offset_id];

		if (pQmapDev->link_state) {
			netif_carrier_on(usbnetdev->net);
		} else {
			netif_carrier_off(usbnetdev->net);
		}

		if (qmap_net) {
			if (pQmapDev->link_state && (1 << offset_id))
				netif_carrier_on(qmap_net);
			else
				netif_carrier_off(qmap_net);
		}

		dev_info(dev, "link_state 0x%x -> 0x%x\n", old_link, pQmapDev->link_state);
	}

	return count;
}

#ifdef QUECTEL_BRIDGE_MODE
static ssize_t bridge_mode_show(struct device *dev, struct device_attribute *attr, char *buf) {
	struct net_device *netdev = to_net_dev(dev);
	uint bridge_mode = 0;

	if (is_qmap_netdev(netdev)) {
		struct qmap_priv *priv = netdev_priv(netdev);
		bridge_mode = priv->bridge_mode;
	}
	else {
		struct usbnet * usbnetdev = netdev_priv( netdev );
		struct qmi_wwan_state *info = (void *)&usbnetdev->data;
		sQmiWwanQmap *pQmapDev = (sQmiWwanQmap *)info->unused;
		bridge_mode = pQmapDev->bridge_mode;
	}

	return snprintf(buf, PAGE_SIZE, "%u\n", bridge_mode);
}

static ssize_t bridge_ipv4_show(struct device *dev, struct device_attribute *attr, char *buf) {
	struct net_device *netdev = to_net_dev(dev);
	unsigned int bridge_ipv4 = 0;
	unsigned char ipv4[4];

	if (is_qmap_netdev(netdev)) {
		struct qmap_priv *priv = netdev_priv(netdev);
		bridge_ipv4 = priv->bridge_ipv4;
	}
	else {
		struct usbnet * usbnetdev = netdev_priv( netdev );
		struct qmi_wwan_state *info = (void *)&usbnetdev->data;
		sQmiWwanQmap *pQmapDev = (sQmiWwanQmap *)info->unused;
		bridge_ipv4 = pQmapDev->bridge_ipv4;
	}

	ipv4[0]  = (bridge_ipv4 >> 24) & 0xFF;
	ipv4[1]  = (bridge_ipv4 >> 16) & 0xFF;
	ipv4[2]  = (bridge_ipv4 >> 8) & 0xFF;
	ipv4[3]  = (bridge_ipv4 >> 0) & 0xFF;

	return snprintf(buf, PAGE_SIZE, "%d.%d.%d.%d\n",  ipv4[0], ipv4[1], ipv4[2], ipv4[3]);
}

static ssize_t bridge_ipv4_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	struct net_device *netdev = to_net_dev(dev);

	if (is_qmap_netdev(netdev)) {
		struct qmap_priv *priv = netdev_priv(netdev);
		priv->bridge_ipv4 = simple_strtoul(buf, NULL, 16);
	}
	else {
		struct usbnet * usbnetdev = netdev_priv( netdev );
		struct qmi_wwan_state *info = (void *)&usbnetdev->data;
		sQmiWwanQmap *pQmapDev = (sQmiWwanQmap *)info->unused;
		pQmapDev->bridge_ipv4 = simple_strtoul(buf, NULL, 16);
	}

	return count;
}
#endif

static DEVICE_ATTR(link_state, S_IWUSR | S_IRUGO, link_state_show, link_state_store);
#ifdef QUECTEL_BRIDGE_MODE
static DEVICE_ATTR(bridge_mode,  S_IRUGO, bridge_mode_show, NULL);
static DEVICE_ATTR(bridge_ipv4, S_IWUSR | S_IRUGO, bridge_ipv4_show, bridge_ipv4_store);
#endif

static struct attribute *qmi_wwan_sysfs_attrs[] = {
	&dev_attr_link_state.attr,
	&dev_attr_qmap_mode.attr,
	&dev_attr_qmap_size.attr,
#if defined(QUECTEL_UL_DATA_AGG)
	&dev_attr_qmap_debug.attr,
#endif
#ifdef QUECTEL_BRIDGE_MODE
	&dev_attr_bridge_mode.attr,
	&dev_attr_bridge_ipv4.attr,
#endif
	NULL,
};

static struct attribute_group qmi_wwan_sysfs_attr_group = {
	.attrs = qmi_wwan_sysfs_attrs,
};

#ifdef QUECTEL_BRIDGE_MODE
static struct attribute *qmi_qmap_sysfs_attrs[] = {
	&dev_attr_bridge_mode.attr,
	&dev_attr_bridge_ipv4.attr,
	NULL,
};

static struct attribute_group qmi_qmap_sysfs_attr_group = {
	.attrs = qmi_qmap_sysfs_attrs,
};
#endif

static int qmap_open(struct net_device *dev)
{
	struct qmap_priv *priv = netdev_priv(dev);
	struct net_device *real_dev = priv->real_dev;

	if (!(priv->real_dev->flags & IFF_UP))
		return -ENETDOWN;

	if (netif_carrier_ok(real_dev))
		netif_carrier_on(dev);
	return 0;
}

static int qmap_stop(struct net_device *pNet)
{
	netif_carrier_off(pNet);
	return 0;
}

static struct sk_buff * add_qhdr(struct sk_buff *skb, u8 mux_id) {
	struct qmap_hdr *qhdr;
	int pad = 0;

	pad = skb->len%4;
	if (pad) {
		pad = 4 - pad;
		if (skb_tailroom(skb) < pad) {
			printk("skb_tailroom small!\n");
			pad = 0;
		}
		if (pad)
			__skb_put(skb, pad);
	}
					
	qhdr = (struct qmap_hdr *)skb_push(skb, sizeof(struct qmap_hdr));
	qhdr->cd_rsvd_pad = pad;
	qhdr->mux_id = mux_id;
	qhdr->pkt_len = cpu_to_be16(skb->len - sizeof(struct qmap_hdr));

	return skb;
}

static struct sk_buff * add_qhdr_v5(struct sk_buff *skb, u8 mux_id) {
	struct rmnet_map_header *map_header;
	struct rmnet_map_v5_csum_header *ul_header;
	u32 padding, map_datalen;

	map_datalen = skb->len;
	padding = map_datalen%4;
	if (padding) {
		padding = 4 - padding;
		if (skb_tailroom(skb) < padding) {
			printk("skb_tailroom small!\n");
			padding = 0;
		}
		if (padding)
			__skb_put(skb, padding);
	}
					
	map_header = (struct rmnet_map_header *)skb_push(skb, (sizeof(struct rmnet_map_header) + sizeof(struct rmnet_map_v5_csum_header)));
	map_header->cd_bit = 0;
	map_header->next_hdr = 1;
	map_header->pad_len = padding;
	map_header->mux_id = mux_id;
	map_header->pkt_len = htons(map_datalen + padding);

	ul_header = (struct rmnet_map_v5_csum_header *)(map_header + 1);
	memset(ul_header, 0, sizeof(*ul_header));
	ul_header->header_type = RMNET_MAP_HEADER_TYPE_CSUM_OFFLOAD;
	if (skb->ip_summed == CHECKSUM_PARTIAL) {
#if 0 //TODO
		skb->ip_summed = CHECKSUM_NONE;
		/* Ask for checksum offloading */
		ul_header->csum_valid_required = 1;
#endif
	}

	return skb;
}

#if defined(QUECTEL_UL_DATA_AGG)
static int rmnet_usb_tx_agg_skip(struct sk_buff *skb, int offset)
{
	u8 *packet_start = skb->data + offset;
	int ready2send = 0;

	if (skb->protocol == htons(ETH_P_IP)) {
		struct iphdr *ip4h = (struct iphdr *)(packet_start);

		if (ip4h->protocol == IPPROTO_TCP) {
			const struct tcphdr *th = (const struct tcphdr *)(packet_start + sizeof(struct iphdr));
			if (th->psh) {
				ready2send = 1;
			}
		}
		else if (ip4h->protocol == IPPROTO_ICMP)
			ready2send = 1;

	} else if (skb->protocol == htons(ETH_P_IPV6)) {
		struct ipv6hdr *ip6h = (struct ipv6hdr *)(packet_start);

		if (ip6h->nexthdr == IPPROTO_ICMPV6) {
			ready2send = 1;
		} else if (ip6h->nexthdr == NEXTHDR_FRAGMENT) {
			struct frag_hdr *frag;

			frag = (struct frag_hdr *)(packet_start
						   + sizeof(struct ipv6hdr));
			if (frag->nexthdr == IPPROTO_ICMPV6)
				ready2send = 1;
		}
	}

	return ready2send;
}

static void rmnet_usb_tx_agg_work(struct work_struct *work)
{
	struct qmap_priv *priv =
			container_of(work, struct qmap_priv, agg_wq);
	struct sk_buff *skb = NULL;
	unsigned long flags;

	spin_lock_irqsave(&priv->agg_lock, flags);
	if (likely(priv->agg_skb)) {
		skb = priv->agg_skb;
		priv->agg_skb = NULL;
		priv->agg_count = 0;
		skb->protocol = htons(ETH_P_MAP);
		skb->dev = priv->real_dev;
		ktime_get_ts64(&priv->agg_time);
	}
	spin_unlock_irqrestore(&priv->agg_lock, flags);
	
	if (skb)
		dev_queue_xmit(skb);
}

static enum hrtimer_restart  rmnet_usb_tx_agg_timer_cb(struct hrtimer *timer)
{
	struct qmap_priv *priv =
			container_of(timer, struct qmap_priv, agg_hrtimer);

	schedule_work(&priv->agg_wq);
	return HRTIMER_NORESTART;
}

static int rmnet_usb_tx_agg(struct sk_buff *skb, struct qmap_priv *priv) {
	struct qmi_wwan_state *info = (void *)&priv->dev->data;
	sQmiWwanQmap *pQmapDev = (sQmiWwanQmap *)info->unused;
	struct tx_agg_ctx *ctx = &pQmapDev->tx_ctx;
	int ready2send = 0;
	int xmit_more = 0;
	struct timespec64 diff, now;
	struct sk_buff *agg_skb = NULL;
	unsigned long flags;

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,1,0) //6b16f9ee89b8d5709f24bc3ac89ae8b5452c0d7c
	xmit_more = skb->xmit_more;
#else
	xmit_more = netdev_xmit_more();
#endif

	if (ctx->ul_data_aggregation_max_datagrams == 1) {
		skb->protocol = htons(ETH_P_MAP);
		skb->dev = priv->real_dev;
		dev_queue_xmit(skb);
		return NET_XMIT_SUCCESS;
	}

new_packet:
	spin_lock_irqsave(&priv->agg_lock, flags);
	agg_skb = NULL;
	ready2send = 0;
	ktime_get_ts64(&now);
	diff = timespec64_sub(now, priv->agg_time);

	if (priv->agg_skb) {
		if ((priv->agg_skb->len + skb->len) < ctx->ul_data_aggregation_max_size) {
			memcpy(skb_put(priv->agg_skb, skb->len), skb->data, skb->len);
			priv->agg_count++;

			if (diff.tv_sec > 0 || diff.tv_nsec > (NSEC_PER_MSEC)) {
				ready2send = 1;
			}
			else if (priv->agg_count == ctx->ul_data_aggregation_max_datagrams) {
				ready2send = 1;
			}
			else if (xmit_more == 0) {
				struct rmnet_map_header *map_header = (struct rmnet_map_header *)skb->data;
				size_t offset = sizeof(struct rmnet_map_header);
				if (map_header->next_hdr)
					offset += sizeof(struct rmnet_map_v5_csum_header);

				ready2send = rmnet_usb_tx_agg_skip(skb, offset);
			}
			
			dev_kfree_skb_any(skb);
			skb = NULL;
		}
		else {
			ready2send = 1;
		}

		if (ready2send) {
			agg_skb = priv->agg_skb;
			priv->agg_skb = NULL;
			priv->agg_count = 0;
		}
	}
	else if (skb) {
		if (diff.tv_sec > 0 || diff.tv_nsec > NSEC_PER_MSEC) {
			ready2send = 1;
		}
		else if (xmit_more == 0) {
			struct rmnet_map_header *map_header = (struct rmnet_map_header *)skb->data;
			size_t offset = sizeof(struct rmnet_map_header);
			if (map_header->next_hdr)
				offset += sizeof(struct rmnet_map_v5_csum_header);

			ready2send = rmnet_usb_tx_agg_skip(skb, offset);
		}

		if (ready2send == 0) {
			priv->agg_skb = alloc_skb(ctx->ul_data_aggregation_max_size, GFP_ATOMIC);
			if (priv->agg_skb) {
				memcpy(skb_put(priv->agg_skb, skb->len), skb->data, skb->len);
				priv->agg_count++;
				dev_kfree_skb_any(skb);
				skb = NULL;
			}
			else {
				ready2send = 1;
			}
		}

		if (ready2send) {
			agg_skb = skb;
			skb = NULL;
		}
	}

	if (ready2send) {
		priv->agg_time = now;
	}
	spin_unlock_irqrestore(&priv->agg_lock, flags);

	if (agg_skb) {
		agg_skb->protocol = htons(ETH_P_MAP);
		agg_skb->dev = priv->real_dev;
		dev_queue_xmit(agg_skb);
	}

	if (skb) {
		goto new_packet;
	}

	if (priv->agg_skb) {
		if (!hrtimer_is_queued(&priv->agg_hrtimer))
			hrtimer_start(&priv->agg_hrtimer, ms_to_ktime(2), HRTIMER_MODE_REL);
	}

	return NET_XMIT_SUCCESS;
}
#endif

static netdev_tx_t rmnet_vnd_start_xmit(struct sk_buff *skb,
					struct net_device *pNet)
{
	int err;
	struct qmap_priv *priv = netdev_priv(pNet);
	int skb_len = skb->len;

	//printk("%s 1 skb=%p, len=%d, protocol=%x, hdr_len=%d\n", __func__, skb, skb->len, skb->protocol, skb->hdr_len);
	if (pNet->type == ARPHRD_ETHER) {
		skb_reset_mac_header(skb);

#ifdef QUECTEL_BRIDGE_MODE
		if (priv->bridge_mode && bridge_mode_tx_fixup(pNet, skb, priv->bridge_ipv4, priv->bridge_mac) == NULL) {
		      dev_kfree_skb_any (skb);
		      return NETDEV_TX_OK;
		}
#endif

		if (skb_pull(skb, ETH_HLEN) == NULL) {
			dev_kfree_skb_any (skb);
			return NETDEV_TX_OK;
		}
	}
	//printk("%s 2 skb=%p, len=%d, protocol=%x, hdr_len=%d\n", __func__, skb, skb->len, skb->protocol, skb->hdr_len);

	if (priv->qmap_version == 5) {
		add_qhdr(skb, priv->mux_id);
	}
	else if (priv->qmap_version == 9) {
		add_qhdr_v5(skb, priv->mux_id);
	}
	else {
		dev_kfree_skb_any (skb);
		return NETDEV_TX_OK;
	}
	//printk("%s skb=%p, len=%d, protocol=%x, hdr_len=%d\n", __func__, skb, skb->len, skb->protocol, skb->hdr_len);

#if 0
	skb->protocol = htons(ETH_P_MAP);
	skb->dev = priv->real_dev;
	err = dev_queue_xmit(skb);
#else
	err = rmnet_usb_tx_agg(skb, priv);
#endif
	if (err == NET_XMIT_SUCCESS) {
		pNet->stats.tx_packets++;
		pNet->stats.tx_bytes += skb_len;
	} else {
		pNet->stats.tx_errors++;
	}

	return err;
}

static int rmnet_vnd_change_mtu(struct net_device *rmnet_dev, int new_mtu)
{
	if (new_mtu < 0 || new_mtu > 1500)
		return -EINVAL;

	rmnet_dev->mtu = new_mtu;
	return 0;
}

/* drivers may override default ethtool_ops in their bind() routine */
static const struct ethtool_ops rmnet_vnd_ethtool_ops = {
	.get_link		= ethtool_op_get_link,
};

static int qmap_start_xmit(struct sk_buff *skb, struct net_device *pNet)
{
	int err;
	struct qmap_priv *priv = netdev_priv(pNet);

	//printk("%s 1 skb=%p, len=%d, protocol=%x, hdr_len=%d\n", __func__, skb, skb->len, skb->protocol, skb->hdr_len);
	skb_reset_mac_header(skb);

#ifdef QUECTEL_BRIDGE_MODE
	if (priv->bridge_mode && bridge_mode_tx_fixup(pNet, skb, priv->bridge_ipv4, priv->bridge_mac) == NULL) {
	      dev_kfree_skb_any (skb);
	      return NETDEV_TX_OK;
	}
#endif

	if (skb_pull(skb, ETH_HLEN) == NULL) {
	      dev_kfree_skb_any (skb);
	      return NETDEV_TX_OK;
   	}

	add_qhdr(skb, QUECTEL_QMAP_MUX_ID + priv->offset_id);

	skb->dev = priv->real_dev;
	err = dev_queue_xmit(skb);
	if (err == NET_XMIT_SUCCESS) {
		pNet->stats.tx_packets++;
		pNet->stats.tx_bytes += skb->len;
	} else {
		pNet->stats.tx_errors++;
	}

	return err;
}

static const struct net_device_ops qmap_netdev_ops = {
	.ndo_open       = qmap_open,
	.ndo_stop       = qmap_stop,
	.ndo_start_xmit = qmap_start_xmit,
};

static const struct net_device_ops rmnet_vnd_ops = {
	.ndo_open       = qmap_open,
	.ndo_stop       = qmap_stop,
	.ndo_start_xmit = rmnet_vnd_start_xmit,
	.ndo_change_mtu = rmnet_vnd_change_mtu,
};

static void rmnet_usb_vnd_setup(struct net_device *rmnet_dev)
{
	rmnet_dev->needed_headroom = 16;

	/* Raw IP mode */
	rmnet_dev->header_ops = NULL;  /* No header */
	rmnet_dev->type = ARPHRD_RAWIP;
	rmnet_dev->hard_header_len = 0;
	rmnet_dev->flags &= ~(IFF_BROADCAST | IFF_MULTICAST);
}

static rx_handler_result_t rmnet_usb_rx_priv_handler(struct sk_buff **pskb)
{
	struct sk_buff *skb = *pskb;
	struct rmnet_nss_cb *nss_cb;

	if (!skb)
		return RX_HANDLER_CONSUMED;
	
	//printk("%s skb=%p, len=%d, protocol=%x, hdr_len=%d\n", __func__, skb, skb->len, skb->protocol, skb->hdr_len);

	if (skb->pkt_type == PACKET_LOOPBACK)
		return RX_HANDLER_PASS;

	/* Check this so that we dont loop around netif_receive_skb */
	if (skb->cb[0] == 1) {
		skb->cb[0] = 0;

		return RX_HANDLER_PASS;
	}

	nss_cb = rcu_dereference(rmnet_nss_callbacks);
	if (nss_cb) {
		nss_cb->nss_tx(skb);
		return RX_HANDLER_CONSUMED;
	}

	return RX_HANDLER_PASS;
}

static int qmap_register_device(sQmiWwanQmap * pDev, u8 offset_id)
{
    struct net_device *real_dev = pDev->mpNetDev->net;
    struct net_device *qmap_net;
    struct qmap_priv *priv;
    int err;
	struct rmnet_nss_cb *nss_cb;

    qmap_net = alloc_etherdev(sizeof(*priv));
    if (!qmap_net)
        return -ENOBUFS;

    SET_NETDEV_DEV(qmap_net, &real_dev->dev);
    priv = netdev_priv(qmap_net);
    priv->offset_id = offset_id;
    priv->real_dev = real_dev;
    priv->self_dev = qmap_net;
    priv->dev = pDev->mpNetDev;
    priv->qmap_version = pDev->qmap_version;
    priv->mux_id = QUECTEL_QMAP_MUX_ID + offset_id;
    sprintf(qmap_net->name, "%s.%d", real_dev->name, offset_id + 1);
    qmap_net->netdev_ops = &qmap_netdev_ops;
    memcpy (qmap_net->dev_addr, real_dev->dev_addr, ETH_ALEN);

#ifdef QUECTEL_BRIDGE_MODE
	priv->bridge_mode = !!(pDev->bridge_mode & BIT(offset_id));
	qmap_net->sysfs_groups[0] = &qmi_qmap_sysfs_attr_group;
#endif

	priv->agg_skb = NULL;
	priv->agg_count = 0;
	hrtimer_init(&priv->agg_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	priv->agg_hrtimer.function = rmnet_usb_tx_agg_timer_cb;
	INIT_WORK(&priv->agg_wq, rmnet_usb_tx_agg_work);
	ktime_get_ts64(&priv->agg_time);
	spin_lock_init(&priv->agg_lock);

	if (pDev->use_rmnet_usb) {
		qmap_net->ethtool_ops = &rmnet_vnd_ethtool_ops;
		qmap_net->netdev_ops = &rmnet_vnd_ops;
	}

	nss_cb = rcu_dereference(rmnet_nss_callbacks);
	if (nss_cb) {
		rmnet_usb_vnd_setup(qmap_net);
	}

    err = register_netdev(qmap_net);
    dev_info(&real_dev->dev, "%s(%s)=%d\n", __func__, qmap_net->name, err);
    if (err < 0)
        goto out_free_newdev;
    netif_device_attach (qmap_net);
    netif_carrier_off(qmap_net);

	nss_cb = rcu_dereference(rmnet_nss_callbacks);
	if (nss_cb) {
		int rc = nss_cb->nss_create(qmap_net);
		if (rc) {
			/* Log, but don't fail the device creation */
			netdev_err(qmap_net, "Device will not use NSS path: %d\n", rc);
		} else {
			netdev_info(qmap_net, "NSS context created\n");
			rtnl_lock();
			netdev_rx_handler_register(qmap_net, rmnet_usb_rx_priv_handler, NULL);
			rtnl_unlock();			
		}
	}

	if (pDev->use_rmnet_usb) {
		strcpy(pDev->rmnet_info.ifname[offset_id], qmap_net->name);
		pDev->rmnet_info.mux_id[offset_id] = priv->mux_id;
	}

    pDev->mpQmapNetDev[offset_id] = qmap_net;
    qmap_net->flags |= IFF_NOARP;
    qmap_net->flags &= ~(IFF_BROADCAST | IFF_MULTICAST);

    dev_info(&real_dev->dev, "%s %s\n", __func__, qmap_net->name);

    return 0;

out_free_newdev:
    free_netdev(qmap_net);
    return err;
}

static void qmap_unregister_device(sQmiWwanQmap * pDev, u8 offset_id) {
	struct net_device *qmap_net = pDev->mpQmapNetDev[offset_id];

	if (qmap_net != NULL && qmap_net != pDev->mpNetDev->net) {
		struct rmnet_nss_cb *nss_cb;

		pr_info("qmap_unregister_device(%s)\n", qmap_net->name);
		pDev->mpQmapNetDev[offset_id] = NULL;
		netif_carrier_off( qmap_net );
		nss_cb = rcu_dereference(rmnet_nss_callbacks);
		if (nss_cb) {
			rtnl_lock();
			netdev_rx_handler_unregister(qmap_net);
			rtnl_unlock();
			nss_cb->nss_free(qmap_net);
		}
		unregister_netdev (qmap_net);
		free_netdev(qmap_net);
	}
}

#if 1//def CONFIG_ANDROID
typedef struct {
    unsigned int size;
    unsigned int rx_urb_size;
    unsigned int ep_type;
    unsigned int iface_id;
    unsigned int MuxId;
    unsigned int ul_data_aggregation_max_datagrams; //0x17
    unsigned int ul_data_aggregation_max_size ;//0x18
    unsigned int dl_minimum_padding; //0x1A
} QMAP_SETTING;

int qma_setting_store(struct device *dev, QMAP_SETTING *qmap_settings, size_t size) {
	struct net_device *netdev = to_net_dev(dev);
	struct usbnet * usbnetdev = netdev_priv( netdev );
	struct qmi_wwan_state *info = (void *)&usbnetdev->data;
	sQmiWwanQmap *pQmapDev = (sQmiWwanQmap *)info->unused;

	if (qmap_settings->size != size) {
		dev_err(dev, "ERROR: qmap_settings.size donot match!\n");
		return -EOPNOTSUPP;
	}

#ifdef QUECTEL_UL_DATA_AGG
	netif_tx_lock_bh(netdev);
	if (pQmapDev->tx_ctx.ul_data_aggregation_max_datagrams == 1 && qmap_settings->ul_data_aggregation_max_datagrams > 1) {
		pQmapDev->tx_ctx.ul_data_aggregation_max_datagrams = qmap_settings->ul_data_aggregation_max_datagrams;
		pQmapDev->tx_ctx.ul_data_aggregation_max_size = qmap_settings->ul_data_aggregation_max_size;
		pQmapDev->tx_ctx.dl_minimum_padding = qmap_settings->dl_minimum_padding;
		dev_info(dev, "ul_data_aggregation_max_datagrams=%d, ul_data_aggregation_max_size=%d, dl_minimum_padding=%d\n",
			pQmapDev->tx_ctx.ul_data_aggregation_max_datagrams,
			pQmapDev->tx_ctx.ul_data_aggregation_max_size,
			pQmapDev->tx_ctx.dl_minimum_padding);
	}
	netif_tx_unlock_bh(netdev);
	return 0;
#endif

	return -EOPNOTSUPP;
}

static int qmap_ndo_do_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd) {
	struct usbnet * usbnetdev = netdev_priv( dev );
	struct qmi_wwan_state *info = (void *)&usbnetdev->data;
	sQmiWwanQmap *pQmapDev = (sQmiWwanQmap *)info->unused;
	int rc = -EOPNOTSUPP;
	uint link_state = 0;
 	QMAP_SETTING qmap_settings = {0};
 
	switch (cmd) {
	case 0x89F1: //SIOCDEVPRIVATE
		rc = copy_from_user(&link_state, ifr->ifr_ifru.ifru_data, sizeof(link_state));
		if (!rc) {
			char buf[32];
			snprintf(buf, sizeof(buf), "%u", link_state);
			link_state_store(&dev->dev, NULL, buf, strlen(buf));
		}
	break;

	case 0x89F2: //SIOCDEVPRIVATE
		rc = copy_from_user(&qmap_settings, ifr->ifr_ifru.ifru_data, sizeof(qmap_settings));
		if (!rc) {
			rc = qma_setting_store(&dev->dev, &qmap_settings, sizeof(qmap_settings));
		}
	break;

	case 0x89F3: //SIOCDEVPRIVATE
		if (pQmapDev->use_rmnet_usb) {
			rc = copy_to_user(ifr->ifr_ifru.ifru_data, &pQmapDev->rmnet_info, sizeof(pQmapDev->rmnet_info));
		}
	break;

	default:
	break;
	}

	return rc;
}
#endif

#ifdef QUECTEL_BRIDGE_MODE
static int is_qmap_netdev(const struct net_device *netdev) {
	return netdev->netdev_ops == &qmap_netdev_ops;
}
#endif
#endif

static struct sk_buff *qmi_wwan_tx_fixup(struct usbnet *dev, struct sk_buff *skb, gfp_t flags) {
	//MDM9x07,MDM9628,MDM9x40,SDX20,SDX24 only work on RAW IP mode
	if ((dev->driver_info->flags & FLAG_NOARP) == 0)
		return skb;

	// Skip Ethernet header from message
	if (dev->net->hard_header_len == 0)
		return skb;
	else
		skb_reset_mac_header(skb);

#ifdef QUECTEL_BRIDGE_MODE
{
	struct qmi_wwan_state *info = (void *)&dev->data;
	sQmiWwanQmap *pQmapDev = (sQmiWwanQmap *)info->unused;

	if (pQmapDev->bridge_mode && bridge_mode_tx_fixup(dev->net, skb, pQmapDev->bridge_ipv4, pQmapDev->bridge_mac) == NULL) {
	      dev_kfree_skb_any (skb);
	      return NULL;
	}
}
#endif

	if (skb_pull(skb, ETH_HLEN)) {
		return skb;
	} else {
		dev_err(&dev->intf->dev,  "Packet Dropped ");
	}

	// Filter the packet out, release it
	dev_kfree_skb_any(skb);
	return NULL;
}
#endif

/* Make up an ethernet header if the packet doesn't have one.
 *
 * A firmware bug common among several devices cause them to send raw
 * IP packets under some circumstances.  There is no way for the
 * driver/host to know when this will happen.  And even when the bug
 * hits, some packets will still arrive with an intact header.
 *
 * The supported devices are only capably of sending IPv4, IPv6 and
 * ARP packets on a point-to-point link. Any packet with an ethernet
 * header will have either our address or a broadcast/multicast
 * address as destination.  ARP packets will always have a header.
 *
 * This means that this function will reliably add the appropriate
 * header iff necessary, provided our hardware address does not start
 * with 4 or 6.
 *
 * Another common firmware bug results in all packets being addressed
 * to 00:a0:c6:00:00:00 despite the host address being different.
 * This function will also fixup such packets.
 */
static int qmi_wwan_rx_fixup(struct usbnet *dev, struct sk_buff *skb)
{
	__be16 proto;

	/* This check is no longer done by usbnet */
	if (skb->len < dev->net->hard_header_len)
		return 0;

	switch (skb->data[0] & 0xf0) {
	case 0x40:
		proto = htons(ETH_P_IP);
		break;
	case 0x60:
		proto = htons(ETH_P_IPV6);
		break;
	case 0x00:
		if (is_multicast_ether_addr(skb->data))
			return 1;
		/* possibly bogus destination - rewrite just in case */
		skb_reset_mac_header(skb);
		goto fix_dest;
	default:
		/* pass along other packets without modifications */
		return 1;
	}
	if (skb_headroom(skb) < ETH_HLEN)
		return 0;
	skb_push(skb, ETH_HLEN);
	skb_reset_mac_header(skb);
	eth_hdr(skb)->h_proto = proto;
	memset(eth_hdr(skb)->h_source, 0, ETH_ALEN);
#if 1 //Added by Quectel
	//some kernel will drop ethernet packet which's souce mac is all zero
	memcpy(eth_hdr(skb)->h_source, default_modem_addr, ETH_ALEN);
#endif

fix_dest:
#ifdef QUECTEL_BRIDGE_MODE
{
	struct qmi_wwan_state *info = (void *)&dev->data;
	sQmiWwanQmap *pQmapDev = (sQmiWwanQmap *)info->unused;
	bridge_mode_rx_fixup(pQmapDev, dev->net, skb);
}
#else
	memcpy(eth_hdr(skb)->h_dest, dev->net->dev_addr, ETH_ALEN);
#endif

	return 1;
}

#if defined(QUECTEL_WWAN_QMAP)
#if defined(QUECTEL_UL_DATA_AGG)
static void qmap_qmi_wwan_tx_bh(unsigned long param)
{
	struct sQmiWwanQmap *pQmapDev = (struct sQmiWwanQmap *)param;
	struct tx_agg_ctx *ctx = &pQmapDev->tx_ctx;
	struct net_device *dev = pQmapDev->mpNetDev->net;

	if (pQmapDev->link_state && ctx->tx_pending_count) {
		netif_tx_lock_bh(dev);
		usbnet_start_xmit(NULL, dev);
		netif_tx_unlock_bh(dev);
	}
}

static enum hrtimer_restart qmap_qmi_wwan_tx_timer_cb(struct hrtimer *timer)
{
	struct sQmiWwanQmap *pQmapDev =
			container_of(timer, struct sQmiWwanQmap, tx_ctx.tx_timer);
	struct tx_agg_ctx *ctx = &pQmapDev->tx_ctx;

	if (pQmapDev->link_state && ctx->tx_pending_count) {
		if (ctx->tx_timer_pkts != ctx->tx_sending_pkts) {
			ctx->tx_timer_pkts = ctx->tx_sending_pkts;
			return HRTIMER_RESTART;
		}
		tasklet_schedule(&ctx->tx_bh);
	}
	ctx->tx_timer_active = 0;
	return HRTIMER_NORESTART;
}

static struct sk_buff *qmap_qmi_wwan_tx_agg(struct usbnet *dev, struct sk_buff *skb, gfp_t flags) {
	struct net_device *net = dev->net;
	struct qmi_wwan_state *info = (void *)&dev->data;
	sQmiWwanQmap *pQmapDev = (sQmiWwanQmap *)info->unused;
	struct tx_agg_ctx *ctx = &pQmapDev->tx_ctx;
	struct sk_buff *skb_out = NULL;
	int ready2send = 0;

	if (ctx->ul_data_aggregation_max_datagrams == 1) {
		if (skb) {
			ctx->qmap_tx_pkts[0]++;
			usbnet_set_skb_tx_stats(skb, 1, 0);
		}
		return skb;
	}

	if (skb) {
		if (ctx->tx_pending_skb[ctx->tx_pending_count]) {
			dev_info(&net->dev, "tx_pending_skb[%d] = %p\n",
				ctx->tx_pending_count, ctx->tx_pending_skb[ctx->tx_pending_count]);
		}
		ctx->tx_pending_skb[ctx->tx_pending_count++] = skb;
		ctx->tx_pending_size += skb->len;

		if (ctx->tx_pending_count >= ctx->ul_data_aggregation_max_datagrams) {
			ctx->tx_agg_match[0]++;
			ready2send = 1;
		} else if (ctx->tx_pending_size >= ctx->ul_data_aggregation_max_size) {
			ctx->tx_agg_match[1]++;
			ready2send = 1;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,1,0) //6b16f9ee89b8d5709f24bc3ac89ae8b5452c0d7c
		} else if (skb->xmit_more == 0) {
#else
		} else if (netdev_xmit_more() == 0) {
#endif
			if (ctx->tx_timer_active == 0 || agg_time_limit == 0) {
				ctx->tx_agg_match[2]++;
				ready2send = 1;
			}
			else if (skb->data[4] == 0x45) {
				const struct iphdr *iph = (const struct iphdr *)(&(skb->data[4]));
				if (iph->protocol == IPPROTO_TCP) {
					const struct tcphdr *th = (const struct tcphdr *)(&(skb->data[24]));
					if (th->psh) {
						ctx->tx_agg_match[3]++;
						ready2send = 1;
					}
				}
			}
		}
	}
	else if (ctx->tx_pending_count) {
		ctx->tx_agg_match[4]++;
		ready2send = 1;
	}

	if (ready2send && ctx->tx_pending_count)
	{
		uint i, skb_count = 0, skb_size;

		if (ctx->tx_pending_count == 1)
		{
			skb_out = ctx->tx_pending_skb[0];
			ctx->tx_pending_skb[0] = NULL;
			ctx->tx_pending_count = 0;
			ctx->tx_pending_size = 0;
			skb_count = 1;
			skb_size = ctx->tx_pending_size;
		}
		else if (ctx->tx_pending_count > 1)
		{
			if (ctx->tx_pending_size <= ctx->ul_data_aggregation_max_size) {
				skb_count = ctx->tx_pending_count;
				skb_size = ctx->tx_pending_size;
			}
			else {
				skb_count = ctx->tx_pending_count - 1;
				skb_size = ctx->tx_pending_size - ctx->tx_pending_skb[skb_count]->len;
			}

			skb_out = alloc_skb(skb_size, flags);

			if (skb_out) {
				for (i = 0; i < skb_count; i++) {
					skb = ctx->tx_pending_skb[i];
					memcpy(skb_put(skb_out, skb->len), skb->data, skb->len);
					dev_kfree_skb_any(skb);
					ctx->tx_pending_skb[i] = NULL;
				}

				if (skb_count == ctx->tx_pending_count) {
					ctx->tx_pending_count = 0;
					ctx->tx_pending_size = 0;
				}
				else {
					skb = ctx->tx_pending_skb[0] = ctx->tx_pending_skb[skb_count];
					ctx->tx_pending_skb[skb_count] = NULL;
					ctx->tx_pending_count = 1;
					ctx->tx_pending_size = skb->len;
				}
			}
			else {
				dev_info(&net->dev, "alloc skb_out fail len = %d\n", skb_size);
				for (i = 0; i < ctx->tx_pending_count; i++) {
					skb = ctx->tx_pending_skb[i];
					dev_kfree_skb_any(skb);
					ctx->tx_pending_skb[i] = NULL;
				}
				ctx->tx_pending_count = 0;
				ctx->tx_pending_size = 0;
			}
		}

		if (skb_out && skb_count) {
			if (skb_count && (dev->driver_info->flags&FLAG_MULTI_PACKET)) {
				skb_out->dev = dev->net;
				usbnet_set_skb_tx_stats(skb_out, skb_count, 0);
			}
		
			if (skb_count > ARRAY_SIZE(ctx->qmap_tx_pkts))
				skb_count = ARRAY_SIZE(ctx->qmap_tx_pkts);
			ctx->qmap_tx_pkts[skb_count - 1]++;
		}
	}

	if (skb_out || ctx->tx_pending_count) {
		if (ctx->tx_timer_active == 0 && agg_time_limit) {
			ctx->tx_timer_active = 1;
			ctx->tx_timer_pkts = ctx->tx_sending_pkts;
			hrtimer_start(&ctx->tx_timer, ms_to_ktime(agg_time_limit), HRTIMER_MODE_REL);
		}
		else if (skb_out) {
			ctx->tx_sending_pkts++;
		}
	}

	return skb_out;
}
#endif

static struct sk_buff *qmap_qmi_wwan_tx_fixup(struct usbnet *dev, struct sk_buff *skb, gfp_t flags) {
	struct qmi_wwan_state *info = (void *)&dev->data;
	sQmiWwanQmap *pQmapDev = (sQmiWwanQmap *)info->unused;
	struct qmap_hdr *qhdr;

	if (unlikely(pQmapDev == NULL)) {
		goto drop_skb;
	} else if (unlikely(pQmapDev->qmap_mode && !pQmapDev->link_state)) {
		dev_dbg(&dev->net->dev, "link_state 0x%x, drop skb, len = %u\n", pQmapDev->link_state, skb->len);
		goto drop_skb;
	} else if (pQmapDev->qmap_mode == 0) {
		skb = qmi_wwan_tx_fixup(dev, skb, flags);
	}
	else if (pQmapDev->qmap_mode > 1) {
		if (likely(skb)) {	
			qhdr = (struct qmap_hdr *)skb->data;
			if ((qhdr->mux_id&0xF0) != 0x80 || ((be16_to_cpu(qhdr->pkt_len) + sizeof(struct qmap_hdr)) != skb->len)) {
				goto drop_skb;
			}
		}
	}
	else {
		if (likely(skb)) {
			skb = qmi_wwan_tx_fixup(dev, skb, flags);

			if (skb) {
				add_qhdr(skb, QUECTEL_QMAP_MUX_ID);
			}
			else {
				return NULL;
			}
		}
	}

#if defined(QUECTEL_UL_DATA_AGG)
	if (pQmapDev->qmap_mode) {
		skb = qmap_qmi_wwan_tx_agg(dev, skb, flags);
	}
#else
	if (skb && (dev->driver_info->flags&FLAG_MULTI_PACKET)) {
		usbnet_set_skb_tx_stats(skb, 1, 0);
 	}
#endif


	return skb;
drop_skb:
	dev_kfree_skb_any (skb);
	return NULL;
}

static int qmap_qmi_wwan_rx_fixup(struct usbnet *dev, struct sk_buff *skb_in)
{
	struct qmi_wwan_state *info = (void *)&dev->data;
	sQmiWwanQmap *pQmapDev = (sQmiWwanQmap *)info->unused;
#if defined(QUECTEL_UL_DATA_AGG)
	unsigned debug_pkts = 0;
#endif
	unsigned headroom = 0;
	const unsigned need_headroot = ETH_HLEN;
	struct sk_buff *qmap_skb;
		
	if (pQmapDev->qmap_mode == 0)
		return qmi_wwan_rx_fixup(dev, skb_in);

	headroom = skb_headroom(skb_in);

	while (skb_in->len > sizeof(struct qmap_hdr)) {
		struct qmap_hdr *qhdr = (struct qmap_hdr *)skb_in->data;
		struct net_device *qmap_net;
		int pkt_len = be16_to_cpu(qhdr->pkt_len);
		int skb_len;
		__be16 protocol;
		int mux_id;

		skb_len = pkt_len - (qhdr->cd_rsvd_pad&0x3F);
		if (skb_len > 1500) {
			dev_info(&dev->net->dev, "drop skb_len=%x larger than 1500\n", skb_len);
			goto error_pkt;
		}

		if (skb_in->len < (pkt_len + sizeof(struct qmap_hdr))) {
			dev_info(&dev->net->dev, "drop qmap unknow pkt, len=%d, pkt_len=%d\n", skb_in->len, pkt_len);
			goto error_pkt;
		}

		if (qhdr->cd_rsvd_pad & 0x80) {
			dev_info(&dev->net->dev, "skip qmap command packet %x\n", qhdr->cd_rsvd_pad);
			goto skip_pkt;
		}

		switch (skb_in->data[sizeof(struct qmap_hdr)] & 0xf0) {
			case 0x40:
				protocol = htons(ETH_P_IP);
			break;
			case 0x60:
				protocol = htons(ETH_P_IPV6);
			break;
			default:
				dev_info(&dev->net->dev, "unknow skb->protocol %02x\n", skb_in->data[sizeof(struct qmap_hdr)]);
				goto error_pkt;
		}
		
		mux_id = qhdr->mux_id - QUECTEL_QMAP_MUX_ID;
		if (mux_id >= pQmapDev->qmap_mode) {
			dev_info(&dev->net->dev, "drop qmap unknow mux_id %x\n", qhdr->mux_id);
			goto error_pkt;
		}

		qmap_net = pQmapDev->mpQmapNetDev[mux_id];

		if (qmap_net == NULL) {
			dev_info(&dev->net->dev, "drop qmap unknow mux_id %x\n", qhdr->mux_id);
			goto skip_pkt;
		}

		if (headroom >= need_headroot) {
			qmap_skb = skb_clone(skb_in, GFP_ATOMIC);
			if (qmap_skb) {
				qmap_skb->dev = qmap_net;
				skb_pull(qmap_skb, sizeof(struct qmap_hdr));
				skb_trim(qmap_skb, skb_len);
			}
			headroom = (qhdr->cd_rsvd_pad&0x3F);
		}
		else {	
			qmap_skb = netdev_alloc_skb(qmap_net, need_headroot + skb_len);
			if (qmap_skb) {
				skb_reserve(qmap_skb, need_headroot);
				skb_put(qmap_skb, skb_len);
				memcpy(qmap_skb->data, skb_in->data + sizeof(struct qmap_hdr), skb_len);
			}
			headroom = pkt_len;
		}		

		if (qmap_skb == NULL) {
			dev_info(&dev->net->dev, "fail to alloc skb, pkt_len = %d\n", skb_len);
			return 0;
		}

		skb_push(qmap_skb, ETH_HLEN);
		skb_reset_mac_header(qmap_skb);
		memcpy(eth_hdr(qmap_skb)->h_source, default_modem_addr, ETH_ALEN);
		memcpy(eth_hdr(qmap_skb)->h_dest, qmap_net->dev_addr, ETH_ALEN);
		eth_hdr(qmap_skb)->h_proto =  protocol;
#ifdef QUECTEL_BRIDGE_MODE
		bridge_mode_rx_fixup(pQmapDev, qmap_net, qmap_skb);
#endif

		if (qmap_net != dev->net) {
			qmap_net->stats.rx_packets++;
			qmap_net->stats.rx_bytes += qmap_skb->len;
		}
		
		skb_queue_tail(&pQmapDev->skb_chain, qmap_skb);

skip_pkt:
#if defined(QUECTEL_UL_DATA_AGG)
		debug_pkts++;
#endif
		skb_pull(skb_in, pkt_len + sizeof(struct qmap_hdr));
	}

	while ((qmap_skb = skb_dequeue (&pQmapDev->skb_chain))) {
		if (qmap_skb->dev != dev->net) {
			qmap_skb->protocol = eth_type_trans (qmap_skb, qmap_skb->dev);
			netif_rx(qmap_skb);
		}
		else {
			usbnet_skb_return(dev, qmap_skb);
		}
	}

#if defined(QUECTEL_UL_DATA_AGG)
	if (debug_pkts) {
		struct tx_agg_ctx *ctx = &pQmapDev->tx_ctx;
		if (debug_pkts > ARRAY_SIZE(ctx->qmap_rx_pkts))
			debug_pkts = ARRAY_SIZE(ctx->qmap_rx_pkts);
		ctx->qmap_rx_pkts[debug_pkts - 1]++;
	}
#endif

error_pkt:
    return 0;
}
#endif

/* very simplistic detection of IPv4 or IPv6 headers */
static bool possibly_iphdr(const char *data)
{
	return (data[0] & 0xd0) == 0x40;
}

/* disallow addresses which may be confused with IP headers */
static int qmi_wwan_mac_addr(struct net_device *dev, void *p)
{
	int ret;
	struct sockaddr *addr = p;

	ret = eth_prepare_mac_addr_change(dev, p);
	if (ret < 0)
		return ret;
	if (possibly_iphdr(addr->sa_data))
		return -EADDRNOTAVAIL;
	eth_commit_mac_addr_change(dev, p);
	return 0;
}

static const struct net_device_ops qmi_wwan_netdev_ops = {
	.ndo_open		= usbnet_open,
	.ndo_stop		= usbnet_stop,
	.ndo_start_xmit		= usbnet_start_xmit,
	.ndo_tx_timeout		= usbnet_tx_timeout,
	.ndo_change_mtu		= usbnet_change_mtu,
#if (LINUX_VERSION_CODE > KERNEL_VERSION( 4,11,0 ))
	.ndo_get_stats64	= usbnet_get_stats64,
#endif
	.ndo_set_mac_address	= qmi_wwan_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
#if defined(QUECTEL_WWAN_QMAP)// && defined(CONFIG_ANDROID)
	.ndo_do_ioctl = qmap_ndo_do_ioctl,
#endif
};

static void ql_net_get_drvinfo(struct net_device *net, struct ethtool_drvinfo *info)
{
	/* Inherit standard device info */
	usbnet_get_drvinfo(net, info);
	strlcpy(info->driver, driver_name, sizeof(info->driver));
	strlcpy(info->version, VERSION_NUMBER, sizeof(info->version));
}

static struct ethtool_ops ql_net_ethtool_ops;

/* using a counter to merge subdriver requests with our own into a
 * combined state
 */
static int qmi_wwan_manage_power(struct usbnet *dev, int on)
{
	struct qmi_wwan_state *info = (void *)&dev->data;
	int rv;

	dev_dbg(&dev->intf->dev, "%s() pmcount=%d, on=%d\n", __func__,
		atomic_read(&info->pmcount), on);

	if ((on && atomic_add_return(1, &info->pmcount) == 1) ||
	    (!on && atomic_dec_and_test(&info->pmcount))) {
		/* need autopm_get/put here to ensure the usbcore sees
		 * the new value
		 */
		rv = usb_autopm_get_interface(dev->intf);
		dev->intf->needs_remote_wakeup = on;
		if (!rv)
			usb_autopm_put_interface(dev->intf);
	}
	return 0;
}

static int qmi_wwan_cdc_wdm_manage_power(struct usb_interface *intf, int on)
{
	struct usbnet *dev = usb_get_intfdata(intf);

	/* can be called while disconnecting */
	if (!dev)
		return 0;
	return qmi_wwan_manage_power(dev, on);
}

/* collect all three endpoints and register subdriver */
static int qmi_wwan_register_subdriver(struct usbnet *dev)
{
	int rv;
	struct usb_driver *subdriver = NULL;
	struct qmi_wwan_state *info = (void *)&dev->data;

	/* collect bulk endpoints */
	rv = usbnet_get_endpoints(dev, info->data);
	if (rv < 0)
		goto err;

	/* update status endpoint if separate control interface */
	if (info->control != info->data)
		dev->status = &info->control->cur_altsetting->endpoint[0];

	/* require interrupt endpoint for subdriver */
	if (!dev->status) {
		rv = -EINVAL;
		goto err;
	}

	/* for subdriver power management */
	atomic_set(&info->pmcount, 0);

	/* register subdriver */
	subdriver = usb_cdc_wdm_register(info->control, &dev->status->desc,
					 4096, &qmi_wwan_cdc_wdm_manage_power);
	if (IS_ERR(subdriver)) {
		dev_err(&info->control->dev, "subdriver registration failed\n");
		rv = PTR_ERR(subdriver);
		goto err;
	}

	/* prevent usbnet from using status endpoint */
	dev->status = NULL;

	/* save subdriver struct for suspend/resume wrappers */
	info->subdriver = subdriver;

err:
	return rv;
}

static int qmi_wwan_bind(struct usbnet *dev, struct usb_interface *intf)
{
	int status = -1;
	struct usb_driver *driver = driver_of(intf);
	struct qmi_wwan_state *info = (void *)&dev->data;

	BUILD_BUG_ON((sizeof(((struct usbnet *)0)->data) <
		      sizeof(struct qmi_wwan_state)));

	/* set up initial state */
	info->control = intf;
	info->data = intf;

	status = qmi_wwan_register_subdriver(dev);
	if (status < 0 && info->control != info->data) {
		usb_set_intfdata(info->data, NULL);
		usb_driver_release_interface(driver, info->data);
	}

	/* Never use the same address on both ends of the link, even
	 * if the buggy firmware told us to.
	 */
	if (ether_addr_equal(dev->net->dev_addr, default_modem_addr))
		eth_hw_addr_random(dev->net);

	/* make MAC addr easily distinguishable from an IP header */
	if (possibly_iphdr(dev->net->dev_addr)) {
		dev->net->dev_addr[0] |= 0x02;	/* set local assignment bit */
		dev->net->dev_addr[0] &= 0xbf;	/* clear "IP" bit */
	}
	dev->net->netdev_ops = &qmi_wwan_netdev_ops;

	ql_net_ethtool_ops = *dev->net->ethtool_ops;
	ql_net_ethtool_ops.get_drvinfo = ql_net_get_drvinfo;
	dev->net->ethtool_ops = &ql_net_ethtool_ops;

#if 1 //Added by Quectel
	if (dev->driver_info->flags & FLAG_NOARP) {
		dev_info(&intf->dev, "Quectel EC25&EC21&EG91&EG95&EG06&EP06&EM06&EG12&EP12&EM12&EG16&EG18&BG96&AG35 work on RawIP mode\n");
		dev->net->flags |= IFF_NOARP;
		dev->net->flags &= ~(IFF_BROADCAST | IFF_MULTICAST);

		usb_control_msg(
			interface_to_usbdev(intf),
			usb_sndctrlpipe(interface_to_usbdev(intf), 0),
			0x22, //USB_CDC_REQ_SET_CONTROL_LINE_STATE
			0x21, //USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE
			1, //active CDC DTR
			intf->cur_altsetting->desc.bInterfaceNumber,
			NULL, 0, 100);
	}

	//to advoid module report mtu 1460, but rx 1500 bytes IP packets, and cause the customer's system crash
	//next setting can make usbnet.c:usbnet_change_mtu() do not modify rx_urb_size according to hard mtu
	dev->rx_urb_size = ETH_DATA_LEN + ETH_HLEN + 6;

#if defined(QUECTEL_WWAN_QMAP)
	if (qmap_mode > QUECTEL_WWAN_QMAP)
		qmap_mode = QUECTEL_WWAN_QMAP;

	if (!status)
	{
		sQmiWwanQmap *pQmapDev = (sQmiWwanQmap *)kzalloc(sizeof(sQmiWwanQmap), GFP_KERNEL);

		if (pQmapDev == NULL)
			return -ENODEV;

#ifdef QUECTEL_BRIDGE_MODE
		pQmapDev->bridge_mode = bridge_mode;
#endif
		pQmapDev->mpNetDev = dev;
		pQmapDev->link_state = 1;
		//on OpenWrt, if set rmnet_usb0.1 as WAN, '/sbin/netifd' will auto create VLAN for rmnet_usb0
		dev->net->features |= (NETIF_F_VLAN_CHALLENGED);

		skb_queue_head_init(&pQmapDev->skb_chain);

		if (dev->driver_info->flags & FLAG_NOARP)
		{
			int idProduct = le16_to_cpu(dev->udev->descriptor.idProduct);
			int lte_a = (idProduct == 0x0306 || idProduct == 0x0512 || idProduct == 0x0620 || idProduct == 0x0800);

			pQmapDev->qmap_mode = qmap_mode;
			if (lte_a || dev->udev->speed == USB_SPEED_SUPER) {
				if (pQmapDev->qmap_mode == 0) {
					pQmapDev->qmap_mode = 1; //force use QMAP
					if(qmap_mode == 0)
						qmap_mode = 1; //old quectel-CM only check sys/module/wwan0/parameters/qmap_mode
				}
			}

			if (pQmapDev->qmap_mode) {
				pQmapDev->qmap_version = 5;
				if (idProduct == 0x0121 || idProduct == 0x0125 || idProduct == 0x0435) //MDM9x07
					dev->rx_urb_size = 4*1024;
				else if (idProduct == 0x0306 || idProduct == 0x0512) //EM06&EM12
					dev->rx_urb_size = 16*1024;					
				else if (idProduct == 0x0800)  //if set as 32K, x55 will rx error pkt
				{
					dev->rx_urb_size = 31*1024;
					pQmapDev->qmap_version = 9;
				}
				else
					dev->rx_urb_size = 32*1024;

				pQmapDev->qmap_size = dev->rx_urb_size;
				//for these modules, if send pakcet before qmi_start_network, or cause host PC crash, or cause modules crash
				if (lte_a || dev->udev->speed == USB_SPEED_SUPER)
					pQmapDev->link_state = 0;
			}

#if defined(QUECTEL_UL_DATA_AGG)
			if (pQmapDev->qmap_mode) {
				struct tx_agg_ctx *ctx = &pQmapDev->tx_ctx;
				tasklet_init(&ctx->tx_bh, qmap_qmi_wwan_tx_bh, (unsigned long)pQmapDev);
				hrtimer_init(&ctx->tx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
				ctx->tx_timer.function = &qmap_qmi_wwan_tx_timer_cb;
				ctx->ul_data_aggregation_max_datagrams = 1;
				ctx->ul_data_aggregation_max_size = 2048;
			}
#endif

			if (pQmapDev->qmap_mode == 0) {
				pQmapDev->driver_info = *dev->driver_info;
				pQmapDev->driver_info.flags &= ~(FLAG_MULTI_PACKET); //see usbnet.c rx_process()
				dev->driver_info = &pQmapDev->driver_info;
			}
		}

		info->unused = (unsigned long)pQmapDev;
		dev->net->sysfs_groups[0] = &qmi_wwan_sysfs_attr_group;

		dev_info(&intf->dev, "rx_urb_size = %zd\n", dev->rx_urb_size);
	}
#endif
#endif

	return status;
}

static void qmi_wwan_unbind(struct usbnet *dev, struct usb_interface *intf)
{
	struct qmi_wwan_state *info = (void *)&dev->data;
	struct usb_driver *driver = driver_of(intf);
	struct usb_interface *other;

	if (dev->udev && dev->udev->state == USB_STATE_CONFIGURED) {
		usb_control_msg(
			interface_to_usbdev(intf),
			usb_sndctrlpipe(interface_to_usbdev(intf), 0),
			0x22, //USB_CDC_REQ_SET_CONTROL_LINE_STATE
			0x21, //USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE
			0, //deactive CDC DTR
			intf->cur_altsetting->desc.bInterfaceNumber,
			NULL, 0, 100);
	}

	if (info->subdriver && info->subdriver->disconnect)
		info->subdriver->disconnect(info->control);

	/* allow user to unbind using either control or data */
	if (intf == info->control)
		other = info->data;
	else
		other = info->control;

	/* only if not shared */
	if (other && intf != other) {
		usb_set_intfdata(other, NULL);
		usb_driver_release_interface(driver, other);
	}

	info->subdriver = NULL;
	info->data = NULL;
	info->control = NULL;
}

/* suspend/resume wrappers calling both usbnet and the cdc-wdm
 * subdriver if present.
 *
 * NOTE: cdc-wdm also supports pre/post_reset, but we cannot provide
 * wrappers for those without adding usbnet reset support first.
 */
static int qmi_wwan_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct usbnet *dev = usb_get_intfdata(intf);
	struct qmi_wwan_state *info = (void *)&dev->data;
	int ret;

	/* Both usbnet_suspend() and subdriver->suspend() MUST return 0
	 * in system sleep context, otherwise, the resume callback has
	 * to recover device from previous suspend failure.
	 */
	ret = usbnet_suspend(intf, message);
	if (ret < 0)
		goto err;

	if (intf == info->control && info->subdriver &&
	    info->subdriver->suspend)
		ret = info->subdriver->suspend(intf, message);
	if (ret < 0)
		usbnet_resume(intf);
err:
	return ret;
}

static int qmi_wwan_resume(struct usb_interface *intf)
{
	struct usbnet *dev = usb_get_intfdata(intf);
	struct qmi_wwan_state *info = (void *)&dev->data;
	int ret = 0;
	bool callsub = (intf == info->control && info->subdriver &&
			info->subdriver->resume);

	if (callsub)
		ret = info->subdriver->resume(intf);
	if (ret < 0)
		goto err;
	ret = usbnet_resume(intf);
	if (ret < 0 && callsub)
		info->subdriver->suspend(intf, PMSG_SUSPEND);
err:
	return ret;
}

static int qmi_wwan_reset_resume(struct usb_interface *intf)
{
	dev_info(&intf->dev, "device do not support reset_resume\n");
	intf->needs_binding = 1;
	return -EOPNOTSUPP;
}

static int rmnet_usb_bind(struct usbnet *dev, struct usb_interface *intf)
{	
	int status = qmi_wwan_bind(dev, intf);

	if (!status) {
		struct qmi_wwan_state *info = (void *)&dev->data;
		sQmiWwanQmap *pQmapDev = (sQmiWwanQmap *)info->unused;

		if (pQmapDev && pQmapDev->qmap_mode) {
			struct net_device *rmmet_usb = dev->net;
			
			pQmapDev->use_rmnet_usb = 1;
			pQmapDev->rmnet_info.size = sizeof(RMNET_INFO);
			pQmapDev->rmnet_info.rx_urb_size = (31*1024);
			pQmapDev->rmnet_info.ep_type = 2; //DATA_EP_TYPE_HSUSB
			pQmapDev->rmnet_info.iface_id = 4;
			pQmapDev->rmnet_info.qmap_mode = pQmapDev->qmap_mode;
			pQmapDev->rmnet_info.qmap_version = pQmapDev->qmap_version;
			pQmapDev->rmnet_info.dl_minimum_padding = 16;
			
			strcpy(rmmet_usb->name, "rmnet_usb%d");

#if 0
			rmmet_usb->header_ops = NULL; /* No header */
			rmmet_usb->type = ARPHRD_RAWIP;
			rmmet_usb->hard_header_len = 0;
			rmmet_usb->addr_len = 0;
#endif
			rmmet_usb->flags &= ~(IFF_BROADCAST | IFF_MULTICAST);
			rmmet_usb->flags |= (IFF_NOARP);
	
			dev->rx_urb_size = (31*1024);
			pQmapDev->qmap_size = dev->rx_urb_size;
		}
	}

	return status;
}

static struct sk_buff *rmnet_usb_tx_fixup(struct usbnet *dev, struct sk_buff *skb, gfp_t flags)
{
	//printk("%s skb=%p, len=%d, protocol=%x, hdr_len=%d\n", __func__, skb, skb->len, skb->protocol, skb->hdr_len);
	if (skb->protocol != htons(ETH_P_MAP)) {
		dev_kfree_skb_any(skb);
		return NULL;
	}

	return skb;
}

static int rmnet_usb_rx_fixup(struct usbnet *dev, struct sk_buff *skb)
{
	struct net_device	*net = dev->net;

	//printk("%s skb=%p, len=%d, protocol=%x, hdr_len=%d\n", __func__, skb, skb->len, skb->protocol, skb->hdr_len);
	if (net->type == ARPHRD_ETHER && skb_headroom(skb) >= ETH_HLEN) {
		//usbnet.c rx_process() usbnet_skb_return() eth_type_trans()
		skb_push(skb, ETH_HLEN);
		skb_reset_mac_header(skb);
		memcpy(eth_hdr(skb)->h_source, default_modem_addr, ETH_ALEN);
		memcpy(eth_hdr(skb)->h_dest, net->dev_addr, ETH_ALEN);
		eth_hdr(skb)->h_proto = htons(ETH_P_MAP);

		return 1;
	}
	
	return 0;
}

static void _rmnet_usb_rx_handler(struct usbnet *dev, struct sk_buff *skb_in)
{
	struct qmi_wwan_state *info = (void *)&dev->data;
	sQmiWwanQmap *pQmapDev = (sQmiWwanQmap *)info->unused;
	unsigned headroom = skb_headroom(skb_in);
	const unsigned need_headroot = 16;
	struct sk_buff *qmap_skb;
	struct sk_buff_head skb_chain;
	uint dl_minimum_padding = 0;

	if (pQmapDev->qmap_version == 9)
		dl_minimum_padding = pQmapDev->tx_ctx.dl_minimum_padding;

	__skb_queue_head_init(&skb_chain);

	while (skb_in->len > sizeof(struct qmap_hdr)) {
		struct rmnet_map_header *map_header = (struct rmnet_map_header *)skb_in->data;
		struct rmnet_map_v5_csum_header *ul_header = NULL;
		size_t hdr_size = sizeof(struct rmnet_map_header);	
		struct net_device *qmap_net;
		int pkt_len = ntohs(map_header->pkt_len);
		int skb_len;
		__be16 protocol;
		int mux_id;

		if (map_header->next_hdr) {
			ul_header = (struct rmnet_map_v5_csum_header *)(map_header + 1);
			hdr_size += sizeof(struct rmnet_map_v5_csum_header);
		}
			
		skb_len = pkt_len - (map_header->pad_len&0x3F);
		skb_len -= dl_minimum_padding;
		if (skb_len > 1500) {
			dev_info(&dev->net->dev, "drop skb_len=%x larger than 1500\n", skb_len);
			goto error_pkt;
		}

		if (skb_in->len < (pkt_len + hdr_size)) {
			dev_info(&dev->net->dev, "drop qmap unknow pkt, len=%d, pkt_len=%d\n", skb_in->len, pkt_len);
			goto error_pkt;
		}

		if (map_header->cd_bit) {
			dev_info(&dev->net->dev, "skip qmap command packet\n");
			goto skip_pkt;
		}

		switch (skb_in->data[hdr_size] & 0xf0) {
			case 0x40:
				protocol = htons(ETH_P_IP);
			break;
			case 0x60:
				protocol = htons(ETH_P_IPV6);
			break;
			default:
				dev_info(&dev->net->dev, "unknow skb->protocol %02x\n", skb_in->data[hdr_size]);
				goto error_pkt;
		}
		
		mux_id = map_header->mux_id - QUECTEL_QMAP_MUX_ID;
		if (mux_id >= pQmapDev->qmap_mode) {
			dev_info(&dev->net->dev, "drop qmap unknow mux_id %x\n", map_header->mux_id);
			goto error_pkt;
		}

		qmap_net = pQmapDev->mpQmapNetDev[mux_id];

		if (qmap_net == NULL) {
			dev_info(&dev->net->dev, "drop qmap unknow mux_id %x\n", map_header->mux_id);
			goto skip_pkt;
		}

		if (headroom >= need_headroot) {
			qmap_skb = skb_clone(skb_in, GFP_ATOMIC);
			if (qmap_skb) {
				qmap_skb->dev = qmap_net;
				skb_pull(qmap_skb, hdr_size);
				skb_trim(qmap_skb, skb_len);
			}
			headroom = (map_header->pad_len&0x3F);
			headroom += dl_minimum_padding;
		}
		else {	
			qmap_skb = netdev_alloc_skb(qmap_net, need_headroot + skb_len);
			if (qmap_skb) {
				skb_reserve(qmap_skb, need_headroot);
				skb_put(qmap_skb, skb_len);
				memcpy(qmap_skb->data, skb_in->data + hdr_size, skb_len);
			}
			headroom = pkt_len;
		}		

		if (qmap_skb == NULL) {
			dev_info(&dev->net->dev, "fail to alloc skb, pkt_len = %d\n", skb_len);
			return;
		}

		skb_reset_transport_header(qmap_skb);
		skb_reset_network_header(qmap_skb);
		qmap_skb->pkt_type = PACKET_HOST;
		skb_set_mac_header(qmap_skb, 0);
		qmap_skb->protocol = protocol;

		if (ul_header && ul_header->header_type == RMNET_MAP_HEADER_TYPE_CSUM_OFFLOAD
			&& ul_header->csum_valid_required) {
#if 0 //TODO
			qmap_skb->ip_summed = CHECKSUM_UNNECESSARY;
#endif
		}

		if (qmap_skb->dev->type == ARPHRD_ETHER) {
			skb_push(qmap_skb, ETH_HLEN);
			skb_reset_mac_header(qmap_skb);
			memcpy(eth_hdr(qmap_skb)->h_source, default_modem_addr, ETH_ALEN);
			memcpy(eth_hdr(qmap_skb)->h_dest, qmap_net->dev_addr, ETH_ALEN);
			eth_hdr(qmap_skb)->h_proto = protocol;
#ifdef QUECTEL_BRIDGE_MODE
			bridge_mode_rx_fixup(pQmapDev, qmap_net, qmap_skb);
#endif
			__skb_pull(qmap_skb, ETH_HLEN);
		}

		qmap_net->stats.rx_packets++;
		qmap_net->stats.rx_bytes += skb_len;		
		__skb_queue_tail(&skb_chain, qmap_skb);

skip_pkt:
		skb_pull(skb_in, pkt_len + hdr_size);
	}

error_pkt:
	while ((qmap_skb = __skb_dequeue (&skb_chain))) {
		netif_receive_skb(qmap_skb);
	}
}

static rx_handler_result_t rmnet_usb_rx_handler(struct sk_buff **pskb)
{
	struct sk_buff *skb = *pskb;
	struct usbnet *dev;

	if (!skb)
		goto done;

	//printk("%s skb=%p, protocol=%x, len=%d\n", __func__, skb, skb->protocol, skb->len);

	if (skb->pkt_type == PACKET_LOOPBACK)
		return RX_HANDLER_PASS;

	if (skb->protocol != htons(ETH_P_MAP)) {
		WARN_ON(1);
		return RX_HANDLER_PASS;
	}

	dev = rcu_dereference(skb->dev->rx_handler_data);

	if (dev == NULL) {
		WARN_ON(1);
		return RX_HANDLER_PASS;
	}

	_rmnet_usb_rx_handler(dev, skb);
	consume_skb(skb);

done:
	return RX_HANDLER_CONSUMED;
}

static const struct driver_info	qmi_wwan_info = {
	.description	= "WWAN/QMI device",
	.flags		= FLAG_WWAN,
	.bind		= qmi_wwan_bind,
	.unbind		= qmi_wwan_unbind,
	.manage_power	= qmi_wwan_manage_power,
	.rx_fixup       = qmi_wwan_rx_fixup,
};

static const struct driver_info qmi_wwan_raw_ip_info = {
	.description	= "WWAN/QMI device",
	.flags		= FLAG_WWAN | FLAG_RX_ASSEMBLE | FLAG_NOARP | FLAG_SEND_ZLP | FLAG_MULTI_PACKET,
	.bind		= qmi_wwan_bind,
	.unbind		= qmi_wwan_unbind,
	.manage_power	= qmi_wwan_manage_power,
#if defined(QUECTEL_WWAN_QMAP)
	.tx_fixup       = qmap_qmi_wwan_tx_fixup,
	.rx_fixup       = qmap_qmi_wwan_rx_fixup,
#else
	.tx_fixup       = qmi_wwan_tx_fixup,
	.rx_fixup       = qmi_wwan_rx_fixup,
#endif
};

static const struct driver_info rmnet_usb_info = {
	.description = "RMNET/USB device",
	.flags		=  FLAG_NOARP | FLAG_SEND_ZLP,
	.bind = rmnet_usb_bind,
	.unbind = qmi_wwan_unbind,
	.manage_power = qmi_wwan_manage_power,
	.tx_fixup = rmnet_usb_tx_fixup,
	.rx_fixup = rmnet_usb_rx_fixup,	
};

/* map QMI/wwan function by a fixed interface number */
#define QMI_FIXED_INTF(vend, prod, num) \
	USB_DEVICE_INTERFACE_NUMBER(vend, prod, num), \
	.driver_info = (unsigned long)&qmi_wwan_info

#define QMI_FIXED_RAWIP_INTF(vend, prod, num) \
	USB_DEVICE_INTERFACE_NUMBER(vend, prod, num), \
	.driver_info = (unsigned long)&qmi_wwan_raw_ip_info

#define RMNET_USB_INTF(vend, prod, num) \
		USB_DEVICE_INTERFACE_NUMBER(vend, prod, num), \
		.driver_info = (unsigned long) &rmnet_usb_info

static const struct usb_device_id products[] = {
#if 1 //Added by Quectel
	{ QMI_FIXED_INTF(0x05C6, 0x9003, 4) },  /* Quectel UC20 */
	{ QMI_FIXED_INTF(0x05C6, 0x9215, 4) },  /* Quectel EC20 (MDM9215) */
	{ QMI_FIXED_RAWIP_INTF(0x2C7C, 0x0125, 4) },  /* Quectel EC20 (MDM9X07)/EC25/EG25 */
	{ QMI_FIXED_RAWIP_INTF(0x2C7C, 0x0121, 4) },  /* Quectel EC21 */
	{ QMI_FIXED_RAWIP_INTF(0x2C7C, 0x0191, 4) },  /* Quectel EG91 */
	{ QMI_FIXED_RAWIP_INTF(0x2C7C, 0x0195, 4) },  /* Quectel EG95 */
	{ QMI_FIXED_RAWIP_INTF(0x2C7C, 0x0306, 4) },  /* Quectel EG06/EP06/EM06 */
	{ QMI_FIXED_RAWIP_INTF(0x2C7C, 0x0512, 4) },  /* Quectel EG12/EP12/EM12/EG16/EG18 */
	{ QMI_FIXED_RAWIP_INTF(0x2C7C, 0x0296, 4) },  /* Quectel BG96 */
	{ QMI_FIXED_RAWIP_INTF(0x2C7C, 0x0435, 4) },  /* Quectel AG35 */
	{ QMI_FIXED_RAWIP_INTF(0x2C7C, 0x0620, 4) },  /* Quectel EG20 */
	{ RMNET_USB_INTF(0x2C7C, 0x0800, 4) },  /* Quectel RG500 */
#endif
	{ }					/* END */
};
MODULE_DEVICE_TABLE(usb, products);

static int qmi_wwan_probe(struct usb_interface *intf,
			  const struct usb_device_id *prod)
{
	struct usb_device_id *id = (struct usb_device_id *)prod;

	/* Workaround to enable dynamic IDs.  This disables usbnet
	 * blacklisting functionality.  Which, if required, can be
	 * reimplemented here by using a magic "blacklist" value
	 * instead of 0 in the static device id table
	 */
	if (!id->driver_info) {
		dev_dbg(&intf->dev, "setting defaults for dynamic device id\n");
		id->driver_info = (unsigned long)&qmi_wwan_info;
	}

	if (intf->cur_altsetting->desc.bInterfaceClass != 0xff) {
		dev_info(&intf->dev,  "Quectel module not qmi_wwan mode! please check 'at+qcfg=\"usbnet\"'\n");
		return -ENODEV;
	}

	return usbnet_probe(intf, id);
}

#if defined(QUECTEL_WWAN_QMAP)
static int qmap_qmi_wwan_probe(struct usb_interface *intf,
			  const struct usb_device_id *prod)
{
	int status = qmi_wwan_probe(intf, prod);

	if (!status) {
		struct usbnet *dev = usb_get_intfdata(intf);
		struct qmi_wwan_state *info = (void *)&dev->data;
		sQmiWwanQmap *pQmapDev = (sQmiWwanQmap *)info->unused;
		unsigned i;

		if (pQmapDev) {
			if (pQmapDev->qmap_mode == 1) {
				pQmapDev->mpQmapNetDev[0] = dev->net;
				if (pQmapDev->use_rmnet_usb) {
					pQmapDev->mpQmapNetDev[0] = NULL;
					qmap_register_device(pQmapDev, 0);
				}
			}
			else if (pQmapDev->qmap_mode > 1) {
				for (i = 0; i < pQmapDev->qmap_mode; i++) {
					qmap_register_device(pQmapDev, i);
				}
			}

			if (pQmapDev->use_rmnet_usb) {
				rtnl_lock();
				netdev_rx_handler_register(dev->net, rmnet_usb_rx_handler, dev);
				rtnl_unlock();
			}

			if (pQmapDev->link_state == 0) {
				netif_carrier_off(dev->net);
			}
		}
	}

	return status;
}

static void qmap_qmi_wwan_disconnect(struct usb_interface *intf)
{
	struct usbnet *dev = usb_get_intfdata(intf);
	struct qmi_wwan_state *info;
	sQmiWwanQmap *pQmapDev;
	uint i;

	if (!dev)
		return;

	info = (void *)&dev->data;
	pQmapDev = (sQmiWwanQmap *)info->unused;

	if (!pQmapDev) {
		return usbnet_disconnect(intf);
	}

	pQmapDev->link_state = 0;

#if defined(QUECTEL_UL_DATA_AGG)
	if (pQmapDev->qmap_mode) {
		struct tx_agg_ctx *ctx = &pQmapDev->tx_ctx;

		if (hrtimer_active(&ctx->tx_timer))
			hrtimer_cancel(&ctx->tx_timer);
		tasklet_kill(&ctx->tx_bh);

		for (i = 0; i < ctx->tx_pending_count; i++) {
			dev_kfree_skb_any(ctx->tx_pending_skb[i]);
		}
	}
#endif

	if (pQmapDev->qmap_mode > 1) {
		for (i = 0; i < pQmapDev->qmap_mode; i++) {
			qmap_unregister_device(pQmapDev, i);
		}
	}

	if (pQmapDev->use_rmnet_usb) {
		qmap_unregister_device(pQmapDev, 0);
		rtnl_lock();
		netdev_rx_handler_unregister(dev->net);
		rtnl_unlock();
	}
	
	usbnet_disconnect(intf);
	info->unused = 0;
	kfree(pQmapDev);
}
#endif

static struct usb_driver qmi_wwan_driver = {
	.name		      = "qmi_wwan_q",
	.id_table	      = products,
	.probe		      = qmi_wwan_probe,
#if defined(QUECTEL_WWAN_QMAP)
	.probe		      = qmap_qmi_wwan_probe,
	.disconnect	      = qmap_qmi_wwan_disconnect,
#else
	.probe		      = qmi_wwan_probe,
	.disconnect	      = usbnet_disconnect,
#endif
	.suspend	      = qmi_wwan_suspend,
	.resume		      =	qmi_wwan_resume,
	.reset_resume         = qmi_wwan_reset_resume,
	.supports_autosuspend = 1,
	.disable_hub_initiated_lpm = 1,
};

#ifdef CONFIG_QCA_NSS_DRV
static uint qca_nss_enabled = 1;
module_param( qca_nss_enabled, uint, S_IRUGO);

/*
	EXTRA_CFLAGS="-I$(STAGING_DIR)/usr/include/qca-nss-drv  $(EXTRA_CFLAGS)"
	qsdk/qca/src/data-kernel/drivers/rmnet-nss/rmnet_nss.c 
*/
#include "rmnet_nss.c"
#endif

static int __init qmi_wwan_driver_init(void)
{
	RCU_INIT_POINTER(rmnet_nss_callbacks, NULL);
#ifdef CONFIG_QCA_NSS_DRV
	if (qca_nss_enabled)
		rmnet_nss_init();
#endif
	return usb_register(&qmi_wwan_driver);
}
module_init(qmi_wwan_driver_init);
static void __exit qmi_wwan_driver_exit(void)
{
#ifdef CONFIG_QCA_NSS_DRV
	if (qca_nss_enabled)
		rmnet_nss_exit();
#endif
	usb_deregister(&qmi_wwan_driver);
}
module_exit(qmi_wwan_driver_exit);

MODULE_AUTHOR("Bjørn Mork <bjorn@mork.no>");
MODULE_DESCRIPTION("Qualcomm MSM Interface (QMI) WWAN driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(QUECTEL_WWAN_VERSION);
