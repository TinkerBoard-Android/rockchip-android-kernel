/*
 * Analog Devices ADV7511 HDMI transmitter driver
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef __DRM_I2C_LT9211_H__
#define __DRM_I2C_LT9211_H__

#include <linux/hdmi.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_crtc_helper.h>
#include <drm/drm_mipi_dsi.h>

#include <video/of_videomode.h>
#include <video/videomode.h>

struct lt9211_data {
	//struct backlight_device *backlight;
	struct i2c_client *client;
	struct device *dev;

	unsigned int mipi_lanes;
	unsigned int lvds_format;;
	unsigned int lvds_bpp;
	unsigned int lvds_output;
	bool test_pattern_en;
	bool dual_link;
	bool enabled;
	bool debug;
	bool status;
	bool init;
	bool uboot;

	bool powered;

	struct gpio_desc *lvds_vdd_en_gpio;
	struct gpio_desc *lt9211_en_gpio;
	struct gpio_desc *lvds_hdmi_sel_gpio;
	struct gpio_desc *pwr_source_gpio;

	//enum drm_connector_status status;
	//struct drm_connector connector;
	//struct device_node *host_node;
	//struct drm_bridge bridge;

	//struct mipi_dsi_device *dsi;
	struct videomode  vm;

	//struct drm_display_mode curr_mode;
	unsigned int bus_format;
	unsigned int bpc;
	unsigned int format;
	unsigned int mode_flags;
	unsigned int t1;
};

//////////////////////LT9211 Config////////////////////////////////
#define _Mipi_PortA_
//#define _Mipi_PortB_

#define OUTPUT_LVDS_1_PORT		(1 << 0)
#define OUTPUT_LVDS_2_PORT		(1 << 1)
#define OUTPUT_FORMAT_VESA		(1 << 2)
#define OUTPUT_FORMAT_JEIDA		(1 << 3)
#define OUTPUT_DE_MODE			(1 << 4)
#define OUTPUT_SYNC_MODE		(1 << 5)
#define OUTPUT_BITDEPTH_666		(1 << 6)
#define OUTPUT_BITDEPTH_888		(1 << 7)
#define OUTPUT_MIPI_1_LANE		(1 << 8)
#define OUTPUT_MIPI_2_LANE		(1 << 9)
#define OUTPUT_MIPI_3_LANE		(1 << 10)
#define OUTPUT_MIPI_4_LANE		(1 << 11)
#define MIPI_SETTLE_VALUE 0x05
#define PCR_M_VALUE 0x17
#define Debug 0

struct video_timing{
	uint16_t hfp;
	uint16_t hs;
	uint16_t hbp;
	uint16_t hact;
	uint16_t htotal;
	uint16_t vfp;
	uint16_t vs;
	uint16_t vbp;
	uint16_t vact;
	uint16_t vtotal;
	uint32_t pclk_khz;
};

typedef enum  _MIPI_LANE_NUMBER
{	
	MIPI_1_LANE = 1,
	MIPI_2_LANE = 2,
	MIPI_3_LANE = 3,	
	MIPI_4_LANE = 0   //default 4Lane
} MIPI_LANE_NUMBER__TypeDef;

typedef enum  _REG8235_PIXCK_DIVSEL   ////dessc pll to generate pixel clk
{	
//[1:0]PIXCK_DIVSEL 
//00 176M~352M
//01 88M~176M
//10 44M~88M
//11 22M~44M	
	PIXCLK_LARGER_THAN_176M  = 0x80,
	PIXCLK_88M_176M          = 0x81,
	PIXCLK_44M_88M           = 0x82,	
	PIXCLK_22M_44M           = 0x83   //default 4Lane
} REG8235_PIXCK_DIVSEL_TypeDef;

#endif /* __DRM_I2C_LT9211_H__ */
