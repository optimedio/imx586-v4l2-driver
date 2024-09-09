// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony imx586 cameras.
 *
 * Based on Sony imx477 camera driver
 * Copyright (C) 2019-2020 Raspberry Pi (Trading) Ltd
 * Modified by Will WHANG
 * Modified by sohonomura2020 in Soho Enterprise Ltd.
 * Modified by OCTOPUSCINEMA
 * Copyright (C) 2024 OCTOPUS CINEMA
 */
#include <asm/unaligned.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>


static bool monochrome_mode;
module_param(monochrome_mode, bool, 0644);
MODULE_PARM_DESC(monochrome_mode, "Set for monochrome sensor: 1=mono, 0=color");


// Support for rpi kernel pre git commit 314a685
#ifndef MEDIA_BUS_FMT_SENSOR_DATA
#define MEDIA_BUS_FMT_SENSOR_DATA 		0x7002
#endif

/* Chip ID */
#define IMX586_REG_CHIP_ID				0x30DC
#define IMX586_CHIP_ID					0x32

/* Standby or streaming mode */
#define IMX586_REG_MODE_SELECT			0x3000
#define IMX586_MODE_STANDBY				0x01
#define IMX586_MODE_STREAMING			0x00
#define IMX586_STREAM_DELAY_US			25000
#define IMX586_STREAM_DELAY_RANGE_US	1000

/* In clk */
#define IMX586_XCLK_FREQ				24000000

/* VMAX internal VBLANK*/
#define IMX586_REG_VMAX					0x3028
#define IMX586_VMAX_MAX					0xfffff

/* HMAX internal HBLANK*/
#define IMX586_REG_HMAX					0x302C
#define IMX586_HMAX_MAX					0xffff

/* SHR internal */
#define IMX586_REG_SHR					0x3050
#define IMX586_SHR_MIN					11

/* Exposure control */
#define IMX586_EXPOSURE_MIN				52
#define IMX586_EXPOSURE_STEP			1
#define IMX586_EXPOSURE_DEFAULT			1000
#define IMX586_EXPOSURE_MAX				49865

/* HDR threshold */
#define IMX586_REG_EXP_TH_H				0x36D0
#define IMX586_REG_EXP_TH_L				0x36D4
#define IMX586_REG_EXP_BK				0x36E2

/* Gradation compression control */
#define IMX586_REG_CCMP1_EXP			0x36E8
#define IMX586_REG_CCMP2_EXP			0x36E4
#define IMX586_REG_ACMP1_EXP			0x36EE
#define IMX586_REG_ACMP2_EXP			0x36EC

/* Black level control */
#define IMX586_REG_BLKLEVEL				0x30DC
#define IMX586_BLKLEVEL_DEFAULT			0

/* Digital Clamp */
#define IMX586_REG_DIGITAL_CLAMP		0x3458

/* Analog gain control */
#define IMX586_REG_ANALOG_GAIN			0x306C
#define IMX586_REG_FDG_SEL0				0x3030
#define IMX586_ANA_GAIN_MIN				0
#define IMX586_ANA_GAIN_MAX				240 // x3980= 72db = 0.3db x 240
#define IMX586_ANA_GAIN_STEP			1
#define IMX586_ANA_GAIN_DEFAULT			0
#define IMX586_ANA_GAIN_HCG_LEVEL		51 // = 15.3db / 0.3db
#define IMX586_ANA_GAIN_HCG_THRESHOLD	(IMX586_ANA_GAIN_HCG_LEVEL+29)
#define IMX586_ANA_GAIN_HCG_MIN			34

/* Flip */
#define IMX586_FLIP_WINMODEH    		0x3020
#define IMX586_FLIP_WINMODEV    		0x3021

/* Embedded metadata stream structure */
#define IMX586_EMBEDDED_LINE_WIDTH 		16384
#define IMX586_NUM_EMBEDDED_LINES 		1

#define IMX586_PIXEL_RATE				74250000

enum pad_types {
	IMAGE_PAD,
	METADATA_PAD,
	NUM_PADS
};


/* Gradation compression */
enum v4l2_xfer_func_sony {
	V4L2_XFER_FUNC_GRADATION_COMPRESSION = 10
};

/* imx586 native and active pixel array size. */
#define IMX586_NATIVE_WIDTH			3856U
#define IMX586_NATIVE_HEIGHT		2180U
#define IMX586_PIXEL_ARRAY_LEFT		8U
#define IMX586_PIXEL_ARRAY_TOP		8U
#define IMX586_PIXEL_ARRAY_WIDTH	3840U
#define IMX586_PIXEL_ARRAY_HEIGHT	2160U

struct imx586_reg {
	u16 address;
	u8 val;
};

struct IMX586_reg_list {
	unsigned int num_of_regs;
	const struct imx586_reg *regs;
};

/* Mode : resolution and related config&values */
struct imx586_mode {
	/* Frame width */
	unsigned int width;

	/* Frame height */
	unsigned int height;

	/* mode uses Clear HDR */
	bool hdr;

	/* mode has linear output (gradation compression disabled) */
	bool linear;

	/* minimum H-timing */
	uint64_t min_HMAX;

	/* minimum V-timing */
	uint64_t min_VMAX;

	/* default H-timing */
	uint64_t default_HMAX;

	/* default V-timing */
	uint64_t default_VMAX;

	/* minimum SHR */
	uint64_t min_SHR;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Default register values */
	struct IMX586_reg_list reg_list;
};

/* Common Modes */
static const struct imx586_reg mode_common_regs[] = {
    {0x3002, 0x01},
    {0x301A, 0x00}, //WDMODE Normal mode
    //{0x301A, 0x10}, //WDMODE Clear HDR
    {0x301B, 0x00}, //ADDMODE 0x00 non-binning
    {0x3024, 0x00}, // COMBI_EN 
    // {0x3024, 0x02}, // COMBI_EN 0x02=Clear HDR mode
    {0x3069, 0x00},
    //{0x3069, 0x02}, // for Clear HDR mode
    {0x3074, 0x64},
	{0x30D5, 0x04}, // DIG_CLP_VSTART
    // {0x3074, 0x63}, // for Clear HDR
    {0x3930, 0x0c},//DUR normal mode 12bit
    {0x3931, 0x01},//DUR normal mode 12bit
    // {0x3930, 0xE6},//DUR Clear HDR 12bit
    // {0x3931, 0x00},//DUR Clear HDR 12bit
    {0x3A4C, 0x39},// WAIT_ST0 Normal
    {0x3A4D, 0x01},//  Normal
    {0x3A50, 0x48},// WAIT_ST1 Normal
    {0x3A51, 0x01},//  Normal
    // {0x3A4C, 0x61},// WAIT_ST0
    // {0x3A4D, 0x02},// 
    // {0x3A50, 0x70},// WAIT_ST1
    // {0x3A51, 0x02},// 
    {0x3E10, 0x10},// ADTHEN Normal
    // {0x3E10, 0x17},// ADTHEN
    {0x493C, 0x23},// ADTHEN
    {0x4940, 0x41},// ADTHEN
    // {0x493C, 0x41},// ADTHEN
    // {0x4940, 0x41},// ADTHEN



    {0x3014, 0x04},// INCK_SEL [3:0] 24 MHz
    {0x3015, 0x02},// DATARATE_SEL [3:0]  1782 Mbps
    // {0x302C, 0x4C},// HMAX [15:0]
    // {0x302D, 0x04},// 
    {0x3030, 0x00},// FDG_SEL0 LCG, HCG:0x01
    {0x3040, 0x03},// LANEMODE [2:0] 4 lane
    {0x3023, 0x01},// MDBIT 12-bit
    // {0x3028, 0x94},// VMAX
    // {0x3029, 0x11},// VMAX
    // {0x302A, 0x00},// VMAX
    // {0x3050, 0xFF},// SHR0 [19:0]
    {0x30A6, 0x00},// XVS_DRV [1:0] Hi-Z
	{0x3081, 0x00},// EXP_GAIN, Reset to 0
    {0x3460, 0x21},// -
    {0x3478, 0xA1},// -
    {0x347C, 0x01},// -
    {0x3480, 0x01},// -
    {0x3A4E, 0x14},// -
    {0x3A52, 0x14},// -
    {0x3A56, 0x00},// -
    {0x3A5A, 0x00},// -
    {0x3A5E, 0x00},// -
    {0x3A62, 0x00},// -
    {0x3A6A, 0x20},// -
    {0x3A6C, 0x42},// -
    {0x3A6E, 0xA0},// -
    {0x3B2C, 0x0C},// -
    {0x3B30, 0x1C},// -
    {0x3B34, 0x0C},// -
    {0x3B38, 0x1C},// -
    {0x3BA0, 0x0C},// -
    {0x3BA4, 0x1C},// -
    {0x3BA8, 0x0C},// -
    {0x3BAC, 0x1C},// -
    {0x3D3C, 0x11},// -
    {0x3D46, 0x0B},// -
    {0x3DE0, 0x3F},// -
    {0x3DE1, 0x08},// -
    {0x3E14, 0x87},// -
    {0x3E16, 0x91},// -
    {0x3E18, 0x91},// -
    {0x3E1A, 0x87},// -
    {0x3E1C, 0x78},// -
    {0x3E1E, 0x50},// -
    {0x3E20, 0x50},// -
    {0x3E22, 0x50},// -
    {0x3E24, 0x87},// -
    {0x3E26, 0x91},// -
    {0x3E28, 0x91},// -
    {0x3E2A, 0x87},// -
    {0x3E2C, 0x78},// -
    {0x3E2E, 0x50},// -
    {0x3E30, 0x50},// -
    {0x3E32, 0x50},// -
    {0x3E34, 0x87},// -
    {0x3E36, 0x91},// -
    {0x3E38, 0x91},// -
    {0x3E3A, 0x87},// -
    {0x3E3C, 0x78},// -
    {0x3E3E, 0x50},// -
    {0x3E40, 0x50},// -
    {0x3E42, 0x50},// -
    {0x4054, 0x64},// -
    {0x4148, 0xFE},// -
    {0x4149, 0x05},// -
    {0x414A, 0xFF},// -
    {0x414B, 0x05},// -
    {0x420A, 0x03},// -
    {0x4231, 0x08},// -
    {0x423D, 0x9C},// -
    {0x4242, 0xB4},// -
    {0x4246, 0xB4},// -
    {0x424E, 0xB4},// -
    {0x425C, 0xB4},// -
    {0x425E, 0xB6},// -
    {0x426C, 0xB4},// -
    {0x426E, 0xB6},// -
    {0x428C, 0xB4},// -
    {0x428E, 0xB6},// -
    {0x4708, 0x00},// -
    {0x4709, 0x00},// -
    {0x470A, 0xFF},// -
    {0x470B, 0x03},// -
    {0x470C, 0x00},// -
    {0x470D, 0x00},// -
    {0x470E, 0xFF},// -
    {0x470F, 0x03},// -
    {0x47EB, 0x1C},// -
    {0x47F0, 0xA6},// -
    {0x47F2, 0xA6},// -
    {0x47F4, 0xA0},// -
    {0x47F6, 0x96},// -
    {0x4808, 0xA6},// -
    {0x480A, 0xA6},// -
    {0x480C, 0xA0},// -
    {0x480E, 0x96},// -
    {0x492C, 0xB2},// -
    {0x4930, 0x03},// -
    {0x4932, 0x03},// -
    {0x4936, 0x5B},// -
    {0x4938, 0x82},// -
    {0x493E, 0x23},// -
    {0x4BA8, 0x1C},// -
    {0x4BA9, 0x03},// -
    {0x4BAC, 0x1C},// -
    {0x4BAD, 0x1C},// -
    {0x4BAE, 0x1C},// -
    {0x4BAF, 0x1C},// -
    {0x4BB0, 0x1C},// -
    {0x4BB1, 0x1C},// -
    {0x4BB2, 0x1C},// -
    {0x4BB3, 0x1C},// -
    {0x4BB4, 0x1C},// -
    {0x4BB8, 0x03},// -
    {0x4BB9, 0x03},// -
    {0x4BBA, 0x03},// -
    {0x4BBB, 0x03},// -
    {0x4BBC, 0x03},// -
    {0x4BBD, 0x03},// -
    {0x4BBE, 0x03},// -
    {0x4BBF, 0x03},// -
    {0x4BC0, 0x03},// -
    {0x4C14, 0x87},// -
    {0x4C16, 0x91},// -
    {0x4C18, 0x91},// -
    {0x4C1A, 0x87},// -
    {0x4C1C, 0x78},// -
    {0x4C1E, 0x50},// -
    {0x4C20, 0x50},// -
    {0x4C22, 0x50},// -
    {0x4C24, 0x87},// -
    {0x4C26, 0x91},// -
    {0x4C28, 0x91},// -
    {0x4C2A, 0x87},// -
    {0x4C2C, 0x78},// -
    {0x4C2E, 0x50},// -
    {0x4C30, 0x50},// -
    {0x4C32, 0x50},// -
    {0x4C34, 0x87},// -
    {0x4C36, 0x91},// -
    {0x4C38, 0x91},// -
    {0x4C3A, 0x87},// -
    {0x4C3C, 0x78},// -
    {0x4C3E, 0x50},// -
    {0x4C40, 0x50},// -
    {0x4C42, 0x50},// -
    {0x4D12, 0x1F},// -
    {0x4D13, 0x1E},// -
    {0x4D26, 0x33},// -
    {0x4E0E, 0x59},// -
    {0x4E14, 0x55},// -
    {0x4E16, 0x59},// -
    {0x4E1E, 0x3B},// -
    {0x4E20, 0x47},// -
    {0x4E22, 0x54},// -
    {0x4E26, 0x81},// -
    {0x4E2C, 0x7D},// -
    {0x4E2E, 0x81},// -
    {0x4E36, 0x63},// -
    {0x4E38, 0x6F},// -
    {0x4E3A, 0x7C},// -
    {0x4F3A, 0x3C},// -
    {0x4F3C, 0x46},// -
    {0x4F3E, 0x59},// -
    {0x4F42, 0x64},// -
    {0x4F44, 0x6E},// -
    {0x4F46, 0x81},// -
    {0x4F4A, 0x82},// -
    {0x4F5A, 0x81},// -
    {0x4F62, 0xAA},// -
    {0x4F72, 0xA9},// -
    {0x4F78, 0x36},// -
    {0x4F7A, 0x41},// -
    {0x4F7C, 0x61},// -
    {0x4F7D, 0x01},// -
    {0x4F7E, 0x7C},// -
    {0x4F7F, 0x01},// -
    {0x4F80, 0x77},// -
    {0x4F82, 0x7B},// -
    {0x4F88, 0x37},// -
    {0x4F8A, 0x40},// -
    {0x4F8C, 0x62},// -
    {0x4F8D, 0x01},// -
    {0x4F8E, 0x76},// -
    {0x4F8F, 0x01},// -
    {0x4F90, 0x5E},// -
    {0x4F91, 0x02},// -
    {0x4F92, 0x69},// -
    {0x4F93, 0x02},// -
    {0x4F94, 0x89},// -
    {0x4F95, 0x02},// -
    {0x4F96, 0xA4},// -
    {0x4F97, 0x02},// -
    {0x4F98, 0x9F},// -
    {0x4F99, 0x02},// -
    {0x4F9A, 0xA3},// -
    {0x4F9B, 0x02},// -
    {0x4FA0, 0x5F},// -
    {0x4FA1, 0x02},// -
    {0x4FA2, 0x68},// -
    {0x4FA3, 0x02},// -
    {0x4FA4, 0x8A},// -
    {0x4FA5, 0x02},// -
    {0x4FA6, 0x9E},// -
    {0x4FA7, 0x02},// -
    {0x519E, 0x79},// -
    {0x51A6, 0xA1},// -
    {0x51F0, 0xAC},// -
    {0x51F2, 0xAA},// -
    {0x51F4, 0xA5},// -
    {0x51F6, 0xA0},// -
    {0x5200, 0x9B},// -
    {0x5202, 0x91},// -
    {0x5204, 0x87},// -
    {0x5206, 0x82},// -
    {0x5208, 0xAC},// -
    {0x520A, 0xAA},// -
    {0x520C, 0xA5},// -
    {0x520E, 0xA0},// -
    {0x5210, 0x9B},// -
    {0x5212, 0x91},// -
    {0x5214, 0x87},// -
    {0x5216, 0x82},// -
    {0x5218, 0xAC},// -
    {0x521A, 0xAA},// -
    {0x521C, 0xA5},// -
    {0x521E, 0xA0},// -
    {0x5220, 0x9B},// -
    {0x5222, 0x91},// -
    {0x5224, 0x87},// -
    {0x5226, 0x82},// -
    {0x3002, 0x00}, // Master mode start
};

/* All pixel 4K60. 12-bit (Normal) */
static const struct imx586_reg mode_4k_regs[] = {
	{0x301A, 0x00}, // WDMODE Normal mode
	{0x301B, 0x00}, // ADDMODE non-binning
	{0x3022, 0x02}, // ADBIT 12-bit
	{0x3023, 0x01}, // MDBIT 12-bit
	{0x3024, 0x00}, // COMBI_EN no HDR combining
	{0x36EF, 0x00}, // CCMP_EN Linear
    {0x3069, 0x00}, // Normal mode
	
	{0x3074, 0x64}, // Normal mode
	{0x30D5, 0x04}, // DIG_CLP_VSTART non-binning
    {0x3930, 0x0c}, // DUR normal mode 12bit
    {0x3931, 0x01}, // DUR normal mode 12bit
    {0x3A4C, 0x39}, // WAIT_ST0 Normal mode
    {0x3A4D, 0x01}, // Normal mode
    {0x3A50, 0x48}, // WAIT_ST1 Normal mode
    {0x3A51, 0x01}, // Normal mode
    {0x3E10, 0x10}, // ADTHEN Normal mode
    {0x493C, 0x23}, // ADTHEN Normal mode
    {0x4940, 0x41}, // ADTHEN Normal mode
};

/* 2x2 binned 1080p60. 12-bit (Normal) */
static const struct imx586_reg mode_1080_regs[] = {
	{0x301A, 0x00}, // WDMODE Normal mode
	{0x301B, 0x01}, // ADDMODE binning
	{0x3022, 0x00}, // ADBIT 10-bit
	{0x3023, 0x01}, // MDBIT 12-bit
	{0x3024, 0x00}, // COMBI_EN no HDR combining
	{0x36EF, 0x00}, // CCMP_EN Linear
	{0x3069, 0x00}, // Normal mode
	
	{0x3074, 0x64}, // Normal mode
	{0x30D5, 0x02}, // DIG_CLP_VSTART binning
    {0x3930, 0x0c}, // DUR normal mode 12bit
    {0x3931, 0x01}, // DUR normal mode 12bit
    {0x3A4C, 0x39}, // WAIT_ST0 Normal mode
    {0x3A4D, 0x01}, // Normal mode
    {0x3A50, 0x48}, // WAIT_ST1 Normal mode
    {0x3A51, 0x01}, // Normal mode
    {0x3E10, 0x10}, // ADTHEN Normal mode
    {0x493C, 0x23}, // ADTHEN Normal mode
    {0x4940, 0x41}, // ADTHEN Normal mode
};

/* All pixel 4K30. 12-bit (HDR gradation compression) */
static const struct imx586_reg mode_4k_nonlinear_regs[] = {
    {0x301A, 0x10}, // WDMODE Clear HDR
    {0x301B, 0x00}, // ADDMODE Non-binning
	
	{0x3022, 0x02}, // ADBIT 12-bit
    {0x3023, 0x01}, // MDBIT 12-bit
    {0x3024, 0x02}, // COMBI_EN 

	{0x36EF, 0x01}, // CCMP_EN Non-linear gradation compression
	
    {0x3030, 0x00}, // FDG_SEL0 LCG, HCG:0x01
	
    {0x3069, 0x02}, // for Clear HDR mode
    {0x3074, 0x63}, // for Clear HDR
    {0x3081, 0x02}, // EXP_GAIN, Clear HDR high gain setting, +12dB
    
    {0x30D5, 0x02}, // DIG_CLP_VSTART Non-binning
	
    {0x3930, 0xE6}, // DUR Clear HDR 12bit
    {0x3931, 0x00}, // DUR Clear HDR 12bit
    
    {0x3A4C, 0x61}, // WAIT_ST0 Clear HDR mode
    {0x3A4D, 0x02}, // Clear HDR mode
    {0x3A50, 0x70}, // WAIT_ST1
    {0x3A51, 0x02}, // Clear HDR mode
    
    {0x3E10, 0x17}, // ADTHEN Clear HDR
    {0x493C, 0x41}, // WAIT_10_SHF Clear HDR 10-bit 0x0C disable
    {0x4940, 0x41}, // WAIT_12_SHF Clear HDR 12-bit 0x41 enable
};

/* All pixel 4K30. 16-bit (Clear HDR) */
static const struct imx586_reg mode_4k_16bit_regs[] = {
    {0x301A, 0x10}, // WDMODE Clear HDR
    {0x301B, 0x00}, // ADDMODE Non-binning
	
	{0x3022, 0x02}, // ADBIT 12-bit
    {0x3023, 0x03}, // MDBIT 16-bit
    {0x3024, 0x02}, // COMBI_EN 

	{0x36EF, 0x00}, // CCMP_EN Linear
	
    {0x3030, 0x00}, // FDG_SEL0 LCG, HCG:0x01
	
    {0x3069, 0x02}, // for Clear HDR mode
    {0x3074, 0x63}, // for Clear HDR
    {0x3081, 0x02}, // EXP_GAIN, Clear HDR high gain setting, +12dB
    
    {0x30D5, 0x02}, // DIG_CLP_VSTART Non-binning
	
    {0x3930, 0xE6}, // DUR Clear HDR 12bit
    {0x3931, 0x00}, // DUR Clear HDR 12bit
    
    {0x3A4C, 0x61}, // WAIT_ST0 Clear HDR mode
    {0x3A4D, 0x02}, // Clear HDR mode
    {0x3A50, 0x70}, // WAIT_ST1
    {0x3A51, 0x02}, // Clear HDR mode
    
    {0x3E10, 0x17}, // ADTHEN Clear HDR
    {0x493C, 0x41}, // WAIT_10_SHF Clear HDR 10-bit 0x0C disable
    {0x4940, 0x41}, // WAIT_12_SHF Clear HDR 12-bit 0x41 enable
};

/* 2x2 binned 1080p30. 16-bit (Clear HDR) */
static const struct imx586_reg mode_1080_16bit_regs[] = {
    {0x301A, 0x10}, // WDMODE Clear HDR
    {0x301B, 0x01}, // ADDMODE Binning
	
	{0x3022, 0x02}, // ADBIT 12-bit
    {0x3023, 0x03}, // MDBIT 16-bit
    {0x3024, 0x02}, // COMBI_EN Built-in HDR combining

	{0x36EF, 0x00}, // CCMP_EN Linear
	
    {0x3030, 0x00}, // FDG_SEL0 LCG, HCG:0x01
	
    {0x3069, 0x02}, // for Clear HDR mode
    {0x3074, 0x63}, // for Clear HDR
    {0x3081, 0x02}, // EXP_GAIN, Clear HDR high gain setting, +12dB
    
    {0x30D5, 0x02}, // DIG_CLP_VSTART
	
    {0x3930, 0xE6}, // DUR Clear HDR 12bit
    {0x3931, 0x00}, // DUR Clear HDR 12bit
    
    {0x3A4C, 0x61}, // WAIT_ST0
    {0x3A4D, 0x02}, // Clear HDR mode
    {0x3A50, 0x70}, // WAIT_ST1
    {0x3A51, 0x02}, // Clear HDR mode

    {0x3E10, 0x17}, // ADTHEN Clear HDR
    {0x493C, 0x41}, // WAIT_10_SHF Clear HDR 10-bit 0x0C disable
    {0x4940, 0x41}, // WAIT_12_SHF Clear HDR 12-bit 0x41 enable
};

/* Mode configs */
static const struct imx586_mode supported_modes_12bit[] = {
	{
		/* 4K60 All pixel */
		.width = 3856,
		.height = 2180,
		.hdr = false,
		.linear = true,
		.min_HMAX = 550,
		.min_VMAX = 2250,
		.default_HMAX = 550,
		.default_VMAX = 2250,
		.min_SHR = 20,
		.crop = {
			.left = IMX586_PIXEL_ARRAY_LEFT,
			.top = IMX586_PIXEL_ARRAY_TOP,
			.width = IMX586_PIXEL_ARRAY_WIDTH,
			.height = IMX586_PIXEL_ARRAY_HEIGHT,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_4k_regs),
			.regs = mode_4k_regs,
		},
	},
	{
		/* 1080p90 2x2 binning */
		.width = 1928,
		.height = 1090,
		.hdr = false,
		.linear = true,
		.min_HMAX = 366,
		.min_VMAX = 2250,
		.default_HMAX = 366,
		.default_VMAX = 2250,
		.min_SHR = 20,
		.crop = {
			.left = IMX586_PIXEL_ARRAY_LEFT,
			.top = IMX586_PIXEL_ARRAY_TOP,
			.width = IMX586_PIXEL_ARRAY_WIDTH,
			.height = IMX586_PIXEL_ARRAY_HEIGHT,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1080_regs),
			.regs = mode_1080_regs,
		},
	},
};

static const struct imx586_mode supported_modes_nonlinear_12bit[] = {
	{
		/* 4K30 All pixel */
		.width = 3856,
		.height = 2180,
		.hdr = true,
		.linear = false,
		//.min_HMAX = 760,
		.min_HMAX = 550, // Clear HDR original
		//.min_VMAX = 2250,
		.min_VMAX = 4500, // Clear HDR original
		.default_HMAX = 550,
		.default_VMAX = 4500,
		// .default_HMAX = 550,
		// .default_VMAX = 4500,
		.min_SHR = 20,
		.crop = {
			.left = IMX586_PIXEL_ARRAY_LEFT,
			.top = IMX586_PIXEL_ARRAY_TOP,
			.width = IMX586_PIXEL_ARRAY_WIDTH,
			.height = IMX586_PIXEL_ARRAY_HEIGHT,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_4k_nonlinear_regs),
			.regs = mode_4k_nonlinear_regs,
		},
	},
};

static const struct imx586_mode supported_modes_16bit[] = {
	{
		/* 1080p30 2x2 binning */
		.width = 1928,
		.height = 1090,
		.hdr = true,
		.linear = true,
		//.min_HMAX = 760,
		.min_HMAX = 550, // Clear HDR original
		//.min_VMAX = 2250,
		.min_VMAX = 4500, // Clear HDR original
		.default_HMAX = 550,
		.default_VMAX = 4500,
		// .default_HMAX = 550,
		// .default_VMAX = 4500,
		.min_SHR = 20,
		.crop = {
			.left = IMX586_PIXEL_ARRAY_LEFT,
			.top = IMX586_PIXEL_ARRAY_TOP,
			.width = IMX586_PIXEL_ARRAY_WIDTH,
			.height = IMX586_PIXEL_ARRAY_HEIGHT,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1080_16bit_regs),
			.regs = mode_1080_16bit_regs,
		},
	},
	{
		/* 4K30 All pixel */
		.width = 3856,
		.height = 2180,
		.hdr = true,
		.linear = true,
		//.min_HMAX = 760,
		.min_HMAX = 550, // Clear HDR original
		//.min_VMAX = 2250,
		.min_VMAX = 4500, // Clear HDR original
		.default_HMAX = 550,
		.default_VMAX = 4500,
		// .default_HMAX = 550,
		// .default_VMAX = 4500,
		.min_SHR = 20,
		.crop = {
			.left = IMX586_PIXEL_ARRAY_LEFT,
			.top = IMX586_PIXEL_ARRAY_TOP,
			.width = IMX586_PIXEL_ARRAY_WIDTH,
			.height = IMX586_PIXEL_ARRAY_HEIGHT,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_4k_16bit_regs),
			.regs = mode_4k_16bit_regs,
		},
	},
};

/*
 * The supported formats.
 * This table MUST contain 4 entries per format, to cover the various flip
 * combinations in the order
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 */
static const u32 codes[] = {
	/* 16-bit modes. */
	MEDIA_BUS_FMT_SRGGB16_1X16,
	MEDIA_BUS_FMT_SGRBG16_1X16,
	MEDIA_BUS_FMT_SGBRG16_1X16,
	MEDIA_BUS_FMT_SBGGR16_1X16,
	/* 12-bit modes. */
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SBGGR12_1X12,
};


static const u32 mono_codes[] = {
    /* 16-bit modes. */
    MEDIA_BUS_FMT_Y16_1X16,
    /* 12-bit modes. */
    MEDIA_BUS_FMT_Y12_1X12,
};


/* regulator supplies */
static const char * const imx586_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (3.3V) supply */
	"VDIG",  /* Digital Core (1.1V) supply */
	"VDDL",  /* IF (1.8V) supply */
};

#define imx586_NUM_SUPPLIES ARRAY_SIZE(imx586_supply_name)

/*
 * Initialisation delay between XCLR low->high and the moment when the sensor
 * can start capture (i.e. can leave software standby)
 */
#define imx586_XCLR_MIN_DELAY_US	500000
#define imx586_XCLR_DELAY_RANGE_US	1000

struct imx586_compatible_data {
	unsigned int chip_id;
	struct IMX586_reg_list extra_regs;
};

struct imx586 {
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	unsigned int fmt_code;

	struct clk *xclk;
	u32 xclk_freq;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[imx586_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	/* Current mode */
	const struct imx586_mode *mode;

    /* Mono mode */
    bool mono;

	uint16_t HMAX;
	uint32_t VMAX;
	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;

	/* Rewrite common registers on stream on? */
	bool common_regs_written;

	/* Any extra information related to different compatible sensors */
	const struct imx586_compatible_data *compatible_data;
};

static inline struct imx586 *to_imx586(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx586, sd);
}

static inline void get_mode_table(struct imx586 *imx586, unsigned int code, enum v4l2_xfer_func transfer_function,
				  const struct imx586_mode **mode_list,
				  unsigned int *num_modes)
{

    struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);
    

    if(imx586->mono){
        switch (code) {
        case MEDIA_BUS_FMT_Y16_1X16:
            *mode_list = supported_modes_16bit;
            *num_modes = ARRAY_SIZE(supported_modes_16bit);
            break;
        case MEDIA_BUS_FMT_Y12_1X12:
            if ( transfer_function == (enum v4l2_xfer_func)V4L2_XFER_FUNC_GRADATION_COMPRESSION ) {
                *mode_list = supported_modes_nonlinear_12bit;
                *num_modes = ARRAY_SIZE(supported_modes_nonlinear_12bit);
            } else {
                *mode_list = supported_modes_12bit;
                *num_modes = ARRAY_SIZE(supported_modes_12bit);
            }
            break;
        default:
            *mode_list = NULL;
            *num_modes = 0;
        }
    }
    else{
        switch (code) {
        /* 16-bit */
        case MEDIA_BUS_FMT_SRGGB16_1X16:
        case MEDIA_BUS_FMT_SGRBG16_1X16:
        case MEDIA_BUS_FMT_SGBRG16_1X16:
        case MEDIA_BUS_FMT_SBGGR16_1X16:
            *mode_list = supported_modes_16bit;
            *num_modes = ARRAY_SIZE(supported_modes_16bit);
            break;
        /* 12-bit */
        case MEDIA_BUS_FMT_SRGGB12_1X12:
        case MEDIA_BUS_FMT_SGRBG12_1X12:
        case MEDIA_BUS_FMT_SGBRG12_1X12:
        case MEDIA_BUS_FMT_SBGGR12_1X12:
            if ( transfer_function == (enum v4l2_xfer_func)V4L2_XFER_FUNC_GRADATION_COMPRESSION ) {
                *mode_list = supported_modes_nonlinear_12bit;
                *num_modes = ARRAY_SIZE(supported_modes_nonlinear_12bit);
            } else {
                *mode_list = supported_modes_12bit;
                *num_modes = ARRAY_SIZE(supported_modes_12bit);
            }
            break;
        default:
            *mode_list = NULL;
            *num_modes = 0;
        }
    }
}

/* Read registers up to 2 at a time */
static int imx586_read_reg(struct imx586 *imx586, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf[4] = { 0, };
	int ret;

	if (len > 4)
		return -EINVAL;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

/* Write registers 1 byte at a time */
static int imx586_write_reg_1byte(struct imx586 *imx586, u16 reg, u8 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);
	u8 buf[3];
	int ret;

	put_unaligned_be16(reg, buf);
	buf[2] = val;
	ret = i2c_master_send(client, buf, 3);
	if ( ret != 3 )
		return ret;

	return 0;
}

/* Write registers 2 byte at a time */
static int imx586_write_reg_2byte(struct imx586 *imx586, u16 reg, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);
	u8 buf[4];
	int ret;

	put_unaligned_be16(reg, buf);
	buf[2] = val;
	buf[3] = val>>8;
	ret = i2c_master_send(client, buf, 4);
	if ( ret != 4 )
		return ret;

	return 0;
}

/* Write registers 3 byte at a time */
static int imx586_write_reg_3byte(struct imx586 *imx586, u16 reg, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);
	u8 buf[5];

	put_unaligned_be16(reg, buf);
	buf[2]  = val;
	buf[3]  = val>>8;
	buf[4]  = val>>16;
	if (i2c_master_send(client, buf, 5) != 5)
		return -EIO;

	return 0;
}

/* Write a list of 1 byte registers */
static int imx586_write_regs(struct imx586 *imx586,
			     const struct imx586_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = imx586_write_reg_1byte(imx586, regs[i].address, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
						"Failed to write reg 0x%4.4x. error = %d\n",
						regs[i].address, ret);

			return ret;
		}
	}

	return 0;
}

/* Hold register values until hold is disabled */
static inline void imx586_register_hold(struct imx586 *imx586, bool hold)
{
	imx586_write_reg_1byte(imx586, 0x3001, hold ? 1 : 0);
}

/* Get bayer order based on flip setting. */
static u32 imx586_get_format_code(struct imx586 *imx586, u32 code)
{
	unsigned int i;
	lockdep_assert_held(&imx586->mutex);

    if(imx586->mono){
        for (i = 0; i < ARRAY_SIZE(mono_codes); i++)
            if (mono_codes[i] == code)
                break;
        return mono_codes[i];
    }
    else{
        for (i = 0; i < ARRAY_SIZE(codes); i++)
            if (codes[i] == code)
                break;
        return codes[i];
    }

	
}

static void imx586_set_default_format(struct imx586 *imx586)
{
	/* Set default mode to max resolution */
	imx586->mode = &supported_modes_12bit[0];
    if(imx586->mono){
        imx586->fmt_code = MEDIA_BUS_FMT_Y12_1X12;
    }
    else{
        imx586->fmt_code = MEDIA_BUS_FMT_SRGGB12_1X12;
    }
	
}

static int imx586_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx586 *imx586 = to_imx586(sd);
	struct v4l2_mbus_framefmt *try_fmt_img =
		v4l2_subdev_get_try_format(sd, fh->state, IMAGE_PAD);
	struct v4l2_mbus_framefmt *try_fmt_meta =
		v4l2_subdev_get_try_format(sd, fh->state, METADATA_PAD);
	struct v4l2_rect *try_crop;

	mutex_lock(&imx586->mutex);

	/* Initialize try_fmt for the image pad */
	try_fmt_img->width = supported_modes_12bit[0].width;
	try_fmt_img->height = supported_modes_12bit[0].height;
    if(imx586->mono){
        try_fmt_img->code = imx586_get_format_code(imx586, MEDIA_BUS_FMT_Y12_1X12);
    }
    else{
        try_fmt_img->code = imx586_get_format_code(imx586, MEDIA_BUS_FMT_SRGGB12_1X12);
    }
	
	try_fmt_img->field = V4L2_FIELD_NONE;

	/* Initialize try_fmt for the embedded metadata pad */
	try_fmt_meta->width = IMX586_EMBEDDED_LINE_WIDTH;
	try_fmt_meta->height = IMX586_NUM_EMBEDDED_LINES;
	try_fmt_meta->code = MEDIA_BUS_FMT_SENSOR_DATA;
	try_fmt_meta->field = V4L2_FIELD_NONE;

	/* Initialize try_crop */
	try_crop = v4l2_subdev_get_try_crop(sd, fh->state, IMAGE_PAD);
	try_crop->left = IMX586_PIXEL_ARRAY_LEFT;
	try_crop->top = IMX586_PIXEL_ARRAY_TOP;
	try_crop->width = IMX586_PIXEL_ARRAY_WIDTH;
	try_crop->height = IMX586_PIXEL_ARRAY_HEIGHT;

	mutex_unlock(&imx586->mutex);

	return 0;
}


static u64 calculate_v4l2_cid_exposure(u64 hmax, u64 vmax, u64 shr, u64 svr, u64 offset) {
    u64 numerator;
    numerator = (vmax * (svr + 1) - shr) * hmax + offset;

    do_div(numerator, hmax);
    numerator = clamp_t(uint32_t, numerator, 0, 0xFFFFFFFF);
    return numerator;
}

static void calculate_min_max_v4l2_cid_exposure(u64 hmax, u64 vmax, u64 min_shr, u64 svr, u64 offset, u64 *min_exposure, u64 *max_exposure) {
    u64 max_shr = (svr + 1) * vmax - 4;
    max_shr = min_t(uint64_t, max_shr, 0xFFFF);

    *min_exposure = calculate_v4l2_cid_exposure(hmax, vmax, max_shr, svr, offset);
    *max_exposure = calculate_v4l2_cid_exposure(hmax, vmax, min_shr, svr, offset);
}


/*
Integration Time [s] = [{VMAX × (SVR + 1) – (SHR)}
 × HMAX + offset] / (72 × 10^6)

Integration Time [s] = exposure * HMAX / (72 × 10^6)
*/

static uint32_t calculate_shr(uint32_t exposure, uint32_t hmax, uint64_t vmax, uint32_t svr, uint32_t offset)
{
    uint64_t temp;
    uint32_t shr;

    temp = ((uint64_t)exposure * hmax - offset);
    do_div(temp, hmax);
    shr = (uint32_t)(vmax * (svr + 1) - temp);

    return shr;
}

static int imx586_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx586 *imx586 = container_of(ctrl->handler, struct imx586, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);
	const struct imx586_mode *mode = imx586->mode;

	int ret = 0;

	/*
	 * The VBLANK control may change the limits of usable exposure, so check
	 * and adjust if necessary.
	 */
	if (ctrl->id == V4L2_CID_VBLANK){
		/* Honour the VBLANK limits when setting exposure. */
		u64 current_exposure, max_exposure, min_exposure, vmax;
		vmax = ((u64)mode->height + ctrl->val) ;
		imx586 -> VMAX = vmax;
		
		calculate_min_max_v4l2_cid_exposure(imx586 -> HMAX, imx586 -> VMAX, (u64)mode->min_SHR, 0, 209, &min_exposure, &max_exposure);
		current_exposure = clamp_t(uint32_t, current_exposure, min_exposure, max_exposure);

		dev_info(&client->dev,"exposure_max:%lld, exposure_min:%lld, current_exposure:%lld\n",max_exposure, min_exposure, current_exposure);
		dev_info(&client->dev,"\tVMAX:%d, HMAX:%d\n",imx586->VMAX, imx586->HMAX);
		__v4l2_ctrl_modify_range(imx586->exposure, min_exposure,max_exposure, 1,current_exposure);
	}

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		{
			u64 shr;
			dev_info(&client->dev,"V4L2_CID_EXPOSURE : %d\n",ctrl->val);
			dev_info(&client->dev,"\tvblank:%d, hblank:%d\n",imx586->vblank->val, imx586->hblank->val);
			dev_info(&client->dev,"\tVMAX:%d, HMAX:%d\n",imx586->VMAX, imx586->HMAX);
			shr = calculate_shr(ctrl->val, imx586->HMAX, imx586->VMAX, 0, 209);
			dev_info(&client->dev,"\tSHR:%lld\n",shr);
			ret = imx586_write_reg_2byte(imx586, IMX586_REG_SHR, shr);
		}
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		{
			int gain = ctrl->val;

			// Use HCG mode when gain is over the HGC level
			// This can only be done when HDR is disabled
			bool useHGC = false;
			if (!mode->hdr && gain >= IMX586_ANA_GAIN_HCG_THRESHOLD) {
				useHGC = true;
				gain -= IMX586_ANA_GAIN_HCG_LEVEL;
				if ( gain < IMX586_ANA_GAIN_HCG_MIN )
					gain = IMX586_ANA_GAIN_HCG_MIN;
			}
			dev_info(&client->dev,"V4L2_CID_ANALOGUE_GAIN: %d, HGC: %d\n",gain, (int)useHGC);

			// Apply gain
			imx586_register_hold(imx586, true);
			ret = imx586_write_reg_2byte(imx586, IMX586_REG_ANALOG_GAIN, gain);
			if (ret)
				dev_err_ratelimited(&client->dev, "Failed to write reg 0x%4.4x. error = %d\n", IMX586_REG_ANALOG_GAIN, ret);
			
			// Set HGC/LCG channel			
			ret = imx586_write_reg_1byte(imx586, IMX586_REG_FDG_SEL0, (u16)(useHGC ? 0x01 : 0x00));
			imx586_register_hold(imx586, false);
		}
		break;
	case V4L2_CID_VBLANK:
		{
			dev_info(&client->dev,"V4L2_CID_VBLANK : %d\n",ctrl->val);
			imx586->VMAX = ((u64)mode->height + ctrl->val);
			dev_info(&client->dev,"\tVMAX : %d\n",imx586 -> VMAX);
			ret = imx586_write_reg_3byte(imx586, IMX586_REG_VMAX, imx586 -> VMAX);
		}
		break;
	case V4L2_CID_HBLANK:
		{
			u64 pixel_rate;
			u64 hmax;
			dev_info(&client->dev,"V4L2_CID_HBLANK : %d\n",ctrl->val);
			pixel_rate = (u64)mode->width * IMX586_PIXEL_RATE;
			do_div(pixel_rate,mode->min_HMAX);
			hmax = (u64)(mode->width + ctrl->val) * IMX586_PIXEL_RATE;
			do_div(hmax,pixel_rate);
			imx586 -> HMAX = hmax;
			dev_info(&client->dev,"\tHMAX : %d\n",imx586 -> HMAX);
			ret = imx586_write_reg_2byte(imx586, IMX586_REG_HMAX, hmax);
		}
		break;
    case V4L2_CID_HFLIP:
		ret = imx586_write_reg_1byte(imx586, IMX586_FLIP_WINMODEH, ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		ret = imx586_write_reg_1byte(imx586, IMX586_FLIP_WINMODEV, ctrl->val);
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		//ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx586_ctrl_ops = {
	.s_ctrl = imx586_set_ctrl,
};

static int imx586_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx586 *imx586 = to_imx586(sd);

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == IMAGE_PAD) {
        if(imx586->mono){
            if (code->index >= (ARRAY_SIZE(mono_codes)))
                return -EINVAL;

            code->code = imx586_get_format_code(imx586,
                                mono_codes[code->index]);
        }
        else{
            if (code->index >= (ARRAY_SIZE(codes) / 4))
                return -EINVAL;

            code->code = imx586_get_format_code(imx586,
                                codes[code->index * 4]);
        }

	} else {
		if (code->index > 0)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}

	return 0;
}

static int imx586_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx586 *imx586 = to_imx586(sd);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == IMAGE_PAD) {
		const struct imx586_mode *mode_list;
		unsigned int num_modes;

		get_mode_table(imx586, fse->code, V4L2_XFER_FUNC_DEFAULT, &mode_list, &num_modes);

		if (fse->index >= num_modes)
			return -EINVAL;

		if (fse->code != imx586_get_format_code(imx586, fse->code))
			return -EINVAL;

		fse->min_width = mode_list[fse->index].width;
		fse->max_width = fse->min_width;
		fse->min_height = mode_list[fse->index].height;
		fse->max_height = fse->min_height;
	} else {
		if (fse->code != MEDIA_BUS_FMT_SENSOR_DATA || fse->index > 0)
			return -EINVAL;

		fse->min_width = IMX586_EMBEDDED_LINE_WIDTH;
		fse->max_width = fse->min_width;
		fse->min_height = IMX586_NUM_EMBEDDED_LINES;
		fse->max_height = fse->min_height;
	}

	return 0;
}

static void imx586_reset_colorspace(const struct imx586_mode *mode, struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = mode->linear ? V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace) : V4L2_XFER_FUNC_GRADATION_COMPRESSION;
}

static void imx586_update_image_pad_format(struct imx586 *imx586,
					   const struct imx586_mode *mode,
					   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	imx586_reset_colorspace(mode, &fmt->format);
}

static void imx586_update_metadata_pad_format(struct v4l2_subdev_format *fmt)
{
	fmt->format.width = IMX586_EMBEDDED_LINE_WIDTH;
	fmt->format.height = IMX586_NUM_EMBEDDED_LINES;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int imx586_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx586 *imx586 = to_imx586(sd);
	struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);

	dev_info(&client->dev,"xfer_func: %d\n", (int)fmt->format.xfer_func);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx586->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(&imx586->sd, sd_state,
						   fmt->pad);
		/* update the code which could change due to vflip or hflip: */
		try_fmt->code = fmt->pad == IMAGE_PAD ?
				imx586_get_format_code(imx586, try_fmt->code) :
				MEDIA_BUS_FMT_SENSOR_DATA;
		fmt->format = *try_fmt;
	} else {
		if (fmt->pad == IMAGE_PAD) {
			imx586_update_image_pad_format(imx586, imx586->mode,
						       fmt);
			fmt->format.code =
			       imx586_get_format_code(imx586, imx586->fmt_code);
		} else {
			imx586_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&imx586->mutex);
	return 0;
}

/* TODO */
static void imx586_set_framing_limits(struct imx586 *imx586)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);
	const struct imx586_mode *mode = imx586->mode;
	u64 def_hblank;
	u64 pixel_rate;


	imx586->VMAX = mode->default_VMAX;
	imx586->HMAX = mode->default_HMAX;

	pixel_rate = (u64)mode->width * IMX586_PIXEL_RATE;
	do_div(pixel_rate,mode->min_HMAX);
	dev_info(&client->dev,"Pixel Rate : %lld\n",pixel_rate);


	//int def_hblank = mode->default_HMAX * IMX586_PIXEL_RATE / 72000000 - IMX586_NATIVE_WIDTH;
	def_hblank = mode->default_HMAX * pixel_rate;
	do_div(def_hblank,IMX586_PIXEL_RATE);
	def_hblank = def_hblank - mode->width;
	__v4l2_ctrl_modify_range(imx586->hblank, 0,
				 IMX586_HMAX_MAX, 1, def_hblank);


	__v4l2_ctrl_s_ctrl(imx586->hblank, def_hblank);



	/* Update limits and set FPS to default */
	__v4l2_ctrl_modify_range(imx586->vblank, mode->min_VMAX - mode->height,
				 IMX586_VMAX_MAX - mode->height,
				 1, mode->default_VMAX - mode->height);
	__v4l2_ctrl_s_ctrl(imx586->vblank, mode->default_VMAX - mode->height);

	/* Setting this will adjust the exposure limits as well. */

	__v4l2_ctrl_modify_range(imx586->pixel_rate, pixel_rate, pixel_rate, 1, pixel_rate);

	dev_info(&client->dev,"Setting default HBLANK : %lld, VBLANK : %lld with PixelRate: %lld\n",def_hblank,mode->default_VMAX - mode->height, pixel_rate);
}

/* TODO */
static int imx586_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;
	const struct imx586_mode *mode;
	struct imx586 *imx586 = to_imx586(sd);
	struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);

	dev_info(&client->dev,"xfer_func: %d\n", (int)fmt->format.xfer_func);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx586->mutex);

	if (fmt->pad == IMAGE_PAD) {
		const struct imx586_mode *mode_list;
		unsigned int num_modes;

		/* Bayer order varies with flips */
		fmt->format.code = imx586_get_format_code(imx586,
							  fmt->format.code);

		get_mode_table(imx586, fmt->format.code, fmt->format.xfer_func, &mode_list, &num_modes);

		mode = v4l2_find_nearest_size(mode_list,
					      num_modes,
					      width, height,
					      fmt->format.width,
					      fmt->format.height);
		imx586_update_image_pad_format(imx586, mode, fmt);
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else if (imx586->mode != mode) {
			imx586->mode = mode;
			imx586->fmt_code = fmt->format.code;
			imx586_set_framing_limits(imx586);
		}
	} else {
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else {
			/* Only one embedded data mode is supported */
			imx586_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&imx586->mutex);

	return 0;
}

/* TODO */
static const struct v4l2_rect *
__imx586_get_pad_crop(struct imx586 *imx586,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&imx586->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &imx586->mode->crop;
	}

	return NULL;
}

/* Start streaming */
static int imx586_start_streaming(struct imx586 *imx586)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);
	const struct IMX586_reg_list *reg_list;
	int ret;
	
	dev_info(&client->dev,"imx586_start_streaming\n");

	if (!imx586->common_regs_written) {
		ret = imx586_write_regs(imx586, mode_common_regs, ARRAY_SIZE(mode_common_regs));
		if (ret) {
			dev_err(&client->dev, "%s failed to set common settings\n", __func__);
			return ret;
		}
		imx586_write_reg_2byte(imx586, IMX586_REG_BLKLEVEL, IMX586_BLKLEVEL_DEFAULT);
		imx586->common_regs_written = true;
		dev_info(&client->dev,"common_regs_written\n");
	}

	/* Apply default values of current mode */
	reg_list = &imx586->mode->reg_list;
	ret = imx586_write_regs(imx586, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	/* Apply gradation compression curve for non-linear mode */
	if ( !imx586->mode->linear ) {
		imx586_write_reg_3byte(imx586, IMX586_REG_CCMP1_EXP, 500);
		imx586_write_reg_1byte(imx586, IMX586_REG_ACMP1_EXP, 0x2);
		imx586_write_reg_3byte(imx586, IMX586_REG_CCMP2_EXP, 11500);
		imx586_write_reg_1byte(imx586, IMX586_REG_ACMP2_EXP, 0x6);
	} else {
		imx586_write_reg_3byte(imx586, IMX586_REG_CCMP1_EXP, 0);
		imx586_write_reg_1byte(imx586, IMX586_REG_ACMP1_EXP, 0);
		imx586_write_reg_3byte(imx586, IMX586_REG_CCMP2_EXP, 0);
		imx586_write_reg_1byte(imx586, IMX586_REG_ACMP2_EXP, 0);
	}
	
	/* Apply HDR combining options */
	if ( imx586->mode->hdr ) {
		imx586_write_reg_2byte(imx586, IMX586_REG_EXP_TH_H, 4095);
		imx586_write_reg_2byte(imx586, IMX586_REG_EXP_TH_L, 512);
		imx586_write_reg_1byte(imx586, IMX586_REG_EXP_BK, 0);
	}
	
	/* Disable digital clamp */
	imx586_write_reg_1byte(imx586, IMX586_REG_DIGITAL_CLAMP, 0);
	
	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(imx586->sd.ctrl_handler);
	if(ret) {
		dev_err(&client->dev, "%s failed to apply user values\n", __func__);
		return ret;
	}

	/* Set stream on register */
	ret = imx586_write_reg_1byte(imx586, IMX586_REG_MODE_SELECT, IMX586_MODE_STREAMING);
	usleep_range(IMX586_STREAM_DELAY_US, IMX586_STREAM_DELAY_US + IMX586_STREAM_DELAY_RANGE_US);
	return ret;
}

/* Stop streaming */
static void imx586_stop_streaming(struct imx586 *imx586)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);
	int ret;
	
	dev_info(&client->dev,"imx586_stop_streaming\n");

	/* set stream off register */
	ret = imx586_write_reg_1byte(imx586, IMX586_REG_MODE_SELECT, IMX586_MODE_STANDBY);
	if (ret)
		dev_err(&client->dev, "%s failed to stop stream\n", __func__);
}

static int imx586_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx586 *imx586 = to_imx586(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&imx586->mutex);
	if (imx586->streaming == enable) {
		mutex_unlock(&imx586->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto err_unlock;
		}

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = imx586_start_streaming(imx586);
		if (ret)
			goto err_rpm_put;
	} else {
		imx586_stop_streaming(imx586);
		pm_runtime_put(&client->dev);
	}

	imx586->streaming = enable;

	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(imx586->vflip, enable);

	mutex_unlock(&imx586->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&imx586->mutex);

	return ret;
}

/* Power/clock management functions */
static int imx586_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx586 *imx586 = to_imx586(sd);
	int ret;

	ret = regulator_bulk_enable(imx586_NUM_SUPPLIES,
				    imx586->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(imx586->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	gpiod_set_value_cansleep(imx586->reset_gpio, 1);
	usleep_range(imx586_XCLR_MIN_DELAY_US,
		     imx586_XCLR_MIN_DELAY_US + imx586_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	regulator_bulk_disable(imx586_NUM_SUPPLIES, imx586->supplies);
	return ret;
}

static int imx586_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx586 *imx586 = to_imx586(sd);

	gpiod_set_value_cansleep(imx586->reset_gpio, 0);
	regulator_bulk_disable(imx586_NUM_SUPPLIES, imx586->supplies);
	clk_disable_unprepare(imx586->xclk);

	/* Force reprogramming of the common registers when powered up again. */
	imx586->common_regs_written = false;

	return 0;
}

static int __maybe_unused imx586_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx586 *imx586 = to_imx586(sd);

	if (imx586->streaming)
		imx586_stop_streaming(imx586);

	return 0;
}

static int __maybe_unused imx586_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx586 *imx586 = to_imx586(sd);
	int ret;

	if (imx586->streaming) {
		ret = imx586_start_streaming(imx586);
		if (ret)
			goto error;
	}

	return 0;

error:
	imx586_stop_streaming(imx586);
	imx586->streaming = 0;
	return ret;
}

static int imx586_get_regulators(struct imx586 *imx586)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);
	unsigned int i;

	for (i = 0; i < imx586_NUM_SUPPLIES; i++)
		imx586->supplies[i].supply = imx586_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       imx586_NUM_SUPPLIES,
				       imx586->supplies);
}

/* Verify chip ID */
static int imx586_identify_module(struct imx586 *imx586, u32 expected_id)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);
	int ret;
	u32 val;

	ret = imx586_read_reg(imx586, IMX586_REG_CHIP_ID,
			      1, &val);
	if (ret) {
		dev_err(&client->dev, "failed to read chip id %x, with error %d\n",
			expected_id, ret);
		return ret;
	}

	dev_info(&client->dev, "Device found, ID: %x\n", val);

	return 0;
}

static int imx586_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct imx586 *imx586 = to_imx586(sd);

		mutex_lock(&imx586->mutex);
		sel->r = *__imx586_get_pad_crop(imx586, sd_state, sel->pad,
						sel->which);
		mutex_unlock(&imx586->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX586_NATIVE_WIDTH;
		sel->r.height = IMX586_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = IMX586_PIXEL_ARRAY_LEFT;
		sel->r.top = IMX586_PIXEL_ARRAY_TOP;
		sel->r.width = IMX586_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX586_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}


static const struct v4l2_subdev_core_ops imx586_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx586_video_ops = {
	.s_stream = imx586_set_stream,
};

static const struct v4l2_subdev_pad_ops imx586_pad_ops = {
	.enum_mbus_code = imx586_enum_mbus_code,
	.get_fmt = imx586_get_pad_format,
	.set_fmt = imx586_set_pad_format,
	.get_selection = imx586_get_selection,
	.enum_frame_size = imx586_enum_frame_size,
};

static const struct v4l2_subdev_ops imx586_subdev_ops = {
	.core = &imx586_core_ops,
	.video = &imx586_video_ops,
	.pad = &imx586_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx586_internal_ops = {
	.open = imx586_open,
};

/* Initialize control handlers */
static int imx586_init_controls(struct imx586 *imx586)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);
	struct v4l2_fwnode_device_properties props;
	int ret;

	ctrl_hdlr = &imx586->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	mutex_init(&imx586->mutex);
	ctrl_hdlr->lock = &imx586->mutex;

	/*
	 * Create the controls here, but mode specific limits are setup
	 * in the imx586_set_framing_limits() call below.
	 */
	/* By default, PIXEL_RATE is read only */
	imx586->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx586_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       0xffff,
					       0xffff, 1,
					       0xffff);
	imx586->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx586_ctrl_ops,
					   V4L2_CID_VBLANK, 0, 0xfffff, 1, 0);
	imx586->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx586_ctrl_ops,
					   V4L2_CID_HBLANK, 0, 0xffff, 1, 0);

	imx586->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx586_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX586_EXPOSURE_MIN,
					     IMX586_EXPOSURE_MAX,
					     IMX586_EXPOSURE_STEP,
					     IMX586_EXPOSURE_DEFAULT);
/*
	v4l2_ctrl_new_std(&imx586->ctrls, &imx586_ctrl_ops,
                          V4L2_CID_ANALOGUE_GAIN, 0, 240, 1, 0);
*/
	v4l2_ctrl_new_std(ctrl_hdlr, &imx586_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX586_ANA_GAIN_MIN, IMX586_ANA_GAIN_MAX,
			  IMX586_ANA_GAIN_STEP, IMX586_ANA_GAIN_DEFAULT);


    imx586->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx586_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);

    imx586->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx586_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1, 0);

/*
	if (imx586->vflip)
		imx586->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
*/

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx586_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	imx586->sd.ctrl_handler = ctrl_hdlr;

	/* Setup exposure and frame/line length limits. */
	imx586_set_framing_limits(imx586);

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx586->mutex);

	return ret;
}

static void imx586_free_controls(struct imx586 *imx586)
{
	v4l2_ctrl_handler_free(imx586->sd.ctrl_handler);
	mutex_destroy(&imx586->mutex);
}

static const struct imx586_compatible_data imx586_compatible = {
	.chip_id = IMX586_CHIP_ID,
	.extra_regs = {
		.num_of_regs = 0,
		.regs = NULL
	}
};

static const struct of_device_id imx586_dt_ids[] = {
	{ .compatible = "sony,imx586", .data = &imx586_compatible },
	{ /* sentinel */ }
};

static int imx586_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx586 *imx586;
	const struct of_device_id *match;
	int ret;
    u32 tm_of;

	imx586 = devm_kzalloc(&client->dev, sizeof(*imx586), GFP_KERNEL);
	if (!imx586)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx586->sd, client, &imx586_subdev_ops);

	match = of_match_device(imx586_dt_ids, dev);
	if (!match)
		return -ENODEV;
	imx586->compatible_data =
		(const struct imx586_compatible_data *)match->data;

    /* From imx477.c */
    /* Default the mono mode from OF to -1, which means invalid */
    ret = of_property_read_u32(dev->of_node, "mono-mode", &tm_of);
    imx586->mono = (ret == 0);
    dev_info(dev, "IMX586 mono option: %d\n", imx586->mono);


	/* Get system clock (xclk) */
	imx586->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx586->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(imx586->xclk);
	}

	imx586->xclk_freq = clk_get_rate(imx586->xclk);
	if (imx586->xclk_freq != IMX586_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			imx586->xclk_freq);
		return -EINVAL;
	}

	ret = imx586_get_regulators(imx586);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	imx586->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	
	/*
	 * The sensor must be powered for imx586_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = imx586_power_on(dev);
	if (ret)
		return ret;

	ret = imx586_identify_module(imx586, imx586->compatible_data->chip_id);
	if (ret)
		goto error_power_off;

	/* Initialize default format */
	imx586_set_default_format(imx586);

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	/* This needs the pm runtime to be registered. */
	ret = imx586_init_controls(imx586);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	imx586->sd.internal_ops = &imx586_internal_ops;
	imx586->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	imx586->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	imx586->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	imx586->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx586->sd.entity, NUM_PADS, imx586->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&imx586->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	return 0;

error_media_entity:
	media_entity_cleanup(&imx586->sd.entity);

error_handler_free:
	imx586_free_controls(imx586);

error_power_off:
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	imx586_power_off(&client->dev);

	return ret;
}

static void imx586_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx586 *imx586 = to_imx586(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx586_free_controls(imx586);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx586_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

}

MODULE_DEVICE_TABLE(of, imx586_dt_ids);

static const struct dev_pm_ops imx586_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx586_suspend, imx586_resume)
	SET_RUNTIME_PM_OPS(imx586_power_off, imx586_power_on, NULL)
};

static struct i2c_driver imx586_i2c_driver = {
	.driver = {
		.name = "imx586",
		.of_match_table	= imx586_dt_ids,
		.pm = &imx586_pm_ops,
	},
	.probe = imx586_probe,
	.remove = imx586_remove,
};

module_i2c_driver(imx586_i2c_driver);

MODULE_AUTHOR("Will Whang <will@willwhang.com>");
MODULE_AUTHOR("Tetsuya NOMURA <tetsuya.nomura@soho-enterprise.com>");
MODULE_AUTHOR("Russell Newman <russellnewman@octopuscinema.com>");
MODULE_AUTHOR("Marcin Paszkuta <marcin.paszkuta@optimedio.com>");
MODULE_DESCRIPTION("Sony imx586 sensor driver");
MODULE_LICENSE("GPL v2");