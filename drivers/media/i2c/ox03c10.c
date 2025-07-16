// SPDX-License-Identifier: GPL-2.0
/*
 * OmniVision OX03C10 sensor driver
 *
 * Copyright (C) 2025 O(log n) Technology Co., Ltd.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/rk-camera-module.h>
#include <linux/rk-preisp.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/version.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x01)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define OX03C10_XVCLK_FREQ		24000000

#define OX03C10_LANES			2
#define OX03C10_BITS_PER_SAMPLE		12
#define OX03C10_LINK_FREQ_240		240000000

#define PIXEL_RATE_WITH_240M_12BIT	(OX03C10_LINK_FREQ_240 * 2 * \
					OX03C10_LANES / OX03C10_BITS_PER_SAMPLE)

#define CHIP_ID				0x58034a
#define OX03C10_REG_CHIP_ID		0x010a

#define OX03C10_REG_CTRL_MODE		0x0100
#define OX03C10_MODE_SW_STANDBY		0x0
#define OX03C10_MODE_STREAMING		BIT(0)

#define OX03C10_REG_EXPOSURE		0x3500
#define OX03C10_EXPOSURE_MIN		4
#define OX03C10_EXPOSURE_STEP		0xf
#define OX03C10_VTS_MAX			0xffff

#define OX03C10_REG_ANALOG_GAIN		0x350a
#define ANALOG_GAIN_MASK		0x3ff
#define ANALOG_GAIN_MIN			0x10
#define ANALOG_GAIN_MAX			0x3e0
#define ANALOG_GAIN_STEP		1
#define ANALOG_GAIN_DEFAULT		0x20

#define OX03C10_REG_TEST_PATTERN		0x5e00
#define	OX03C10_TEST_PATTERN_ENABLE	0x80
#define	OX03C10_TEST_PATTERN_DISABLE	0x0

#define OX03C10_REG_VTS			0x380e

#define OX03C10_MIRROR_REG		0x3821
#define OX03C10_FLIP_REG		0x3820

#define OX03C10_FETCH_MIRROR(VAL, ENABLE)	(ENABLE ? VAL | 0x01 : VAL & 0xf9)
#define OX03C10_FETCH_FLIP(VAL, ENABLE)		(ENABLE ? VAL | 0x01 : VAL & 0x9f)

#define REG_DELAY			0xFFFE
#define REG_NULL			0xFFFF

#define OX03C10_REG_VALUE_08BIT		1
#define OX03C10_REG_VALUE_16BIT		2
#define OX03C10_REG_VALUE_24BIT		3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
#define OX03C10_NAME			"ox03c10"

static const char * const ox03c10_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define OX03C10_NUM_SUPPLIES ARRAY_SIZE(ox03c10_supply_names)

struct regval {
	u16 addr;
	u16 val;
};

struct ox03c10_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
	u32 hdr_mode;
	u32 vc[PAD_MAX];
};

struct ox03c10 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct regulator_bulk_data supplies[OX03C10_NUM_SUPPLIES];

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*test_pattern;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct ox03c10_mode *cur_mode;
	struct v4l2_fract	cur_fps;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	u32			cur_vts;
};

#define to_ox03c10(sd) container_of(sd, struct ox03c10, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval ox03c10_global_regs[] = {
	// { 0x0101, 0x00 },
	// { 0x0102, 0x00 },
	// { 0x0104, 0x00 },
	// { 0x0105, 0x00 },
	// { 0x0106, 0x00 },
	// { 0x0108, 0x00 },
	{ 0x0109, 0xa8 },
	{ 0x010a, 0x58 },
	{ 0x010b, 0x03 },
	{ 0x010c, 0x4a },
	{ 0x010d, 0x00 },
	{ 0x010e, 0x00 },
	{ 0x010f, 0x00 },
	{ 0x0110, 0x00 },
	{ 0x0111, 0x00 },
	{ 0x0112, 0x02 },
	{ 0x0113, 0x00 },
	{ 0x0114, 0x00 },
	{ 0x0115, 0x00 },
	{ 0x0116, 0x00 },
	{ 0x0117, 0x00 },
	{ 0x0118, 0x00 },
	{ 0x0119, 0x00 },
	{ 0x011a, 0x00 },
	{ 0x011b, 0x00 },
	{ 0x011c, 0x00 },
	{ 0x011d, 0x00 },
	{ 0x011e, 0x00 },
	{ 0x011f, 0x01 },
	{ 0x0120, 0x00 },
	{ 0x0121, 0x00 },
	{ 0x0122, 0x00 },
	{ 0x0123, 0x00 },
	{ 0x0124, 0x00 },
	{ 0x0125, 0x00 },
	{ 0x0126, 0x00 },
	{ 0x0127, 0x00 },
	{ 0x0128, 0x00 },
	{ 0x0129, 0x00 },
	{ 0x012a, 0x10 },
	{ 0x012b, 0x44 },
	{ 0x0300, 0x3a },
	{ 0x0301, 0xc8 },
	{ 0x0302, 0x21 },
	{ 0x0303, 0x04 },
	{ 0x0304, 0x01 },
	{ 0x0305, 0x68 },
	{ 0x0306, 0x04 },
	{ 0x0307, 0x02 },
	{ 0x0308, 0x13 },
	{ 0x0309, 0x03 },
	{ 0x030a, 0x00 },
	{ 0x030b, 0x00 },
	{ 0x030c, 0x00 },
	{ 0x030d, 0x00 },
	{ 0x030e, 0x00 },
	{ 0x030f, 0x00 },
	{ 0x0310, 0x00 },
	{ 0x0311, 0x00 },
	{ 0x0312, 0x07 },
	{ 0x0313, 0x00 },
	{ 0x0314, 0x00 },
	{ 0x0315, 0x00 },
	{ 0x0316, 0x00 },
	{ 0x0317, 0x01 },
	{ 0x0318, 0x00 },
	{ 0x0319, 0x00 },
	{ 0x031a, 0x00 },
	{ 0x031b, 0x00 },
	{ 0x031c, 0x00 },
	{ 0x031d, 0x00 },
	{ 0x031e, 0x00 },
	{ 0x031f, 0x00 },
	{ 0x0320, 0x02 },
	{ 0x0321, 0x31 },
	{ 0x0322, 0x03 },
	{ 0x0323, 0x04 },
	{ 0x0324, 0x02 },
	{ 0x0325, 0x52 },
	{ 0x0326, 0x8a },
	{ 0x0327, 0x0a },
	{ 0x0328, 0x00 },
	{ 0x0329, 0x00 },
	{ 0x032a, 0x0a },
	{ 0x032b, 0x02 },
	{ 0x032c, 0x01 },
	{ 0x032d, 0x01 },
	{ 0x032e, 0x01 },
	{ 0x032f, 0x00 },
	{ 0x0330, 0x02 },
	{ 0x0331, 0x0c },
	{ 0x0332, 0x22 },
	{ 0x0333, 0x31 },
	{ 0x0334, 0x03 },
	{ 0x0335, 0x00 },
	{ 0x0336, 0x00 },
	{ 0x0337, 0x4b },
	{ 0x0338, 0x04 },
	{ 0x0339, 0x08 },
	{ 0x033a, 0x00 },
	{ 0x033b, 0x0b },
	{ 0x033c, 0x00 },
	{ 0x033d, 0x01 },
	{ 0x033e, 0x01 },
	{ 0x033f, 0x01 },
	{ 0x0340, 0x00 },
	{ 0x0341, 0x02 },
	{ 0x0342, 0x0d },
	{ 0x0343, 0x00 },
	{ 0x0344, 0x00 },
	{ 0x0345, 0x00 },
	{ 0x0346, 0x00 },
	{ 0x0347, 0x00 },
	{ 0x0348, 0x00 },
	{ 0x0349, 0x00 },
	{ 0x034a, 0x00 },
	{ 0x034b, 0x00 },
	{ 0x034c, 0x00 },
	{ 0x034d, 0x00 },
	{ 0x034e, 0x00 },
	{ 0x034f, 0x00 },
	{ 0x0350, 0x00 },
	{ 0x0351, 0x00 },
	{ 0x0352, 0x00 },
	{ 0x0353, 0x00 },
	{ 0x0354, 0x00 },
	{ 0x0355, 0x00 },
	{ 0x0356, 0x00 },
	{ 0x0357, 0x00 },
	{ 0x0358, 0x00 },
	{ 0x0359, 0x00 },
	{ 0x035a, 0x00 },
	{ 0x035b, 0x00 },
	{ 0x035c, 0x00 },
	{ 0x035d, 0x00 },
	{ 0x035e, 0x00 },
	{ 0x035f, 0x00 },
	{ 0x0360, 0x00 },
	{ 0x0361, 0x00 },
	{ 0x0362, 0xcc },
	{ 0x0363, 0x0c },
	{ 0x0313, 0x00 },
	{ 0x0001, 0x01 },
	{ 0x02be, 0x10 },
	{ 0x005b, 0x01 },
	{ 0x0318, 0x5e },
	{ 0x02d3, 0x00 },
	{ 0x02d3, 0x10 },
	{ 0x0320, 0x2c },
	{ 0x0313, 0x02 },

	// RSVD
	{ 0x3700, 0x35 },
	{ 0x3701, 0x18 },
	{ 0x3702, 0x1c },
	{ 0x3703, 0x41 },
	{ 0x3704, 0x0c },
	{ 0x3705, 0x00 },
	{ 0x3706, 0x38 },
	{ 0x3707, 0x08 },
	{ 0x3708, 0x28 },
	{ 0x3709, 0x6e },
	{ 0x370a, 0x00 },
	{ 0x370b, 0x7a },
	{ 0x370c, 0x0c },
	{ 0x370d, 0x08 },
	{ 0x370e, 0x55 },
	{ 0x370f, 0x24 },
	{ 0x3710, 0x14 },
	{ 0x3711, 0x16 },
	{ 0x3712, 0x13 },
	{ 0x3713, 0x01 },
	{ 0x3714, 0x02 },
	{ 0x3715, 0x80 },
	{ 0x3716, 0x0c },
	{ 0x3717, 0x02 },
	{ 0x3718, 0x72 },
	{ 0x3719, 0x4f },
	{ 0x371a, 0x54 },
	{ 0x371b, 0x31 },
	{ 0x371c, 0x00 },
	{ 0x371d, 0x08 },
	{ 0x371e, 0x00 },
	{ 0x371f, 0x02 },
	{ 0x3720, 0x03 },
	{ 0x3721, 0x5c },
	{ 0x3722, 0x64 },
	{ 0x3723, 0x06 },
	{ 0x3724, 0x01 },
	{ 0x3725, 0x06 },
	{ 0x3726, 0x0a },
	{ 0x3727, 0x06 },
	{ 0x3728, 0x03 },
	{ 0x3729, 0x09 },
	{ 0x372a, 0x01 },
	{ 0x372b, 0x03 },
	{ 0x372c, 0x17 },
	{ 0x372d, 0x4a },
	{ 0x372e, 0x00 },
	{ 0x372f, 0x00 },
	{ 0x3730, 0x02 },
	{ 0x3731, 0x48 },
	{ 0x3732, 0x04 },
	{ 0x3733, 0x01 },
	{ 0x3734, 0x0a },
	{ 0x3735, 0x11 },
	{ 0x3736, 0x11 },
	{ 0x3737, 0x00 },
	{ 0x3738, 0x50 },
	{ 0x3739, 0x48 },
	{ 0x373a, 0x48 },
	{ 0x373b, 0x48 },
	{ 0x373c, 0x11 },
	{ 0x373d, 0x11 },
	{ 0x373e, 0x00 },
	{ 0x373f, 0x36 },
	{ 0x3740, 0x36 },
	{ 0x3741, 0x36 },
	{ 0x3742, 0x36 },
	{ 0x3743, 0x01 },
	{ 0x3744, 0x15 },
	{ 0x3745, 0x06 },
	{ 0x3746, 0x03 },
	{ 0x3747, 0x35 },
	{ 0x3748, 0x35 },
	{ 0x3749, 0x1c },
	{ 0x374a, 0x00 },
	{ 0x374b, 0x83 },
	{ 0x374c, 0x14 },
	{ 0x374d, 0x01 },
	{ 0x374e, 0x01 },
	{ 0x374f, 0x04 },
	{ 0x3750, 0x4f },
	{ 0x3751, 0x02 },
	{ 0x3752, 0x03 },
	{ 0x3753, 0xa0 },
	{ 0x3754, 0x06 },
	{ 0x3755, 0x17 },
	{ 0x3756, 0x05 },
	{ 0x3757, 0x19 },
	{ 0x3758, 0x00 },
	{ 0x3759, 0x30 },
	{ 0x375a, 0x00 },
	{ 0x375b, 0x60 },
	{ 0x375c, 0x00 },
	{ 0x375d, 0x08 },
	{ 0x375e, 0x0c },
	{ 0x375f, 0x03 },
	{ 0x3760, 0x0f },
	{ 0x3761, 0x0e },
	{ 0x3762, 0x15 },
	{ 0x3763, 0x03 },
	{ 0x3764, 0x0a },
	{ 0x3765, 0x1c },
	{ 0x3766, 0x06 },
	{ 0x3767, 0x06 },
	{ 0x3768, 0x15 },
	{ 0x3769, 0x01 },
	{ 0x376a, 0x01 },
	{ 0x376b, 0x00 },
	{ 0x376c, 0x07 },
	{ 0x376d, 0x08 },
	{ 0x376e, 0x08 },
	{ 0x376f, 0x08 },
	{ 0x3770, 0x91 },
	{ 0x3771, 0x08 },
	{ 0x3772, 0x01 },
	{ 0x3773, 0x00 },
	{ 0x3774, 0x02 },
	{ 0x3775, 0x00 },
	{ 0x3776, 0x00 },
	{ 0x3777, 0x99 },
	{ 0x3778, 0x00 },
	{ 0x3779, 0x22 },
	{ 0x377a, 0x02 },
	{ 0x377b, 0x01 },
	{ 0x377c, 0x48 },
	{ 0x377d, 0x00 },
	{ 0x377e, 0x00 },
	{ 0x377f, 0x06 },
	{ 0x3780, 0x00 },
	{ 0x3781, 0x02 },
	{ 0x3782, 0x07 },
	{ 0x3783, 0x02 },
	{ 0x3784, 0x09 },
	{ 0x3785, 0x0f },
	{ 0x3786, 0x04 },
	{ 0x3787, 0x03 },
	{ 0x3788, 0x02 },
	{ 0x3789, 0x02 },
	{ 0x378a, 0x04 },
	{ 0x378b, 0x00 },
	{ 0x378c, 0x00 },
	{ 0x378d, 0x00 },
	{ 0x378e, 0x00 },
	{ 0x378f, 0x00 },
	{ 0x3790, 0x13 },
	{ 0x3791, 0x05 },
	{ 0x3792, 0x31 },
	{ 0x3793, 0x00 },
	{ 0x3794, 0x14 },
	{ 0x3795, 0x00 },
	{ 0x3796, 0x00 },
	{ 0x3797, 0x00 },
	{ 0x3798, 0x00 },
	{ 0x3799, 0x00 },
	{ 0x379a, 0x00 },
	{ 0x379b, 0x10 },
	{ 0x379c, 0x00 },
	{ 0x379d, 0x48 },
	{ 0x379e, 0xa0 },
	{ 0x379f, 0x00 },
	{ 0x37a0, 0x08 },
	{ 0x37a1, 0x80 },
	{ 0x37a2, 0x03 },
	{ 0x37a3, 0x04 },
	{ 0x37a4, 0x03 },
	{ 0x37a5, 0x0f },
	{ 0x37a6, 0x12 },
	{ 0x37a7, 0x0f },
	{ 0x37a8, 0x04 },
	{ 0x37a9, 0x06 },
	{ 0x37aa, 0x04 },
	{ 0x37ab, 0x05 },
	{ 0x37ac, 0x04 },
	{ 0x37ad, 0x0a },
	{ 0x37ae, 0x0a },
	{ 0x37af, 0x01 },
	{ 0x37b0, 0x09 },
	{ 0x37b1, 0x04 },
	{ 0x37b2, 0x06 },
	{ 0x37b3, 0x08 },
	{ 0x37b4, 0x06 },
	{ 0x37b5, 0x06 },
	{ 0x37b6, 0x06 },
	{ 0x37b7, 0x06 },
	{ 0x37b8, 0xc0 },
	{ 0x37b9, 0x01 },
	{ 0x37ba, 0x06 },
	{ 0x37bb, 0x58 },
	{ 0x37bc, 0x22 },
	{ 0x37bd, 0x01 },
	{ 0x37be, 0x01 },
	{ 0x37bf, 0x00 },
	{ 0x37c0, 0x01 },
	{ 0x37c1, 0x11 },
	{ 0x37c2, 0x11 },
	{ 0x37c3, 0x00 },
	{ 0x37c4, 0x50 },
	{ 0x37c5, 0x4c },
	{ 0x37c6, 0x4c },
	{ 0x37c7, 0x4c },
	{ 0x37c8, 0x02 },
	{ 0x37c9, 0x0f },
	{ 0x37ca, 0x10 },
	{ 0x37cb, 0x34 },
	{ 0x37cc, 0x10 },
	{ 0x37cd, 0x34 },
	{ 0x37ce, 0x04 },
	{ 0x37cf, 0x10 },
	{ 0x37d0, 0x00 },
	{ 0x37d1, 0x33 },
	{ 0x37d2, 0x00 },
	{ 0x37d3, 0x64 },
	{ 0x37d4, 0x00 },
	{ 0x37d5, 0x32 },
	{ 0x37d6, 0x00 },
	{ 0x37d7, 0x63 },
	{ 0x37d8, 0x01 },
	{ 0x37d9, 0x00 },
	{ 0x37da, 0x00 },
	{ 0x37db, 0x00 },
	{ 0x37dc, 0x54 },
	{ 0x37dd, 0x00 },
	{ 0x37de, 0x28 },
	{ 0x37df, 0x00 },
	{ 0x37e0, 0x00 },
	{ 0x37e1, 0x01 },
	{ 0x37e2, 0x00 },
	{ 0x37e3, 0x28 },
	{ 0x37e4, 0x00 },
	{ 0x37e5, 0x00 },
	{ 0x37e6, 0x00 },
	{ 0x37e7, 0x00 },
	{ 0x37e8, 0x00 },
	{ 0x37e9, 0x00 },
	{ 0x37ea, 0x00 },
	{ 0x37eb, 0x00 },
	{ 0x37ec, 0x00 },
	{ 0x37ed, 0x00 },
	{ 0x37ee, 0x00 },
	{ 0x37ef, 0x00 },
	{ 0x37f0, 0x00 },
	{ 0x37f1, 0x00 },
	{ 0x37f2, 0x00 },
	{ 0x37f3, 0x00 },
	{ 0x37f4, 0x00 },
	{ 0x37f5, 0x00 },
	{ 0x37f6, 0x00 },
	{ 0x37f7, 0x00 },
	{ 0x37f8, 0x00 },
	{ 0x37f9, 0x00 },
	{ 0x37fa, 0x00 },
	{ 0x37fb, 0x00 },
	{ 0x37fc, 0x00 },
	{ 0x37fd, 0x00 },
	{ 0x37fe, 0x00 },
	{ 0x37ff, 0x00 },

	// DCG
	{ 0x3501, 0x02 },
	{ 0x3502, 0x00 },
	{ 0x3503, 0xa8 },
	{ 0x3504, 0x08 },
	{ 0x3505, 0x00 },
	{ 0x3506, 0x40 },
	{ 0x3507, 0x00 },
	{ 0x3508, 0x01 },
	{ 0x3509, 0x70 },
	{ 0x350a, 0x01 },
	{ 0x350b, 0x00 },
	{ 0x350c, 0x00 },
	{ 0x350d, 0x00 },
	{ 0x350e, 0x02 },
	{ 0x350f, 0x00 },
	{ 0x3510, 0x40 },
	{ 0x3511, 0x00 },
	{ 0x3512, 0x00 },
	{ 0x3513, 0x17 },
	{ 0x3514, 0x00 },
	{ 0x3515, 0x17 },
	{ 0x3516, 0x00 },
	{ 0x3517, 0x07 },
	{ 0x3518, 0x01 },
	{ 0x3519, 0x00 },
	{ 0x351a, 0x00 },
	{ 0x351b, 0x01 },
	{ 0x351c, 0x00 },
	{ 0x351d, 0x00 },
	{ 0x351e, 0x01 },
	{ 0x351f, 0x07 },
	{ 0x3520, 0x01 },
	{ 0x3521, 0x70 },
	{ 0x3522, 0x00 },
	{ 0x3523, 0x00 },
	{ 0x3524, 0x00 },
	{ 0x3525, 0x00 },
	{ 0x3526, 0x00 },
	{ 0x3527, 0x00 },
	{ 0x3528, 0x00 },

	// Unknown
	{ 0x3529, 0x00 },
	{ 0x352a, 0x00 },
	{ 0x352b, 0x00 },
	{ 0x352c, 0x00 },
	{ 0x352d, 0x00 },
	{ 0x352e, 0x00 },
	{ 0x352f, 0x00 },
	{ 0x3530, 0x00 },
	{ 0x3531, 0x00 },
	{ 0x3532, 0x00 },
	{ 0x3533, 0x00 },
	{ 0x3534, 0x00 },
	{ 0x3535, 0x00 },
	{ 0x3536, 0x00 },
	{ 0x3537, 0x00 },
	{ 0x3538, 0x00 },
	{ 0x3539, 0x00 },
	{ 0x353a, 0x00 },
	{ 0x353b, 0x00 },
	{ 0x353c, 0x00 },
	{ 0x353d, 0x00 },
	{ 0x353e, 0x00 },
	{ 0x353f, 0x00 },
	{ 0x3540, 0x00 },

	// SPD
	{ 0x3541, 0x02 },
	{ 0x3542, 0x00 },
	{ 0x3543, 0xa8 },
	{ 0x3544, 0x08 },
	{ 0x3545, 0x00 },
	{ 0x3546, 0x20 },
	{ 0x3547, 0x00 },
	{ 0x3548, 0x05 },
	{ 0x3549, 0x00 },
	{ 0x354a, 0x01 },
	{ 0x354b, 0x00 },
	{ 0x354c, 0x00 },
	{ 0x354d, 0x00 },
	{ 0x354e, 0x02 },
	{ 0x354f, 0x00 },
	{ 0x3550, 0x20 },
	{ 0x3551, 0x00 },
	{ 0x3552, 0x00 },
	{ 0x3553, 0x50 },
	{ 0x3554, 0x00 },
	{ 0x3555, 0x50 },
	{ 0x3556, 0x03 },
	{ 0x3557, 0x04 },
	{ 0x3558, 0x01 },
	{ 0x3559, 0x00 },
	{ 0x355a, 0x00 },
	{ 0x355b, 0x01 },
	{ 0x355c, 0x00 },
	{ 0x355d, 0x00 },
	{ 0x355e, 0x05 },
	{ 0x355f, 0x00 },
	{ 0x3560, 0x05 },
	{ 0x3561, 0x00 },
	{ 0x3562, 0x00 },
	{ 0x3563, 0x00 },
	{ 0x3564, 0x00 },
	{ 0x3565, 0x00 },
	{ 0x3566, 0x00 },
	{ 0x3567, 0x00 },
	{ 0x3568, 0x00 },

	// // Format
	// { 0x4319, 0x03 }, // spd dcg
	// { 0x431f, 0x20 }, // enable PWL (pwl0_en), 12 bits

	// BLC
	{ 0x4000, 0x78 },
	{ 0x4001, 0x2b },
	{ 0x4002, 0x00 },
	{ 0x4003, 0x80 },
	{ 0x4004, 0x00 },
	{ 0x4005, 0x80 },
	{ 0x4006, 0x0f },
	{ 0x4007, 0x00 },
	{ 0x4008, 0x00 },
	{ 0x4009, 0x07 },
	{ 0x400a, 0x08 },
	{ 0x400b, 0x00 },
	{ 0x400c, 0x0f },
	{ 0x400d, 0x10 },
	{ 0x400e, 0x02 },
	{ 0x400f, 0x2d },
	{ 0x4010, 0xdf },
	{ 0x4011, 0x01 },
	{ 0x4012, 0x30 },
	{ 0x4013, 0x01 },
	{ 0x4014, 0x3f },
	{ 0x4015, 0xff },
	{ 0x4016, 0x00 },
	{ 0x4017, 0x07 },
	{ 0x4018, 0x12 },
	{ 0x4019, 0x7f },
	{ 0x401a, 0x08 },
	{ 0x401b, 0x02 },
	{ 0x401c, 0x02 },
	{ 0x401d, 0x02 },
	{ 0x401e, 0x00 },
	{ 0x401f, 0x04 },
	{ 0x4020, 0x58 },
	{ 0x4021, 0x0d },
	{ 0x4022, 0x30 },
	{ 0x4023, 0x10 },
	{ 0x4024, 0x00 },
	{ 0x4025, 0x00 },
	{ 0x4026, 0x00 },
	{ 0x4027, 0x40 },
	{ 0x4028, 0x00 },
	{ 0x4029, 0x40 },
	{ 0x402a, 0x00 },
	{ 0x402b, 0x40 },
	{ 0x402c, 0x00 },
	{ 0x402d, 0x40 },
	{ 0x402e, 0x00 },
	{ 0x402f, 0x00 },
	{ 0x4030, 0x00 },
	{ 0x4031, 0x00 },
	{ 0x4032, 0x00 },
	{ 0x4033, 0x00 },
	{ 0x4034, 0x00 },
	{ 0x4035, 0x00 },
	{ 0x4036, 0x00 },
	{ 0x4037, 0x00 },
	{ 0x4038, 0x00 },
	{ 0x4039, 0x00 },
	{ 0x403a, 0x00 },
	{ 0x403b, 0x00 },
	{ 0x403c, 0x00 },
	{ 0x403d, 0x00 },
	{ 0x403e, 0x01 },
	{ 0x403f, 0xbf },
	{ 0x4040, 0x01 },
	{ 0x4041, 0xbf },
	{ 0x4042, 0x01 },
	{ 0x4043, 0xbf },
	{ 0x4044, 0x01 },
	{ 0x4045, 0xbf },
	{ 0x4046, 0x01 },
	{ 0x4047, 0x98 },
	{ 0x4048, 0x01 },
	{ 0x4049, 0x98 },
	{ 0x404a, 0x01 },
	{ 0x404b, 0x98 },
	{ 0x404c, 0x01 },
	{ 0x404d, 0x98 },
	{ 0x404e, 0x02 },
	{ 0x404f, 0x00 },
	{ 0x4050, 0x02 },
	{ 0x4051, 0x00 },
	{ 0x4052, 0x02 },
	{ 0x4053, 0x00 },
	{ 0x4054, 0x02 },
	{ 0x4055, 0x00 },
	{ 0x4056, 0x02 },
	{ 0x4057, 0x00 },
	{ 0x4058, 0x02 },
	{ 0x4059, 0x00 },
	{ 0x405a, 0x02 },
	{ 0x405b, 0x00 },
	{ 0x405c, 0x02 },
	{ 0x405d, 0x00 },
	{ 0x405e, 0x00 },
	{ 0x405f, 0x00 },
	{ 0x4060, 0x00 },
	{ 0x4061, 0x00 },
	{ 0x4062, 0x00 },
	{ 0x4063, 0x00 },
	{ 0x4064, 0x00 },
	{ 0x4065, 0x00 },
	{ 0x4066, 0x00 },
	{ 0x4067, 0x00 },
	{ 0x4068, 0x00 },
	{ 0x4069, 0x00 },
	{ 0x406a, 0x00 },
	{ 0x406b, 0x00 },
	{ 0x406c, 0x00 },
	{ 0x406d, 0x00 },
	{ 0x406e, 0x00 },
	{ 0x406f, 0x00 },
	{ 0x4070, 0x00 },
	{ 0x4071, 0x00 },
	{ 0x4072, 0x00 },
	{ 0x4073, 0x00 },
	{ 0x4074, 0x00 },
	{ 0x4075, 0x00 },
	{ 0x4076, 0x00 },
	{ 0x4077, 0x00 },
	{ 0x4078, 0x00 },
	{ 0x4079, 0x00 },
	{ 0x407a, 0x00 },
	{ 0x407b, 0x00 },
	{ 0x407c, 0x00 },
	{ 0x407d, 0x00 },
	{ 0x407e, 0xff },
	{ 0x407f, 0xff },
	{ 0x4080, 0x00 },
	{ 0x4081, 0xed },
	{ 0x4082, 0x00 },
	{ 0x4083, 0xa1 },
	{ 0x4084, 0x00 },
	{ 0x4085, 0xb7 },
	{ 0x4086, 0x00 },
	{ 0x4087, 0x7f },
	{ 0x4088, 0x10 },
	{ 0x4089, 0x0f },
	{ 0x408a, 0x01 },
	{ 0x408b, 0x00 },
	{ 0x408c, 0x00 },
	{ 0x408d, 0xf0 },
	{ 0x408e, 0x0f },
	{ 0x408f, 0x00 },
	{ 0x4090, 0x08 },
	{ 0x4091, 0x00 },
	{ 0x4092, 0x08 },
	{ 0x4093, 0x00 },
	{ 0x4094, 0x08 },
	{ 0x4095, 0x00 },
	{ 0x4096, 0x08 },
	{ 0x4097, 0x00 },
	{ 0x4098, 0x07 },
	{ 0x4099, 0xfe },
	{ 0x409a, 0x07 },
	{ 0x409b, 0xff },
	{ 0x409c, 0x07 },
	{ 0x409d, 0xff },
	{ 0x409e, 0x07 },
	{ 0x409f, 0xfe },
	{ 0x40a0, 0x08 },
	{ 0x40a1, 0x0c },
	{ 0x40a2, 0x08 },
	{ 0x40a3, 0x0c },
	{ 0x40a4, 0x08 },
	{ 0x40a5, 0x0c },
	{ 0x40a6, 0x08 },
	{ 0x40a7, 0x0c },
	{ 0x40a8, 0x08 },
	{ 0x40a9, 0x02 },
	{ 0x40aa, 0x08 },
	{ 0x40ab, 0x02 },
	{ 0x40ac, 0x08 },
	{ 0x40ad, 0x02 },
	{ 0x40ae, 0x08 },
	{ 0x40af, 0x02 },
	{ 0x40b0, 0x3f },
	{ 0x40b1, 0x00 },
	{ 0x40b2, 0x00 },
	{ 0x40b3, 0x00 },
	{ 0x40b4, 0x00 },
	{ 0x40b5, 0x00 },
	{ 0x40b6, 0x00 },
	{ 0x40b7, 0x00 },
	{ 0x40b8, 0x00 },
	{ 0x40b9, 0x00 },
	{ 0x40ba, 0x00 },
	{ 0x40bb, 0x00 },
	{ 0x40bc, 0x00 },
	{ 0x40bd, 0x00 },
	{ 0x40be, 0x00 },
	{ 0x40bf, 0x00 },
	{ 0x40c0, 0x3f },
	{ 0x40c1, 0xff },
	{ 0x40c2, 0x00 },
	{ 0x40c3, 0xff },
	{ 0x40c4, 0x3f },
	{ 0x40c5, 0xfe },
	{ 0x40c6, 0x00 },
	{ 0x40c7, 0x00 },
	{ 0x40c8, 0x3f },
	{ 0x40c9, 0xff },
	{ 0x40ca, 0x00 },
	{ 0x40cb, 0x00 },
	{ 0x40cc, 0x00 },
	{ 0x40cd, 0x00 },
	{ 0x40ce, 0x00 },
	{ 0x40cf, 0x00 },
	{ 0x40d0, 0x00 },
	{ 0x40d1, 0xd0 },
	{ 0x40d2, 0x00 },
	{ 0x40d3, 0xd0 },
	{ 0x40d4, 0x00 },
	{ 0x40d5, 0xd0 },
	{ 0x40d6, 0x00 },
	{ 0x40d7, 0xd0 },
	{ 0x40d8, 0x00 },
	{ 0x40d9, 0x00 },
	{ 0x40da, 0x00 },
	{ 0x40db, 0x00 },
	{ 0x40dc, 0x00 },
	{ 0x40dd, 0x00 },
	{ 0x40de, 0x00 },
	{ 0x40df, 0x00 },
	{ 0x40e0, 0x30 },
	{ 0x40e1, 0x30 },
	{ 0x40e2, 0x30 },
	{ 0x40e3, 0x30 },
	{ 0x40e4, 0x00 },
	{ 0x40e5, 0x00 },
	{ 0x40e6, 0x00 },
	{ 0x40e7, 0x00 },
	{ 0x40e8, 0x00 },
	{ 0x40e9, 0x03 },

	{ 0x4880, 0x00 },
	{ 0x4881, 0x00 },
	{ 0x4882, 0x00 },
	{ 0x4883, 0x00 },
	{ 0x4884, 0x08 },
	{ 0x4885, 0x10 },
	{ 0x4886, 0x00 },
	{ 0x4887, 0x51 },
	{ 0x4888, 0x90 },
	{ 0x4889, 0x03 },
	{ 0x488a, 0x00 },
	{ 0x4900, 0x06 },
	{ 0x4901, 0x00 },
	{ 0x4902, 0x00 },
	{ 0x4903, 0x80 },
	{ 0x1600, 0xc0 },
	{ 0x1601, 0x28 },
	{ 0x1602, 0xc8 },
	{ 0x1603, 0x10 },
	{ 0x1604, 0x00 },
	{ 0x1605, 0x10 },
	{ 0x1606, 0x01 },
	{ 0x1607, 0x47 },
	{ 0x1608, 0x03 },
	{ 0x1609, 0x00 },
	{ 0x160a, 0x00 },
	{ 0x160b, 0x00 },
	{ 0x160c, 0x40 },
	{ 0x160d, 0x00 },
	{ 0x160e, 0x00 },
	{ 0x160f, 0x00 },
	{ 0x1610, 0x00 },
	{ 0x1611, 0x02 },
	{ 0x1612, 0x00 },
	{ 0x1613, 0x00 },
	{ 0x1614, 0x00 },
	{ 0x1615, 0x00 },
	{ 0x1616, 0x00 },
	{ 0x1617, 0xff },
	{ 0x1618, 0xbf },
	{ 0x1619, 0x00 },
	{ 0x161a, 0x00 },
	{ 0x161b, 0x20 },
	{ 0x161c, 0x00 },
	{ 0x161d, 0x88 },
	{ 0x161e, 0x00 },
	{ 0x161f, 0x01 },
	{ 0x1620, 0x01 },
	{ 0x1621, 0x00 },
	{ 0x1622, 0x00 },
	{ 0x1623, 0x00 },
	{ 0x1624, 0x20 },
	{ 0x1625, 0x00 },
	{ 0x1626, 0x18 },
	{ 0x1627, 0x00 },
	{ 0x1628, 0xdf },
	{ 0x1629, 0x8f },
	{ 0x162a, 0x00 },
	{ 0x162b, 0x00 },
	{ 0x162c, 0xff },
	{ 0x162d, 0xff },
	{ 0x162e, 0x00 },
	{ 0x162f, 0x00 },
	{ 0x1630, 0x00 },
	{ 0x1631, 0x00 },
	{ 0x1632, 0x00 },
	{ 0x1633, 0x00 },
	{ 0x1634, 0x00 },
	{ 0x1635, 0x00 },
	{ 0x1636, 0x00 },
	{ 0x1637, 0x00 },
	{ 0x1638, 0x00 },
	{ 0x1639, 0x00 },
	{ 0x163a, 0x00 },
	{ 0x163b, 0x00 },
	{ 0x163c, 0x00 },
	{ 0x163d, 0x00 },
	{ 0x163e, 0x00 },
	{ 0x163f, 0x00 },
	{ 0x1640, 0x00 },
	{ 0x1641, 0x00 },
	{ 0x1642, 0x00 },
	{ 0x1643, 0x00 },
	{ 0x1644, 0x00 },
	{ 0x1645, 0x00 },
	{ 0x1646, 0x00 },
	{ 0x1647, 0x00 },
	{ 0x1648, 0x00 },
	{ 0x1649, 0x00 },
	{ 0x164a, 0x00 },
	{ 0x164b, 0x00 },
	{ 0x164c, 0x00 },
	{ 0x164d, 0x00 },
	{ 0x164e, 0x00 },
	{ 0x164f, 0x00 },
	{ 0x1650, 0x55 },
	{ 0x1651, 0x85 },
	{ 0x1652, 0x01 },
	{ 0x1653, 0x84 },
	{ 0x1654, 0x52 },
	{ 0x1655, 0x04 },
	{ 0x1656, 0x00 },
	{ 0x1657, 0x08 },
	{ 0x1658, 0x01 },
	{ 0x1659, 0x01 },
	{ 0x165a, 0x01 },
	{ 0x165b, 0x01 },
	{ 0x165c, 0x01 },
	{ 0x165d, 0x04 },
	{ 0x165e, 0x01 },
	{ 0x165f, 0x01 },
	{ 0x1660, 0x01 },
	{ 0x1661, 0x01 },
	{ 0x1662, 0x01 },
	{ 0x1663, 0x00 },
	{ 0x1664, 0x08 },
	{ 0x1665, 0x00 },
	{ 0x1666, 0x00 },
	{ 0x1667, 0x00 },
	{ 0x1668, 0xff },
	{ 0x1669, 0xf4 },
	{ 0x166a, 0xcf },
	{ 0x166b, 0x03 },
	{ 0x166c, 0xff },
	{ 0x166d, 0x7e },
	{ 0x166e, 0x1e },
	{ 0x166f, 0x00 },
	{ 0x1670, 0xff },
	{ 0x1671, 0xff },
	{ 0x1672, 0x07 },
	{ 0x1673, 0x00 },
	{ 0x1674, 0x80 },
	{ 0x1675, 0x04 },
	{ 0x1676, 0x86 },
	{ 0x1677, 0x01 },
	{ 0x1678, 0x00 },
	{ 0x1679, 0x01 },
	{ 0x167a, 0x02 },
	{ 0x167b, 0x00 },
	{ 0x167c, 0x00 },
	{ 0x167d, 0x00 },
	{ 0x167e, 0x00 },
	{ 0x167f, 0x00 },
	{ 0x1680, 0x00 },
	{ 0x1681, 0x00 },
	{ 0x1682, 0x00 },
	{ 0x1683, 0x00 },
	{ 0x1684, 0x80 },
	{ 0x1685, 0x02 },
	{ 0x1686, 0x00 },
	{ 0x1687, 0x00 },
	{ 0x1688, 0x7f },
	{ 0x1689, 0x01 },
	{ 0x168a, 0x01 },
	{ 0x168b, 0x00 },
	{ 0x168c, 0x00 },
	{ 0x168d, 0x07 },
	{ 0x168e, 0x00 },
	{ 0x168f, 0x00 },
	{ 0x1690, 0x77 },
	{ 0x1691, 0x03 },
	{ 0x1692, 0x00 },
	{ 0x1693, 0x00 },
	{ 0x1694, 0xff },
	{ 0x1695, 0xff },
	{ 0x1696, 0x00 },
	{ 0x1697, 0x00 },
	{ 0x1698, 0x88 },
	{ 0x1699, 0xfc },
	{ 0x169a, 0x00 },
	{ 0x169b, 0x00 },
	{ 0x169c, 0x00 },
	{ 0x169d, 0x00 },
	{ 0x169e, 0x00 },
	{ 0x169f, 0x00 },
	{ 0x16a0, 0x04 },
	{ 0x16a1, 0x00 },
	{ 0x16a2, 0x00 },
	{ 0x16a3, 0x00 },
	{ 0x16a4, 0x00 },
	{ 0x16a5, 0x00 },
	{ 0x16a6, 0x00 },
	{ 0x16a7, 0x00 },
	{ 0x16a8, 0x00 },
	{ 0x16a9, 0x00 },
	{ 0x16aa, 0x00 },
	{ 0x16ab, 0x00 },
	{ 0x16ac, 0x00 },
	{ 0x16ad, 0x00 },
	{ 0x16ae, 0x00 },
	{ 0x16af, 0x00 },
	{ 0x16b0, 0x06 },
	{ 0x16b1, 0x06 },
	{ 0x16b2, 0x01 },
	{ 0x16b3, 0x00 },
	{ 0x16b4, 0x06 },
	{ 0x16b5, 0x00 },
	{ 0x16b6, 0x00 },
	{ 0x16b7, 0x00 },
	{ 0x16b8, 0x00 },
	{ 0x16b9, 0x00 },
	{ 0x16ba, 0x00 },
	{ 0x16bb, 0x00 },
	{ 0x16bc, 0x00 },
	{ 0x16bd, 0x00 },
	{ 0x16be, 0x00 },
	{ 0x16bf, 0x00 },
	{ 0x16c0, 0x80 },
	{ 0x16c1, 0x04 },
	{ 0x16c2, 0x14 },
	{ 0x16c3, 0x00 },
	{ 0x16c4, 0x93 },
	{ 0x16c5, 0x00 },
	{ 0x16c6, 0x00 },
	{ 0x16c7, 0x21 },
	{ 0x16c8, 0x58 },
	{ 0x16c9, 0xd0 },
	{ 0x16ca, 0x22 },
	{ 0x16cb, 0x06 },
	{ 0x16cc, 0x22 },
	{ 0x16cd, 0x51 },
	{ 0x16ce, 0x00 },
	{ 0x16cf, 0x22 },
	{ 0x16d0, 0x09 },
	{ 0x16d1, 0x00 },
	{ 0x16d2, 0x00 },
	{ 0x16d3, 0x1f },
	{ 0x16d4, 0x00 },
	{ 0x16d5, 0x00 },
	{ 0x16d6, 0x00 },
	{ 0x16d7, 0x00 },
	{ 0x16d8, 0x00 },
	{ 0x16d9, 0x00 },
	{ 0x16da, 0x00 },
	{ 0x16db, 0x00 },
	{ 0x16dc, 0x00 },
	{ 0x16dd, 0x00 },
	{ 0x16de, 0x00 },
	{ 0x16df, 0x00 },
	{ 0x16e0, 0x00 },
	{ 0x16e1, 0x00 },
	{ 0x16e2, 0x00 },
	{ 0x16e3, 0x00 },
	{ 0x16e4, 0x00 },
	{ 0x16e5, 0x00 },
	{ 0x16e6, 0x00 },
	{ 0x16e7, 0x00 },
	{ 0x16e8, 0x00 },
	{ 0x16e9, 0x00 },
	{ 0x16ea, 0x00 },
	{ 0x16eb, 0x00 },
	{ 0x16ec, 0x00 },
	{ 0x16ed, 0x00 },
	{ 0x16ee, 0x00 },
	{ 0x16ef, 0x00 },
	{ 0x16f0, 0x00 },
	{ 0x16f1, 0x00 },
	{ 0x16f2, 0x00 },
	{ 0x16f3, 0x00 },
	{ 0x16f4, 0x00 },
	{ 0x16f5, 0x00 },
	{ 0x16f6, 0x00 },
	{ 0x16f7, 0x00 },
	{ 0x16f8, 0x00 },
	{ 0x16f9, 0x00 },
	{ 0x16fa, 0x00 },
	{ 0x16fb, 0x00 },
	{ 0x16fc, 0x00 },
	{ 0x16fd, 0x00 },
	{ 0x16fe, 0x00 },
	{ 0x16ff, 0x00 },
	{ 0x1900, 0x33 },
	{ 0x1901, 0x08 },
	{ 0x1902, 0x10 },
	{ 0x1903, 0x02 },
	{ 0x1904, 0x03 },
	{ 0x1905, 0x28 },
	{ 0x1906, 0x00 },
	{ 0x1907, 0x33 },
	{ 0x1908, 0x00 },
	{ 0x1909, 0x00 },
	{ 0x190a, 0x00 },
	{ 0x190b, 0x00 },
	{ 0x190c, 0x00 },
	{ 0x190d, 0xff },
	{ 0x190e, 0x55 },
	{ 0x190f, 0xaa },
	{ 0x1910, 0xff },
	{ 0x1911, 0xfe },
	{ 0x1912, 0xff },
	{ 0x1913, 0xff },
	{ 0x1914, 0xff },
	{ 0x1915, 0xff },
	{ 0x1916, 0xff },
	{ 0x1917, 0xff },
	{ 0x1918, 0x00 },
	{ 0x1919, 0x00 },
	{ 0x191a, 0x00 },
	{ 0x191b, 0x00 },
	{ 0x191c, 0x00 },
	{ 0x191d, 0x00 },
	{ 0x191e, 0x00 },
	{ 0x191f, 0x00 },
	{ 0x1920, 0xff },
	{ 0x1921, 0xff },
	{ 0x1922, 0xff },
	{ 0x1923, 0xff },
	{ 0x1924, 0xff },
	{ 0x1925, 0x8f },
	{ 0x1926, 0xcb },
	{ 0x1927, 0xff },
	{ 0x1928, 0x00 },
	{ 0x1929, 0x00 },
	{ 0x192a, 0x00 },
	{ 0x192b, 0x00 },
	{ 0x192c, 0x00 },
	{ 0x192d, 0x00 },
	{ 0x192e, 0x00 },
	{ 0x192f, 0x00 },
	{ 0x1930, 0x00 },
	{ 0x1931, 0x00 },
	{ 0x1932, 0x00 },
	{ 0x1933, 0x00 },
	{ 0x1934, 0x00 },
	{ 0x1935, 0x00 },
	{ 0x1936, 0x00 },
	{ 0x1937, 0x00 },
	{ 0x1938, 0x00 },
	{ 0x1939, 0x00 },
	{ 0x193a, 0x00 },
	{ 0x193b, 0x00 },
	{ 0x193c, 0x00 },
	{ 0x193d, 0x00 },
	{ 0x193e, 0x00 },
	{ 0x193f, 0x00 },
	{ 0x1940, 0x00 },
	{ 0x1941, 0x00 },
	{ 0x1942, 0x00 },
	{ 0x1943, 0x00 },
	{ 0x1944, 0x00 },
	{ 0x1945, 0x00 },
	{ 0x1946, 0x00 },
	{ 0x1947, 0x00 },
	{ 0x1948, 0x00 },
	{ 0x1949, 0x00 },
	{ 0x194a, 0x00 },
	{ 0x194b, 0x00 },
	{ 0x194c, 0x00 },
	{ 0x194d, 0x00 },
	{ 0x194e, 0x00 },
	{ 0x194f, 0x00 },
	{ 0x1950, 0x00 },
	{ 0x1951, 0x00 },
	{ 0x1952, 0x00 },
	{ 0x1953, 0x00 },
	{ 0x1954, 0x00 },
	{ 0x1955, 0x00 },
	{ 0x1956, 0x10 },
	{ 0x1957, 0x00 },
	{ 0x1958, 0x00 },
	{ 0x1959, 0x00 },
	{ 0x195a, 0x00 },
	{ 0x195b, 0x00 },
	{ 0x195c, 0x00 },
	{ 0x195d, 0x00 },
	{ 0x195e, 0x00 },
	{ 0x195f, 0x00 },
	{ 0x1960, 0x00 },
	{ 0x1961, 0x00 },
	{ 0x1962, 0x00 },
	{ 0x1963, 0x00 },
	{ 0x1964, 0x00 },
	{ 0x1965, 0x00 },
	{ 0x1966, 0x00 },
	{ 0x1967, 0x00 },
	{ 0x1968, 0x00 },
	{ 0x1969, 0x00 },
	{ 0x196a, 0x00 },
	{ 0x196b, 0x00 },
	{ 0x196c, 0x00 },
	{ 0x196d, 0x00 },
	{ 0x196e, 0x00 },
	{ 0x196f, 0x00 },
	{ 0x1970, 0x00 },
	{ 0x1971, 0x00 },
	{ 0x1972, 0x00 },
	{ 0x1973, 0x00 },
	{ 0x1974, 0x00 },
	{ 0x1975, 0x00 },
	{ 0x1976, 0x00 },
	{ 0x1977, 0x00 },
	{ 0x1978, 0x00 },
	{ 0x1979, 0x00 },
	{ 0x197a, 0x00 },
	{ 0x197b, 0x00 },
	{ 0x197c, 0x00 },
	{ 0x197d, 0x00 },
	{ 0x197e, 0x00 },
	{ 0x197f, 0x00 },
	{ 0x1980, 0x00 },
	{ 0x1981, 0x00 },
	{ 0x1982, 0x00 },
	{ 0x1983, 0x00 },
	{ 0x1984, 0x00 },
	{ 0x1985, 0x70 },
	{ 0x1986, 0x34 },
	{ 0x1987, 0x00 },
	{ 0x1988, 0x00 },
	{ 0x1989, 0x00 },
	{ 0x198a, 0x00 },
	{ 0x198b, 0x00 },
	{ 0x198c, 0x00 },
	{ 0x198d, 0x00 },
	{ 0x198e, 0x00 },
	{ 0x198f, 0xc0 },
	{ 0x1990, 0xff },
	{ 0x1991, 0xff },
	{ 0x1992, 0xff },
	{ 0x1993, 0xff },
	{ 0x1994, 0xff },
	{ 0x1995, 0xff },
	{ 0x1996, 0xff },
	{ 0x1997, 0x3f },

	// // Watchdog
	// { 0x4f00, 0x00 },
	// { 0x4f01, 0x00 },
	// { 0x4f02, 0x80 },
	// { 0x4f03, 0x2c },
	// { 0x4f04, 0xf8 },

	{ REG_NULL, 0x00 },
};

static const struct regval ox03c10_1920x1536_2_lanes_regs[] = {
	// { 0x3012, 0x21 }, // SC_PHY_CTRL = 2 lane MIPI

	{ REG_NULL, 0x00 },
};

static const struct ox03c10_mode supported_modes[] = {
	{
		.width = 1920,
		.height = 1536,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x00f8,
		.hts_def = 0x0420,
		.vts_def = 0x069e,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR12_1X12,
		.reg_list = ox03c10_1920x1536_2_lanes_regs,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	}
};

static const s64 link_freq_menu_items[] = {
	OX03C10_LINK_FREQ_240
};

static const char * const ox03c10_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int ox03c10_write_reg(struct i2c_client *client, u16 reg,
			    u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;
	return 0;
}

static int ox03c10_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		if (regs[i].addr == REG_DELAY) {
        		usleep_range(regs[i].val * 1000, regs[i].val * 1000 + 1000);
            		continue;
        	}
		ret = ox03c10_write_reg(client, regs[i].addr,
					OX03C10_REG_VALUE_08BIT, regs[i].val);
	}
	return ret;
}

/* Read registers up to 4 at a time */
static int ox03c10_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
			    u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

static int ox03c10_get_reso_dist(const struct ox03c10_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct ox03c10_mode *
ox03c10_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = ox03c10_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int ox03c10_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct ox03c10 *ox03c10 = to_ox03c10(sd);
	const struct ox03c10_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&ox03c10->mutex);

	mode = ox03c10_find_best_fit(fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&ox03c10->mutex);
		return -ENOTTY;
#endif
	} else {
		ox03c10->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(ox03c10->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(ox03c10->vblank, vblank_def,
					 OX03C10_VTS_MAX - mode->height,
					 1, vblank_def);
		ox03c10->cur_fps = mode->max_fps;
	}

	mutex_unlock(&ox03c10->mutex);

	return 0;
}

static int ox03c10_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct ox03c10 *ox03c10 = to_ox03c10(sd);
	const struct ox03c10_mode *mode = ox03c10->cur_mode;

	mutex_lock(&ox03c10->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&ox03c10->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = mode->bus_fmt;
		fmt->format.field = V4L2_FIELD_NONE;
		/* format info: width/height/data type/virtual channel */
		if (fmt->pad < PAD_MAX && mode->hdr_mode != NO_HDR)
			fmt->reserved[0] = mode->vc[fmt->pad];
		else
			fmt->reserved[0] = mode->vc[PAD0];
	}
	mutex_unlock(&ox03c10->mutex);

	return 0;
}

static int ox03c10_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct ox03c10 *ox03c10 = to_ox03c10(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = ox03c10->cur_mode->bus_fmt;

	return 0;
}

static int ox03c10_enum_frame_sizes(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != supported_modes[0].bus_fmt)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int ox03c10_enable_test_pattern(struct ox03c10 *ox03c10, u32 pattern)
{
	u32 val;

	if (pattern)
		val = (pattern - 1) | OX03C10_TEST_PATTERN_ENABLE;
	else
		val = OX03C10_TEST_PATTERN_DISABLE;

	return ox03c10_write_reg(ox03c10->client, OX03C10_REG_TEST_PATTERN,
				OX03C10_REG_VALUE_08BIT, val);
}

static int ox03c10_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct ox03c10 *ox03c10 = to_ox03c10(sd);
	const struct ox03c10_mode *mode = ox03c10->cur_mode;

	if (ox03c10->streaming)
		fi->interval = ox03c10->cur_fps;
	else
		fi->interval = mode->max_fps;

	return 0;
}

static int ox03c10_g_mbus_config(struct v4l2_subdev *sd,
				unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	struct ox03c10 *ox03c10 = to_ox03c10(sd);
	const struct ox03c10_mode *mode = ox03c10->cur_mode;
	u32 val = 1 << (OX03C10_LANES - 1) |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	if (mode->hdr_mode != NO_HDR)
		val |= V4L2_MBUS_CSI2_CHANNEL_1;
	if (mode->hdr_mode == HDR_X3)
		val |= V4L2_MBUS_CSI2_CHANNEL_2;

	config->type = V4L2_MBUS_CSI2_DPHY;
	config->flags = val;

	return 0;
}

static void ox03c10_get_module_inf(struct ox03c10 *ox03c10,
				   struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, OX03C10_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, ox03c10->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, ox03c10->len_name, sizeof(inf->base.lens));
}

static long ox03c10_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct ox03c10 *ox03c10 = to_ox03c10(sd);
	struct rkmodule_hdr_cfg *hdr;
	u32 i, h, w;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		ox03c10_get_module_inf(ox03c10, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = ox03c10->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = ox03c10->cur_mode->width;
		h = ox03c10->cur_mode->height;
		for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr->hdr_mode) {
				ox03c10->cur_mode = &supported_modes[i];
				break;
			}
		}
		if (i == ARRAY_SIZE(supported_modes)) {
			dev_err(&ox03c10->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			w = ox03c10->cur_mode->hts_def - ox03c10->cur_mode->width;
			h = ox03c10->cur_mode->vts_def - ox03c10->cur_mode->height;
			__v4l2_ctrl_modify_range(ox03c10->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(ox03c10->vblank, h,
						 OX03C10_VTS_MAX - ox03c10->cur_mode->height, 1, h);
		}
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = ox03c10_write_reg(ox03c10->client, OX03C10_REG_CTRL_MODE,
				 OX03C10_REG_VALUE_08BIT, OX03C10_MODE_STREAMING);
		else
			ret = ox03c10_write_reg(ox03c10->client, OX03C10_REG_CTRL_MODE,
				 OX03C10_REG_VALUE_08BIT, OX03C10_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long ox03c10_compat_ioctl32(struct v4l2_subdev *sd,
				   unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;
	long ret;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = ox03c10_ioctl(sd, cmd, inf);
		if (!ret) {
			if (copy_to_user(up, inf, sizeof(*inf)))
				ret = -EFAULT;
		}
		kfree(inf);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = ox03c10_ioctl(sd, cmd, hdr);
		if (!ret) {
			if (copy_to_user(up, hdr, sizeof(*hdr)))
				ret = -EFAULT;
		}
		kfree(hdr);
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(hdr, up, sizeof(*hdr));
		if (!ret)
			ret = ox03c10_ioctl(sd, cmd, hdr);
		else
			ret = -EFAULT;
		kfree(hdr);
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		hdrae = kzalloc(sizeof(*hdrae), GFP_KERNEL);
		if (!hdrae) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(hdrae, up, sizeof(*hdrae));
		if (!ret)
			ret = ox03c10_ioctl(sd, cmd, hdrae);
		else
			ret = -EFAULT;
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = ox03c10_ioctl(sd, cmd, &stream);
		else
			ret = -EFAULT;
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __ox03c10_start_stream(struct ox03c10 *ox03c10)
{
	int ret;

	ret = ox03c10_write_array(ox03c10->client, ox03c10->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	ret = __v4l2_ctrl_handler_setup(&ox03c10->ctrl_handler);
	if (ret)
		return ret;

	return ox03c10_write_reg(ox03c10->client, OX03C10_REG_CTRL_MODE,
				 OX03C10_REG_VALUE_08BIT, OX03C10_MODE_STREAMING);
}

static int __ox03c10_stop_stream(struct ox03c10 *ox03c10)
{
	return ox03c10_write_reg(ox03c10->client, OX03C10_REG_CTRL_MODE,
				 OX03C10_REG_VALUE_08BIT, OX03C10_MODE_SW_STANDBY);
}

static int ox03c10_s_stream(struct v4l2_subdev *sd, int on)
{
	struct ox03c10 *ox03c10 = to_ox03c10(sd);
	struct i2c_client *client = ox03c10->client;
	int ret = 0;

	mutex_lock(&ox03c10->mutex);
	on = !!on;
	if (on == ox03c10->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __ox03c10_start_stream(ox03c10);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__ox03c10_stop_stream(ox03c10);
		pm_runtime_put(&client->dev);
	}

	ox03c10->streaming = on;

unlock_and_return:
	mutex_unlock(&ox03c10->mutex);

	return ret;
}

static int ox03c10_s_power(struct v4l2_subdev *sd, int on)
{
	struct ox03c10 *ox03c10 = to_ox03c10(sd);
	struct i2c_client *client = ox03c10->client;
	int ret = 0;

	mutex_lock(&ox03c10->mutex);

	/* If the power state is not modified - no work to do. */
	if (ox03c10->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = ox03c10_write_array(ox03c10->client, ox03c10_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ox03c10->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		ox03c10->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&ox03c10->mutex);

	return ret;
}

static int __ox03c10_power_on(struct ox03c10 *ox03c10)
{
	int ret;
	struct device *dev = &ox03c10->client->dev;

	if (!IS_ERR_OR_NULL(ox03c10->pins_default)) {
		ret = pinctrl_select_state(ox03c10->pinctrl,
					   ox03c10->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(ox03c10->xvclk, OX03C10_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(ox03c10->xvclk) != OX03C10_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(ox03c10->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (!IS_ERR(ox03c10->reset_gpio))
		gpiod_set_value_cansleep(ox03c10->reset_gpio, 0);

	ret = regulator_bulk_enable(OX03C10_NUM_SUPPLIES, ox03c10->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	usleep_range(5 * 1000, 10 * 1000);
	if (!IS_ERR(ox03c10->reset_gpio))
		gpiod_set_value_cansleep(ox03c10->reset_gpio, 1);

	usleep_range(16 * 1000, 32 * 1000);

	return 0;

disable_clk:
	clk_disable_unprepare(ox03c10->xvclk);

	return ret;
}

static void __ox03c10_power_off(struct ox03c10 *ox03c10)
{
	int ret;
	struct device *dev = &ox03c10->client->dev;

	clk_disable_unprepare(ox03c10->xvclk);
	if (!IS_ERR(ox03c10->reset_gpio))
		gpiod_direction_output(ox03c10->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(ox03c10->pins_sleep)) {
		ret = pinctrl_select_state(ox03c10->pinctrl,
					   ox03c10->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}

	regulator_bulk_disable(OX03C10_NUM_SUPPLIES, ox03c10->supplies);
}

static int ox03c10_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ox03c10 *ox03c10 = to_ox03c10(sd);

	return __ox03c10_power_on(ox03c10);
}

static int ox03c10_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ox03c10 *ox03c10 = to_ox03c10(sd);

	__ox03c10_power_off(ox03c10);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int ox03c10_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ox03c10 *ox03c10 = to_ox03c10(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct ox03c10_mode *def_mode = &supported_modes[0];

	mutex_lock(&ox03c10->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&ox03c10->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int ox03c10_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	fie->code = supported_modes[fie->index].bus_fmt;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static const struct dev_pm_ops ox03c10_pm_ops = {
	SET_RUNTIME_PM_OPS(ox03c10_runtime_suspend,
			   ox03c10_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops ox03c10_internal_ops = {
	.open = ox03c10_open,
};
#endif

static const struct v4l2_subdev_core_ops ox03c10_core_ops = {
	.s_power = ox03c10_s_power,
	.ioctl = ox03c10_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = ox03c10_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops ox03c10_video_ops = {
	.s_stream = ox03c10_s_stream,
	.g_frame_interval = ox03c10_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops ox03c10_pad_ops = {
	.enum_mbus_code = ox03c10_enum_mbus_code,
	.enum_frame_size = ox03c10_enum_frame_sizes,
	.enum_frame_interval = ox03c10_enum_frame_interval,
	.get_fmt = ox03c10_get_fmt,
	.set_fmt = ox03c10_set_fmt,
	.get_mbus_config = ox03c10_g_mbus_config,
};

static const struct v4l2_subdev_ops ox03c10_subdev_ops = {
	.core	= &ox03c10_core_ops,
	.video	= &ox03c10_video_ops,
	.pad	= &ox03c10_pad_ops,
};

static int ox03c10_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ox03c10 *ox03c10 = container_of(ctrl->handler,
					       struct ox03c10, ctrl_handler);
	struct i2c_client *client = ox03c10->client;
	s64 max;
	int ret = 0;
	u32 val = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = ox03c10->cur_mode->height + ctrl->val - 20;
		__v4l2_ctrl_modify_range(ox03c10->exposure,
					 ox03c10->exposure->minimum, max,
					 ox03c10->exposure->step,
					 ox03c10->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		ret = ox03c10_write_reg(ox03c10->client, OX03C10_REG_EXPOSURE,
				       OX03C10_REG_VALUE_24BIT, ctrl->val << 4);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = ox03c10_write_reg(ox03c10->client, OX03C10_REG_ANALOG_GAIN,
				       OX03C10_REG_VALUE_16BIT,
				       ctrl->val & ANALOG_GAIN_MASK);
		break;
	case V4L2_CID_VBLANK:
		ret = ox03c10_write_reg(ox03c10->client, OX03C10_REG_VTS,
				       OX03C10_REG_VALUE_16BIT,
				       ctrl->val + ox03c10->cur_mode->height);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = ox03c10_enable_test_pattern(ox03c10, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = ox03c10_read_reg(ox03c10->client, OX03C10_MIRROR_REG,
				       OX03C10_REG_VALUE_08BIT, &val);
		ret |= ox03c10_write_reg(ox03c10->client, OX03C10_MIRROR_REG,
					 OX03C10_REG_VALUE_08BIT,
					 OX03C10_FETCH_MIRROR(val, ctrl->val));
		break;
	case V4L2_CID_VFLIP:
		ret = ox03c10_read_reg(ox03c10->client, OX03C10_FLIP_REG,
				       OX03C10_REG_VALUE_08BIT, &val);
		ret |= ox03c10_write_reg(ox03c10->client, OX03C10_FLIP_REG,
					 OX03C10_REG_VALUE_08BIT,
					 OX03C10_FETCH_FLIP(val, ctrl->val));
		break;

	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ox03c10_ctrl_ops = {
	.s_ctrl = ox03c10_set_ctrl,
};

static int ox03c10_initialize_controls(struct ox03c10 *ox03c10)
{
	const struct ox03c10_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &ox03c10->ctrl_handler;
	mode = ox03c10->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &ox03c10->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, PIXEL_RATE_WITH_240M_12BIT, 1, PIXEL_RATE_WITH_240M_12BIT);

	h_blank = mode->hts_def - mode->width;
	ox03c10->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					    h_blank, h_blank, 1, h_blank);
	if (ox03c10->hblank)
		ox03c10->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	vblank_def = mode->vts_def - mode->height;
	ox03c10->vblank = v4l2_ctrl_new_std(handler, &ox03c10_ctrl_ops,
					    V4L2_CID_VBLANK, vblank_def,
					    OX03C10_VTS_MAX - mode->height,
					    1, vblank_def);
	ox03c10->cur_fps = mode->max_fps;
	exposure_max = mode->vts_def - 20;
	ox03c10->exposure = v4l2_ctrl_new_std(handler, &ox03c10_ctrl_ops,
					      V4L2_CID_EXPOSURE, OX03C10_EXPOSURE_MIN,
					      exposure_max, OX03C10_EXPOSURE_STEP,
					      mode->exp_def);
	ox03c10->anal_gain = v4l2_ctrl_new_std(handler, &ox03c10_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, ANALOG_GAIN_MIN,
				ANALOG_GAIN_MAX, ANALOG_GAIN_STEP,
				ANALOG_GAIN_DEFAULT);

	ox03c10->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
							    &ox03c10_ctrl_ops,
					V4L2_CID_TEST_PATTERN,
					ARRAY_SIZE(ox03c10_test_pattern_menu) - 1,
					0, 0, ox03c10_test_pattern_menu);
	v4l2_ctrl_new_std(handler, &ox03c10_ctrl_ops,
				V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(handler, &ox03c10_ctrl_ops,
				V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (handler->error) {
		ret = handler->error;
		dev_err(&ox03c10->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	ox03c10->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int ox03c10_check_sensor_id(struct ox03c10 *ox03c10,
				   struct i2c_client *client)
{
	struct device *dev = &ox03c10->client->dev;
	u32 id = 0;
	int ret;

	ret = ox03c10_read_reg(client, OX03C10_REG_CHIP_ID,
			       OX03C10_REG_VALUE_24BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected OX03C10 sensor\n");

	return 0;
}

static int ox03c10_configure_regulators(struct ox03c10 *ox03c10)
{
	unsigned int i;

	for (i = 0; i < OX03C10_NUM_SUPPLIES; i++)
		ox03c10->supplies[i].supply = ox03c10_supply_names[i];

	return devm_regulator_bulk_get(&ox03c10->client->dev,
				       OX03C10_NUM_SUPPLIES,
				       ox03c10->supplies);
}

static int ox03c10_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct ox03c10 *ox03c10;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	ox03c10 = devm_kzalloc(dev, sizeof(*ox03c10), GFP_KERNEL);
	if (!ox03c10)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &ox03c10->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &ox03c10->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &ox03c10->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &ox03c10->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	ox03c10->client = client;
	ox03c10->cur_mode = &supported_modes[0];

	ox03c10->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(ox03c10->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	ox03c10->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ox03c10->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	ox03c10->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(ox03c10->pinctrl)) {
		ox03c10->pins_default =
			pinctrl_lookup_state(ox03c10->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(ox03c10->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		ox03c10->pins_sleep =
			pinctrl_lookup_state(ox03c10->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(ox03c10->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = ox03c10_configure_regulators(ox03c10);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&ox03c10->mutex);

	sd = &ox03c10->subdev;
	v4l2_i2c_subdev_init(sd, client, &ox03c10_subdev_ops);
	ret = ox03c10_initialize_controls(ox03c10);
	if (ret)
		goto err_destroy_mutex;

	ret = __ox03c10_power_on(ox03c10);
	if (ret)
		goto err_free_handler;

	ret = ox03c10_check_sensor_id(ox03c10, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &ox03c10_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	ox03c10->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &ox03c10->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(ox03c10->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 ox03c10->module_index, facing,
		 OX03C10_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__ox03c10_power_off(ox03c10);
err_free_handler:
	v4l2_ctrl_handler_free(&ox03c10->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&ox03c10->mutex);

	return ret;
}

static int ox03c10_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ox03c10 *ox03c10 = to_ox03c10(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&ox03c10->ctrl_handler);
	mutex_destroy(&ox03c10->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__ox03c10_power_off(ox03c10);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ox03c10_of_match[] = {
	{ .compatible = "ovti,ox03c10" },
	{},
};
MODULE_DEVICE_TABLE(of, ox03c10_of_match);
#endif

static const struct i2c_device_id ox03c10_match_id[] = {
	{ "ovti,ox03c10", 0 },
	{ },
};

static struct i2c_driver ox03c10_i2c_driver = {
	.driver = {
		.name = OX03C10_NAME,
		.pm = &ox03c10_pm_ops,
		.of_match_table = of_match_ptr(ox03c10_of_match),
	},
	.probe		= &ox03c10_probe,
	.remove		= &ox03c10_remove,
	.id_table	= ox03c10_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&ox03c10_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&ox03c10_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_AUTHOR("O(log n) Technology Co., Ltd.");
MODULE_DESCRIPTION("OmniVision OX03C10 sensor driver");
MODULE_LICENSE("GPL v2");
