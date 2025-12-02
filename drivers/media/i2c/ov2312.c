// SPDX-License-Identifier: GPL-2.0
/*
 * OmniVision OV2312 sensor driver
 *
 * Copyright (C) 2025 O(log n) Technology Co., Ltd.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/rk-camera-module.h>
#include <linux/version.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x01)

#define MIPI_FREQ_160M			160000000

#define OV2312_LANES			2
#define OV2312_BITS_PER_SAMPLE		8

#define PIXEL_RATE_WITH_160M		(MIPI_FREQ_160M / OV2312_BITS_PER_SAMPLE * 2 * OV2312_LANES)

#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"

#define OV2312_XVCLK_FREQ		24000000

#define CHIP_ID				0x2311
#define OV2312_REG_CHIP_ID		0x300a

#define OV2312_REG_CTRL_MODE		0x0100
#define OV2312_MODE_SW_STANDBY		0x0
#define OV2312_MODE_STREAMING		BIT(0)

#define OV2312_REG_V_FLIP		0x3820
#define OV2312_REG_H_FLIP		0x3821
#define OV2312_FLIP_BIT			BIT(2)

#define OV2312_REG_EXPOSURE		0x3501
#define OV2312_EXPOSURE_MIN		4
#define OV2312_EXPOSURE_STEP		1
#define OV2312_VTS_MAX			0xffff

#define OV2312_REG_GAIN_H		0x3508
#define OV2312_REG_GAIN_L		0x3509
#define OV2312_GAIN_H_MASK		0x07
#define OV2312_GAIN_H_SHIFT		8
#define OV2312_GAIN_L_MASK		0xff
#define OV2312_GAIN_MIN			0x100
#define OV2312_GAIN_MAX			0x780
#define OV2312_GAIN_STEP		1
#define OV2312_GAIN_DEFAULT		OV2312_GAIN_MIN

#define OV2312_REG_TEST_PATTERN		0x5e00
#define OV2312_TEST_PATTERN_ENABLE	0x80
#define OV2312_TEST_PATTERN_DISABLE	0x0

#define OV2312_REG_VTS			0x380e

#define REG_NULL			0xFFFF

#define OV2312_REG_VALUE_08BIT		1
#define OV2312_REG_VALUE_16BIT		2

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define OV2312_NAME			"ov2312"

static const char *const ov2312_supply_names[] = {
	"avdd", /* Analog power */
	"dovdd", /* Digital I/O power */
	"dvdd", /* Digital core power */
};

#define OV2312_NUM_SUPPLIES ARRAY_SIZE(ov2312_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct ov2312_mode {
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

struct ov2312 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct regulator_bulk_data supplies[OV2312_NUM_SUPPLIES];

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct ov2312_mode *cur_mode;
	struct v4l2_fract	cur_fps;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
};

#define to_ov2312(sd) container_of(sd, struct ov2312, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval ov2312_1600x1296_regs[] = {
	{ 0x0103, 0x01 },
	{ 0x0100, 0x00 },
	{ 0x0106, 0x00 },
	{ 0x010c, 0x02 },
	{ 0x010b, 0x01 },
	{ 0x0300, 0x01 },
	{ 0x0302, 0x32 },
	{ 0x0303, 0x00 },
	{ 0x0304, 0x03 },
	{ 0x0305, 0x02 },
	{ 0x0306, 0x01 },
	{ 0x030d, 0x70 },
	{ 0x030e, 0x04 },
	{ 0x3001, 0x02 },
	{ 0x3004, 0x00 },
	{ 0x3005, 0x00 },
	{ 0x3006, 0x0a },
	{ 0x3011, 0x0d },
	{ 0x3014, 0x04 },
	{ 0x301c, 0xf0 },
	{ 0x3020, 0x20 },
	{ 0x302c, 0x00 },
	{ 0x302d, 0x12 },
	{ 0x302e, 0x33 },
	{ 0x302f, 0x84 },
	{ 0x3030, 0x10 },
	{ 0x303f, 0x03 },
	{ 0x3103, 0x00 },
	{ 0x3106, 0x08 },
	{ 0x31ff, 0x01 },
	{ 0x3206, 0xc0 },
	{ 0x3501, 0x05 },
	{ 0x3502, 0x7c },
	{ 0x3506, 0x00 },
	{ 0x3507, 0x00 },
	{ 0x3620, 0x67 },
	{ 0x3633, 0x78 },
	{ 0x3662, 0x67 },
	{ 0x3664, 0xb0 },
	{ 0x3666, 0x70 },
	{ 0x3670, 0x68 },
	{ 0x3674, 0x10 },
	{ 0x3675, 0x00 },
	{ 0x367e, 0x90 },
	{ 0x3680, 0x84 },
	{ 0x3683, 0x96 },
	{ 0x36a2, 0x04 },
	{ 0x36a3, 0x80 },
	{ 0x36b0, 0x00 },
	{ 0x3700, 0x35 },
	{ 0x3704, 0x39 },
	{ 0x370a, 0x50 },
	{ 0x3712, 0x00 },
	{ 0x3713, 0x02 },
	{ 0x3778, 0x00 },
	{ 0x379b, 0x01 },
	{ 0x379c, 0x10 },
	{ 0x3800, 0x00 },
	{ 0x3801, 0x00 },
	{ 0x3802, 0x00 },
	{ 0x3803, 0x00 },
	{ 0x3804, 0x06 },
	{ 0x3805, 0x4f },
	{ 0x3806, 0x05 },
	{ 0x3807, 0x23 },
	{ 0x3808, 0x06 },
	{ 0x3809, 0x40 },
	{ 0x380a, 0x05 },
	{ 0x380b, 0x10 },
	{ 0x380c, 0x03 },
	{ 0x380d, 0x88 },
	{ 0x380e, 0x05 },
	{ 0x380f, 0x88 },
	{ 0x3810, 0x00 },
	{ 0x3811, 0x0a },
	{ 0x3812, 0x00 },
	{ 0x3813, 0x08 },
	{ 0x3814, 0x11 },
	{ 0x3815, 0x11 },
	{ 0x3816, 0x00 },
	{ 0x3817, 0x01 },
	{ 0x3818, 0x00 },
	{ 0x3819, 0x05 },
	{ 0x3820, 0x00 },
	{ 0x3821, 0x00 },
	{ 0x382b, 0x32 },
	{ 0x382c, 0x0a },
	{ 0x382d, 0xf8 },
	{ 0x3881, 0x44 },
	{ 0x3882, 0x02 },
	{ 0x3883, 0x8c },
	{ 0x3885, 0x07 },
	{ 0x389d, 0x03 },
	{ 0x38a6, 0x00 },
	{ 0x38a7, 0x01 },
	{ 0x38b3, 0x07 },
	{ 0x38b1, 0x03 },
	{ 0x38e5, 0x02 },
	{ 0x38e7, 0x00 },
	{ 0x38e8, 0x00 },
	{ 0x3910, 0xff },
	{ 0x3911, 0xff },
	{ 0x3912, 0x08 },
	{ 0x3913, 0x00 },
	{ 0x3914, 0x00 },
	{ 0x3915, 0x00 },
	{ 0x391c, 0x00 },
	{ 0x3920, 0xff },
	{ 0x3921, 0x80 },
	{ 0x3922, 0x00 },
	{ 0x3923, 0x00 },
	{ 0x3924, 0x05 },
	{ 0x3925, 0x00 },
	{ 0x3926, 0x00 },
	{ 0x3927, 0x00 },
	{ 0x3928, 0x1a },
	{ 0x392d, 0x03 },
	{ 0x392e, 0xa8 },
	{ 0x392f, 0x08 },
	{ 0x4001, 0x00 },
	{ 0x4003, 0x40 },
	{ 0x4008, 0x04 },
	{ 0x4009, 0x1b },
	{ 0x400c, 0x04 },
	{ 0x400d, 0x1b },
	{ 0x4010, 0xf4 },
	{ 0x4011, 0x00 },
	{ 0x4016, 0x00 },
	{ 0x4017, 0x04 },
	{ 0x4042, 0x11 },
	{ 0x4043, 0x70 },
	{ 0x4045, 0x00 },
	{ 0x4409, 0x1f },
	{ 0x440e, 0xff },
	{ 0x4509, 0x00 },
	{ 0x450b, 0x00 },
	{ 0x4600, 0x00 },
	{ 0x4601, 0x64 },
	{ 0x4708, 0x09 },
	{ 0x470c, 0x81 },
	{ 0x4710, 0x06 },
	{ 0x4711, 0x00 },
	{ 0x4800, 0x00 },
	{ 0x481f, 0x30 },
	{ 0x4837, 0x14 },
	{ 0x4f00, 0x00 },
	{ 0x4f07, 0x00 },
	{ 0x4f08, 0x03 },
	{ 0x4f09, 0x08 },
	{ 0x4f0c, 0x05 },
	{ 0x4f0d, 0xb4 },
	{ 0x4f10, 0x00 },
	{ 0x4f11, 0x00 },
	{ 0x4f12, 0x07 },
	{ 0x4f13, 0xe2 },
	{ 0x5000, 0x17 },
	{ 0x5001, 0x20 },
	{ 0x5026, 0x00 },
	{ 0x5b00, 0x03 },
	{ 0x5c00, 0x00 },
	{ 0x5c01, 0x2c },
	{ 0x5c02, 0x00 },
	{ 0x5c03, 0x7f },
	{ 0x5e00, 0x00 },
	{ 0x5e01, 0x41 },

	{ 0x3501, 0x05 },
	{ 0x3502, 0x7c },
	{ REG_NULL, 0x00 },
};

/*
 * The width and height must be configured to be
 * the same as the current output resolution of the sensor.
 * The input width of the isp needs to be 16 aligned.
 * The input height of the isp needs to be 8 aligned.
 * If the width or height does not meet the alignment rules,
 * you can configure the cropping parameters with the following function to
 * crop out the appropriate resolution.
 * struct v4l2_subdev_pad_ops {
 *	.get_selection
 * }
 */
static const struct ov2312_mode supported_modes[] = {
	{
		.width = 1600,
		.height = 1296,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.bus_fmt = MEDIA_BUS_FMT_SBGGR8_1X8,
		.exp_def = 0x0320,
		.hts_def = (0x0388 * 2),/* Registers 0x380c / 0x380d  * 2 */
		.vts_def = 0x5c2,	/* Registers 0x380e / 0x380f
					 * 60fps for 10bpp
					 */
		.reg_list = ov2312_1600x1296_regs,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const s64 link_freq_menu_items[] = {
	MIPI_FREQ_160M,
};

static const char * const ov2312_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int ov2312_write_reg(struct i2c_client *client, u16 reg, u32 len,
			    u32 val)
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

static int ov2312_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = ov2312_write_reg(client, regs[i].addr,
				       OV2312_REG_VALUE_08BIT, regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int ov2312_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
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

static int ov2312_get_reso_dist(const struct ov2312_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct ov2312_mode *
ov2312_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = ov2312_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int ov2312_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ov2312 *ov2312 = to_ov2312(sd);
	const struct ov2312_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&ov2312->mutex);

	mode = ov2312_find_best_fit(fmt);
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = mode->bus_fmt;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&ov2312->mutex);
		return -ENOTTY;
#endif
	} else {
		ov2312->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(ov2312->hblank, h_blank, h_blank, 1,
					 h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(ov2312->vblank, vblank_def,
					 OV2312_VTS_MAX - mode->height, 1,
					 vblank_def);
		ov2312->cur_fps = mode->max_fps;
	}

	mutex_unlock(&ov2312->mutex);

	return 0;
}

static int ov2312_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ov2312 *ov2312 = to_ov2312(sd);
	const struct ov2312_mode *mode = ov2312->cur_mode;

	mutex_lock(&ov2312->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&ov2312->mutex);
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
	mutex_unlock(&ov2312->mutex);

	return 0;
}

static int ov2312_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ov2312 *ov2312 = to_ov2312(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = ov2312->cur_mode->bus_fmt;

	return 0;
}

static int ov2312_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != supported_modes[0].bus_fmt)
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int ov2312_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ov2312 *ov2312 = to_ov2312(sd);
	const struct ov2312_mode *mode = ov2312->cur_mode;

	if (ov2312->streaming)
		fi->interval = ov2312->cur_fps;
	else
		fi->interval = mode->max_fps;

	return 0;
}

static int ov2312_enable_test_pattern(struct ov2312 *ov2312, u32 pattern)
{
	u32 val;

	if (pattern)
		val = (pattern - 1) | OV2312_TEST_PATTERN_ENABLE;
	else
		val = OV2312_TEST_PATTERN_DISABLE;

	return ov2312_write_reg(ov2312->client, OV2312_REG_TEST_PATTERN,
				OV2312_REG_VALUE_08BIT, val);
}

static int ov2312_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	struct ov2312 *ov2312 = to_ov2312(sd);
	const struct ov2312_mode *mode = ov2312->cur_mode;
	u32 val = 1 << (OV2312_LANES - 1) | V4L2_MBUS_CSI2_CHANNEL_0 |
		  V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	if (mode->hdr_mode != NO_HDR)
		val |= V4L2_MBUS_CSI2_CHANNEL_1;
	if (mode->hdr_mode == HDR_X3)
		val |= V4L2_MBUS_CSI2_CHANNEL_2;

	config->type = V4L2_MBUS_CSI2_DPHY;
	config->flags = val;

	return 0;
}

static int ov2312_get_channel_info(struct ov2312 *ov2312,
				   struct rkmodule_channel_info *ch_info)
{
	if (ch_info->index < PAD0 || ch_info->index >= PAD_MAX)
		return -EINVAL;
	ch_info->vc = ov2312->cur_mode->vc[ch_info->index];
	ch_info->width = ov2312->cur_mode->width;
	ch_info->height = ov2312->cur_mode->height;
	ch_info->bus_fmt = ov2312->cur_mode->bus_fmt;
	return 0;
}

static void ov2312_get_module_inf(struct ov2312 *ov2312,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, OV2312_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, ov2312->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, ov2312->len_name, sizeof(inf->base.lens));
}

static long ov2312_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct ov2312 *ov2312 = to_ov2312(sd);
	struct rkmodule_channel_info *ch_info;
	struct rkmodule_hdr_cfg *hdr;
	long ret = 0;
	u32 i, h, w;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = (struct rkmodule_channel_info *)arg;
		ret = ov2312_get_channel_info(ov2312, ch_info);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = ov2312->cur_mode->hdr_mode;
		break;
	case RKMODULE_GET_MODULE_INFO:
		ov2312_get_module_inf(ov2312, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = ov2312->cur_mode->width;
		h = ov2312->cur_mode->height;
		for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr->hdr_mode) {
				ov2312->cur_mode = &supported_modes[i];
				break;
			}
		}
		if (i == ARRAY_SIZE(supported_modes)) {
			dev_err(&ov2312->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			w = ov2312->cur_mode->hts_def - ov2312->cur_mode->width;
			h = ov2312->cur_mode->vts_def -
			    ov2312->cur_mode->height;
			__v4l2_ctrl_modify_range(ov2312->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(
				ov2312->vblank, h,
				OV2312_VTS_MAX - ov2312->cur_mode->height, 1,
				h);
		}
		break;
	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);

		if (stream)
			ret = ov2312_write_reg(ov2312->client,
					       OV2312_REG_CTRL_MODE,
					       OV2312_REG_VALUE_08BIT,
					       OV2312_MODE_STREAMING);
		else
			ret = ov2312_write_reg(ov2312->client,
					       OV2312_REG_CTRL_MODE,
					       OV2312_REG_VALUE_08BIT,
					       OV2312_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long ov2312_compat_ioctl32(struct v4l2_subdev *sd, unsigned int cmd,
				  unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_hdr_cfg *hdr;
	long ret;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = ov2312_ioctl(sd, cmd, hdr);
		if (!ret) {
			if (copy_to_user(up, hdr, sizeof(*hdr)))
				ret = -EFAULT;
		}
		kfree(hdr);
		break;
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = ov2312_ioctl(sd, cmd, inf);
		if (!ret) {
			if (copy_to_user(up, inf, sizeof(*inf)))
				ret = -EFAULT;
		}
		kfree(inf);
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(hdr, up, sizeof(*hdr));
		if (!ret)
			ret = ov2312_ioctl(sd, cmd, hdr);
		else
			ret = -EFAULT;
		kfree(hdr);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = ov2312_ioctl(sd, cmd, &stream);
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

static int __ov2312_start_stream(struct ov2312 *ov2312)
{
	int ret;

	ret = ov2312_write_array(ov2312->client, ov2312->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	ret = __v4l2_ctrl_handler_setup(&ov2312->ctrl_handler);
	if (ret)
		return ret;

	return ov2312_write_reg(ov2312->client, OV2312_REG_CTRL_MODE,
				OV2312_REG_VALUE_08BIT, OV2312_MODE_STREAMING);
}

static int __ov2312_stop_stream(struct ov2312 *ov2312)
{
	return ov2312_write_reg(ov2312->client, OV2312_REG_CTRL_MODE,
				OV2312_REG_VALUE_08BIT, OV2312_MODE_SW_STANDBY);
}

static int ov2312_s_stream(struct v4l2_subdev *sd, int on)
{
	struct ov2312 *ov2312 = to_ov2312(sd);
	struct i2c_client *client = ov2312->client;
	int ret = 0;

	mutex_lock(&ov2312->mutex);
	on = !!on;
	if (on == ov2312->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __ov2312_start_stream(ov2312);
		if (ret) {
			dev_err(&ov2312->client->dev,
				"start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__ov2312_stop_stream(ov2312);
		pm_runtime_put(&client->dev);
	}

	ov2312->streaming = on;

unlock_and_return:
	mutex_unlock(&ov2312->mutex);

	return ret;
}

static int ov2312_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov2312 *ov2312 = to_ov2312(sd);
	struct i2c_client *client = ov2312->client;
	int ret = 0;

	mutex_lock(&ov2312->mutex);

	/* If the power state is not modified - no work to do. */
	if (ov2312->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ov2312->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		ov2312->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&ov2312->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 ov2312_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, OV2312_XVCLK_FREQ / 1000 / 1000);
}

static int __ov2312_power_on(struct ov2312 *ov2312)
{
	int ret;
	u32 delay_us;
	struct device *dev = &ov2312->client->dev;

	if (!IS_ERR_OR_NULL(ov2312->pins_default)) {
		ret = pinctrl_select_state(ov2312->pinctrl,
					   ov2312->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(ov2312->xvclk, OV2312_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(ov2312->xvclk) != OV2312_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(ov2312->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (!IS_ERR(ov2312->reset_gpio))
		gpiod_set_value_cansleep(ov2312->reset_gpio, 0);

	ret = regulator_bulk_enable(OV2312_NUM_SUPPLIES, ov2312->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	usleep_range(5 * 1000, 10 * 1000);
	if (!IS_ERR(ov2312->reset_gpio))
		gpiod_set_value_cansleep(ov2312->reset_gpio, 1);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = ov2312_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(ov2312->xvclk);

	return ret;
}

static void __ov2312_power_off(struct ov2312 *ov2312)
{
	int ret;
	struct device *dev = &ov2312->client->dev;

	clk_disable_unprepare(ov2312->xvclk);
	if (!IS_ERR(ov2312->reset_gpio))
		gpiod_direction_output(ov2312->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(ov2312->pins_sleep)) {
		ret = pinctrl_select_state(ov2312->pinctrl, ov2312->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}

	regulator_bulk_disable(OV2312_NUM_SUPPLIES, ov2312->supplies);
}

static int ov2312_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov2312 *ov2312 = to_ov2312(sd);

	return __ov2312_power_on(ov2312);
}

static int ov2312_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov2312 *ov2312 = to_ov2312(sd);

	__ov2312_power_off(ov2312);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int ov2312_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ov2312 *ov2312 = to_ov2312(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct ov2312_mode *def_mode = &supported_modes[0];

	mutex_lock(&ov2312->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&ov2312->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int
ov2312_enum_frame_interval(struct v4l2_subdev *sd,
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

static const struct dev_pm_ops ov2312_pm_ops = {
	SET_RUNTIME_PM_OPS(ov2312_runtime_suspend,
			   ov2312_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops ov2312_internal_ops = {
	.open = ov2312_open,
};
#endif

static const struct v4l2_subdev_core_ops ov2312_core_ops = {
	.s_power = ov2312_s_power,
	.ioctl = ov2312_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = ov2312_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops ov2312_video_ops = {
	.s_stream = ov2312_s_stream,
	.g_frame_interval = ov2312_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops ov2312_pad_ops = {
	.enum_mbus_code = ov2312_enum_mbus_code,
	.enum_frame_size = ov2312_enum_frame_sizes,
	.enum_frame_interval = ov2312_enum_frame_interval,
	.get_fmt = ov2312_get_fmt,
	.set_fmt = ov2312_set_fmt,
	.get_mbus_config = ov2312_g_mbus_config,
};

static const struct v4l2_subdev_ops ov2312_subdev_ops = {
	.core = &ov2312_core_ops,
	.video = &ov2312_video_ops,
	.pad = &ov2312_pad_ops,
};

static int ov2312_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov2312 *ov2312 =
		container_of(ctrl->handler, struct ov2312, ctrl_handler);
	struct i2c_client *client = ov2312->client;
	s64 max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = ov2312->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(ov2312->exposure,
					 ov2312->exposure->minimum, max,
					 ov2312->exposure->step,
					 ov2312->exposure->default_value);
		break;
	}

	if (pm_runtime_get(&client->dev) <= 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = ov2312_write_reg(ov2312->client, OV2312_REG_EXPOSURE,
				       OV2312_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = ov2312_write_reg(ov2312->client, OV2312_REG_GAIN_H,
				       OV2312_REG_VALUE_08BIT,
				       (ctrl->val >> OV2312_GAIN_H_SHIFT) &
					OV2312_GAIN_H_MASK);
		ret |= ov2312_write_reg(ov2312->client, OV2312_REG_GAIN_L,
					OV2312_REG_VALUE_08BIT,
					ctrl->val & OV2312_GAIN_L_MASK);
		break;
	case V4L2_CID_VBLANK:
		ret = ov2312_write_reg(ov2312->client, OV2312_REG_VTS,
				       OV2312_REG_VALUE_16BIT,
				       ctrl->val + ov2312->cur_mode->height);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = ov2312_enable_test_pattern(ov2312, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = ov2312_write_reg(ov2312->client, OV2312_REG_H_FLIP,
				       OV2312_REG_VALUE_08BIT,
				       ctrl->val ? OV2312_FLIP_BIT : 0);
		break;
	case V4L2_CID_VFLIP:
		ret = ov2312_write_reg(ov2312->client, OV2312_REG_V_FLIP,
				       OV2312_REG_VALUE_08BIT,
				       ctrl->val ? OV2312_FLIP_BIT : 0);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ov2312_ctrl_ops = {
	.s_ctrl = ov2312_set_ctrl,
};

static int ov2312_initialize_controls(struct ov2312 *ov2312)
{
	struct v4l2_fwnode_device_properties props;
	const struct ov2312_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &ov2312->ctrl_handler;
	mode = ov2312->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 11);
	if (ret)
		return ret;
	handler->lock = &ov2312->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ, 0, 0,
				      link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE, PIXEL_RATE_WITH_160M,
			  PIXEL_RATE_WITH_160M, 1, PIXEL_RATE_WITH_160M);

	h_blank = mode->hts_def - mode->width;
	ov2312->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					   h_blank, h_blank, 1, h_blank);
	if (ov2312->hblank)
		ov2312->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	ov2312->vblank =
		v4l2_ctrl_new_std(handler, &ov2312_ctrl_ops, V4L2_CID_VBLANK,
				  vblank_def, OV2312_VTS_MAX - mode->height, 1,
				  vblank_def);

	exposure_max = mode->vts_def - 4;
	ov2312->exposure =
		v4l2_ctrl_new_std(handler, &ov2312_ctrl_ops, V4L2_CID_EXPOSURE,
				  OV2312_EXPOSURE_MIN, exposure_max,
				  OV2312_EXPOSURE_STEP, mode->exp_def);

	v4l2_ctrl_new_std(handler, &ov2312_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  OV2312_GAIN_MIN, OV2312_GAIN_MAX, OV2312_GAIN_STEP,
			  OV2312_GAIN_DEFAULT);

	v4l2_ctrl_new_std_menu_items(handler, &ov2312_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ov2312_test_pattern_menu) - 1,
				     0, 0, ov2312_test_pattern_menu);

	v4l2_ctrl_new_std(handler, &ov2312_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1,
			  0);

	v4l2_ctrl_new_std(handler, &ov2312_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1,
			  0);

	ret = v4l2_fwnode_device_parse(&ov2312->client->dev, &props);
	if (ret)
		goto err_free_handler;

	ret = v4l2_ctrl_new_fwnode_properties(handler, &ov2312_ctrl_ops,
					      &props);
	if (ret)
		goto err_free_handler;

	if (handler->error) {
		ret = handler->error;
		dev_err(&ov2312->client->dev, "Failed to init controls(%d)\n",
			ret);
		goto err_free_handler;
	}

	ov2312->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int ov2312_check_sensor_id(struct ov2312 *ov2312,
				  struct i2c_client *client)
{
	struct device *dev = &ov2312->client->dev;
	u32 id = 0;
	int ret;

	ret = ov2312_read_reg(client, OV2312_REG_CHIP_ID,
			      OV2312_REG_VALUE_16BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected OV2312 sensor\n");

	return 0;
}

static int ov2312_configure_regulators(struct ov2312 *ov2312)
{
	unsigned int i;

	for (i = 0; i < OV2312_NUM_SUPPLIES; i++)
		ov2312->supplies[i].supply = ov2312_supply_names[i];

	return devm_regulator_bulk_get(&ov2312->client->dev,
				       OV2312_NUM_SUPPLIES, ov2312->supplies);
}

static int ov2312_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct ov2312 *ov2312;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	u32 i, hdr_mode = 0;

	dev_info(dev, "driver version: %02x.%02x.%02x", DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8, DRIVER_VERSION & 0x00ff);

	ov2312 = devm_kzalloc(dev, sizeof(*ov2312), GFP_KERNEL);
	if (!ov2312)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &ov2312->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &ov2312->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &ov2312->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &ov2312->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(node, OF_CAMERA_HDR_MODE, &hdr_mode);
	if (ret) {
		hdr_mode = NO_HDR;
		dev_warn(dev, " Get hdr mode failed! no hdr default\n");
	}
	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		if (hdr_mode == supported_modes[i].hdr_mode) {
			ov2312->cur_mode = &supported_modes[i];
			break;
		}
	}
	if (i == ARRAY_SIZE(supported_modes))
		ov2312->cur_mode = &supported_modes[0];

	ov2312->client = client;

	ov2312->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(ov2312->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	ov2312->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ov2312->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	ov2312->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(ov2312->pinctrl)) {
		ov2312->pins_default = pinctrl_lookup_state(
			ov2312->pinctrl, OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(ov2312->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		ov2312->pins_sleep = pinctrl_lookup_state(
			ov2312->pinctrl, OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(ov2312->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = ov2312_configure_regulators(ov2312);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&ov2312->mutex);

	sd = &ov2312->subdev;
	v4l2_i2c_subdev_init(sd, client, &ov2312_subdev_ops);
	ret = ov2312_initialize_controls(ov2312);
	if (ret)
		goto err_destroy_mutex;

	ret = __ov2312_power_on(ov2312);
	if (ret)
		goto err_free_handler;

	ret = ov2312_check_sensor_id(ov2312, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &ov2312_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	ov2312->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &ov2312->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(ov2312->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 ov2312->module_index, facing, OV2312_NAME, dev_name(sd->dev));
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
	__ov2312_power_off(ov2312);
err_free_handler:
	v4l2_ctrl_handler_free(&ov2312->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&ov2312->mutex);

	return ret;
}

static int ov2312_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov2312 *ov2312 = to_ov2312(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&ov2312->ctrl_handler);
	mutex_destroy(&ov2312->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__ov2312_power_off(ov2312);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ov2312_of_match[] = {
	{ .compatible = "ovti,ov2312" },
	{},
};
MODULE_DEVICE_TABLE(of, ov2312_of_match);
#endif

static const struct i2c_device_id ov2312_match_id[] = {
	{ "ovti,ov2312", 0 },
	{},
};

static struct i2c_driver ov2312_i2c_driver = {
	.driver = {
		.name = OV2312_NAME,
		.pm = &ov2312_pm_ops,
		.of_match_table = of_match_ptr(ov2312_of_match),
	},
	.probe		= &ov2312_probe,
	.remove		= &ov2312_remove,
	.id_table	= ov2312_match_id,
};

module_i2c_driver(ov2312_i2c_driver);

MODULE_AUTHOR("O(log n) Technology Co., Ltd.");
MODULE_DESCRIPTION("OmniVision OV2312 sensor driver");
MODULE_LICENSE("GPL v2");
