// SPDX-License-Identifier: GPL-2.0
/*
 * ov6211 driver
 *
 * Copyright (C) 2024 Huy Duong <huy.duong@ologn.tech>
 *
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <linux/rk-preisp.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x01)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define OV6211_LANES			1
#define OV6211_BITS_PER_SAMPLE		10
#define OV6211_LINK_FREQ_240		240000000

#define PIXEL_RATE_WITH_240M_10BIT	(OV6211_LINK_FREQ_240 * 2 * \
					OV6211_LANES / OV6211_BITS_PER_SAMPLE)
#define OV6211_XVCLK_FREQ		24000000

#define CHIP_ID				0x67
#define OV6211_REG_CHIP_ID		0x300a
#define OV6211_REG_OPT_LOAD_CTRL	0x3d81

#define OV6211_REG_CTRL_MODE		0x0100
#define OV6211_MODE_SW_STANDBY		0x0
#define OV6211_MODE_STREAMING		BIT(0)

#define OV6211_REG_EXPOSURE		0x3500
#define OV6211_EXPOSURE_MIN		4
#define OV6211_EXPOSURE_STEP		0xf
#define OV6211_VTS_MAX			0xffff

#define OV6211_REG_ANALOG_GAIN		0x350a
#define ANALOG_GAIN_MASK		0x3ff
#define ANALOG_GAIN_MIN			0x10
#define ANALOG_GAIN_MAX			0x3e0
#define ANALOG_GAIN_STEP		1
#define ANALOG_GAIN_DEFAULT		0x20

#define OV6211_REG_TEST_PATTERN		0x5e00
#define	OV6211_TEST_PATTERN_ENABLE	0x80
#define	OV6211_TEST_PATTERN_DISABLE	0x0

#define OV6211_REG_VTS			0x380e

#define OV6211_MIRROR_REG		0x3821
#define OV6211_FLIP_REG		0x3820

#define OV6211_FETCH_MIRROR(VAL, ENABLE)	(ENABLE ? VAL | 0x01 : VAL & 0xf9)
#define OV6211_FETCH_FLIP(VAL, ENABLE)		(ENABLE ? VAL | 0x01 : VAL & 0x9f)

#define REG_DELAY			0xFFFE
#define REG_NULL			0xFFFF

#define OV6211_REG_VALUE_08BIT		1
#define OV6211_REG_VALUE_16BIT		2
#define OV6211_REG_VALUE_24BIT		3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
#define OV6211_NAME			"ov6211"

static const char * const ov6211_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define OV6211_NUM_SUPPLIES ARRAY_SIZE(ov6211_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct ov6211_mode {
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

struct ov6211 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct regulator_bulk_data supplies[OV6211_NUM_SUPPLIES];

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
	const struct ov6211_mode *cur_mode;
	struct v4l2_fract	cur_fps;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	u32			cur_vts;
};

#define to_ov6211(sd) container_of(sd, struct ov6211, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval ov6211_global_regs[] = {
	{REG_NULL, 0x00},
};


static const struct regval ov6211_400x400_120fps_regs[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x3005, 0x00},
	{0x3013, 0x12},
	{0x3014, 0x04},
	{0x3016, 0x10},
	{0x3017, 0x00},
	{0x3018, 0x00},
	{0x301a, 0x00},
	{0x301b, 0x00},
	{0x301c, 0x00},
	{0x3037, 0xf0},
	{0x3080, 0x01},
	{0x3081, 0x00},
	{0x3082, 0x01},
	{0x3098, 0x04},
	{0x3099, 0x28},
	{0x309a, 0x06},
	{0x309b, 0x04},
	{0x309c, 0x00},
	{0x309d, 0x00},
	{0x309e, 0x01},
	{0x309f, 0x00},
	{0x30b0, 0x0a},
	{0x30b1, 0x02},
	{0x30b2, 0x00},
	{0x30b3, 0x32},
	{0x30b4, 0x02},
	{0x30b5, 0x05},
	{0x3106, 0xd9},
	{0x3500, 0x00},
	{0x3501, 0x1b},
	{0x3502, 0x20},
	{0x3503, 0x07},
	{0x3509, 0x10},
	{0x350b, 0x10},
	{0x3620, 0xb7},
	{0x3621, 0x05},
	{0x3626, 0x31},
	{0x3627, 0x40},
	{0x3632, 0xa3},
	{0x3633, 0x34},
	{0x3634, 0x40},
	{0x3636, 0x00},
	{0x3660, 0x80},
	{0x3662, 0x01},
	{0x3664, 0xf0},
	{0x366a, 0x00},
	{0x366b, 0x50},
	{0x3680, 0xf4},
	{0x3681, 0x50},
	{0x3682, 0x00},
	{0x3708, 0x20},
	{0x3709, 0x40},
	{0x370d, 0x03},
	{0x373b, 0x02},
	{0x373c, 0x08},
	{0x3742, 0x00},
	{0x3744, 0x16},
	{0x3745, 0x08},
	{0x3781, 0xfc},
	{0x3788, 0x00},
	{0x3800, 0x00},
	{0x3801, 0x04},
	{0x3802, 0x00},
	{0x3803, 0x04},
	{0x3804, 0x01},
	{0x3805, 0x9b},
	{0x3806, 0x01},
	{0x3807, 0x9b},
	{0x3808, 0x01},
	{0x3809, 0x90},
	{0x380a, 0x01},
	{0x380b, 0x90},
	{0x380c, 0x05},
	{0x380d, 0xf2},
	{0x380e, 0x01},
	{0x380f, 0xb6},
	{0x3810, 0x00},
	{0x3811, 0x04},
	{0x3812, 0x00},
	{0x3813, 0x04},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3820, 0x00},
	{0x3821, 0x00},
	{0x382b, 0xfa},
	{0x382f, 0x04},
	{0x3832, 0x00},
	{0x3833, 0x05},
	{0x3834, 0x00},
	{0x3835, 0x05},
	{0x3882, 0x04},
	{0x3883, 0x00},
	{0x38a4, 0x10},
	{0x38a5, 0x00},
	{0x38b1, 0x03},
	{0x3b80, 0x00},
	{0x3b81, 0xa5},
	{0x3b82, 0x10},
	{0x3b83, 0x00},
	{0x3b84, 0x08},
	{0x3b85, 0x00},
	{0x3b86, 0x01},
	{0x3b87, 0x00},
	{0x3b88, 0x00},
	{0x3b89, 0x00},
	{0x3b8a, 0x00},
	{0x3b8b, 0x05},
	{0x3b8c, 0x00},
	{0x3b8d, 0x00},
	{0x3b8e, 0x00},
	{0x3b8f, 0x1a},
	{0x3b94, 0x05},
	{0x3b95, 0xf2},
	{0x3b96, 0xf0},
	{0x4004, 0x04},
	{0x404e, 0x01},
	{0x4801, 0x0f},
	{0x4806, 0x0f},
	{0x4837, 0x43},
	{0x0100, 0x01},
	{REG_NULL, 0x00},
};

static const struct ov6211_mode supported_modes[] = {
	{
		.width = 400,
		.height = 400,
		.max_fps = {
			.numerator = 10000,
			.denominator = 1200000,
		},
		.exp_def = 0x00f8,
		.hts_def = 0x03a1,
		.vts_def = 0x021a,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.reg_list = ov6211_400x400_120fps_regs,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	}
};

static const s64 link_freq_menu_items[] = {
	OV6211_LINK_FREQ_240
};

static const char * const ov6211_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int ov6211_write_reg(struct i2c_client *client, u16 reg,
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

static int ov6211_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		ret = ov6211_write_reg(client, regs[i].addr,
					OV6211_REG_VALUE_08BIT, regs[i].val);
	}
	return ret;
}

/* Read registers up to 4 at a time */
static int ov6211_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
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



static int ov6211_get_reso_dist(const struct ov6211_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct ov6211_mode *
ov6211_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = ov6211_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int ov6211_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct ov6211 *ov6211 = to_ov6211(sd);
	const struct ov6211_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&ov6211->mutex);

	mode = ov6211_find_best_fit(fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&ov6211->mutex);
		return -ENOTTY;
#endif
	} else {
		ov6211->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(ov6211->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(ov6211->vblank, vblank_def,
					 OV6211_VTS_MAX - mode->height,
					 1, vblank_def);
		ov6211->cur_fps = mode->max_fps;
	}

	mutex_unlock(&ov6211->mutex);

	return 0;
}

static int ov6211_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct ov6211 *ov6211 = to_ov6211(sd);
	const struct ov6211_mode *mode = ov6211->cur_mode;

	mutex_lock(&ov6211->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&ov6211->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = mode->bus_fmt;
		fmt->format.field = V4L2_FIELD_NONE;
		/* format info: width/height/data type/virctual channel */
		if (fmt->pad < PAD_MAX && mode->hdr_mode != NO_HDR)
			fmt->reserved[0] = mode->vc[fmt->pad];
		else
			fmt->reserved[0] = mode->vc[PAD0];
	}
	mutex_unlock(&ov6211->mutex);

	return 0;
}

static int ov6211_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct ov6211 *ov6211 = to_ov6211(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = ov6211->cur_mode->bus_fmt;

	return 0;
}

static int ov6211_enum_frame_sizes(struct v4l2_subdev *sd,
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

static int ov6211_enable_test_pattern(struct ov6211 *ov6211, u32 pattern)
{
	u32 val;

	if (pattern)
		val = (pattern - 1) | OV6211_TEST_PATTERN_ENABLE;
	else
		val = OV6211_TEST_PATTERN_DISABLE;

	return ov6211_write_reg(ov6211->client, OV6211_REG_TEST_PATTERN,
				OV6211_REG_VALUE_08BIT, val);
}

static int ov6211_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct ov6211 *ov6211 = to_ov6211(sd);
	const struct ov6211_mode *mode = ov6211->cur_mode;

	if (ov6211->streaming)
		fi->interval = ov6211->cur_fps;
	else
		fi->interval = mode->max_fps;

	return 0;
}

static int ov6211_g_mbus_config(struct v4l2_subdev *sd,
				unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	struct ov6211 *ov6211 = to_ov6211(sd);
	const struct ov6211_mode *mode = ov6211->cur_mode;
	u32 val = 1 << (OV6211_LANES - 1) |
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

static void ov6211_get_module_inf(struct ov6211 *ov6211,
				   struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, OV6211_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, ov6211->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, ov6211->len_name, sizeof(inf->base.lens));
}

static long ov6211_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct ov6211 *ov6211 = to_ov6211(sd);
	struct rkmodule_hdr_cfg *hdr;
	u32 i, h, w;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		ov6211_get_module_inf(ov6211, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = ov6211->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = ov6211->cur_mode->width;
		h = ov6211->cur_mode->height;
		for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr->hdr_mode) {
				ov6211->cur_mode = &supported_modes[i];
				break;
			}
		}
		if (i == ARRAY_SIZE(supported_modes)) {
			dev_err(&ov6211->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			w = ov6211->cur_mode->hts_def - ov6211->cur_mode->width;
			h = ov6211->cur_mode->vts_def - ov6211->cur_mode->height;
			__v4l2_ctrl_modify_range(ov6211->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(ov6211->vblank, h,
						 OV6211_VTS_MAX - ov6211->cur_mode->height, 1, h);
		}
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = ov6211_write_reg(ov6211->client, OV6211_REG_CTRL_MODE,
				 OV6211_REG_VALUE_08BIT, OV6211_MODE_STREAMING);
		else
			ret = ov6211_write_reg(ov6211->client, OV6211_REG_CTRL_MODE,
				 OV6211_REG_VALUE_08BIT, OV6211_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long ov6211_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = ov6211_ioctl(sd, cmd, inf);
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

		ret = ov6211_ioctl(sd, cmd, hdr);
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
			ret = ov6211_ioctl(sd, cmd, hdr);
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
			ret = ov6211_ioctl(sd, cmd, hdrae);
		else
			ret = -EFAULT;
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = ov6211_ioctl(sd, cmd, &stream);
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

static int __ov6211_start_stream(struct ov6211 *ov6211)
{
	int ret;

	ret = ov6211_write_array(ov6211->client, ov6211->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	ret = __v4l2_ctrl_handler_setup(&ov6211->ctrl_handler);
	if (ret)
		return ret;

	return ov6211_write_reg(ov6211->client, OV6211_REG_CTRL_MODE,
				 OV6211_REG_VALUE_08BIT, OV6211_MODE_STREAMING);
}

static int __ov6211_stop_stream(struct ov6211 *ov6211)
{
	return ov6211_write_reg(ov6211->client, OV6211_REG_CTRL_MODE,
				 OV6211_REG_VALUE_08BIT, OV6211_MODE_SW_STANDBY);
}

static int ov6211_s_stream(struct v4l2_subdev *sd, int on)
{
	struct ov6211 *ov6211 = to_ov6211(sd);
	struct i2c_client *client = ov6211->client;
	int ret = 0;

	mutex_lock(&ov6211->mutex);
	on = !!on;
	if (on == ov6211->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __ov6211_start_stream(ov6211);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
		usleep_range(10 * 1000, 12 * 1000);
	} else {
		__ov6211_stop_stream(ov6211);
		pm_runtime_put(&client->dev);
	}

	ov6211->streaming = on;

unlock_and_return:
	mutex_unlock(&ov6211->mutex);

	return ret;
}

static int ov6211_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov6211 *ov6211 = to_ov6211(sd);
	struct i2c_client *client = ov6211->client;
	int ret = 0;

	mutex_lock(&ov6211->mutex);

	/* If the power state is not modified - no work to do. */
	if (ov6211->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = ov6211_write_array(ov6211->client, ov6211_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ov6211->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		ov6211->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&ov6211->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 ov6211_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, OV6211_XVCLK_FREQ / 1000 / 1000);
}

static int __ov6211_power_on(struct ov6211 *ov6211)
{
	int ret;
	u32 delay_us;
	struct device *dev = &ov6211->client->dev;

	if (!IS_ERR_OR_NULL(ov6211->pins_default)) {
		ret = pinctrl_select_state(ov6211->pinctrl,
					   ov6211->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(ov6211->xvclk, OV6211_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(ov6211->xvclk) != OV6211_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(ov6211->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (!IS_ERR(ov6211->reset_gpio))
		gpiod_set_value_cansleep(ov6211->reset_gpio, 0);

	ret = regulator_bulk_enable(OV6211_NUM_SUPPLIES, ov6211->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	usleep_range(5 * 1000, 10 * 1000);
	if (!IS_ERR(ov6211->reset_gpio))
		gpiod_set_value_cansleep(ov6211->reset_gpio, 1);

	if (!IS_ERR(ov6211->reset_gpio))
		usleep_range(6000, 8000);
	else
		usleep_range(12000, 16000);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = ov6211_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(ov6211->xvclk);

	return ret;
}

static void __ov6211_power_off(struct ov6211 *ov6211)
{
	int ret;
	struct device *dev = &ov6211->client->dev;

	clk_disable_unprepare(ov6211->xvclk);
	if (!IS_ERR(ov6211->reset_gpio))
		gpiod_set_value_cansleep(ov6211->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(ov6211->pins_sleep)) {
		ret = pinctrl_select_state(ov6211->pinctrl,
					   ov6211->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(OV6211_NUM_SUPPLIES, ov6211->supplies);
}

static int ov6211_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov6211 *ov6211 = to_ov6211(sd);

	return __ov6211_power_on(ov6211);
}

static int ov6211_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov6211 *ov6211 = to_ov6211(sd);

	__ov6211_power_off(ov6211);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int ov6211_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ov6211 *ov6211 = to_ov6211(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct ov6211_mode *def_mode = &supported_modes[0];

	mutex_lock(&ov6211->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&ov6211->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int ov6211_enum_frame_interval(struct v4l2_subdev *sd,
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

static const struct dev_pm_ops ov6211_pm_ops = {
	SET_RUNTIME_PM_OPS(ov6211_runtime_suspend,
			   ov6211_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops ov6211_internal_ops = {
	.open = ov6211_open,
};
#endif

static const struct v4l2_subdev_core_ops ov6211_core_ops = {
	.s_power = ov6211_s_power,
	.ioctl = ov6211_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = ov6211_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops ov6211_video_ops = {
	.s_stream = ov6211_s_stream,
	.g_frame_interval = ov6211_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops ov6211_pad_ops = {
	.enum_mbus_code = ov6211_enum_mbus_code,
	.enum_frame_size = ov6211_enum_frame_sizes,
	.enum_frame_interval = ov6211_enum_frame_interval,
	.get_fmt = ov6211_get_fmt,
	.set_fmt = ov6211_set_fmt,
	.get_mbus_config = ov6211_g_mbus_config,
};

static const struct v4l2_subdev_ops ov6211_subdev_ops = {
	.core	= &ov6211_core_ops,
	.video	= &ov6211_video_ops,
	.pad	= &ov6211_pad_ops,
};

static int ov6211_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov6211 *ov6211 = container_of(ctrl->handler,
					       struct ov6211, ctrl_handler);
	struct i2c_client *client = ov6211->client;
	s64 max;
	int ret = 0;
	u32 val = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = ov6211->cur_mode->height + ctrl->val - 20;
		__v4l2_ctrl_modify_range(ov6211->exposure,
					 ov6211->exposure->minimum, max,
					 ov6211->exposure->step,
					 ov6211->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		ret = ov6211_write_reg(ov6211->client, OV6211_REG_EXPOSURE,
				       OV6211_REG_VALUE_24BIT, ctrl->val << 4);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = ov6211_write_reg(ov6211->client, OV6211_REG_ANALOG_GAIN,
				       OV6211_REG_VALUE_16BIT,
				       ctrl->val & ANALOG_GAIN_MASK);
		break;
	case V4L2_CID_VBLANK:
		ret = ov6211_write_reg(ov6211->client, OV6211_REG_VTS,
				       OV6211_REG_VALUE_16BIT,
				       ctrl->val + ov6211->cur_mode->height);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = ov6211_enable_test_pattern(ov6211, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = ov6211_read_reg(ov6211->client, OV6211_MIRROR_REG,
				       OV6211_REG_VALUE_08BIT, &val);
		ret |= ov6211_write_reg(ov6211->client, OV6211_MIRROR_REG,
					 OV6211_REG_VALUE_08BIT,
					 OV6211_FETCH_MIRROR(val, ctrl->val));
		break;
	case V4L2_CID_VFLIP:
		ret = ov6211_read_reg(ov6211->client, OV6211_FLIP_REG,
				       OV6211_REG_VALUE_08BIT, &val);
		ret |= ov6211_write_reg(ov6211->client, OV6211_FLIP_REG,
					 OV6211_REG_VALUE_08BIT,
					 OV6211_FETCH_FLIP(val, ctrl->val));
		break;

	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ov6211_ctrl_ops = {
	.s_ctrl = ov6211_set_ctrl,
};

static int ov6211_initialize_controls(struct ov6211 *ov6211)
{
	const struct ov6211_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &ov6211->ctrl_handler;
	mode = ov6211->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &ov6211->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, PIXEL_RATE_WITH_240M_10BIT, 1, PIXEL_RATE_WITH_240M_10BIT);

	h_blank = mode->hts_def - mode->width;
	ov6211->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					    h_blank, h_blank, 1, h_blank);
	if (ov6211->hblank)
		ov6211->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	vblank_def = mode->vts_def - mode->height;
	ov6211->vblank = v4l2_ctrl_new_std(handler, &ov6211_ctrl_ops,
					    V4L2_CID_VBLANK, vblank_def,
					    OV6211_VTS_MAX - mode->height,
					    1, vblank_def);
	ov6211->cur_fps = mode->max_fps;
	exposure_max = mode->vts_def - 20;
	ov6211->exposure = v4l2_ctrl_new_std(handler, &ov6211_ctrl_ops,
					      V4L2_CID_EXPOSURE, OV6211_EXPOSURE_MIN,
					      exposure_max, OV6211_EXPOSURE_STEP,
					      mode->exp_def);
	ov6211->anal_gain = v4l2_ctrl_new_std(handler, &ov6211_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, ANALOG_GAIN_MIN,
				ANALOG_GAIN_MAX, ANALOG_GAIN_STEP,
				ANALOG_GAIN_DEFAULT);

	ov6211->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
							    &ov6211_ctrl_ops,
					V4L2_CID_TEST_PATTERN,
					ARRAY_SIZE(ov6211_test_pattern_menu) - 1,
					0, 0, ov6211_test_pattern_menu);
	v4l2_ctrl_new_std(handler, &ov6211_ctrl_ops,
				V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(handler, &ov6211_ctrl_ops,
				V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (handler->error) {
		ret = handler->error;
		dev_err(&ov6211->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	ov6211->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int ov6211_check_sensor_id(struct ov6211 *ov6211,
				   struct i2c_client *client)
{
	struct device *dev = &ov6211->client->dev;
	u32 id = 0;
	int ret;

	ret = ov6211_read_reg(client, OV6211_REG_CHIP_ID,
			       OV6211_REG_VALUE_08BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected OV%06x sensor\n", CHIP_ID);

	return 0;
}

static int ov6211_configure_regulators(struct ov6211 *ov6211)
{
	unsigned int i;

	for (i = 0; i < OV6211_NUM_SUPPLIES; i++)
		ov6211->supplies[i].supply = ov6211_supply_names[i];

	return devm_regulator_bulk_get(&ov6211->client->dev,
				       OV6211_NUM_SUPPLIES,
				       ov6211->supplies);
}

static int ov6211_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct ov6211 *ov6211;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	ov6211 = devm_kzalloc(dev, sizeof(*ov6211), GFP_KERNEL);
	if (!ov6211)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &ov6211->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &ov6211->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &ov6211->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &ov6211->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	ov6211->client = client;
	ov6211->cur_mode = &supported_modes[0];

	ov6211->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(ov6211->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	ov6211->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ov6211->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	ov6211->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(ov6211->pinctrl)) {
		ov6211->pins_default =
			pinctrl_lookup_state(ov6211->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(ov6211->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		ov6211->pins_sleep =
			pinctrl_lookup_state(ov6211->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(ov6211->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = ov6211_configure_regulators(ov6211);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&ov6211->mutex);

	sd = &ov6211->subdev;
	v4l2_i2c_subdev_init(sd, client, &ov6211_subdev_ops);
	ret = ov6211_initialize_controls(ov6211);
	if (ret)
		goto err_destroy_mutex;

	ret = __ov6211_power_on(ov6211);
	if (ret)
		goto err_free_handler;

	ret = ov6211_check_sensor_id(ov6211, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &ov6211_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	ov6211->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &ov6211->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(ov6211->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 ov6211->module_index, facing,
		 OV6211_NAME, dev_name(sd->dev));
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
	__ov6211_power_off(ov6211);
err_free_handler:
	v4l2_ctrl_handler_free(&ov6211->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&ov6211->mutex);

	return ret;
}

static int ov6211_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov6211 *ov6211 = to_ov6211(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&ov6211->ctrl_handler);
	mutex_destroy(&ov6211->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__ov6211_power_off(ov6211);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ov6211_of_match[] = {
	{ .compatible = "ovti,ov6211" },
	{},
};
MODULE_DEVICE_TABLE(of, ov6211_of_match);
#endif

static const struct i2c_device_id ov6211_match_id[] = {
	{ "ovti,ov6211", 0 },
	{ },
};

static struct i2c_driver ov6211_i2c_driver = {
	.driver = {
		.name = OV6211_NAME,
		.pm = &ov6211_pm_ops,
		.of_match_table = of_match_ptr(ov6211_of_match),
	},
	.probe		= &ov6211_probe,
	.remove		= &ov6211_remove,
	.id_table	= ov6211_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&ov6211_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&ov6211_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("OmniVision ov6211 sensor driver");
MODULE_LICENSE("GPL v2");
