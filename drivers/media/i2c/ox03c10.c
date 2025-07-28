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

#define OX03C10_XVCLK_FREQ		27000000

#define OX03C10_LANES			2
#define OX03C10_BITS_PER_SAMPLE		12
#define OX03C10_LINK_FREQ_384M		384000000

#define PIXEL_RATE_WITH_384M_12BIT	(OX03C10_LINK_FREQ_384M * 2 * \
					OX03C10_LANES / OX03C10_BITS_PER_SAMPLE)

#define OX03C10_CHIP_ID			0x580343
#define OX03C10_REG_CHIP_ID		0x300a

#define OX03C10_VTS_MAX			0xffff

#define OX03C10_REG_EXP_LONG_H		0x3501

#define OX03C10_REG_AGAIN_LONG_H	0x3508
#define OX03C10_REG_DGAIN_LONG_H	0x350a

#define OX03C10_REG_CTRL_MODE		0x0100
#define OX03C10_MODE_SW_STANDBY		0x0
#define OX03C10_MODE_STREAMING		BIT(0)

#define OX03C10_GAIN_MIN		0x0080
#define OX03C10_GAIN_MAX		0x7820
#define OX03C10_GAIN_STEP		1
#define OX03C10_GAIN_DEFAULT	0x0080

#define OX03C10_REG_VTS			0x380e

#define OX03C10_EXPOSURE_MIN		2
#define OX03C10_EXPOSURE_STEP		1

#define OX03C10_FLIP_REG		0x3820
#define MIRROR_BIT_MASK			BIT(5)
#define FLIP_BIT_MASK			BIT(2)

#define OX03C10_REG_VALUE_08BIT		1
#define OX03C10_REG_VALUE_16BIT		2
#define OX03C10_REG_VALUE_24BIT		3

#define OX03C10_NAME			"ox03c10"

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define REG_DELAY			0xFFFE
#define REG_NULL			0xFFFF

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
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct ox03c10_mode *cur_mode;
	struct v4l2_fract	cur_fps;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
};

#define to_ox03c10(sd) container_of(sd, struct ox03c10, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval ox03c10_global_regs[] = {
	// RSVD
	{ 0x3700, 0x28 },
	{ 0x3701, 0x15 },
	{ 0x3702, 0x19 },
	{ 0x3703, 0x23 },
	{ 0x3704, 0x0a },
	{ 0x3705, 0x00 },
	{ 0x3706, 0x3e },
	{ 0x3707, 0x0d },
	{ 0x3708, 0x50 },
	{ 0x3709, 0x5a },
	{ 0x370a, 0x00 },
	{ 0x370b, 0x96 },
	{ 0x3711, 0x11 },
	{ 0x3712, 0x13 },
	{ 0x3717, 0x02 },
	{ 0x3718, 0x73 },
	{ 0x372c, 0x40 },
	{ 0x3733, 0x01 },
	{ 0x3738, 0x36 },
	{ 0x3739, 0x36 },
	{ 0x373a, 0x25 },
	{ 0x373b, 0x25 },
	{ 0x373f, 0x21 },
	{ 0x3740, 0x21 },
	{ 0x3741, 0x21 },
	{ 0x3742, 0x21 },
	{ 0x3747, 0x28 },
	{ 0x3748, 0x28 },
	{ 0x3749, 0x19 },
	{ 0x3755, 0x1a },
	{ 0x3756, 0x0a },
	{ 0x3757, 0x1c },
	{ 0x3765, 0x19 },
	{ 0x3766, 0x05 },
	{ 0x3767, 0x05 },
	{ 0x3768, 0x13 },
	{ 0x376c, 0x07 },
	{ 0x3778, 0x20 },
	{ 0x377c, 0xc8 },
	{ 0x3781, 0x02 },
	{ 0x3783, 0x02 },
	{ 0x379c, 0x58 },
	{ 0x379e, 0x00 },
	{ 0x379f, 0x00 },
	{ 0x37a0, 0x00 },
	{ 0x37bc, 0x22 },
	{ 0x37c0, 0x01 },
	{ 0x37c4, 0x3e },
	{ 0x37c5, 0x3e },
	{ 0x37c6, 0x2a },
	{ 0x37c7, 0x28 },
	{ 0x37c8, 0x02 },
	{ 0x37c9, 0x12 },
	{ 0x37cb, 0x29 },
	{ 0x37cd, 0x29 },
	{ 0x37d2, 0x00 },
	{ 0x37d3, 0x73 },
	{ 0x37d6, 0x00 },
	{ 0x37d7, 0x6b },
	{ 0x37dc, 0x00 },
	{ 0x37df, 0x54 },
	{ 0x37e2, 0x00 },
	{ 0x37e3, 0x00 },
	{ 0x37f8, 0x00 },
	{ 0x37f9, 0x01 },
	{ 0x37fa, 0x00 },
	{ 0x37fb, 0x19 },

	// Mirror
	{ 0x3820, 0x20 },

	// Format
	{ 0x4319, 0x03 }, // spd dcg
	{ 0x431f, 0x20 }, // enable PWL (pwl0_en), 12 bits

	// BLC
	{ 0x4008, 0x02 },
	{ 0x4009, 0x03 },

	// Watchdog
	{ 0x4f00, 0x00 },
	{ 0x4f01, 0x00 },
	{ 0x4f02, 0x80 },
	{ 0x4f03, 0x2c },
	{ 0x4f04, 0xf8 },

	{ REG_NULL, 0x00 },
};

static const struct regval ox03c10_1920x1280_2_lanes_regs[] = {
	{ 0x3012, 0x21 }, // SC_PHY_CTRL = 2 lane MIPI

	{ REG_NULL, 0x00 },
};

static const struct ox03c10_mode supported_modes[] = {
	{
		.width = 1920,
		.height = 1280,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 256,
		.hts_def = 2140,
		.vts_def = 1574,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR12_1X12,
		.reg_list = ox03c10_1920x1280_2_lanes_regs,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	}
};

static const s64 link_freq_menu_items[] = {
	OX03C10_LINK_FREQ_384M
};

/* Write registers up to 4 at a time */
static int ox03c10_write_reg(struct i2c_client *client, u16 reg, u32 len,
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

static int ox03c10_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		if (regs[i].addr == REG_DELAY) {
			usleep_range(regs[i].val * 1000,
				     regs[i].val * 1000 + 1000);
			continue;
		}
		ret = ox03c10_write_reg(client, regs[i].addr,
					OX03C10_REG_VALUE_08BIT, regs[i].val);
	}
	return ret;
}

/* Read registers up to 4 at a time */
static int ox03c10_read_reg(struct i2c_client *client, u16 reg,
			    unsigned int len, u32 *val)
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
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = mode->bus_fmt;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;
	fmt->format.ycbcr_enc = V4L2_YCBCR_ENC_601;
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
		__v4l2_ctrl_modify_range(ox03c10->hblank, h_blank, h_blank, 1,
					 h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(ox03c10->vblank, vblank_def,
					 OX03C10_VTS_MAX - mode->height, 1,
					 vblank_def);
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

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
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

static int ox03c10_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				 struct v4l2_mbus_config *config)
{
	struct ox03c10 *ox03c10 = to_ox03c10(sd);
	const struct ox03c10_mode *mode = ox03c10->cur_mode;
	u32 val = 1 << (OX03C10_LANES - 1) | V4L2_MBUS_CSI2_CHANNEL_0 |
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
			w = ox03c10->cur_mode->hts_def -
			    ox03c10->cur_mode->width;
			h = ox03c10->cur_mode->vts_def -
			    ox03c10->cur_mode->height;
			__v4l2_ctrl_modify_range(ox03c10->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(
				ox03c10->vblank, h,
				OX03C10_VTS_MAX - ox03c10->cur_mode->height, 1,
				h);
		}
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = ox03c10_write_reg(ox03c10->client,
						OX03C10_REG_CTRL_MODE,
						OX03C10_REG_VALUE_08BIT,
						OX03C10_MODE_STREAMING);
		else
			ret = ox03c10_write_reg(ox03c10->client,
						OX03C10_REG_CTRL_MODE,
						OX03C10_REG_VALUE_08BIT,
						OX03C10_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long ox03c10_compat_ioctl32(struct v4l2_subdev *sd, unsigned int cmd,
				   unsigned long arg)
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
				 OX03C10_REG_VALUE_08BIT,
				 OX03C10_MODE_STREAMING);
}

static int __ox03c10_stop_stream(struct ox03c10 *ox03c10)
{
	return ox03c10_write_reg(ox03c10->client, OX03C10_REG_CTRL_MODE,
				 OX03C10_REG_VALUE_08BIT,
				 OX03C10_MODE_SW_STANDBY);
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

static int
ox03c10_enum_frame_interval(struct v4l2_subdev *sd,
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
	SET_RUNTIME_PM_OPS(ox03c10_runtime_suspend, ox03c10_runtime_resume, NULL)
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
	.core = &ox03c10_core_ops,
	.video = &ox03c10_video_ops,
	.pad = &ox03c10_pad_ops,
};

static int ox03c10_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ox03c10 *ox03c10 =
		container_of(ctrl->handler, struct ox03c10, ctrl_handler);
	struct i2c_client *client = ox03c10->client;
	s64 max;
	int ret = 0;
	u32 again = 0, dgain = 0, flip = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = ox03c10->cur_mode->height + ctrl->val - 8;
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
		ret = ox03c10_write_reg(ox03c10->client, OX03C10_REG_EXP_LONG_H,
					OX03C10_REG_VALUE_16BIT, ctrl->val);
		dev_dbg(&client->dev, "set exposure 0x%x\n", ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		if (ctrl->val > 1984) {
			dgain = ctrl->val * 1024 / 1984;
			again = 1984;
		} else {
			dgain = 1024;
			again = ctrl->val;
		}
		ret = ox03c10_write_reg(ox03c10->client,
					OX03C10_REG_AGAIN_LONG_H,
					OX03C10_REG_VALUE_16BIT,
					again & 0x1fff);
		ret |= ox03c10_write_reg(ox03c10->client,
					 OX03C10_REG_DGAIN_LONG_H,
					 OX03C10_REG_VALUE_16BIT,
					 dgain & 0x3fff);
		dev_dbg(&client->dev, "set analog gain 0x%x\n", ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = ox03c10_write_reg(ox03c10->client, OX03C10_REG_VTS,
					OX03C10_REG_VALUE_16BIT,
					ctrl->val + ox03c10->cur_mode->height);
		dev_dbg(&client->dev, "set vblank 0x%x\n", ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = ox03c10_read_reg(ox03c10->client, OX03C10_FLIP_REG,
				       OX03C10_REG_VALUE_08BIT, &flip);
		if (ctrl->val)
			flip &= ~MIRROR_BIT_MASK;
		else
			flip |= MIRROR_BIT_MASK;
		ret |= ox03c10_write_reg(ox03c10->client, OX03C10_FLIP_REG,
					 OX03C10_REG_VALUE_08BIT, flip);
		break;
	case V4L2_CID_VFLIP:
		ret = ox03c10_read_reg(ox03c10->client, OX03C10_FLIP_REG,
				       OX03C10_REG_VALUE_08BIT, &flip);
		if (ctrl->val) 
			flip |= FLIP_BIT_MASK;
		else
			flip &= ~FLIP_BIT_MASK;

		ret |= ox03c10_write_reg(ox03c10->client, OX03C10_FLIP_REG,
					 OX03C10_REG_VALUE_08BIT, flip);
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

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ, 0, 0,
				      link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE, 0,
			  PIXEL_RATE_WITH_384M_12BIT, 1,
			  PIXEL_RATE_WITH_384M_12BIT);

	h_blank = mode->hts_def - mode->width;
	ox03c10->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					    h_blank, h_blank, 1, h_blank);
	if (ox03c10->hblank)
		ox03c10->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	vblank_def = mode->vts_def - mode->height;
	ox03c10->vblank =
		v4l2_ctrl_new_std(handler, &ox03c10_ctrl_ops, V4L2_CID_VBLANK,
				  vblank_def, OX03C10_VTS_MAX - mode->height, 1,
				  vblank_def);
	ox03c10->cur_fps = mode->max_fps;

	exposure_max = mode->vts_def - 8;
	ox03c10->exposure =
		v4l2_ctrl_new_std(handler, &ox03c10_ctrl_ops, V4L2_CID_EXPOSURE,
				  OX03C10_EXPOSURE_MIN, exposure_max,
				  OX03C10_EXPOSURE_STEP, mode->exp_def);

	v4l2_ctrl_new_std(handler, &ox03c10_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  OX03C10_GAIN_MIN, OX03C10_GAIN_MAX, OX03C10_GAIN_STEP,
			  OX03C10_GAIN_DEFAULT);

	v4l2_ctrl_new_std(handler, &ox03c10_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1,
			  0);

	v4l2_ctrl_new_std(handler, &ox03c10_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1,
			  0);

	if (handler->error) {
		ret = handler->error;
		dev_err(&ox03c10->client->dev, "Failed to init controls(%d)\n",
			ret);
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
	if (id != OX03C10_CHIP_ID) {
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
				       OX03C10_NUM_SUPPLIES, ox03c10->supplies);
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

	dev_info(dev, "driver version: %02x.%02x.%02x", DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8, DRIVER_VERSION & 0x00ff);

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
		ox03c10->pins_default = pinctrl_lookup_state(
			ox03c10->pinctrl, OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(ox03c10->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		ox03c10->pins_sleep = pinctrl_lookup_state(
			ox03c10->pinctrl, OF_CAMERA_PINCTRL_STATE_SLEEP);
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
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
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
		 ox03c10->module_index, facing, OX03C10_NAME,
		 dev_name(sd->dev));
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
	{},
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
