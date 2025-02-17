/*
 *
 * Copyright (C) 2012-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (c) 2020 VeriSilicon Holdings Co., Ltd.
 * Copyright 2018,2023-2024 NXP
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include "vvsensor.h"

#define DEFAULT_WIDTH                   1280
#define DEFAULT_HEIGHT                  800
#define DEFAULT_FPS                     60

#define AR0144_NUM_CONSUMERS	3
#define AR0144_SENS_PADS_NUM	1

#define AR0144_CHIP_ID                  0x356
#define AR0144_CHIP_VERSION_REG 		0x3000
#define AR0144_Y_ADDR_START     		0x3002
#define AR0144_X_ADDR_START     		0x3004
#define AR0144_Y_ADDR_END       		0x3006
#define AR0144_X_ADDR_END       		0x3008
#define AR0144_SERIAL_FORMAT            0x31AE
#define AR0144_DATA_FORMAT_BITS         0x31AC

//#define TEST_EBD

static struct vvcam_sccb_data_s ar0144_1280x800_60fps[] = {
	//{0x301A, 0x00D9, 0}, // RESET_REGISTER
	{0x301A, 0x3058}, // RESET_REGISTER
	{0x3F4C, 0x003F}, // PIX_DEF_1D_DDC_LO_DEF
	{0x3F4E, 0x0018}, // PIX_DEF_1D_DDC_HI_DEF
	{0x3F50, 0x17DF}, // PIX_DEF_1D_DDC_EDGE
	{0x30B0, 0x0028}, // DIGITAL_TEST
	{0x3060, 0x000D}, // ANALOG_GAIN
	{0x30FE, 0x00A8}, // NOISE_PEDESTAL
	{0x306E, 0x4810}, // DATAPATH_SELECT
	{0x3064, 0x1802}, // SMIA_TEST
	{0x302A, 0x0006}, // VT_PIX_CLK_DIV
	{0x302C, 0x0001}, // VT_SYS_CLK_DIV
	{0x302E, 0x0004}, // PRE_PLL_CLK_DIV
	{0x3030, 0x0042}, // PLL_MULTIPLIER
	{0x3036, 0x000C}, // OP_PIX_CLK_DIV
	{0x3038, 0x0001}, // OP_SYS_CLK_DIV
	{0x30B0, 0x0028}, // DIGITAL_TEST
	{0x31B0, 0x005A}, // FRAME_PREAMBLE
	{0x31B2, 0x002E}, // LINE_PREAMBLE
	{0x31B4, 0x2633}, // MIPI_TIMING_0
	{0x31B6, 0x210E}, // MIPI_TIMING_1
	{0x31B8, 0x20C7}, // MIPI_TIMING_2
	{0x31BA, 0x0105}, // MIPI_TIMING_3
	{0x31BC, 0x0004}, // MIPI_TIMING_4
	{0x3354, 0x002C}, // MIPI_CNTRL
	{0x31AE, 0x0202}, // SERIAL_FORMAT
	{0x3002, 0x0000}, // Y_ADDR_START
	{0x3004, 0x0004}, // X_ADDR_START
	{0x3006, 0x031F}, // Y_ADDR_END
	{0x3008, 0x0503}, // X_ADDR_END
	{0x300A, 0x033B}, // FRAME_LENGTH_LINES
	{0x300C, 0x05D0}, // LINE_LENGTH_PCK
	{0x3012, 0x033A}, // COARSE_INTEGRATION_TIME
	{0x31AC, 0x0C0C}, // DATA_FORMAT_BITS
	{0x306E, 0x9010}, // DATAPATH_SELECT
	{0x30A2, 0x0001}, // X_ODD_INC
	{0x30A6, 0x0001}, // Y_ODD_INC
	{0x3082, 0x0003}, // OPERATION_MODE_CTRL
	{0x3040, 0x0000}, // READ_MODE
	{0x31D0, 0x0000}, // COMPANDING
	{0x301A, 0x005C}, // RESET_REGISTER
	{0x311C, 0x033B}, // AE_MAX_EXPOSURE_REG
	{0x3060, 0x0030}, //gain
#if 1
	{0x3070, 0x1}, //test_pattern_mode, solid color
	{0x3072, 0x111}, //test_data_red GRBG
	{0x3074, 0x222}, //test_data_greenr
	{0x3076, 0x333}, //test_data_blue
	{0x3078, 0x444}, //test_data_greenb
#endif
};

struct ar0144_datafmt {
	u32						code;
	enum v4l2_colorspace	colorspace;
};

static struct vvcam_mode_info_s par0144_mode_info[] = {
	{
		.index          = 0,
		.size           = {
			.bounds_width  = 1280,
			.bounds_height = 800,
			.top           = 0,
			.left          = 0,
			.width         = 1280,
			.height        = 800,
		},
		.hdr_mode       = SENSOR_MODE_LINEAR,
		.bit_width      = 12,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_GRBG,
		.ae_info = {
			.def_frm_len_lines     = 0x33B,
			.curr_frm_len_lines    = 0x33B - 1,
			.one_line_exp_time_ns  = 20190,

			.max_integration_line  = 0x33B - 1,
			.min_integration_line  = 8,

			.max_again             = 8 * (1 << SENSOR_FIX_FRACBITS),
			.min_again             = 2 * (1 << SENSOR_FIX_FRACBITS),
			.max_dgain             = 2 * (1 << SENSOR_FIX_FRACBITS),
			.min_dgain             = 1 * (1 << SENSOR_FIX_FRACBITS),
			.gain_step             = 1,

			.start_exposure        = 3 * 100 * (1 << SENSOR_FIX_FRACBITS),
			.cur_fps               = 60 * (1 << SENSOR_FIX_FRACBITS),
			.max_fps               = 60 * (1 << SENSOR_FIX_FRACBITS),
			.min_fps               = 5 * (1 << SENSOR_FIX_FRACBITS),
			.min_afps              = 5 * (1 << SENSOR_FIX_FRACBITS),
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 2,
		},
		.preg_data      = ar0144_1280x800_60fps,
		.reg_data_count = ARRAY_SIZE(ar0144_1280x800_60fps),
	},
};

struct ar0144 {
	struct i2c_client *i2c_client;
	struct media_pad pad;
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_captureparm streamcap;
	struct regulator_bulk_data supplies[AR0144_NUM_CONSUMERS];
	struct gpio_desc *reset;
	struct gpio_desc *isp_en;
	vvcam_mode_info_t cur_mode;
	vvcam_lens_t focus_lens;

	struct v4l2_subdev subdev;
	struct media_pad pads[AR0144_SENS_PADS_NUM];

	struct mutex lock;
	bool mode_change;
	u32 resume_status;
	u32 stream_status;
};

/* regulator supplies */
static const char * const ar0144_supply_name[] = {
	"AVDD",
	"DVDD",
	"VDDIO",
};

static const struct ar0144_datafmt ar0144_colour_fmts[] = {
	{MEDIA_BUS_FMT_SGRBG12_1X12, V4L2_COLORSPACE_RAW},
};

static inline struct ar0144 *to_ar0144_device(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ar0144, subdev);
}

static int ar0144_write_reg(struct ar0144 *sensor, u16 reg, u16 val)
{
	struct device *dev = &sensor->i2c_client->dev;
	u8 data[4] = { reg >> 8, reg & 0xff, val >> 8, val & 0xff };
	int ret;

	ret = i2c_master_send(sensor->i2c_client, data, 4);
	if (ret < 0){
		dev_err(dev, "%s: i2c write error, reg: %x\n", __func__, reg);
		return ret;
	}

	return 0;
}

static int ar0144_read_reg(struct ar0144 *sensor, u16 reg, u16 *val)
{
	struct device *dev = &sensor->i2c_client->dev;
	u8 RegBuf[2] = { reg >> 8, reg & 0xff };
	u8 ValBuf[2] = {0};
	int ret;

	ret = i2c_master_send(sensor->i2c_client, RegBuf, 2);
	if (ret < 0) {
		dev_err(dev, "%s: i2c write error, reg: %x\n", __func__, reg);
		return ret;
	}

	ret = i2c_master_recv(sensor->i2c_client, ValBuf, 2);
	if (ret < 0){
		dev_err(dev, "%s: i2c read error, reg: %x\n", __func__, reg);
		return ret;
	}

	*val = ((u16)ValBuf[0] << 8) | (u16)ValBuf[1];

	return 0;
}

static int ar0144_write_array(struct ar0144 *sensor,
				struct vvcam_sccb_data_s *mode_setting, int array_size)
{
	register u16 reg_addr = 0;
	register u16 data = 0;
	int i, retval = 0;

	for (i = 0; i < array_size; ++i, ++mode_setting) {

		reg_addr = mode_setting->addr;
		data = mode_setting->data;

		retval = ar0144_write_reg(sensor, reg_addr, data);
		if (retval < 0)
			break;
		if (reg_addr == 0x301A  && i == 0)
			msleep(100);
	}

	return 0;
}

static int ar0144_stream_on(struct ar0144 *sensor)
{
	int ret;
	u16 val = 0;
	ret = ar0144_read_reg(sensor, 0x301A, &val);
	if (ret < 0)
		return ret;
	val |= 0x0004;
	ret = ar0144_write_reg(sensor, 0x301A, val);
	return ret;
}

static int ar0144_stream_off(struct ar0144 *sensor)
{
	int ret;
	u16 val = 0;
	ret = ar0144_read_reg(sensor, 0x301A, &val);
	if (ret < 0)
		return ret;
	val &= (~0x0004);
	ret = ar0144_write_reg(sensor, 0x301A, val);
	return ret;
}

static int ar0144_s_power(struct v4l2_subdev *sd, int on)
{

	return 0;
}

static int ar0144_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0144 *sensor = to_ar0144_device(client);

	if (enable)
		return ar0144_stream_on(sensor);
	else
		return ar0144_stream_off(sensor);
}

static int ar0144_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_mbus_code_enum *code)
{


	if (code->pad || code->index >= ARRAY_SIZE(ar0144_colour_fmts))
		return -EINVAL;

	code->code = ar0144_colour_fmts[code->index].code;

	return 0;
}

static int ar0144_set_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_state *sd_state,
			struct v4l2_subdev_format *fmt)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0144 *sensor = to_ar0144_device(client);


	mutex_lock(&sensor->lock);
	pr_info("enter %s\n", __func__);
	if ((fmt->format.width != sensor->cur_mode.size.bounds_width) ||
	    (fmt->format.height != sensor->cur_mode.size.bounds_height)) {
		pr_err("%s:set sensor format %dx%d error\n",
			__func__,fmt->format.width,fmt->format.height);
		mutex_unlock(&sensor->lock);
		return -EINVAL;
	}

	if (sensor->mode_change) {
		//ar0144_init(sensor);
		ret |= ar0144_write_array(sensor,
			sensor->cur_mode.preg_data,
			sensor->cur_mode.reg_data_count);
	
		if (ret < 0) {
			pr_err("%s:ar0144_write_reg_arry error\n",__func__);
			mutex_unlock(&sensor->lock);
			return -EINVAL;
		}
		sensor->mode_change = 0;
	}
	fmt->format.code = MEDIA_BUS_FMT_SGBRG12_1X12;
	fmt->format.field = V4L2_FIELD_NONE;
	sensor->fmt = fmt->format;
	mutex_unlock(&sensor->lock);
	return 0;
}

static int ar0144_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0144 *sensor = to_ar0144_device(client);
	struct v4l2_mbus_framefmt *fmt = &sensor->fmt;



	if (format->pad)
		return -EINVAL;

	format->format = *fmt;

	dev_info(&client->dev, "get_fmt: code=0x%x, wxh=%dx%d\n",
					format->format.code, format->format.width, format->format.height);

	return 0;
}

static void ar0144_reset(struct ar0144 *sensor)
{
	gpiod_set_value_cansleep(sensor->reset, 1);
	udelay(5000);

	gpiod_set_value_cansleep(sensor->reset, 0);
	msleep(20);
}

static int ar0144_get_regulators(struct ar0144 *sensor)
{
	int i;

	for (i = 0; i < AR0144_NUM_CONSUMERS; i++)
		sensor->supplies[i].supply = ar0144_supply_name[i];

	return devm_regulator_bulk_get(&sensor->i2c_client->dev,
				       AR0144_NUM_CONSUMERS, sensor->supplies);
}
int ar0144_get_clk(struct ar0144 *sensor, void *clk)
{
	struct vvcam_clk_s vvcam_clk;
	int ret = 0;
	vvcam_clk.sensor_mclk = 24000000;
	vvcam_clk.csi_max_pixel_clk = 266000000;
	ret = copy_to_user(clk, &vvcam_clk, sizeof(struct vvcam_clk_s));
	if (ret != 0)
		ret = -EINVAL;
	return ret;
}
static int ar0144_query_capability(struct ar0144 *sensor, void *arg)
{
	struct v4l2_capability *pcap = (struct v4l2_capability *)arg;

	strcpy((char *)pcap->driver, "ar0144");
	sprintf((char *)pcap->bus_info, "csi%d",0);
	if(sensor->i2c_client->adapter) {
		pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] =
			(__u8)sensor->i2c_client->adapter->nr;
	} else {
		pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] = 0xFF;
	}
	return 0;
}

static int ar0144_query_supports(struct ar0144 *sensor, void* parry)
{
	int ret = 0;
	struct vvcam_mode_info_array_s *psensor_mode_arry = parry;
	uint32_t support_counts = ARRAY_SIZE(par0144_mode_info);

	ret = copy_to_user(&psensor_mode_arry->count, &support_counts, sizeof(support_counts));
	ret |= copy_to_user(&psensor_mode_arry->modes, par0144_mode_info,
			   sizeof(par0144_mode_info));
	if (ret != 0)
		ret = -ENOMEM;
	return ret;
}
static int ar0144_get_sensor_id(struct ar0144 *sensor, void* pchip_id)
{
	int ret = 0;
	u16 chip_id;

	ret = ar0144_read_reg(sensor, 0x3000, &chip_id);
	ret = copy_to_user(pchip_id, &chip_id, sizeof(u16));
	if (ret != 0)
		ret = -ENOMEM;
	return ret;
}
static int ar0144_get_reserve_id(struct ar0144 *sensor, void* preserve_id)
{
	int ret = 0;
	u16 reserve_id = 0x2770;
	ret = copy_to_user(preserve_id, &reserve_id, sizeof(u16));
	if (ret != 0)
		ret = -ENOMEM;
	return ret;
}
static int ar0144_get_sensor_mode(struct ar0144 *sensor, void* pmode)
{
	int ret = 0;
	ret = copy_to_user(pmode, &sensor->cur_mode,
		sizeof(struct vvcam_mode_info_s));
	if (ret != 0)
		ret = -ENOMEM;
	return ret;
}

static int ar0144_set_sensor_mode(struct ar0144 *sensor, void* pmode)
{
	int ret = 0;
	int i = 0;
	struct vvcam_mode_info_s sensor_mode;

	ret = copy_from_user(&sensor_mode, pmode,
		sizeof(struct vvcam_mode_info_s));
	if (ret != 0)
		return -ENOMEM;
	for (i = 0; i < ARRAY_SIZE(par0144_mode_info); i++) 
    {
		if (par0144_mode_info[i].index == sensor_mode.index) 
        {
			memcpy(&sensor->cur_mode, &par0144_mode_info[i],sizeof(struct vvcam_mode_info_s));
			return 0;
		}
	}
	return -ENXIO;
}

static int ar0144_set_exp(struct ar0144 *sensor, u32 exp)
{
	int ret = 0;
	ret |= ar0144_write_reg(sensor, 0x3012, exp);
	return ret;
}
//TBD
static int ar0144_set_gain(struct ar0144 *sensor, u32 gain)
{
	int ret = 0;
	u16 new_ana_gain = 0;
	u16 new_dig_gain = 0;
	u16 dig_gain_hi = 0;
	u16 dig_gain_lo = 0;
	if (gain < (2 << SENSOR_FIX_FRACBITS)) {
		new_ana_gain = (gain-0x400)/33;
	} else if (gain < (4 * (1 << SENSOR_FIX_FRACBITS))) {
		new_ana_gain =0x10 +( (gain-0x800)/33);
	} else if (gain < (8 * (1 << SENSOR_FIX_FRACBITS))) {
		new_ana_gain =0x20 +( (gain-0x1000)/33);
	} else {
		new_ana_gain =0x30 +( (gain-0x2000)/33);
	}
	//c*1000/128=2937-2000
	dig_gain_hi = (gain/1024) <<7;
	dig_gain_lo = (gain%1024)/8;
	new_dig_gain = dig_gain_hi + dig_gain_lo;
	ret = ar0144_write_reg(sensor, 0x3060, new_ana_gain);
	//ret = ar0144_write_reg(sensor, 0x305E, new_dig_gain);
    return ret;
}

static int ar0144_set_fps(struct ar0144 *sensor, u32 fps)
{
	u32 vts;
	int ret = 0;

	if (fps > sensor->cur_mode.ae_info.max_fps) {
		fps = sensor->cur_mode.ae_info.max_fps;
	}
	else if (fps < sensor->cur_mode.ae_info.min_fps) {
		fps = sensor->cur_mode.ae_info.min_fps;
	}
	vts = sensor->cur_mode.ae_info.max_fps *
	      sensor->cur_mode.ae_info.def_frm_len_lines / fps;

	ret |= ar0144_write_reg(sensor, 0x3012, vts);
	sensor->cur_mode.ae_info.cur_fps = fps;

	if (sensor->cur_mode.hdr_mode == SENSOR_MODE_LINEAR) {
		sensor->cur_mode.ae_info.max_integration_line = vts - 1;
	} else {
		if (sensor->cur_mode.stitching_mode ==
		    SENSOR_STITCHING_DUAL_DCG){
			sensor->cur_mode.ae_info.max_vsintegration_line = 44;
			sensor->cur_mode.ae_info.max_integration_line = vts -
				4 - sensor->cur_mode.ae_info.max_vsintegration_line;
		} else {
			sensor->cur_mode.ae_info.max_integration_line = vts - 1;
		}
	}
	sensor->cur_mode.ae_info.curr_frm_len_lines = vts;
	return ret;
}

static int ar0144_get_fps(struct ar0144 *sensor, u32 *pfps)
{
	*pfps = sensor->cur_mode.ae_info.cur_fps;
	return 0;
}

static int ar0144_set_test_pattern(struct ar0144 *sensor, void * arg)
{

	int ret;
	struct sensor_test_pattern_s test_pattern;

	ret = copy_from_user(&test_pattern, arg, sizeof(test_pattern));
	if (ret != 0)
		return -ENOMEM;
	if (test_pattern.enable) {
		switch (test_pattern.pattern) {
		case 0:
			ret |= ar0144_write_reg(sensor, 0x0600, 0x0001);
			break;
		case 1:
			ret |= ar0144_write_reg(sensor, 0x0600, 0x0002);
			break;
		case 2:
			ret |= ar0144_write_reg(sensor, 0x0600, 0x0003);
			break;
		default:
			ret = -1;
			break;
		}
	}
	return ret;
}
static int ar0144_get_lens(struct ar0144 *sensor, void * arg) {

	vvcam_lens_t *pfocus_lens = (vvcam_lens_t *)arg;

	if (!arg)
		return -ENOMEM;

	if (strlen(sensor->focus_lens.name) == 0)
		return -1;

	return copy_to_user(pfocus_lens, &sensor->focus_lens, sizeof(vvcam_lens_t));
}
static long ar0144_priv_ioctl(struct v4l2_subdev *sd,
                              unsigned int cmd,
                              void *arg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0144 *sensor = to_ar0144_device(client);
	long ret = 0;
	struct vvcam_sccb_data_s sensor_reg;
	uint32_t value = 0;

	mutex_lock(&sensor->lock);
	switch (cmd){
	case VVSENSORIOC_S_POWER:
		ret = 0;
		break;
	case VVSENSORIOC_S_CLK:
		ret = 0;
		break;
	case VVSENSORIOC_G_CLK:
		ret = ar0144_get_clk(sensor,arg);
		break;
	case VVSENSORIOC_RESET:
		ret = 0;
		break;
	case VIDIOC_QUERYCAP:
		ret = ar0144_query_capability(sensor, arg);
		break;
	case VVSENSORIOC_QUERY:
		ret = ar0144_query_supports(sensor, arg);
		break;
	case VVSENSORIOC_G_CHIP_ID:
		ret = ar0144_get_sensor_id(sensor, arg);
		break;
	case VVSENSORIOC_G_RESERVE_ID:
		ret = ar0144_get_reserve_id(sensor, arg);
		break;
	case VVSENSORIOC_G_SENSOR_MODE:
		ret = ar0144_get_sensor_mode(sensor, arg);
		break;
	case VVSENSORIOC_S_SENSOR_MODE:
		ret = ar0144_set_sensor_mode(sensor, arg);
		break;
	case VVSENSORIOC_S_STREAM:
		ret = copy_from_user(&value, arg, sizeof(value));
		ret |= ar0144_s_stream(&sensor->subdev, value);
		break;
	case VVSENSORIOC_WRITE_REG:
		ret = copy_from_user(&sensor_reg, arg,
			sizeof(struct vvcam_sccb_data_s));
		ret |= ar0144_write_reg(sensor, sensor_reg.addr,
			sensor_reg.data);
		break;
	case VVSENSORIOC_READ_REG:
		ret = copy_from_user(&sensor_reg, arg, sizeof(struct vvcam_sccb_data_s));
		ret |= ar0144_read_reg(sensor, (u16)sensor_reg.addr, (u16 *)&sensor_reg.data);
		ret |= copy_to_user(arg, &sensor_reg, sizeof(struct vvcam_sccb_data_s));
		break;
	case VVSENSORIOC_S_EXP:
		ret = copy_from_user(&value, arg, sizeof(value));
		ret |= ar0144_set_exp(sensor, value);
		break;
	case VVSENSORIOC_S_GAIN:
		ret = copy_from_user(&value, arg, sizeof(value));
		ret |= ar0144_set_gain(sensor, value);
		break;
	case VVSENSORIOC_S_FPS:
		ret = copy_from_user(&value, arg, sizeof(value));
		ret |= ar0144_set_fps(sensor, value);
		break;
	case VVSENSORIOC_G_FPS:
		ret = ar0144_get_fps(sensor, &value);
		ret |= copy_to_user(arg, &value, sizeof(value));
		break;
	case VVSENSORIOC_S_TEST_PATTERN:
		ret= ar0144_set_test_pattern(sensor, arg);
		break;
	case VVSENSORIOC_G_LENS:
		ret = ar0144_get_lens(sensor, arg);
		break;
	default:
		break;
	}

	mutex_unlock(&sensor->lock);
	return ret;
}

static const struct v4l2_subdev_video_ops ar0144_video_ops = {
	.s_stream = ar0144_s_stream,
};

static const struct v4l2_subdev_pad_ops ar0144_pad_ops = {
	.enum_mbus_code = ar0144_enum_mbus_code,
	.get_fmt = ar0144_get_fmt,
	.set_fmt = ar0144_set_fmt,
};

static const struct v4l2_subdev_core_ops ar0144_core_ops = {
	.s_power = ar0144_s_power,
	.ioctl = ar0144_priv_ioctl,
};

static const struct v4l2_subdev_ops ar0144_subdev_ops = {
	.core = &ar0144_core_ops,
	.video = &ar0144_video_ops,
	.pad = &ar0144_pad_ops,
};

static int ar0144_check_chip_id(struct ar0144 *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;
	u16 val;
	int ret;
	ret = ar0144_read_reg(sensor, AR0144_CHIP_VERSION_REG, &val);
	if ((ret < 0) | (val != AR0144_CHIP_ID)) {
		dev_err(dev, "Sensor AR0144 is not found\n");
        return -ENODEV;
    }

	dev_info(dev, "Sensor AR0144 is found\n");
	return 0;
}

static int ar0144_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations ar0144_sd_media_ops = {
	.link_setup = ar0144_link_setup,
};

static int ar0144_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ar0144 *sensor;
	//struct v4l2_mbus_framefmt *fmt;
	int ret;
	struct v4l2_subdev *sd;

	sensor = devm_kmalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;
	memset(sensor, 0, sizeof(*sensor));

	sensor->i2c_client = client;

	/* request reset pin */
	sensor->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset)) {
		ret = PTR_ERR(sensor->reset);
		dev_err(dev, "fail to get reset pin for AP0144 ret=%d\n", ret);
		return ret;
	}

	sensor->isp_en = devm_gpiod_get_optional(dev, "isp_en", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->isp_en)) {
		ret = PTR_ERR(sensor->isp_en);
		dev_err(dev, "fail to get enable/bypass isp pin for AR0144 ret=%d\n", ret);
		return ret;
	}
	/*Bypass ISP by default*/
	gpiod_set_value_cansleep(sensor->isp_en, 0);

	ret = ar0144_get_regulators(sensor);
	if (ret) {
		dev_err(dev, "Fail to get regulators for AR0144\n");
		return ret;
	}

	ret = regulator_bulk_enable(AR0144_NUM_CONSUMERS, sensor->supplies);
	if (ret) {
		dev_err(dev, "Fail to enable regulators for AR0144\n");
		return ret;
	}

	ar0144_reset(sensor);

	ret = ar0144_check_chip_id(sensor);
	if (ret < 0)
		return ret;

	sd= &sensor->subdev;
	v4l2_i2c_subdev_init(sd, client, &ar0144_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->dev = &client->dev;
	sd->entity.ops = &ar0144_sd_media_ops;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pads[0].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity,
				1,
				sensor->pads);
	if (ret < 0)
		return ret;

	ret = v4l2_async_register_subdev_sensor(sd);
	if (ret < 0) {
		dev_err(&client->dev,"%s--Async register failed, ret=%d\n",
			__func__,ret);
		return ret;
	}
	
	memcpy(&sensor->cur_mode, &par0144_mode_info[0],
			sizeof(struct vvcam_mode_info_s));
	mutex_init(&sensor->lock);
	return ret;
}
static int ar0144_power_off(struct ar0144 *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;
	int ret;
	pr_debug("enter %s\n", __func__);
	ret = regulator_bulk_disable(AR0144_NUM_CONSUMERS, sensor->supplies);
	if (ret) {
		dev_err(dev, "Fail to enable regulators for AR0144\n");
		return ret;
	}
	return 0;
}
static void ar0144_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0144 *sensor = to_ar0144_device(client);

	pr_info("enter %s, %d\n", __func__, __LINE__);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	ar0144_power_off(sensor);
	regulator_bulk_free(AR0144_NUM_CONSUMERS, sensor->supplies);
	mutex_destroy(&sensor->lock);
}

static int __maybe_unused ar0144_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ar0144 *sensor = to_ar0144_device(client);

	sensor->resume_status = sensor->stream_status;
	if (sensor->resume_status) {
		ar0144_s_stream(&sensor->subdev,0);
	}

	return 0;
}

static int __maybe_unused ar0144_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ar0144 *sensor = to_ar0144_device(client);

	if (sensor->resume_status) {
		ar0144_s_stream(&sensor->subdev,1);
	}

	return 0;
}

static const struct dev_pm_ops ar0144_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ar0144_suspend, ar0144_resume)
};
static const struct i2c_device_id ar0144_id[] = {
	{"ar0144", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, ar0144_id);

static const struct of_device_id ar0144_dt_ids[] = {
	{ .compatible = "onsemi,ar0144" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ar0144_dt_ids);

static struct i2c_driver ar0144_i2c_driver = {
	.driver = {
		.name  = "ar0144",
		.pm = &ar0144_pm_ops,
		.of_match_table	= ar0144_dt_ids,
	},
	.id_table = ar0144_id,
	.probe = ar0144_probe,
	.remove   = ar0144_remove,
};

module_i2c_driver(ar0144_i2c_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("AR0144 MIPI Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
