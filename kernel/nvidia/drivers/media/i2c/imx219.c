/*
 * imx219.c - imx219 sensor driver
 *
 * Copyright (c) 2015-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PUR"/home/host/kernel_src_test1/"POSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>
#include <media/imx219.h>

#include "../platform/tegra/camera/camera_gpio.h"
#include "imx219_mode_tbls.h"

//for start_streaming 
#define OV5647_TIMING_REG20			    0x3820
#define OV5647_TIMING_REG21			    0x3821
#define VERTICAL_FLIP				((0x1 << 1) | (0x1 << 6))
#define HORIZONTAL_MIRROR_MASK			(0x3 << 1)

//for stop_streaming
#define OV5647_MAX_FRAME_LENGTH        (0x7fff)
#define OV5647_DEFAULT_LINE_LENGTH     (2592)
#define OV5647_DEFAULT_PIXEL_CLOCK     (160)

//for get_gain
#define OV5647_GAIN_ADDR_MSB			0x350A
#define OV5647_GAIN_ADDR_LSB			0x350B

//for ov5647_get_coarse_time_regs
#define OV5647_COARSE_TIME_ADDR_1		0x3500
#define OV5647_COARSE_TIME_ADDR_2		0x3501
#define OV5647_COARSE_TIME_ADDR_3		0x3502

//for get_frame_length
#define OV5647_FRAME_LENGTH_ADDR_MSB		0x380E
#define OV5647_FRAME_LENGTH_ADDR_LSB		0x380F

//for set_gruop_hold
#define OV5647_GROUP_HOLD_ADDR			0x3208

//custom
#define OV5647_SIG_MSB 0x56
#define OV5647_SIG_LSB 0x47
#define OV5647_SENSOR_NAME "ov5647"

static const struct i2c_device_id ov5647_sensor_id[] = {
  { OV5647_SENSOR_NAME, 0 },
  { }
};

MODULE_DEVICE_TABLE(i2c, ov5647_sensor_id);


static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

struct ov5647 {
	struct i2c_client		*i2c_client;
	struct v4l2_subdev		*subdev;
	struct media_pad		pad;
	struct mutex			lock;
	struct v4l2_mbus_framefmt	format;
	unsigned int			width;
	unsigned int			height;
	int				        power_count;
	struct clk			    *xclk;

	s32				group_hold_prev;
	u32				frame_length;
	bool			group_hold_en;
	
	struct camera_common_data	*s_data;
	struct camera_common_i2c	i2c_dev;
	struct tegracam_device		*tc_dev;
	
	struct gpio_desc		*pwdn;
	unsigned int			flags;

	struct mutex			streaming_lock;
	bool	    			streaming;
	
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = true,
};


static inline void ov5647_get_coarse_time_regs(ov5647_reg *regs,
				u32 coarse_time)
{
	regs->addr = OV5647_COARSE_TIME_ADDR_1;
	regs->val = (coarse_time >> 12) & 0xff;
	(regs + 1)->addr = OV5647_COARSE_TIME_ADDR_2;
	(regs + 1)->val = (coarse_time >> 4) & 0xff;
	(regs + 2)->addr = OV5647_COARSE_TIME_ADDR_3;
	(regs + 2)->val = (coarse_time & 0xf) << 4;
}

static inline void ov5647_get_gain_regs(ov5647_reg *regs, u16 gain)
{
	printk("ov5647_get_gain_regs");

	//gainlow=(unsigned char)(gain_val&0xff);
  	//gainhigh=(unsigned char)((gain_val>>8)&0x3);
	regs->addr = OV5647_GAIN_ADDR_MSB;
	regs->val = (gain >> 8) & 0x3;

	(regs + 1)->addr = OV5647_GAIN_ADDR_LSB;
	(regs + 1)->val = (gain) & 0xFF;
}

static inline int ov5647_read_reg(struct camera_common_data *s_data,
	u16 addr, u8 *val)
{

	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xff;
printk("ov5647_read_reg");
	return err;
}

static inline int ov5647_write_reg(struct camera_common_data *s_data,
	u16 addr, u8 val)
{
	int err = 0;
	dev_err(s_data->dev, "%s: ov5647_start write_reg  \n", __func__);
	err = regmap_write(s_data->regmap, addr, val);
	if (err)
		dev_err(s_data->dev, "%s: i2c write failed, 0x%x = %x",
			__func__, addr, val);

	return err;
}

static int ov5647_write_table(struct ov5647 *priv, const ov5647_reg table[])
{
	struct camera_common_data *s_data = priv->s_data;
	return regmap_util_write_table_8(s_data->regmap,
					 table,
					 NULL, 0,
					 OV5647_TABLE_WAIT_MS,
					 OV5647_TABLE_END);
}
// static int imx219_set_group_hold(struct tegracam_device *tc_dev, bool val)
// {
// 	/* imx219 does not support group hold */
// 	return 0;
// }

// static int imx219_get_fine_integ_time(struct imx219 *priv, u16 *fine_time)
// {
// 	struct camera_common_data *s_data = priv->s_data;
// 	int err = 0;
// 	u8 reg_val[2];

// printk("imx219_get_fine_integ_time");

// 	err = imx219_read_reg(s_data, IMX219_FINE_INTEG_TIME_ADDR_MSB,
// 		&reg_val[0]);
// 	if (err)
// 		goto done;

// 	err = imx219_read_reg(s_data, IMX219_FINE_INTEG_TIME_ADDR_LSB,
// 		&reg_val[1]);
// 	if (err)
// 		goto done;

// 	*fine_time = (reg_val[0] << 8) | reg_val[1];

// done:
// 	return err;
// }
static int ov5647_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	int err;
	struct ov5647 *priv = tc_dev->priv;
	int gh_prev = switch_ctrl_qmenu[priv->group_hold_prev];
	struct device *dev = tc_dev->dev;

	if (priv->group_hold_en == true && gh_prev == SWITCH_OFF) {
		camera_common_i2c_aggregate(&priv->i2c_dev, true);
		/* enter group hold */
		err = ov5647_write_reg(priv->s_data,
				       OV5647_GROUP_HOLD_ADDR, val);

		if (err){
			dev_err(dev, "%s: ov5647_write_reg \n", __func__);
			goto fail;
		}
		
		priv->group_hold_prev = 1;

		dev_err(dev, "%s: enter group hold\n", __func__);

	} else if (priv->group_hold_en == false && gh_prev == SWITCH_ON) {
		/* leave group hold */
		err = ov5647_write_reg(priv->s_data,
				       OV5647_GROUP_HOLD_ADDR, 0x11);
		if (err){
			dev_err(dev, "%s: OV5647_GROUP_HOLD_ADDR 0x11\n", __func__);
			goto fail;
		}
		err = ov5647_write_reg(priv->s_data,
				       OV5647_GROUP_HOLD_ADDR, 0x61);
		if (err){
			dev_err(dev, "%s: OV5647_GROUP_HOLD_ADDR 0x61\n", __func__);
			goto fail;
		}

		camera_common_i2c_aggregate(&priv->i2c_dev, false);

		priv->group_hold_prev = 0;

		dev_err(dev, "%s: leave group hold\n", __func__);
	}

	return 0;

fail:
	dev_err(dev, "%s: Group hold control error\n", __func__);
	return err;
}

static int ov5647_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	

	struct camera_common_data *s_data = tc_dev->s_data;
	struct ov5647 *priv = (struct ov5647 *)tc_dev->priv;
	struct device *dev = s_data->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];

	int err = 0;
	ov5647_reg reg_list[2];
	u16 gain;
	int i;

printk("start ov5647_set_gain");

	if (!priv->group_hold_prev)
		ov5647_set_group_hold(tc_dev, 1);

	/* translate value */
	gain = (u16) (((val * 16) +
			(mode->control_properties.gain_factor / 2)) /
			mode->control_properties.gain_factor);

	ov5647_get_gain_regs(reg_list, gain);

	for (i = 0; i < 2; i++) {
		err = ov5647_write_reg(s_data, reg_list[i].addr, reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_err(dev, "%s: GAIN control error\n", __func__);
	printk("GAIN contorl error");
	return err;
}
static inline void ov5647_get_frame_length_regs(ov5647_reg *regs,
				u32 frame_length)
{
	regs->addr = OV5647_FRAME_LENGTH_ADDR_MSB;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = OV5647_FRAME_LENGTH_ADDR_LSB;
	(regs + 1)->val = (frame_length) & 0xff;
}

static int ov5647_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct ov5647 *priv = (struct ov5647 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	int err = 0;
	ov5647_reg reg_list[2];
	u32 frame_length;
	int i;

printk("ov5647_set_frame_rate");
	if (!priv->group_hold_prev)
		ov5647_set_group_hold(tc_dev, 1);
	
	frame_length =  mode->signal_properties.pixel_clock.val *
		mode->control_properties.framerate_factor /
		mode->image_properties.line_length / val;

	ov5647_get_frame_length_regs(reg_list, frame_length);

	dev_err(dev,
		"%s: val: %llde-6 [fps], frame_length: %u [lines]\n",
		__func__, val, frame_length);

	for (i = 0; i < 2; i++) {
		err = ov5647_write_reg(s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	priv->frame_length = frame_length;

	return 0;

fail:
	dev_err(dev, "%s: FRAME_LENGTH control error\n", __func__);
	printk("FRAME_LENGTH control error\n");
	return err;

}

static int ov5647_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	struct ov5647 *priv = tc_dev->priv;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	ov5647_reg reg_list[3];
	int err;
	u32 coarse_time;
	int i;

printk("ov5647_set_exposure");

	if (!priv->group_hold_prev)
		ov5647_set_group_hold(tc_dev, 1);

	coarse_time = (u32)(((mode->signal_properties.pixel_clock.val*val)
			/mode->image_properties.line_length)/
			mode->control_properties.exposure_factor);

	ov5647_get_coarse_time_regs(reg_list, coarse_time);

	for (i = 0; i < 3; i++) {
		err = ov5647_write_reg(s_data, reg_list[i].addr, reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;
fail:
	dev_err(dev, "%s: COARSE_TIME control error\n", __func__);
	printk("COARSE_TIME control error\n");
	return err;
}

static struct tegracam_ctrl_ops ov5647_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = ov5647_set_gain,
	.set_exposure = ov5647_set_exposure,
	.set_frame_rate = ov5647_set_frame_rate,
	.set_group_hold = ov5647_set_group_hold,
};

static void ov5647_gpio_set(struct camera_common_data *s_data,
			    unsigned int gpio, int val)
{
	struct camera_common_pdata *pdata = s_data->pdata;

	if (pdata && pdata->use_cam_gpio)
		cam_gpio_ctrl(s_data->dev, gpio, val, 1);
	else {
		if (gpio_cansleep(gpio))
			gpio_set_value_cansleep(gpio, val);
		else
			gpio_set_value(gpio, val);
	}
}


static int ov5647_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

printk("ov5647_power_on");

	dev_err(dev, "%s: power on\n", __func__);
	if (pdata && pdata->power_on) {
		err = pdata->power_on(pw);
		if (err)
			dev_err(dev, "%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	if (pw->avdd) {
		err = regulator_enable(pw->avdd);
		if (err)
			goto ov5647_avdd_fail;
	}

	if (pw->iovdd) {
		err = regulator_enable(pw->iovdd);
		if (err)
			goto ov5647_iovdd_fail;
	}

	if (pw->dvdd) {
		err = regulator_enable(pw->dvdd);
		if (err)
			goto ov5647_avdd_fail;
	}

	usleep_range(1, 2);
	if (gpio_is_valid(pw->pwdn_gpio))
		ov5647_gpio_set(s_data, pw->pwdn_gpio, 1);

	usleep_range(2000, 2010);

	/*
	 * datasheet 2.9: reset requires ~2ms settling time
	 * a power on reset is generated after core power becomes stable
	 */

	if (gpio_is_valid(pw->reset_gpio))
		ov5647_gpio_set(s_data, pw->reset_gpio, 1);

	/* datasheet fig 2-9: t3 */
	usleep_range(2000, 2010);

	pw->state = SWITCH_ON;

	return 0;

ov5647_avdd_fail:
	printk("error ov5647_avdd_fail");
	regulator_disable(pw->iovdd);

ov5647_iovdd_fail:
	printk("error ov5647_iovdd_fail");
	regulator_disable(pw->avdd);
	return -ENODEV;
}

static int ov5647_power_off(struct camera_common_data *s_data)
{


	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

printk("ov5647_power_off");

	dev_err(dev, "%s: power off\n", __func__);

	if (pdata && pdata->power_off) {
		err = pdata->power_off(pw);
		if (err) {
			dev_err(dev, "%s failed.\n", __func__);
			return err;
		} else {
			goto power_off_done;
		}
	}

	usleep_range(21, 25);
	if (gpio_is_valid(pw->pwdn_gpio))
		ov5647_gpio_set(s_data, pw->pwdn_gpio, 0);
	usleep_range(1, 2);
	if (gpio_is_valid(pw->reset_gpio))
		ov5647_gpio_set(s_data, pw->reset_gpio, 0);

	/* datasheet 2.9: reset requires ~2ms settling time*/
	usleep_range(2000, 2010);

	if (pw->iovdd) regulator_disable(pw->iovdd);
	if (pw->avdd) regulator_disable(pw->avdd);

power_off_done:
	pw->state = SWITCH_OFF;
	return 0;
}

static int ov5647_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = tc_dev->dev;

printk("ov5647_power_put");

	if (unlikely(!pw))
		return -EFAULT;

	if (pdata && pdata->use_cam_gpio)
		cam_gpio_deregister(dev, pw->pwdn_gpio);
	else {
		if (gpio_is_valid(pw->pwdn_gpio))
			gpio_free(pw->pwdn_gpio);
		if (gpio_is_valid(pw->reset_gpio))
			gpio_free(pw->reset_gpio);
	}
	return 0;
}

static int ov5647_power_get(struct tegracam_device *tc_dev)
{
	
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct clk *parent;
	const char *mclk_name;
	const char *parentclk_name;
	int err = 0;
	int ret = 0;


printk("ov5647_power_get");

	if (!pdata) {
		dev_err(dev, "pdata missing\n");
		return -EFAULT;
	}

	mclk_name = pdata->mclk_name ?
		    pdata->mclk_name : "cam_mclk1";
	pw->mclk = devm_clk_get(dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		dev_err(dev, "unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}
	parentclk_name = pdata->parentclk_name;
	if (parentclk_name) {
		parent = devm_clk_get(dev, parentclk_name);
		if (IS_ERR(parent)) {
			dev_err(dev, "unable to get parent clock %s",
				parentclk_name);
		} else
			clk_set_parent(pw->mclk, parent);
	}

	// /* Sensor MCLK (aka. INCK) */
	// if (pdata->mclk_name) {
	// 	pw->mclk = devm_clk_get(dev, pdata->mclk_name);
	// 	if (IS_ERR(pw->mclk)) {
	// 		dev_err(dev, "unable to get clock %s\n",
	// 			pdata->mclk_name);
	// 		return PTR_ERR(pw->mclk);
	// 	}

	// 	if (pdata->parentclk_name) {
	// 		parent = devm_clk_get(dev, pdata->parentclk_name);
	// 		if (IS_ERR(parent)) {
	// 			dev_err(dev, "unable to get parent clock %s",
	// 				pdata->parentclk_name);
	// 		} else
	// 			clk_set_parent(pw->mclk, parent);
	// 	}
	// }

	/* analog 2.8v */
	if (pdata->regulators.avdd)
		err |= camera_common_regulator_get(dev,
				&pw->avdd, pdata->regulators.avdd);
	/* IO 1.8v */
	if (pdata->regulators.iovdd)
		err |= camera_common_regulator_get(dev,
				&pw->iovdd, pdata->regulators.iovdd);
	// /* dig 1.2v */
	// if (pdata->regulators.dvdd)
	// 	err |= camera_common_regulator_get(dev,
	// 			&pw->dvdd, pdata->regulators.dvdd);

	if (!err) {
		pw->reset_gpio = pdata->reset_gpio;
		pw->pwdn_gpio = pdata->pwdn_gpio;
	}
	if (pdata->use_cam_gpio) {
		err = cam_gpio_register(dev, pw->pwdn_gpio);
		if (err)
			dev_err(dev, "%s ERR can't register cam gpio %u!\n",
				 __func__, pw->pwdn_gpio);
	} else {
		if (gpio_is_valid(pw->pwdn_gpio)) {
			ret = gpio_request(pw->pwdn_gpio, "cam_pwdn_gpio");
			if (ret < 0) {
				dev_err(dev, "%s can't request pwdn_gpio %d\n",
					__func__, ret);
			}
			gpio_direction_output(pw->pwdn_gpio, 1);
		}
		if (gpio_is_valid(pw->reset_gpio)) {
			ret = gpio_request(pw->reset_gpio, "cam_reset_gpio");
			if (ret < 0) {
				dev_err(dev, "%s can't request reset_gpio %d\n",
					__func__, ret);
			}
			gpio_direction_output(pw->reset_gpio, 1);
		}
	}

	pw->state = SWITCH_OFF;
	return err;
}

static struct camera_common_pdata *ov5647_parse_dt(
	struct tegracam_device *tc_dev)
{
	// struct device *dev = tc_dev->dev;
	// struct device_node *np = dev->of_node;
	// struct camera_common_pdata *board_priv_pdata;
	// const struct of_device_id *match;
	struct camera_common_pdata *ret = NULL;
	// int err = 0;
	// int gpio;

	printk("ov5647_parse_dt");
	return(ret);

// 	if (!np)
// 		return NULL;

// 	match = of_match_device(imx219_of_match, dev);
// 	if (!match) {
// 		dev_err(dev, "Failed to find matching dt id\n");
// 		return NULL;
// 	}

// 	board_priv_pdata = devm_kzalloc(dev,
// 		sizeof(*board_priv_pdata), GFP_KERNEL);
// 	if (!board_priv_pdata)
// 		return NULL;

// 	gpio = of_get_named_gpio(np, "reset-gpios", 0);
// 	if (gpio < 0) {
// 		if (gpio == -EPROBE_DEFER)
// 			ret = ERR_PTR(-EPROBE_DEFER);
// 		dev_err(dev, "reset-gpios not found\n");
// 		goto error;
// 	}
// 	board_priv_pdata->reset_gpio = (unsigned int)gpio;

// 	err = of_property_read_string(np, "mclk", &board_priv_pdata->mclk_name);
// 	if (err)
// 		dev_err(dev, "mclk name not present, "
// 			"assume sensor driven externally\n");

// 	err = of_property_read_string(np, "avdd-reg",
// 		&board_priv_pdata->regulators.avdd);
// 	err |= of_property_read_string(np, "iovdd-reg",
// 		&board_priv_pdata->regulators.iovdd);
// 	err |= of_property_read_string(np, "dvdd-reg",
// 		&board_priv_pdata->regulators.dvdd);
// 	if (err)
// 		dev_err(dev, "avdd, iovdd and/or dvdd reglrs. not present, "
// 			"assume sensor powered independently\n");

// 	board_priv_pdata->has_eeprom =
// 		of_property_read_bool(np, "has-eeprom");

// 	return board_priv_pdata;

// error:
// 	devm_kfree(dev, board_priv_pdata);

// 	return ret;


}

static int ov5647_set_mode(struct tegracam_device *tc_dev)
{
	
	struct ov5647 *priv = (struct ov5647 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	int err = 0;

printk("ov5647_set_mode");

	err = ov5647_write_table(priv, mode_table[s_data->mode_prop_idx]);
	if (err)
		return err;

	return 0;
}

static int ov5647_start_streaming(struct tegracam_device *tc_dev)
{
	struct ov5647 *priv = (struct ov5647 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;
	int err;
	u8 val;

	mutex_lock(&priv->streaming_lock);
	err = ov5647_write_table(priv, mode_table[OV5647_MODE_START_STREAM]);
	if (err) {
		mutex_unlock(&priv->streaming_lock);
		goto exit;
	} else {
		priv->streaming = true;
		mutex_unlock(&priv->streaming_lock);
	}
	if (pdata->v_flip) {
		ov5647_read_reg(s_data, OV5647_TIMING_REG20, &val);
		ov5647_write_reg(s_data, OV5647_TIMING_REG20,
				 val | VERTICAL_FLIP);
	}
	if (pdata->h_mirror) {
		ov5647_read_reg(s_data, OV5647_TIMING_REG21, &val);
		ov5647_write_reg(s_data, OV5647_TIMING_REG21,
				 val | HORIZONTAL_MIRROR_MASK);
	} else {
		ov5647_read_reg(s_data, OV5647_TIMING_REG21, &val);
		ov5647_write_reg(s_data, OV5647_TIMING_REG21,
				 val & (~HORIZONTAL_MIRROR_MASK));
	}

	return 0;

exit:
	dev_err(dev, "%s: error starting stream\n", __func__);
	printk("error starting stream\n");
	return err;
}

static void ov5647_update_ctrl_range(struct camera_common_data *s_data, s32 frame_length)
{

  //sensor_write(sd, 0x380f, (frame_length & 0xff));
	 ov5647_write_reg(s_data, 0x380f, (frame_length & 0xff) );
  //sensor_write(sd, 0x380e, (frame_length >> 8));
	 ov5647_write_reg(s_data, 0x380e, (frame_length >> 8) );

}

static int ov5647_stop_streaming(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct ov5647 *priv = (struct ov5647 *)tegracam_get_privdata(tc_dev);
	struct device *dev = s_data->dev;
	u32 frame_time;
	int err;

	printk("ov5647_stop_streaming");
	ov5647_update_ctrl_range(s_data, OV5647_MAX_FRAME_LENGTH);

	mutex_lock(&priv->streaming_lock);
	err = ov5647_write_table(priv, mode_table[OV5647_MODE_STOP_STREAM]);
	if (err) {
		mutex_unlock(&priv->streaming_lock);
		goto exit;
	} else {
		priv->streaming = false;
		mutex_unlock(&priv->streaming_lock);
	}
	/*
	 * Wait for one frame to make sure sensor is set to
	 * software standby in V-blank
	 *
	 * frame_time = frame length rows * Tline
	 * Tline = line length / pixel clock (in MHz)
	 */

	frame_time = priv->frame_length *
		OV5647_DEFAULT_LINE_LENGTH / OV5647_DEFAULT_PIXEL_CLOCK;

	usleep_range(frame_time, frame_time + 1000);

	return 0;

exit:
	dev_err(dev, "%s: error stopping stream\n", __func__);
	printk("error stopping stream");
	return err;
}

static struct camera_common_sensor_ops ov5647_common_ops = {
	.numfrmfmts = ARRAY_SIZE(ov5647_frmfmt),
	.frmfmt_table = ov5647_frmfmt,
	.power_on = ov5647_power_on,
	.power_off = ov5647_power_off,
	.write_reg = ov5647_write_reg,
	.read_reg = ov5647_read_reg,
	.parse_dt = ov5647_parse_dt,
	.power_get = ov5647_power_get,
	.power_put = ov5647_power_put,
	.set_mode = ov5647_set_mode,
	.start_streaming = ov5647_start_streaming,
	.stop_streaming = ov5647_stop_streaming,
};

static int ov5647_board_setup(struct ov5647 *priv)
{
	
	
	struct camera_common_data *s_data = priv->s_data;
	struct device *dev = s_data->dev;

	int err = 0;

printk("ov5647_board_setup");
	err = camera_common_mclk_enable(s_data);

	if ( err ) {
		dev_err(dev, "Error %d turning on mclk\n", err);
		printk("ov5674 Error turning on mclk\n");
		return err;
	}

	err = ov5647_power_on(s_data);
	if (err) {
		dev_err(dev, "Error %d during power on sensor\n", err);
		printk("ov5647 Error during power on sensor\n");

		return err;
	}

	/* Probe sensor model id registers */
	// err = ov5647_read_reg(s_data, IMX219_MODEL_ID_ADDR_MSB, &reg_val[0]);
    //     printk("ov5647 reg0 %X",reg_val[0]);
	// if (err) {
	// 	dev_err(dev, "%s: error during i2c read probe (%d)\n",
	// 		__func__, err);
	// 	goto err_reg_probe;
	// }
	// err = imx219_read_reg(s_data, IMX219_MODEL_ID_ADDR_LSB, &reg_val[1]);
    //     printk("ov5647 reg1 %X",reg_val[1]);
	// if (err) {
	// 	dev_err(dev, "%s: error during i2c read probe (%d)\n",
	// 		__func__, err);
	// 	goto err_reg_probe;
	// }
	// if (!((reg_val[0] == OV5647_SIG_MSB) && reg_val[1] == OV5647_SIG_LSB))
	// 	dev_err(dev, "%s: invalid sensor model id: %x%x\n",
	// 		__func__, reg_val[0], reg_val[1]);

// 	/* Sensor fine integration time */
// 	err = imx219_get_fine_integ_time(priv, &priv->fine_integ_time);
// 	if (err)
// 		dev_err(dev, "%s: error querying sensor fine integ. time\n",
// 			__func__);

// err_reg_probe:
// 	imx219_power_off(s_data);

// err_power_on:
// 	if (pdata->mclk_name)
// 		camera_common_mclk_disable(s_data);

// done:
// 	return err;
	ov5647_power_off(s_data);
	camera_common_mclk_disable(s_data);
	return err;
}

static int ov5647_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_err(&client->dev, "1nd %s:\n", __func__);
	printk("ov5647_open");
	return 0;
}

static const struct v4l2_subdev_internal_ops ov5647_subdev_internal_ops = {
	.open = ov5647_open,
};

static int ov5647_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tegracam_device *tc_dev;
	struct ov5647 *priv;
	int err;

		client->addr=0x36;

printk("ov5647_probe");

	dev_err(dev, "probing v4l2 sensor at addr 0x%0x\n", client->addr);

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	priv = devm_kzalloc(dev,
			sizeof(struct ov5647), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev,
			sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "ov5647", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_config;
	tc_dev->sensor_ops = &ov5647_common_ops;
	tc_dev->v4l2sd_internal_ops = &ov5647_subdev_internal_ops;
	tc_dev->tcctrl_ops = &ov5647_ctrl_ops;

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

	err = ov5647_board_setup(priv);
	if (err) {
		tegracam_device_unregister(tc_dev);
		dev_err(dev, "board setup failed\n");
		return err;
	}

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	dev_err(dev, "detected imx219 sensor\n");

	return 0;
}

static int ov5647_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ov5647 *priv = (struct ov5647 *)s_data->priv;
	
	printk("ov5647_remove");

	tegracam_v4l2subdev_unregister(priv->tc_dev);
	ov5647_power_put(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);
	mutex_destroy(&priv->streaming_lock);

	return 0;
}

static struct i2c_driver ov5647_sensor_driver = {
	.driver = {
		.name = OV5647_SENSOR_NAME,
		.owner = THIS_MODULE,
		//.of_match_table = of_match_ptr(imx219_of_match),
	},
	.probe = ov5647_probe,
	.remove = ov5647_remove,
	.id_table = ov5647_sensor_id,

};

module_i2c_driver(ov5647_sensor_driver);

MODULE_DESCRIPTION("Media Controller driver for Sony IMX219");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL v2");
