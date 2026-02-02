// SPDX-License-Identifier: GPL-2.0-only
/*
 * RG56 Pro built-in gamepad driver
 *
 * Polling-based input driver for the RG56 Pro handheld's integrated gamepad.
 * Reads analog sticks and triggers via IIO (SARADC), ADC-threshold buttons
 * via IIO, and GPIO-connected buttons via gpiod.
 *
 * Compatible with the "play_joystick" device tree node shipped in the
 * stock RK3562 firmware.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/iio/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/property.h>

#define DRIVER_NAME	"rg56pro-joystick"

/* Number of stick axes (LX, LY, RX, RY) */
#define NUM_STICK_CHANS	4
/* Number of trigger axes (L2, R2) */
#define NUM_TRIG_CHANS	2
/* Number of ADC-threshold buttons (dpad L/R/D, B, X, Y) */
#define NUM_ADC_BTNS	6
/* Number of GPIO buttons */
#define NUM_GPIO_BTNS	10

/* Reported axis range to userspace */
#define AXIS_MIN	-32767
#define AXIS_MAX	32767
#define TRIG_MIN	0
#define TRIG_MAX	32767

/* DTS key codes that need remapping to BTN_* range for joydev visibility */
#define KEY_HOME_DTS	102
#define KEY_FN_DTS	464

static const char * const stick_chan_names[NUM_STICK_CHANS] = {
	"button0", "button1", "button2", "button3",
};

static const char * const trig_chan_names[NUM_TRIG_CHANS] = {
	"l2-abs", "r2-abs",
};

/* ADC button channel names (same order as io-channel-names in DTS) */
static const char * const adc_btn_chan_names[NUM_ADC_BTNS] = {
	"left-key", "right-key", "down-key", "b-key", "x-key", "y-key",
};

/* Axis ABS codes for sticks */
static const unsigned int stick_abs_codes[NUM_STICK_CHANS] = {
	ABS_X, ABS_Y, ABS_RX, ABS_RY,
};

/* Axis ABS codes for triggers */
static const unsigned int trig_abs_codes[NUM_TRIG_CHANS] = {
	ABS_Z, ABS_RZ,
};

struct rg56pro_joystick {
	struct input_dev *input;

	/* IIO channels */
	struct iio_channel *stick_chans[NUM_STICK_CHANS];
	struct iio_channel *trig_chans[NUM_TRIG_CHANS];
	struct iio_channel *adc_btn_chans[NUM_ADC_BTNS];

	/* GPIO buttons */
	struct gpio_desc *btn_gpios[NUM_GPIO_BTNS];
	unsigned int btn_codes[NUM_GPIO_BTNS];

	/* ADC button codes and thresholds */
	unsigned int adc_btn_codes[NUM_ADC_BTNS];
	int adc_btn_thresh[NUM_ADC_BTNS]; /* in microvolts */

	/* Stick calibration (millivolts from DTS, converted to microvolts) */
	int axis_min_uv;
	int axis_max_uv;
	int axis_dz_lo_uv;  /* dead zone low */
	int axis_dz_hi_uv;  /* dead zone high */

	/* Trigger calibration (millivolts from DTS, converted to microvolts) */
	int trig_min_uv;
	int trig_max_uv;

	/* Axis inversion flags */
	bool lx_swap;
	bool ly_swap;
};

/*
 * Map a raw microvolt ADC reading to the [-32767, 32767] axis range.
 * Values within the dead zone map to 0.
 */
static int rg56pro_map_stick(struct rg56pro_joystick *joy, int uv, bool swap)
{
	int val;

	/* Clamp to calibration range */
	if (uv < joy->axis_min_uv)
		uv = joy->axis_min_uv;
	if (uv > joy->axis_max_uv)
		uv = joy->axis_max_uv;

	/* Dead zone → 0 */
	if (uv >= joy->axis_dz_lo_uv && uv <= joy->axis_dz_hi_uv)
		return 0;

	if (uv < joy->axis_dz_lo_uv) {
		/* Below dead zone: map [min, dz_lo] → [-32767, 0] */
		val = (int)((long long)(uv - joy->axis_dz_lo_uv) * 32767 /
			    (joy->axis_dz_lo_uv - joy->axis_min_uv));
	} else {
		/* Above dead zone: map [dz_hi, max] → [0, 32767] */
		val = (int)((long long)(uv - joy->axis_dz_hi_uv) * 32767 /
			    (joy->axis_max_uv - joy->axis_dz_hi_uv));
	}

	if (swap)
		val = -val;

	if (val < AXIS_MIN)
		val = AXIS_MIN;
	if (val > AXIS_MAX)
		val = AXIS_MAX;

	return val;
}

/*
 * Map a raw microvolt reading to the [0, 32767] trigger range.
 */
static int rg56pro_map_trigger(struct rg56pro_joystick *joy, int uv)
{
	int val;

	if (uv < joy->trig_min_uv)
		uv = joy->trig_min_uv;
	if (uv > joy->trig_max_uv)
		uv = joy->trig_max_uv;

	val = (int)((long long)(uv - joy->trig_min_uv) * 32767 /
		    (joy->trig_max_uv - joy->trig_min_uv));

	if (val < TRIG_MIN)
		val = TRIG_MIN;
	if (val > TRIG_MAX)
		val = TRIG_MAX;

	return val;
}

static void rg56pro_poll(struct input_dev *input)
{
	struct rg56pro_joystick *joy = input_get_drvdata(input);
	int i, ret, raw;
	bool swap;

	/* Read analog sticks */
	for (i = 0; i < NUM_STICK_CHANS; i++) {
		ret = iio_read_channel_processed(joy->stick_chans[i], &raw);
		if (ret < 0)
			continue;

		/* iio_read_channel_processed returns millivolts; convert to uV */
		raw *= 1000;

		swap = false;
		if (i == 0)
			swap = joy->lx_swap;
		else if (i == 1)
			swap = joy->ly_swap;

		input_report_abs(input, stick_abs_codes[i],
				 rg56pro_map_stick(joy, raw, swap));
	}

	/* Read analog triggers */
	for (i = 0; i < NUM_TRIG_CHANS; i++) {
		ret = iio_read_channel_processed(joy->trig_chans[i], &raw);
		if (ret < 0)
			continue;

		raw *= 1000;
		input_report_abs(input, trig_abs_codes[i],
				 rg56pro_map_trigger(joy, raw));
	}

	/* Read ADC-threshold buttons */
	for (i = 0; i < NUM_ADC_BTNS; i++) {
		ret = iio_read_channel_processed(joy->adc_btn_chans[i], &raw);
		if (ret < 0)
			continue;

		raw *= 1000; /* mV → uV */
		input_report_key(input, joy->adc_btn_codes[i],
				 raw < joy->adc_btn_thresh[i] ? 1 : 0);
	}

	/* Read GPIO buttons (active low: gpiod handles inversion) */
	for (i = 0; i < NUM_GPIO_BTNS; i++) {
		input_report_key(input, joy->btn_codes[i],
				 gpiod_get_value_cansleep(joy->btn_gpios[i]));
	}

	input_sync(input);
}

static int rg56pro_parse_adc_buttons(struct rg56pro_joystick *joy,
				     struct device *dev)
{
	struct device_node *node = dev->of_node;
	struct device_node *child;
	int i = 0;

	for_each_child_of_node(node, child) {
		u32 code, thresh;

		if (i >= NUM_ADC_BTNS) {
			of_node_put(child);
			break;
		}

		if (of_property_read_u32(child, "linux,code", &code)) {
			dev_warn(dev, "ADC button %s missing linux,code\n",
				 child->name);
			continue;
		}

		if (of_property_read_u32(child, "press-threshold-microvolt",
					 &thresh))
			thresh = 80000; /* default 80 mV */

		joy->adc_btn_codes[i] = code;
		joy->adc_btn_thresh[i] = thresh;
		i++;
	}

	if (i != NUM_ADC_BTNS) {
		dev_err(dev, "Expected %d ADC button children, found %d\n",
			NUM_ADC_BTNS, i);
		return -EINVAL;
	}

	return 0;
}

/*
 * Remap DTS key codes that fall outside the BTN_* range (and would be
 * invisible to joydev) into BTN_TL2/BTN_TR2.
 */
static unsigned int rg56pro_remap_code(unsigned int dts_code)
{
	switch (dts_code) {
	case KEY_HOME_DTS:
		return BTN_TL2;   /* 312 — Home → joydev b6 */
	case KEY_FN_DTS:
		return BTN_TR2;   /* 313 — Menu → joydev b7 */
	default:
		return dts_code;
	}
}

static int rg56pro_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rg56pro_joystick *joy;
	struct input_dev *input;
	u32 gpio_codes[NUM_GPIO_BTNS];
	u32 poll_interval = 16;
	int axis_min_mv, axis_max_mv, dz_lo_mv, dz_hi_mv;
	int trig_min_mv, trig_max_mv;
	int i, ret, count;

	joy = devm_kzalloc(dev, sizeof(*joy), GFP_KERNEL);
	if (!joy)
		return -ENOMEM;

	/* --- Parse DT properties --- */

	of_property_read_u32(dev->of_node, "poll-interval", &poll_interval);

	joy->lx_swap = of_property_read_bool(dev->of_node, "l_x_swap");
	joy->ly_swap = of_property_read_bool(dev->of_node, "l_y_swap");

	/* Stick calibration (millivolts → microvolts) */
	if (of_property_read_u32(dev->of_node, "axis-min-value-mv",
				 &axis_min_mv))
		axis_min_mv = 250;
	if (of_property_read_u32(dev->of_node, "axis-max-value-mv",
				 &axis_max_mv))
		axis_max_mv = 1550;
	if (of_property_read_u32(dev->of_node, "axis-dead-zone-l", &dz_lo_mv))
		dz_lo_mv = 790;
	if (of_property_read_u32(dev->of_node, "axis-dead-zone-h", &dz_hi_mv))
		dz_hi_mv = 1010;

	joy->axis_min_uv = axis_min_mv * 1000;
	joy->axis_max_uv = axis_max_mv * 1000;
	joy->axis_dz_lo_uv = dz_lo_mv * 1000;
	joy->axis_dz_hi_uv = dz_hi_mv * 1000;

	/* Trigger calibration */
	if (of_property_read_u32(dev->of_node, "l2-r2-min-value-mv",
				 &trig_min_mv))
		trig_min_mv = 50;
	if (of_property_read_u32(dev->of_node, "l2-r2-max-value-mv",
				 &trig_max_mv))
		trig_max_mv = 800;

	joy->trig_min_uv = trig_min_mv * 1000;
	joy->trig_max_uv = trig_max_mv * 1000;

	/* --- Acquire IIO channels --- */

	for (i = 0; i < NUM_STICK_CHANS; i++) {
		joy->stick_chans[i] = devm_iio_channel_get(dev,
							    stick_chan_names[i]);
		if (IS_ERR(joy->stick_chans[i])) {
			ret = PTR_ERR(joy->stick_chans[i]);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "Failed to get IIO channel %s: %d\n",
					stick_chan_names[i], ret);
			return ret;
		}
	}

	for (i = 0; i < NUM_TRIG_CHANS; i++) {
		joy->trig_chans[i] = devm_iio_channel_get(dev,
							   trig_chan_names[i]);
		if (IS_ERR(joy->trig_chans[i])) {
			ret = PTR_ERR(joy->trig_chans[i]);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "Failed to get IIO channel %s: %d\n",
					trig_chan_names[i], ret);
			return ret;
		}
	}

	for (i = 0; i < NUM_ADC_BTNS; i++) {
		joy->adc_btn_chans[i] = devm_iio_channel_get(dev,
							      adc_btn_chan_names[i]);
		if (IS_ERR(joy->adc_btn_chans[i])) {
			ret = PTR_ERR(joy->adc_btn_chans[i]);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "Failed to get IIO channel %s: %d\n",
					adc_btn_chan_names[i], ret);
			return ret;
		}
	}

	/* --- Parse ADC button child nodes --- */

	ret = rg56pro_parse_adc_buttons(joy, dev);
	if (ret)
		return ret;

	/* --- Acquire GPIO buttons --- */

	count = of_property_count_u32_elems(dev->of_node, "key-gpios-map");
	if (count != NUM_GPIO_BTNS) {
		dev_err(dev, "Expected %d GPIO key codes, got %d\n",
			NUM_GPIO_BTNS, count);
		return -EINVAL;
	}

	ret = of_property_read_u32_array(dev->of_node, "key-gpios-map",
					 gpio_codes, NUM_GPIO_BTNS);
	if (ret) {
		dev_err(dev, "Failed to read key-gpios-map: %d\n", ret);
		return ret;
	}

	for (i = 0; i < NUM_GPIO_BTNS; i++) {
		joy->btn_gpios[i] = devm_gpiod_get_index(dev, "key",
							  i, GPIOD_IN);
		if (IS_ERR(joy->btn_gpios[i])) {
			ret = PTR_ERR(joy->btn_gpios[i]);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "Failed to get GPIO %d: %d\n",
					i, ret);
			return ret;
		}
		joy->btn_codes[i] = rg56pro_remap_code(gpio_codes[i]);
	}

	/* --- Setup input device --- */

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	joy->input = input;
	input->name = DRIVER_NAME;
	input->id.bustype = BUS_HOST;
	input->id.vendor = 0;
	input->id.product = 0;
	input->id.version = 0x0100;

	input_set_drvdata(input, joy);

	/* Register stick axes */
	for (i = 0; i < NUM_STICK_CHANS; i++) {
		input_set_abs_params(input, stick_abs_codes[i],
				     AXIS_MIN, AXIS_MAX, 16, 128);
	}

	/* Register trigger axes */
	for (i = 0; i < NUM_TRIG_CHANS; i++) {
		input_set_abs_params(input, trig_abs_codes[i],
				     TRIG_MIN, TRIG_MAX, 16, 128);
	}

	/* Register GPIO buttons */
	for (i = 0; i < NUM_GPIO_BTNS; i++)
		input_set_capability(input, EV_KEY, joy->btn_codes[i]);

	/* Register ADC buttons */
	for (i = 0; i < NUM_ADC_BTNS; i++)
		input_set_capability(input, EV_KEY, joy->adc_btn_codes[i]);

	/* Setup polling */
	ret = input_setup_polling(input, rg56pro_poll);
	if (ret) {
		dev_err(dev, "Failed to setup polling: %d\n", ret);
		return ret;
	}

	input_set_poll_interval(input, poll_interval);
	input_set_min_poll_interval(input, 8);
	input_set_max_poll_interval(input, 100);

	ret = input_register_device(input);
	if (ret) {
		dev_err(dev, "Failed to register input device: %d\n", ret);
		return ret;
	}

	dev_info(dev, "RG56 Pro joystick registered (poll %u ms)\n",
		 poll_interval);

	return 0;
}

static const struct of_device_id rg56pro_of_match[] = {
	{ .compatible = "play_joystick" },
	{ },
};
MODULE_DEVICE_TABLE(of, rg56pro_of_match);

static struct platform_driver rg56pro_driver = {
	.probe = rg56pro_probe,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = rg56pro_of_match,
	},
};
module_platform_driver(rg56pro_driver);

MODULE_DESCRIPTION("RG56 Pro built-in gamepad driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("dArkOS contributors");
