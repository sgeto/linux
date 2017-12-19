// SPDX-License-Identifier: GPL-2.0+
//
// Driver for Panasonic AN30259A 3-channel LED driver
//
// Copyright (c) 2018 Simon Shields <simon@lineageos.org>
//
// Datasheet:
// https://www.alliedelec.com/m/d/a9d2b3ee87c2d1a535a41dd747b1c247.pdf

#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <uapi/linux/uleds.h>

#define MAX_LEDS 3

#define REG_SRESET 0x00
#define LED_SRESET BIT(0)

/* LED power registers */
#define REG_LED_ON 0x01
#define LED_EN(x) BIT(x - 1)
#define LED_SLOPE(x) BIT((x - 1) + 4)

#define REG_LEDCC(x) (0x03 + (x - 1))

/* slope control registers */
#define REG_SLOPE(x) (0x06 + (x - 1))
#define LED_SLOPETIME1(x) (x)
#define LED_SLOPETIME2(x) ((x) << 4)

#define REG_LEDCNT1(x) (0x09 + (4 * (x - 1)))
#define LED_DUTYMAX(x) ((x) << 4)
#define LED_DUTYMID(x) (x)

#define REG_LEDCNT2(x) (0x0A + (4 * (x - 1)))
#define LED_DELAY(x) ((x) << 4)
#define LED_DUTYMIN(x) (x)

/* detention time control (length of each slope step) */
#define REG_LEDCNT3(x) (0x0B + (4 * (x - 1)))
#define LED_DT1(x) (x)
#define LED_DT2(x) ((x) << 4)

#define REG_LEDCNT4(x) (0x0C + (4 * (x - 1)))
#define LED_DT3(x) (x)
#define LED_DT4(x) ((x) << 4)

#define REG_MAX 0x14

#define BLINK_MAX_TIME 7500 /* ms */
#define SLOPE_RESOLUTION 500 /* ms */

#define STATE_OFF 0
#define STATE_KEEP 1
#define STATE_ON 2

struct an30259a;

struct an30259a_led {
	struct an30259a *chip;
	struct led_classdev cdev;
	u32 num;
	u32 default_state;
	char label[LED_MAX_NAME_SIZE];
};

struct an30259a {
	struct mutex mutex; /* held when writing to registers */
	struct i2c_client *client;
	struct an30259a_led leds[MAX_LEDS];
	struct regmap *regmap;
	int num_leds;
};

/*
 * When doing sloping, AN30259A only allows us
 * to set the first four bits of a 7-bit brightness
 * value, as opposed to the full 8-bit range
 * allowed in constant output mode.
 *
 * This function returns the best approximation
 * of the 8-bit brightness value in the 7-bit range
 * we have available.
 */
static u8 an30259a_get_dutymax(u8 brightness)
{
	u8 duty_max, floor, ceil;

	/* squash 8 bit number into 7-bit PWM range. */
	duty_max = brightness >> 1;

	/*
	 * Bottom 3 bits are always set for DUTYMAX,
	 * so figure out the closest value.
	 */
	ceil = duty_max | 0x7;
	floor = ceil - 0x8;

	if ((duty_max - floor) < (ceil - duty_max))
		duty_max = floor >> 3;
	else
		duty_max = ceil >> 3;

	return duty_max;
}

static int an30259a_brightness_set(struct led_classdev *cdev,
				   enum led_brightness brightness)
{
	struct an30259a_led *led;
	int ret;
	unsigned int led_on;
	u8 dutymax;

	led = container_of(cdev, struct an30259a_led, cdev);
	mutex_lock(&led->chip->mutex);

	ret = regmap_read(led->chip->regmap, REG_LED_ON, &led_on);
	if (ret)
		goto error;

	switch (brightness) {
	case LED_OFF:
		led_on &= ~LED_EN(led->num);
		led_on &= ~LED_SLOPE(led->num);
		break;
	default:
		led_on |= LED_EN(led->num);
		dutymax = an30259a_get_dutymax(brightness & 0xff);
		ret = regmap_write(led->chip->regmap, REG_LEDCNT1(led->num),
				   LED_DUTYMAX(dutymax) | LED_DUTYMID(dutymax));
		if (ret)
			goto error;
		break;
	}
	ret = regmap_write(led->chip->regmap, REG_LED_ON, led_on);
	if (ret)
		goto error;

	ret = regmap_write(led->chip->regmap, REG_LEDCC(led->num),
			   brightness & 0xff);

error:
	mutex_unlock(&led->chip->mutex);

	return ret;
}

static int an30259a_blink_set(struct led_classdev *cdev,
			      unsigned long *delay_off, unsigned long *delay_on)
{
	struct an30259a_led *led;
	int ret, num;
	unsigned int led_on;
	unsigned long off = *delay_off, on = *delay_on;

	led = container_of(cdev, struct an30259a_led, cdev);

	mutex_lock(&led->chip->mutex);
	num = led->num;

	/* slope time - multiples of 500ms only, floored */
	off -= off % SLOPE_RESOLUTION;
	/* don't floor off time to zero if a non-zero time was requested */
	if (!off && *delay_off)
		off += SLOPE_RESOLUTION;
	else if (off > BLINK_MAX_TIME)
		off = BLINK_MAX_TIME;
	*delay_off = off;

	on -= on % SLOPE_RESOLUTION;
	/* don't floor on time to zero if a non-zero time was requested */
	if (!on && *delay_on)
		on += SLOPE_RESOLUTION;
	else if (on > BLINK_MAX_TIME)
		on = BLINK_MAX_TIME;
	*delay_on = on;

	/* convert into values the HW will understand */
	off /= SLOPE_RESOLUTION;
	on /= SLOPE_RESOLUTION;

	/* duty min should be zero (=off), delay should be zero */
	ret = regmap_write(led->chip->regmap, REG_LEDCNT2(num),
			   LED_DELAY(0) | LED_DUTYMIN(0));
	if (ret)
		goto error;

	/* reset detention time (no "breathing" effect) */
	ret = regmap_write(led->chip->regmap, REG_LEDCNT3(num),
			   LED_DT1(0) | LED_DT2(0));
	if (ret)
		goto error;
	ret = regmap_write(led->chip->regmap, REG_LEDCNT4(num),
			   LED_DT3(0) | LED_DT4(0));
	if (ret)
		goto error;

	/* slope time controls on/off cycle length */
	ret = regmap_write(led->chip->regmap, REG_SLOPE(num),
			   LED_SLOPETIME1(off) | LED_SLOPETIME2(on));
	if (ret)
		goto error;

	/* Finally, enable slope mode. */
	ret = regmap_read(led->chip->regmap, REG_LED_ON, &led_on);
	if (ret)
		goto error;

	led_on |= LED_SLOPE(num);

	ret = regmap_write(led->chip->regmap, REG_LED_ON, led_on);

error:
	mutex_unlock(&led->chip->mutex);

	return ret;
}

static int an30259a_dt_init(struct i2c_client *client,
			    struct an30259a *chip)
{
	struct device_node *np = client->dev.of_node, *child;
	int count, ret;
	int i = 0;
	const char *str;
	struct an30259a_led *led;

	count = of_get_child_count(np);
	if (!count || count > MAX_LEDS)
		return -EINVAL;

	for_each_available_child_of_node(np, child) {
		u32 source;

		ret = of_property_read_u32(child, "reg", &source);
		if (ret != 0 || !source || source > MAX_LEDS) {
			dev_err(&client->dev, "Couldn't read LED address: %d\n",
				ret);
			count--;
			continue;
		}

		led = &chip->leds[i];

		led->num = source;
		led->chip = chip;

		if (of_property_read_string(child, "label", &str))
			snprintf(led->label, sizeof(led->label), "an30259a::");
		else
			snprintf(led->label, sizeof(led->label), "an30259a:%s",
				 str);

		led->cdev.name = led->label;

		if (!of_property_read_string(child, "default-state", &str)) {
			if (!strcmp(str, "on"))
				led->default_state = STATE_ON;
			else if (!strcmp(str, "keep"))
				led->default_state = STATE_KEEP;
			else
				led->default_state = STATE_OFF;
		}

		of_property_read_string(child, "linux,default-trigger",
					&led->cdev.default_trigger);

		i++;
	}

	if (!count)
		return -EINVAL;

	chip->num_leds = i;

	return 0;
}

static const struct regmap_config an30259a_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_MAX,
};

static void an30259a_init_default_state(struct an30259a_led *led)
{
	struct an30259a *chip = led->chip;
	int led_on, err;

	switch (led->default_state) {
	case STATE_ON:
		led->cdev.brightness = LED_FULL;
		break;
	case STATE_KEEP:
		err = regmap_read(chip->regmap, REG_LED_ON, &led_on);
		if (err)
			break;

		if (!(led_on & LED_EN(led->num))) {
			led->cdev.brightness = LED_OFF;
			break;
		}
		regmap_read(chip->regmap, REG_LEDCC(led->num),
			    &led->cdev.brightness);
		break;
	default:
		led->cdev.brightness = LED_OFF;
	}

	an30259a_brightness_set(&led->cdev, led->cdev.brightness);
}

static int an30259a_probe(struct i2c_client *client)
{
	struct an30259a *chip;
	int i, err;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	err = an30259a_dt_init(client, chip);
	if (err < 0)
		return err;

	mutex_init(&chip->mutex);
	chip->client = client;
	i2c_set_clientdata(client, chip);

	chip->regmap = devm_regmap_init_i2c(client, &an30259a_regmap_config);

	for (i = 0; i < chip->num_leds; i++) {
		an30259a_init_default_state(&chip->leds[i]);
		chip->leds[i].cdev.brightness_set_blocking =
			an30259a_brightness_set;
		chip->leds[i].cdev.blink_set = an30259a_blink_set;

		err = devm_led_classdev_register(&client->dev,
						 &chip->leds[i].cdev);
		if (err < 0)
			goto exit;
	}
	return 0;

exit:
	mutex_destroy(&chip->mutex);
	return err;
}

static int an30259a_remove(struct i2c_client *client)
{
	struct an30259a *chip = i2c_get_clientdata(client);

	mutex_destroy(&chip->mutex);

	return 0;
}

static const struct of_device_id an30259a_match_table[] = {
	{ .compatible = "panasonic,an30259a", },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, an30259a_match_table);

static const struct i2c_device_id an30259a_id[] = {
	{ "an30259a", 0 },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, an30259a_id);

static struct i2c_driver an30259a_driver = {
	.driver = {
		.name = "leds-an32059a",
		.of_match_table = of_match_ptr(an30259a_match_table),
	},
	.probe_new = an30259a_probe,
	.remove = an30259a_remove,
	.id_table = an30259a_id,
};

module_i2c_driver(an30259a_driver);

MODULE_AUTHOR("Simon Shields <simon@lineageos.org>");
MODULE_DESCRIPTION("AN32059A LED driver");
MODULE_LICENSE("GPL v2");
