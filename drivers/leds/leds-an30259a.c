// SPDX-License-Identifier: GPL-2.0
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regmap.h>

#define MAX_LEDS 3

#define REG_SRESET 0x00
#define SRESET (1 << 0)

#define REG_LEDON 0x01
#define LED_EN(x) (1 << x)

#define REG_LEDCC(x) (0x03 + x)

#define REG_MAX 0x14

struct an30259a;

struct an30259a_led {
	struct an30259a *chip;
	struct led_classdev cdev;
	int num;
	char name[32];
};

struct an30259a {
	struct mutex mutex;
	struct i2c_client *client;
	struct an30259a_led leds[MAX_LEDS];
	struct regmap *regmap;
};

static int an30259a_brightness(struct an30259a_led *led,
		enum led_brightness brightness)
{
	int ret;
	u8 ledon = i2c_smbus_read_byte_data(led->chip->client, REG_LEDON);

	switch (brightness) {
	case LED_OFF:
		ledon &= ~LED_EN(led->num);
		break;
	default:
		ledon |= LED_EN(led->num);
		break;
	}
	ret = i2c_smbus_write_byte_data(led->chip->client, REG_LEDON,
				ledon);

	if (ret)
		return ret;

	ret = i2c_smbus_write_byte_data(led->chip->client, REG_LEDCC(led->num),
			brightness & 0xff);

	return ret;
}

static int an30259a_led_set(struct led_classdev *cdev,
		enum led_brightness value)
{
	struct an30259a_led *led;
	int ret;

	led = container_of(cdev, struct an30259a_led, cdev);

	mutex_lock(&led->chip->mutex);

	ret = an30259a_brightness(led, value);
	pr_err("set led %d/%s to %d => %d\n", led->num, led->name, value, ret);

	mutex_unlock(&led->chip->mutex);
	return ret;
}

static struct led_platform_data *
an30259a_dt_init(struct i2c_client *client)
{
	struct led_platform_data *pdata;
	struct device_node *np = client->dev.of_node, *child;
	struct led_info *leds;
	int count;
	int res;

	count = of_get_child_count(np);
	if (!count || count > MAX_LEDS)
		return ERR_PTR(-EINVAL);

	leds = devm_kzalloc(&client->dev, sizeof(*leds) * count, GFP_KERNEL);
	if (!leds)
		return ERR_PTR(-ENOMEM);

	for_each_child_of_node(np, child) {
		u32 source;
		struct led_info led = {};
		res = of_property_read_u32(child, "led-sources", &source);
		if ((res != 0) || source >= MAX_LEDS)
			continue;

		led.name = of_get_property(child, "label", NULL);
		led.default_trigger =
			of_get_property(child, "linux,default-trigger", NULL);
		leds[source] = led;
	}

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->leds = leds;
	pdata->num_leds = MAX_LEDS;

	return pdata;
}

static const struct regmap_config an30259a_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_MAX,
};

static int an30259a_probe(struct i2c_client *client)
{
	struct led_platform_data *pdata;
	struct an30259a *chip;
	int i, err;

	pdata = an30259a_dt_init(client);
	if (IS_ERR(pdata)) {
		return PTR_ERR(pdata);
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	mutex_init(&chip->mutex);
	chip->client = client;
	chip->regmap = devm_regmap_init_i2c(client, &regmap_config);

	i2c_set_clientdata(client, chip);

	/* Reset the IC */
	i2c_smbus_write_byte_data(client, REG_SRESET, SRESET);

	for (i = 0; i < pdata->num_leds; i++) {
		chip->leds[i].num = i;
		chip->leds[i].chip = chip;

		if (pdata->leds[i].name)
			snprintf(chip->leds[i].name, sizeof(chip->leds[i].name),
					"an30259a:%s", pdata->leds[i].name);
		else
			snprintf(chip->leds[i].name, sizeof(chip->leds[i].name),
					"an30259a:%d:%.2x:%d", client->adapter->nr,
					client->addr, i);
		if (pdata->leds[i].default_trigger)
			chip->leds[i].cdev.default_trigger =
				pdata->leds[i].default_trigger;

		chip->leds[i].cdev.name = chip->leds[i].name;

		chip->leds[i].cdev.brightness_set_blocking = an30259a_led_set;

		err = led_classdev_register(&client->dev, &chip->leds[i].cdev);
		if (err < 0)
			goto exit;
	}
	return 0;

exit:
	while (i--)
		led_classdev_unregister(&chip->leds[i].cdev);
	return err;
}

static int an30259a_remove(struct i2c_client *client)
{
	struct an30259a *chip = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < MAX_LEDS; i++)
		led_classdev_unregister(&chip->leds[i].cdev);

	return 0;
}

static const struct of_device_id an30259a_match_table[] = {
	{ .compatible = "panasonic,an30259a", },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, an30259a_match_table);

static struct i2c_driver an30259a_driver = {
	.driver = {
		.name = "leds-an32059a",
		.of_match_table = of_match_ptr(an30259a_match_table),
	},
	.probe_new = an30259a_probe,
	.remove = an30259a_remove,
};

module_i2c_driver(an30259a_driver);

MODULE_AUTHOR("Simon Shields <simon@lineageos.org>");
MODULE_DESCRIPTION("AN32059A LED driver");
MODULE_LICENSE("GPL v2");
