// SPDX-License-Identifier: GPL-2.0
//
// Copyright (C) 2018 Simon Shields <simon@lineageos.org>
//

#include <linux/bitmap.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include "ssp.h"

const struct firmware *ssp_load_firmware(struct ssp_dev *dev)
{
	const struct firmware *fw;
	int ret = request_firmware(&fw, dev->data->fw_name, dev->dev);
	if (ret) {
		dev_err(dev->dev, "Failed to load firmware '%s': %d\n",
				dev->data->fw_name, ret);
		return NULL;
	}

	return fw;
}

static irqreturn_t ssp_irq_handler(int irq, void *data)
{
	pr_info("%s: %d\n", __func__, irq);

	return IRQ_HANDLED;
}

static irqreturn_t ssp_top_irq_handler(int irq, void *data)
{
	return IRQ_WAKE_THREAD;
}

static int ssp_check_firmware(struct ssp_dev *dev)
{
	const struct firmware *firmware = NULL;
	const struct ssp_ops *ops = dev->ops;
	int ret = 0;
	/* if bootloader mode isn't supported, assume fw rev is ok */
	if (!ops->is_in_bootloader)
		return 0;

	if (ops->is_in_bootloader(dev)) {
		firmware = ssp_load_firmware(dev);
		if (firmware) {
			ret = ops->download_firmware(dev, firmware);
			msleep(3000);
		} else
			ret = ops->leave_bootloader(dev);
	} else {
		int rev = ops->get_firmware_rev(dev);
		if (rev < 0)
			ret = rev;
		else if (rev != dev->data->rev) {
			firmware = ssp_load_firmware(dev);
			if (firmware) {
				ssp_enter_bootloader(dev);
				ret = ops->download_firmware(dev, firmware);
				msleep(3000);
			}
		}
	}

	if (firmware)
		release_firmware(firmware);

	if (ret) {
		dev_err(dev->dev, "SSP firmware download failed: %d\n", ret);
		set_bit(SSP_STATE_SHUTDOWN, &dev->state);
		return ret;
	}

	clear_bit(SSP_STATE_SHUTDOWN, &dev->state);
	dev_info(dev->dev, "SSP firmware version: %d\n",
			ops->get_firmware_rev(dev));
	return 0;
}

int ssp_common_probe(struct ssp_dev *dev, struct device *device,
		const struct ssp_ops *ops, const struct ssp_dev_data *data)
{
	int ret;

	dev->dev = device;
	dev->ops = ops;
	dev->data = data;

	dev->gpio_mcu_nrst = devm_gpiod_get(device, "mcu-nrst", GPIOD_OUT_HIGH);
	if (IS_ERR(dev->gpio_mcu_nrst))
		return PTR_ERR(dev->gpio_mcu_nrst);
	dev->gpio_mcu_int = devm_gpiod_get(device, "ap-mcu-int", GPIOD_OUT_HIGH);
	if (IS_ERR(dev->gpio_mcu_int))
		return PTR_ERR(dev->gpio_mcu_int);
	dev->gpio_mcu_busy = devm_gpiod_get(device, "mcu-busy", GPIOD_IN);
	if (IS_ERR(dev->gpio_mcu_busy))
		return PTR_ERR(dev->gpio_mcu_busy);
	dev->gpio_mcu_ready = devm_gpiod_get(device, "mcu-ready", GPIOD_IN);
	if (IS_ERR(dev->gpio_mcu_ready))
		return PTR_ERR(dev->gpio_mcu_ready);

	dev->irq = gpiod_to_irq(dev->gpio_mcu_ready);
	ret = devm_request_threaded_irq(device, dev->irq, ssp_top_irq_handler,
					ssp_irq_handler, IRQF_TRIGGER_FALLING,
					"ssp-irq", dev);
	if (ret)
		return ret;

	/* disable IRQ while booting */
	disable_irq(dev->irq);

	dev->sensor_pos = devm_kzalloc(dev->dev, sizeof(*dev->sensor_pos) *
				       data->sensor_pos_count, GFP_KERNEL);
	if (!dev->sensor_pos) {
		dev_warn(dev->dev, "Failed to allocate memory for sensor position info\n");
	} else {
		ret = device_property_read_u32_array(dev->dev, "samsung,sensor-positions",
				dev->sensor_pos, data->sensor_pos_count);
		if (ret)
			return ret;
	}

	ret = ssp_check_firmware(dev);
	if (ret) {
		dev_err(dev->dev, "Failed to check firmware version: %d\n",
				ret);
		return ret;
	}

	ret = dev->ops->set_sensor_pos(dev, dev->sensor_pos, data->sensor_pos_count);
	if (ret)
		dev_warn(dev->dev, "Failed to set sensor positions!\n");
	return 0;
}
EXPORT_SYMBOL_GPL(ssp_common_probe);

void ssp_enter_bootloader(struct ssp_dev *dev)
{
	int i;
	for (i = 0; i < 10; i++) {
		gpiod_set_value(dev->gpio_mcu_nrst, 0);
		udelay(10);
		gpiod_set_value(dev->gpio_mcu_nrst, 1);
		msleep(100);
	}

	msleep(50);
	dev_info(dev->dev, "SSP is in bootloader!\n");
}
EXPORT_SYMBOL_GPL(ssp_enter_bootloader);

static int ssp_is_mcu_busy(struct ssp_dev *dev)
{
	return gpiod_get_value(dev->gpio_mcu_busy);
}

static int ssp_is_mcu_ready(struct ssp_dev *dev)
{
	return gpiod_get_value(dev->gpio_mcu_ready);
}

int ssp_wakeup(struct ssp_dev *dev)
{
	int i = 0;

	while ((i++ < 200) && !ssp_is_mcu_busy(dev) &&
		!test_bit(SSP_STATE_SHUTDOWN, &dev->state))
		mdelay(5);
	
	if (i >= 200)
		dev_err(dev->dev, "Timeout waiting for MCU IRQ");

	gpiod_set_value(dev->gpio_mcu_nrst, 0);
	udelay(1);
	gpiod_set_value(dev->gpio_mcu_nrst, 1);

	i = 0;
	while ((i++ < 200) && !ssp_is_mcu_ready(dev) &&
		!test_bit(SSP_STATE_SHUTDOWN, &dev->state))
		mdelay(5);

	if (i >= 200)
		dev_err(dev->dev, "Timeout waiting for MCU ready");

	if (test_bit(SSP_STATE_SHUTDOWN, &dev->state))
		return -ENODEV;

	dev_info(dev->dev, "SSP is awake!\n");
	return 0;
}
EXPORT_SYMBOL_GPL(ssp_wakeup);

MODULE_LICENSE("GPL v2");
