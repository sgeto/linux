// SPDX-License-Identifier: GPL-2.0
//
// Copyright (C) 2018 Simon Shields <simon@lineageos.org>
//

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include "ssp.h"
#include "ssp_cmd.h"

#define I2C_ADDR_SENSORHUB 0x18
#define I2C_ADDR_BOOTLOADER 0x26

#define BL_READ_NEXT			(0x20)
#define BL_APP_STATE_MASK		(0xf0)
#define BL_APP_STATE_CRC_FAIL		(0x40)
#define BL_APP_STATE_WAITING_FRAME	(0x80)
#define BL_APP_STATE_WAITING_CMD	(0xc0)
#define BL_BOOT_STATE_MASK		(0x3f)
#define BL_FRAME_CRC_CHECK		(0x02)
#define BL_FRAME_CRC_FAIL		(0x03)
#define BL_FRAME_CRC_PASS		(0x04)

struct ssp_i2c_priv {
	struct ssp_dev dev;
	struct i2c_client *i2c_boot;
	struct i2c_client *i2c_main;
};
#define to_priv(dev) container_of(dev, struct ssp_i2c_priv, dev)

static int ssp_i2c_check_bl_state(struct ssp_i2c_priv *priv, u8 expected)
{
	u8 val, unused;
	int ret;

recheck:
	dev_info(priv->dev.dev, "address: %#x\n", priv->i2c_boot->addr);
	ret = i2c_master_recv(priv->i2c_boot, &val, 1);
	if (ret < 0) {
		dev_err(priv->dev.dev, "errnk: %d\n", ret);
		return -EIO;
	}

	dev_info(priv->dev.dev, "Got val: %#x\n", val);
	if (val & BL_READ_NEXT) {
		if (i2c_master_recv(priv->i2c_boot, &unused, 1) < 0 ||
			i2c_master_recv(priv->i2c_boot, &unused, 1) < 0) {
			dev_info(priv->dev.dev, "Dummy #1 failed\n");
			return -EIO;
		}
		val &= ~BL_READ_NEXT;
	}

	if ((val & BL_APP_STATE_MASK) == BL_APP_STATE_CRC_FAIL) {
		dev_info(priv->dev.dev, "APP_STATE_CRC_FAIL\n");
		if (i2c_master_recv(priv->i2c_boot, &val, 1) < 0)
			return -EIO;

		if (val & BL_READ_NEXT) {
			if (i2c_master_recv(priv->i2c_boot, &unused, 1) < 0)
				return -EIO;
			if (i2c_master_recv(priv->i2c_boot, &unused, 1) < 0)
				return -EIO;
			val &= ~BL_READ_NEXT;
		}
	}

	switch (expected) {
	case BL_APP_STATE_WAITING_CMD:
	case BL_APP_STATE_WAITING_FRAME:
		val &= ~BL_BOOT_STATE_MASK;
		break;
	case BL_FRAME_CRC_PASS:
		if (val == BL_FRAME_CRC_CHECK)
			goto recheck;
		break;
	default:
		return -EINVAL;
	}

	dev_info(priv->dev.dev, "BL state: %#x, want: %#x\n", val, expected);
	if (val != expected) {
		dev_err(priv->dev.dev, "Bad bootloader state %#x\n", val);
		return -EINVAL;
	}

	return 0;
}

static bool ssp_i2c_is_in_bootloader(struct ssp_dev *dev)
{
	struct ssp_i2c_priv *priv = to_priv(dev);
	int ret;
	ret = ssp_i2c_check_bl_state(priv, BL_APP_STATE_WAITING_CMD);
	dev_info(dev->dev, "is in bootloader? %d\n", ret);
	if (ret == 0)
		return true;
	return false;
}

static int ssp_i2c_unlock_bootloader(struct ssp_i2c_priv *priv)
{
	u8 cmd[2] = {0xdc, 0xaa};

	dev_info(priv->dev.dev, "Unlocking bootloader\n");
	if (i2c_master_send(priv->i2c_boot, cmd, 2) != 2)
		return -EIO;

	return 0;
}

static int ssp_i2c_write_firmware_frame(struct ssp_i2c_priv *priv,
		const u8 *data, size_t size)
{
	int ret = i2c_master_send(priv->i2c_boot, data, size);
	if (ret != size) {
		dev_err(priv->dev.dev, "Failed to send data: %d\n", ret);
		return -EIO;
	}
	return 0;
}

static int ssp_i2c_download_firmware(struct ssp_dev *dev, const struct firmware *fw)
{
	size_t pos = 0;
	size_t frame_size;
	int error_count = 0;
	int ret = 0;
	struct ssp_i2c_priv *priv = to_priv(dev);

	if (ssp_i2c_unlock_bootloader(priv)) {
		dev_err(dev->dev, "Failed to unlock bootloader\n");
		return -EIO;
	}

	while (pos < fw->size) {
		ret = ssp_i2c_check_bl_state(priv, BL_APP_STATE_WAITING_FRAME);
		if (ret) {
			error_count++;
			if (error_count > 10)
				break;
			dev_warn(dev->dev, "Bootloader in unexpected state!\n");
			continue;
		}

		frame_size = (fw->data[pos] << 8) | fw->data[pos + 1];
		/* and CRC bytes */
		frame_size += 2;

		ret = ssp_i2c_write_firmware_frame(priv, &fw->data[pos],
						   frame_size);
		if (!ret) {
			ret = ssp_i2c_check_bl_state(priv, BL_FRAME_CRC_PASS);
		}
		if (ret) {
			error_count++;
			if (error_count > 10)
				break;
			dev_warn(dev->dev, "Failed to write firmware frame\n");
			continue;
		}

		pos += frame_size;
		dev_info(dev->dev, "Sent %zd/%zd bytes of new firmware...\n",
				pos, fw->size);
		mdelay(1);
	}

	return ret;
}

static int ssp_i2c_leave_bootloader(struct ssp_dev *dev)
{
	struct ssp_i2c_priv *priv = to_priv(dev);
	u8 buf[] = {0, 0};
	int ret;

	gpiod_set_value(dev->gpio_mcu_nrst, 0);
	udelay(10);
	gpiod_set_value(dev->gpio_mcu_nrst, 1);
	msleep(50);

	ret = i2c_master_send(priv->i2c_boot, buf, 2);
	if (ret < 0)
		return ret;
	return 0;
}

static int ssp_i2c_read(struct ssp_i2c_priv *priv, u8 *tx, size_t tx_len,
		u8 *rx, size_t rx_len)
{
	int i;
	struct i2c_msg msgs[] = {
		{
			.addr = priv->i2c_main->addr,
			.flags = 0,
			.len = tx_len,
			.buf = tx,
		}, {
			.addr = priv->i2c_main->addr,
			.flags = I2C_M_RD,
			.len = rx_len,
			.buf = rx,
		}
	};
	int ret;

	for (i = 0; i < 3; i++) {
		ret = i2c_transfer(priv->i2c_main->adapter, msgs, 2);
		if (ret == 2)
			break;
	}

	if (ret == 2)
		ret = 0;
	else if (ret >= 0)
		dev_warn(priv->dev.dev, "only transmitted partial i2c message: %d\n", ret);
	else
		dev_err(priv->dev.dev, "i2c error: %d\n", ret);
	return ret;
}

static int ssp_i2c_get_firmware_rev(struct ssp_dev *dev)
{
	struct ssp_i2c_priv *priv = to_priv(dev);
	int ret;
	u8 rx[3];
	u8 tx = SSP_MSG_GET_FW_REV;
	int rev;

	if (ssp_wakeup(dev))
		return -ENODEV;

	ret = ssp_i2c_read(priv, &tx, 1, rx, sizeof(rx));
	if (ret) {
		return ret;
	}

	rev = (rx[0] << 16) | (rx[1] << 8) | rx[2];
	return rev;
}

static int ssp_i2c_set_sensor_pos(struct ssp_dev *dev, u32 *pos,
		size_t count)
{
	struct ssp_i2c_priv *priv = to_priv(dev);
	int i, ret;
	u8 rx;
	u8 *tx = kzalloc(count + 1, GFP_KERNEL);
	if (!tx)
		return -ENOMEM;

	tx[0] = SSP_MSG_SENSOR_FORMATION;
	for (i = 0; i < count; i++) {
		tx[i+1] = (u8)pos[i];
	}

	ret = ssp_i2c_read(priv, tx, count + 1, &rx, 1);
	if (ret)
		return ret;

	if (rx != MSG_SSP_ACK)
		return -EIO;
	return 0;
}

static const struct ssp_ops ssp_i2c_ops = {
	.is_in_bootloader = ssp_i2c_is_in_bootloader,
	.leave_bootloader = ssp_i2c_leave_bootloader,
	.download_firmware = ssp_i2c_download_firmware,

	.get_firmware_rev = ssp_i2c_get_firmware_rev,
	.set_sensor_pos = ssp_i2c_set_sensor_pos,
	.send_instruction = NULL,
};

static int ssp_i2c_probe(struct i2c_client *client)
{
	struct ssp_i2c_priv *priv = devm_kzalloc(&client->dev, sizeof(*priv),
						 GFP_KERNEL);
	struct ssp_dev_data *data;
	int ret;

	if (!priv)
		return -ENOMEM;

	dev_info(&client->dev, "Probing ssp_i2c driver...\n");

	i2c_set_clientdata(client, priv);

	priv->i2c_main = client;

	priv->i2c_boot = i2c_new_dummy(client->adapter, I2C_ADDR_BOOTLOADER);
	if (!priv->i2c_boot) {
		dev_err(&client->dev, "Failed to allocate boot client\n");
		return -ENODEV;
	}
	i2c_set_clientdata(priv->i2c_boot, priv);

	data = (struct ssp_dev_data *)of_device_get_match_data(&client->dev);

	ret = ssp_common_probe(&priv->dev, &client->dev, &ssp_i2c_ops, data);
	if (ret)
		goto err_free_boot_client;

	dev_info(&client->dev, "Probe OK\n");
	return 0;

err_free_boot_client:
	dev_info(&client->dev, "Probe not OK: %d\n", ret);
	i2c_unregister_device(priv->i2c_boot);
	return ret;
}

static int ssp_i2c_remove(struct i2c_client *client)
{
	struct ssp_i2c_priv *priv = i2c_get_clientdata(client);
	i2c_unregister_device(priv->i2c_boot);
	return 0;
}

static const struct ssp_dev_data ssp_data_n710x = {
	.rev = 92800,
	.fw_name = "ssp_n710x.fw",
	.sensor_pos_count = 4,
};

static const struct of_device_id ssp_match_table[] = {
	{
		.compatible = "samsung,n710x-ssp",
		.data = &ssp_data_n710x,
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ssp_match_table);

static struct i2c_driver ssp_i2c_driver = {
	.driver = {
		.name = "ssp-i2c",
		.of_match_table = of_match_ptr(ssp_match_table)
	},
	.probe_new = ssp_i2c_probe,
	.remove = ssp_i2c_remove,
};
module_i2c_driver(ssp_i2c_driver);

MODULE_LICENSE("GPL v2");
