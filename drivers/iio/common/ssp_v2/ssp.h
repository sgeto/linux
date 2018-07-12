// SPDX-License-Identifier: GPL-2.0
//
// Copyright (C) 2018 Simon Shields <simon@lineageos.org>
//

#include <linux/device.h>
#include <linux/firmware.h>

#ifndef _SENSOR_SSP_H
#define _SENSOR_SSP_H
struct ssp_dev;
struct ssp_ops;
struct ssp_dev_data;

/* All sensors supported by the SSP hub */
enum ssp_sensor {
	ACCELEROMETER_SENSOR = 0,
	GYROSCOPE_SENSOR,
	GEOMAGNETIC_SENSOR,
	PRESSURE_SENSOR,
	GESTURE_SENSOR,
	PROXIMITY_SENSOR,
	LIGHT_SENSOR,
	PROXIMITY_RAW_SENSOR,
	ORIENTATION_SENSOR,
	SENSOR_MAX,
};

/**
 * struct ssp_ops: bus-specific SSP operations
 * @send_instruction: send instruction operating on a sensor with some amount of data
 */
struct ssp_ops {
	bool (*is_in_bootloader)(struct ssp_dev *dev);
	int (*leave_bootloader)(struct ssp_dev *dev);
	int (*download_firmware)(struct ssp_dev *dev, const struct firmware *fw);

	int (*get_firmware_rev)(struct ssp_dev *dev);
	int (*set_sensor_pos)(struct ssp_dev *dev, u32 *pos, size_t count);
	int (*send_instruction)(struct ssp_dev *dev, u8 instruction, u8 sensor, u8 *buf, u8 buflen);
};

#define SSP_STATE_SHUTDOWN (SENSOR_MAX)

/**
 * struct ssp_dev: data for a SSP device
 * @dev: struct device associated with this ssp
 * @ops: bus-specific ops (i.e. i2c, spi, ...)
 * @data: board-specific ssp data (e.g. expected firmware version)
 * @gpio_mcu_nrst: MCU_NRST gpio
 * @gpio_mcu_int: AP to MCU interrupt GPIO
 * @gpio_mcu_busy: MCU to AP interrupt 1, merely a "busy" indicator
 * @gpio_mcu_ready: MCU to AP interrupt 2, a "ready" indicator
 * @irq: MCU to AP interrupt 1
 * @state: MCU state
 */
struct ssp_dev {
	struct device *dev;
	const struct ssp_ops *ops;
	const struct ssp_dev_data *data;
	struct gpio_desc *gpio_mcu_nrst;
	struct gpio_desc *gpio_mcu_int;
	struct gpio_desc *gpio_mcu_ready;
	struct gpio_desc *gpio_mcu_busy;
	int irq;
	unsigned long state;
	u32 *sensor_pos;
};

/**
 * struct ssp_dev_data: data for specific sensorhub variant
 */
struct ssp_dev_data {
	unsigned int rev;
	const char *fw_name;
	int sensor_pos_count;
};

int ssp_common_probe(struct ssp_dev *dev, struct device *device,
		const struct ssp_ops *ops, const struct ssp_dev_data *data);
void ssp_enter_bootloader(struct ssp_dev *dev);
int ssp_wakeup(struct ssp_dev *dev);
#endif
