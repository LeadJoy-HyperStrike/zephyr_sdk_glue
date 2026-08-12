/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 *
 * Scan I2C1 (BOARD_APP_I2C): PY03 SCL / PY02 SDA.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define I2C_NODE DT_ALIAS(i2c_0)

static const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);

int main(void)
{
	uint8_t found = 0;

	if (!device_is_ready(i2c_dev)) {
		printk("I2C1 not ready\n");
		return 0;
	}

	printk("I2C1 scan (PY03 SCL / PY02 SDA) @ 100kHz\n");

	while (1) {
		found = 0;
		printk("--- scan ---\n");
		for (uint8_t addr = 0x08; addr < 0x78; addr++) {
			struct i2c_msg msg = {
				.buf = NULL,
				.len = 0,
				.flags = I2C_MSG_WRITE | I2C_MSG_STOP,
			};
			int ret = i2c_transfer(i2c_dev, &msg, 1, addr);

			if (ret == 0) {
				printk("  found 0x%02x\n", addr);
				found++;
			}
		}
		if (!found) {
			printk("  (no ACK — empty bus is OK if nothing is wired)\n");
		} else {
			printk("  %u device(s)\n", found);
		}
		k_msleep(2000);
	}

	return 0;
}
