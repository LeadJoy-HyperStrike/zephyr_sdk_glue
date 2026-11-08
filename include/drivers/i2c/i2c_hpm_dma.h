/*
 * Copyright (c) 2026 HeliumStrike
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SDK_GLUE_INCLUDE_DRIVERS_I2C_I2C_HPM_DMA_H_
#define SDK_GLUE_INCLUDE_DRIVERS_I2C_I2C_HPM_DMA_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

/*
 * What the HDMA path of an hpmicro I2C instance has done since boot
 * (CONFIG_I2C_HPMICRO_DMA; drivers/i2c/i2c_hpmicro.c). For a bench readout:
 * whether the transfers really moved on the HDMA, and whether the path
 * gave up.
 *
 *  xfers  transfers the HDMA path completed (success)
 *  nacks  address NACKs on the HDMA path (-ENXIO, normal while probing)
 *  fails  HDMA-path failures (-EIO / -ETIMEDOUT); the first one turns the
 *         path off for the instance and the transfer is retried on the
 *         interrupt path, so this stays 0 or 1
 *  off    the instance has given up the HDMA path (every transfer since on
 *         the interrupt path, as with the option off)
 */
struct i2c_hpm_dma_stats {
	uint32_t xfers;
	uint32_t nacks;
	uint32_t fails;
	bool off;
};

/*
 * Linked only with CONFIG_I2C_HPMICRO_DMA, for an hpmicro I2C instance.
 * 0 and *out filled; -ENOTSUP when the instance has no "txrx" dmas entry.
 */
int i2c_hpm_dma_get_stats(const struct device *dev, struct i2c_hpm_dma_stats *out);

#endif /* SDK_GLUE_INCLUDE_DRIVERS_I2C_I2C_HPM_DMA_H_ */
