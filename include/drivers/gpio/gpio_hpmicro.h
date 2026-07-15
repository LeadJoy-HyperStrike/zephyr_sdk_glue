/*
 * Copyright (c) 2026 HPMicro / HyperStrike
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_HPMICRO_H_
#define ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_HPMICRO_H_

#include <stdint.h>
#include <zephyr/device.h>

/**
 * @brief System address of the port's raw input value register.
 *
 * Returns &GPIO->DI[port].VALUE for the port behind @p dev. Used by the HS2
 * CPU1 engine to sample button levels with a bare MMIO read from the second
 * hart - the register is read-only and side-effect free, so cross-hart
 * concurrent reads are unrestricted.
 */
uint32_t gpio_hpmicro_di_addr(const struct device *dev);

#endif /* ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_HPMICRO_H_ */
