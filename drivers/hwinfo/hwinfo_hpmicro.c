/*
 * Copyright 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>

#include "hpm_otp_drv.h"
#include "hpm_soc_feature.h"

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	uint8_t id[OTP_SOC_UUID_LEN];
	size_t copy_len;

	for (uint32_t i = 0; i < (OTP_SOC_UUID_LEN / sizeof(uint32_t)); i++) {
		uint32_t word = otp_read_from_shadow(OTP_SOC_UUID_IDX + i);

		memcpy(&id[i * sizeof(uint32_t)], &word, sizeof(word));
	}

	copy_len = MIN(length, sizeof(id));
	memcpy(buffer, id, copy_len);

	return (ssize_t)copy_len;
}
