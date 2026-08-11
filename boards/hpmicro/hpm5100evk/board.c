/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <hpm_common.h>
#include <hpm_soc.h>
#include "hpm_clock_drv.h"

void sys_arch_reboot(int type)
{
	ARG_UNUSED(type);

	HPM_PPOR->RESET_ENABLE = (1UL << 31);
	HPM_PPOR->RESET_TYPE |= (1UL << 31);

	HPM_PPOR->SOFTWARE_RESET = 1000U;
	while (1) {
	}
}
