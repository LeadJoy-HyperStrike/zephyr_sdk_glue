/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/pm/state.h>
#include <zephyr/sys/util.h>
#include <hpm_common.h>
#include <hpm_iomux.h>
#include <hpm_pdgo_drv.h>
#include <hpm_soc.h>
#include "hpm_clock_drv.h"

void hpm_board_pm_prepare_wakeup(enum pm_state state)
{
	if ((state != PM_STATE_RUNTIME_IDLE) &&
	    (state != PM_STATE_SUSPEND_TO_IDLE) &&
	    (state != PM_STATE_SOFT_OFF)) {
		return;
	}

	/*
	 * Route PY07 as GPIO into the always-on PIOC/PDGO WKUP path.
	 *
	 * Do not use IOC_PY07_FUNC_CTL_UART2_TXD here. UART TX idles high,
	 * which holds PDGO.PIN_STS.WKUP active and makes IRQ0_STS[16]
	 * immediately reassert after a W1C clear.
	 */
	HPM_IOC->PAD[IOC_PAD_PY07].FUNC_CTL = IOC_PY07_FUNC_CTL_GPIO_Y_07;
	HPM_PIOC->PAD[IOC_PAD_PY07].FUNC_CTL = PIOC_PY07_FUNC_CTL_SOC_GPIO_Y_07;
	pdgo_enable_pulldown_resistor_for_wakeup_pin(HPM_PDGO);
}

static int hpm5100evk_wkup_pin_init(void)
{
	hpm_board_pm_prepare_wakeup(PM_STATE_RUNTIME_IDLE);

	return 0;
}

SYS_INIT(hpm5100evk_wkup_pin_init, POST_KERNEL, 50);

void sys_arch_reboot(int type)
{
	ARG_UNUSED(type);

	HPM_PPOR->RESET_ENABLE = (1UL << 31);
	HPM_PPOR->RESET_TYPE |= (1UL << 31);

	HPM_PPOR->SOFTWARE_RESET = 1000U;
	while (1) {
	}
}
