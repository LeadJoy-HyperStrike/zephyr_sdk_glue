/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BOARD_HPM5100EVK_H_
#define BOARD_HPM5100EVK_H_

#include <zephyr/pm/state.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Board hook consumed by the HPM5100 SoC PM backend before entering a
 * low-power state. On HPM5100EVK it routes PY07 through PIOC so PDGO can
 * observe the WKUP key even when UART2 stays disabled.
 */
void hpm_board_pm_prepare_wakeup(enum pm_state state);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_HPM5100EVK_H_ */
