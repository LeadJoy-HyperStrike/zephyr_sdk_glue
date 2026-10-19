/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef HPM5100_LP_H_
#define HPM5100_LP_H_

void prepare_soc_low_power(void);
int enter_wait_mode(void);
int enter_stop_mode(void);
int enter_stop_timer_mode(void);
void enter_standby_mode(void);
void enter_shutdown_mode(void);

#endif /* HPM5100_LP_H_ */
