/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 *
 * PMIC-domain PTMR (GPTMRv2) as the Zephyr system timer.
 *
 * Andes mtimer is in the CPU domain and does not run in Stop with the
 * HPM5100 retention mask. PTMR stays in the PMIC domain, so kernel
 * timeouts can wake Stop.
 *
 * The counter is 24-bit. A 24 MHz source with prescaler 24 yields a 1 MHz
 * tick (~16.7 s max timeout), matching CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC.
 */

#include <errno.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys_clock.h>

#include "hpm_clock_drv.h"
#include "hpm_gptmrv2_drv.h"

#define PTMR_NODE DT_NODELABEL(ptmr)

BUILD_ASSERT(DT_NODE_HAS_STATUS(PTMR_NODE, okay),
	     "HPM PTMR sysclock requires &ptmr status okay");
BUILD_ASSERT(DT_PROP(PTMR_NODE, sys_clock),
	     "HPM PTMR sysclock requires &ptmr { sys-clock; }");

#define PTMR_BASE ((GPTMRV2_Type *)DT_REG_ADDR(PTMR_NODE))
#define PTMR_CHANNEL DT_PROP(PTMR_NODE, channel)
#define PTMR_CLOCK ((clock_name_t)DT_PROP(PTMR_NODE, clk_name))
#define PTMR_IRQN DT_IRQN(PTMR_NODE)
#define PTMR_IRQ_PRIO DT_IRQ(PTMR_NODE, priority)
#define PTMR_RLD_MASK GPTMR_CH_RLD_STAT_MASK(PTMR_CHANNEL)

#define CYC_PER_TICK ((uint32_t)(sys_clock_hw_cycles_per_sec() / \
				 CONFIG_SYS_CLOCK_TICKS_PER_SEC))
#define MAX_CYCLES 0xFFFFFFU
#define MAX_TICKS ((int32_t)(MAX_CYCLES / CYC_PER_TICK))

static struct k_spinlock lock;
static uint64_t cycle_offset;
static uint64_t last_count;
static uint64_t last_ticks;
static uint32_t last_elapsed;
static uint32_t period;

static inline uint32_t ptmr_counter(void)
{
	return gptmr_channel_get_counter(PTMR_BASE, PTMR_CHANNEL,
					 gptmr_counter_type_normal);
}

static inline uint64_t ptmr_now(void)
{
	return cycle_offset + ptmr_counter();
}

static void ptmr_program(uint32_t cycles)
{
	uint32_t progress;

	if (cycles < CYC_PER_TICK) {
		cycles = CYC_PER_TICK;
	} else if (cycles > (uint32_t)MAX_TICKS * CYC_PER_TICK) {
		cycles = (uint32_t)MAX_TICKS * CYC_PER_TICK;
	} else {
		cycles = (cycles / CYC_PER_TICK) * CYC_PER_TICK;
	}

	gptmr_stop_counter(PTMR_BASE, PTMR_CHANNEL);

	if (gptmr_check_status(PTMR_BASE, PTMR_RLD_MASK)) {
		gptmr_clear_status(PTMR_BASE, PTMR_RLD_MASK);
		cycle_offset += period;
	} else {
		progress = ptmr_counter();
		cycle_offset += progress;
	}

	period = cycles;
	gptmr_channel_reset_count(PTMR_BASE, PTMR_CHANNEL);
	gptmr_channel_config_update_reload(PTMR_BASE, PTMR_CHANNEL, period);
	gptmr_clear_status(PTMR_BASE, PTMR_RLD_MASK);
	gptmr_start_counter(PTMR_BASE, PTMR_CHANNEL);
}

__attribute__((section(".isr")))
static void ptmr_isr(const void *arg)
{
	uint64_t now;
	uint32_t dticks;

	ARG_UNUSED(arg);

	k_spinlock_key_t key = k_spin_lock(&lock);

	gptmr_clear_status(PTMR_BASE, PTMR_RLD_MASK);
	cycle_offset += period;
	now = cycle_offset;

	dticks = (uint32_t)((now - last_count) / CYC_PER_TICK);
	last_count += (uint64_t)dticks * CYC_PER_TICK;
	last_ticks += dticks;
	last_elapsed = 0;

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		period = CYC_PER_TICK;
		gptmr_channel_config_update_reload(PTMR_BASE, PTMR_CHANNEL,
						   period);
	}

	k_spin_unlock(&lock, key);
	sys_clock_announce(dticks);
}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	uint64_t now;
	uint64_t target;
	uint32_t delay;

	ARG_UNUSED(idle);

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);

	now = ptmr_now();

	if (ticks == K_TICKS_FOREVER) {
		delay = (uint32_t)MAX_TICKS * CYC_PER_TICK;
	} else {
		target = (last_ticks + last_elapsed + (uint64_t)ticks) *
			 CYC_PER_TICK;
		if (target <= now) {
			delay = CYC_PER_TICK;
		} else if ((target - now) > ((uint64_t)MAX_TICKS * CYC_PER_TICK)) {
			delay = (uint32_t)MAX_TICKS * CYC_PER_TICK;
		} else {
			delay = (uint32_t)(target - now);
		}
	}

	ptmr_program(delay);
	k_spin_unlock(&lock, key);
}

uint32_t sys_clock_elapsed(void)
{
	uint32_t dticks;

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);
	uint64_t now = ptmr_now();

	dticks = (uint32_t)((now - last_count) / CYC_PER_TICK);
	last_elapsed = dticks;
	k_spin_unlock(&lock, key);

	return dticks;
}

uint32_t sys_clock_cycle_get_32(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	uint32_t cycles = (uint32_t)ptmr_now();

	k_spin_unlock(&lock, key);
	return cycles;
}

uint64_t sys_clock_cycle_get_64(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	uint64_t cycles = ptmr_now();

	k_spin_unlock(&lock, key);
	return cycles;
}

static int sys_clock_driver_init(void)
{
	gptmr_channel_config_t ch_cfg;
	uint32_t src_freq;
	uint32_t prescaler;

	clock_add_to_group(PTMR_CLOCK, 0);
	src_freq = clock_get_frequency(PTMR_CLOCK);
	if (src_freq == 0U) {
		return -EIO;
	}

	prescaler = src_freq / sys_clock_hw_cycles_per_sec();
	if ((prescaler < 1U) ||
	    ((src_freq / prescaler) != sys_clock_hw_cycles_per_sec())) {
		return -EINVAL;
	}

	gptmr_channel_get_default_config(PTMR_BASE, &ch_cfg);
	ch_cfg.prescaler = (uint16_t)prescaler;
	ch_cfg.reload = CYC_PER_TICK;
	if (gptmr_channel_config(PTMR_BASE, PTMR_CHANNEL, &ch_cfg, false) !=
	    status_success) {
		return -EIO;
	}

	period = CYC_PER_TICK;
	cycle_offset = 0;
	last_count = 0;
	last_ticks = 0;
	last_elapsed = 0;

	IRQ_CONNECT(PTMR_IRQN, PTMR_IRQ_PRIO, ptmr_isr, NULL, 0);
	gptmr_enable_irq(PTMR_BASE, GPTMR_CH_RLD_IRQ_MASK(PTMR_CHANNEL));
	gptmr_start_counter(PTMR_BASE, PTMR_CHANNEL);
	irq_enable(PTMR_IRQN);

	return 0;
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2,
	 CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
