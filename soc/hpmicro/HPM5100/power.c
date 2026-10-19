/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * HPM5151 low-power support:
 * - Wait mode for runtime idle / arch_cpu_idle()
 * - Stop mode for suspend-to-idle
 * - Shutdown for soft-off
 */

#include <zephyr/arch/riscv/arch.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/irq_multilevel.h>
#include <zephyr/pm/pm.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/tracing/tracing.h>

#include <hpm_soc.h>
#include <hpm_sysctl_drv.h>
#include <hpm_pdgo_drv.h>
#include <hpm_pdgo_regs.h>
#include <hpm_pcfg_drv.h>

#define HPM_PM_STOP_RETENTION 0x0FUL
#define HPM_PM_POWER_DOWN_COUNT 0x100U
#define HPM_PM_WKUP_IRQ0_BIT dgo_irq0_wkup_pin
#define HPM_PM_WKUP_STATUS_BIT dgo_wakeup_wkup_pin

#define HPM_PM_WKUP_IRQ \
	(IRQ_TO_L2(IRQn_DGO_PAD_WAKEUP) | DT_IRQN(DT_NODELABEL(plic)))

#define HPM_PM_PUART_NODE DT_NODELABEL(puart)
#define HPM_PM_HAS_PUART_WAKE \
	(DT_NODE_HAS_STATUS(HPM_PM_PUART_NODE, okay) && \
	 DT_NODE_HAS_PROP(HPM_PM_PUART_NODE, wakeup_source))

#define HPM_PM_PTMR_NODE DT_NODELABEL(ptmr)
#define HPM_PM_HAS_PTMR_WAKE \
	(DT_NODE_HAS_STATUS(HPM_PM_PTMR_NODE, okay) && \
	 DT_NODE_HAS_PROP(HPM_PM_PTMR_NODE, wakeup_source))

static volatile bool hpm_pm_wkup_pin_woke;
static uint32_t hpm_pm_pcfg_wake_mask;

__weak void hpm_board_pm_prepare_wakeup(enum pm_state state)
{
	ARG_UNUSED(state);
}

static void hpm5100_wkup_isr(const void *arg)
{
	ARG_UNUSED(arg);

	if ((HPM_PDGO->IRQ0_STS & HPM_PM_WKUP_IRQ0_BIT) != 0U) {
		HPM_PDGO->IRQ0_STS = HPM_PM_WKUP_IRQ0_BIT;
		hpm_pm_wkup_pin_woke = true;
	}

	if ((HPM_PDGO->WKUP_STS & HPM_PM_WKUP_STATUS_BIT) != 0U) {
		HPM_PDGO->WKUP_STS = HPM_PM_WKUP_STATUS_BIT;
	}
}

static int hpm5100_pm_wkup_init(void)
{
	IRQ_CONNECT(HPM_PM_WKUP_IRQ, 2, hpm5100_wkup_isr, NULL, 0);
	irq_disable(HPM_PM_WKUP_IRQ);

	return 0;
}

SYS_INIT(hpm5100_pm_wkup_init, POST_KERNEL, 50);

static void hpm5100_arm_wkup_pin(void)
{
	hpm_pm_wkup_pin_woke = false;

	/* Internal pulldown, edge latch, then clear stale wake indications. */
	HPM_PDGO->PIN_CTRL &= ~PDGO_PIN_CTRL_WKUP_MASK;
	HPM_PDGO->IRQ0_EN |= HPM_PM_WKUP_IRQ0_BIT;
	HPM_PDGO->IRQ0_STS = HPM_PM_WKUP_IRQ0_BIT;
	HPM_PDGO->WKUP_STS = HPM_PM_WKUP_STATUS_BIT;
	irq_enable(HPM_PM_WKUP_IRQ);
}

static void hpm5100_disarm_wakeup_sources(enum pm_state state)
{
	irq_disable(HPM_PM_WKUP_IRQ);

	if (state == PM_STATE_SUSPEND_TO_IDLE) {
#if HPM_PM_HAS_PUART_WAKE
		/*
		 * The SDK disable-with-IRQ helper uses an incorrect >>2 index
		 * on HPM5151. Use the register-array index explicitly.
		 */
		sysctl_disable_cpu0_wakeup_source_with_mask(
			HPM_SYSCTL, IRQn_PUART / 32U,
			BIT(IRQn_PUART % 32U));
#endif
#if HPM_PM_HAS_PTMR_WAKE
		sysctl_disable_cpu0_wakeup_source_with_mask(
			HPM_SYSCTL, IRQn_PTMR / 32U,
			BIT(IRQn_PTMR % 32U));
#endif
		HPM_PCFG->WAKE_MASK = hpm_pm_pcfg_wake_mask;
	}
}

static inline void hpm5100_enter_wait(void)
{
	sysctl_set_cpu0_lp_mode(HPM_SYSCTL, cpu_lp_mode_gate_cpu_clock);
	/*
	 * pm_system_suspend() runs with interrupts masked; WFI only wakes on
	 * IRQ when MIE is set (same pattern as other Zephyr SoC PM backends).
	 */
	irq_unlock(MSTATUS_IEN);
	__asm__ volatile("wfi");
}

static inline void hpm5100_enter_stop(void)
{
	hpm_pm_pcfg_wake_mask = HPM_PCFG->WAKE_MASK;
	sysctl_enable_cpu0_wakeup_source_with_irq(HPM_SYSCTL, IRQn_DGO_PAD_WAKEUP);
#if HPM_PM_HAS_PUART_WAKE
	pcfg_enable_wakeup_source(HPM_PCFG, pcfg_wakeup_src_puart);
	sysctl_enable_cpu0_wakeup_source_with_irq(HPM_SYSCTL, IRQn_PUART);
#endif
#if HPM_PM_HAS_PTMR_WAKE
	pcfg_enable_wakeup_source(HPM_PCFG, pcfg_wakeup_src_ptimer);
	sysctl_enable_cpu0_wakeup_source_with_irq(HPM_SYSCTL, IRQn_PTMR);
#endif
	sysctl_set_cpu0_lp_retention(HPM_SYSCTL, HPM_PM_STOP_RETENTION);
	sysctl_set_cpu0_lp_mode(HPM_SYSCTL, cpu_lp_mode_trigger_system_lp);
	irq_unlock(MSTATUS_IEN);
	__asm__ volatile("wfi");
}

static inline void hpm5100_exit_low_power(void)
{
	sysctl_set_cpu0_lp_mode(HPM_SYSCTL, cpu_lp_mode_ungate_cpu_clock);
}

static inline void hpm5100_enter_soft_off(void)
{
	pdgo_enable_wkup_pin_wakeup(HPM_PDGO);
	pdgo_set_turnoff_counter(HPM_PDGO, HPM_PM_POWER_DOWN_COUNT);

	while (true) {
		;
	}
}

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	switch (state) {
	case PM_STATE_RUNTIME_IDLE:
		hpm_board_pm_prepare_wakeup(state);
		hpm5100_arm_wkup_pin();
		hpm5100_enter_wait();
		break;
	case PM_STATE_SUSPEND_TO_IDLE:
		hpm_board_pm_prepare_wakeup(state);
		hpm5100_arm_wkup_pin();
		hpm5100_enter_stop();
		break;
	case PM_STATE_SOFT_OFF:
		hpm_board_pm_prepare_wakeup(state);
		hpm5100_enter_soft_off();
		break;
	default:
		break;
	}
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	switch (state) {
	case PM_STATE_RUNTIME_IDLE:
	case PM_STATE_SUSPEND_TO_IDLE:
		hpm5100_exit_low_power();
		hpm5100_disarm_wakeup_sources(state);
		break;
	case PM_STATE_SOFT_OFF:
	default:
		break;
	}

	irq_unlock(MSTATUS_IEN);
}

bool hpm_pm_wakeup_from_wkup_pin(void)
{
	return hpm_pm_wkup_pin_woke;
}

FUNC_NORETURN void z_sys_poweroff(void)
{
	pm_state_set(PM_STATE_SOFT_OFF, 1U);

	CODE_UNREACHABLE;
}

#if !defined(CONFIG_PM)
void arch_cpu_idle(void)
{
	sys_trace_idle();
	hpm5100_enter_wait();
	hpm5100_exit_low_power();
	irq_unlock(MSTATUS_IEN);
}

void arch_cpu_atomic_idle(unsigned int key)
{
	sys_trace_idle();
	hpm5100_enter_wait();
	hpm5100_exit_low_power();
	irq_unlock(key);
}
#endif
