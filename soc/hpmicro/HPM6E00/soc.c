/*
 * Copyright (c) 2022-2025 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <hpm_common.h>
#include <hpm_soc.h>
#include "hpm_clock_drv.h"
#include "hpm_pllctlv2_drv.h"
#include "hpm_pcfg_drv.h"
#ifdef CONFIG_NOCACHE_MEMORY
#include <zephyr/linker/linker-defs.h>
#include "hpm_pmp_drv.h"
#endif
#ifdef CONFIG_XIP
#include "hpm_bootheader.h"
#endif

#ifdef CONFIG_XIP
__attribute__((section(".nor_cfg_option"), used)) const uint32_t option[4] = { 0xfcf90001, 0x00000007, 0x0, 0x0 };
__attribute__((section(".last_section"))) const uint32_t rom_marker = CONFIG_LINKER_LAST_SECTION_ID_PATTERN;
#endif
__attribute__((weak)) void c_startup(void)
{
}

static void soc_init_clock(void)
{
    uint32_t cpu0_freq = clock_get_frequency(clock_cpu0);
    if (cpu0_freq == PLLCTL_SOC_PLL_REFCLK_FREQ) {
        /* Configure the External OSC ramp-up time: ~9ms */
        pllctlv2_xtal_set_rampup_time(HPM_PLLCTLV2, 32ul * 1000ul * 9u);

        /* select clock setting preset1 */
        sysctl_clock_set_preset(HPM_SYSCTL, 2);
    }
    /* Add Clocks to group 0 */
    clock_add_to_group(clock_cpu0, 0);
    clock_add_to_group(clock_mchtmr0, 0);
    clock_add_to_group(clock_ahb0, 0);
    clock_add_to_group(clock_axif, 0);
    clock_add_to_group(clock_axis, 0);
    clock_add_to_group(clock_axic, 0);
    clock_add_to_group(clock_axin, 0);
    clock_add_to_group(clock_rom0, 0);
    clock_add_to_group(clock_xpi0, 0);
    clock_add_to_group(clock_lmm0, 0);
    clock_add_to_group(clock_lmm1, 0);
    clock_add_to_group(clock_ram0, 0);
    clock_add_to_group(clock_ram1, 0);
    clock_add_to_group(clock_hdma, 0);
    clock_add_to_group(clock_xdma, 0);
    clock_add_to_group(clock_gpio, 0);
    /* Motor Related */
    clock_add_to_group(clock_pwm0, 0);
    /*
     * The motor-control island (QEI/QEO/RDC/MTG/VSC/CLC/PLB/SEI/EMDS), PWM1-3
     * and PTPC have no enabled devicetree node on the boards this fork carries.
     *
     * These MUST be driven with clock_remove_from_group(): the group registers
     * are set-only from software's point of view (clock_add_to_group() ends in
     * sysctl_enable_group_resource(..., true), which ORs one bit in), and
     * simply not calling add does NOT gate anything. UM V0.8 chapter 14 is
     * explicit that the documented all-zero reset value of GROUP0[VALUE] is
     * not what software observes: "本产品 ROM 的启动代码会配置本模块，因此软件
     * 读取到的寄存器值与本章节描述的复位值有可能不一致". Dropping the add calls
     * measured as no power change at all, which is consistent with the ROM
     * leaving these bits set.
     *
     * PWM0 is deliberately outside this list: it is the ADC TRGO source (see
     * trigger-pwm in the board overlays) and must stay clocked.
     */
    static const clock_name_t unused_clocks[] = {
        clock_ptpc,
        clock_qei0, clock_qei1, clock_qei2, clock_qei3,
        clock_qeo0, clock_qeo1, clock_qeo2, clock_qeo3,
        clock_pwm1, clock_pwm2, clock_pwm3,
        clock_rdc0, clock_rdc1,
        clock_plb0, clock_sei0,
        clock_mtg0, clock_mtg1,
        clock_vsc0, clock_vsc1,
        clock_clc0, clock_clc1,
        clock_emds,
    };

    for (uint32_t i = 0; i < ARRAY_SIZE(unused_clocks); i++) {
#if defined(CONFIG_HPM_SOC_GATE_UNUSED_CLOCKS)
        clock_remove_from_group(unused_clocks[i], 0);
#else
        clock_add_to_group(unused_clocks[i], 0);
#endif
    }
    /* Connect Group0 to CPU0 */
    clock_connect_group_to_cpu(0, 0);

    /* Add the CPU1 clock to Group1 */
    clock_add_to_group(clock_cpu1, 1);
    clock_add_to_group(clock_mchtmr1, 1);
    /* Connect Group1 to CPU1 */
    clock_connect_group_to_cpu(1, 1);

    /* Bump up DCDC voltage to 1275mv */
    pcfg_dcdc_set_voltage(HPM_PCFG, 1275);

    /* Set CPU clock to 600MHz */
    clock_set_source_divider(clock_cpu0, clk_src_pll0_clk0, 1);
    clock_set_source_divider(clock_cpu1, clk_src_pll0_clk0, 1);

    /* Configure mchtmr to 24MHz */
    clock_set_source_divider(clock_mchtmr0, clk_src_osc24m, 1);
    clock_set_source_divider(clock_mchtmr1, clk_src_osc24m, 1);
}

#ifdef CONFIG_NOCACHE_MEMORY
static void soc_init_pma(void)
{
    volatile uint32_t start_addr = (uint32_t) &_nocache_ram_start;
    volatile uint32_t length = (uint32_t) &_nocache_ram_size;

    if (length == 0) {
        return;
    }

    /* Ensure the address and the length are power of 2 aligned */
    assert((length & (length - 1U)) == 0U);
    assert((start_addr & (length - 1U)) == 0U);

    pma_attr_t pma_attrs[1] = { 0 };
    pma_attrs[0].pma_addr = PMA_NAPOT_ADDR(start_addr, length);
    pma_attrs[0].pma_cfg.val = PMA_CFG(ADDR_MATCH_NAPOT, MEM_TYPE_MEM_NON_CACHE_BUF, AMO_EN);
    pma_config_attributes(&pma_attrs[0], ARRAY_SIZE(pma_attrs));
}
#endif

static int hpmicro_soc_init(void)
{
	uint32_t key;

	key = irq_lock();
	soc_init_clock();
#ifdef CONFIG_NOCACHE_MEMORY
	soc_init_pma();
#endif
	irq_unlock(key);

	return 0;
}

SYS_INIT(hpmicro_soc_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
