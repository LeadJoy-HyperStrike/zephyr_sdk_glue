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

    /*
     * VDD_SOC follows the core clock. Datasheet V0.11 table 12 (正常工作条件)
     * gives the sanctioned operating points, and note 2 says to use the
     * typical column:
     *
     *   性能模式  <=600 MHz   1.25 / 1.275 / 1.30 V
     *   平衡模式  <=480 MHz   1.15 / 1.175 / 1.30 V
     *   节能模式  <=400 MHz   1.05 / 1.075 / 1.30 V
     *
     * The stock code pinned 1275 mV unconditionally, so running the part at
     * 480 MHz still burned the 600 MHz voltage. Dynamic power goes with V^2,
     * so this is free once the frequency is already down. Raising the voltage
     * above a bin buys nothing and note 1 warns it shortens device life.
     *
     * ORDERING RULE, and it is directional: raising the frequency needs the
     * voltage up FIRST, lowering it needs the voltage down LAST. Neither
     * single placement is safe on its own, because this function runs from
     * two different entry states -- cold boot from the BootROM (CPU at 24 MHz
     * or the preset value, DCDC at its reset default) and warm chainload from
     * MCUboot (CPU already at whatever MCUboot set). An earlier version set
     * the target voltage here, before the PLL retune, which clocked the cores
     * at MCUboot's 600 MHz on the 400 MHz bin's 1.075 V -- outside every row
     * of table 12.
     *
     * So: park at the 600 MHz voltage across the transition (safe for any
     * frequency this SoC supports), then settle to the target bin once the
     * PLL is already there. This is the same pre-raise the HPM SDK does --
     * boards/hpm6e00evk/board.c board_init_clock() calls
     * pcfg_dcdc_set_voltage(HPM_PCFG, 1275) immediately before
     * init_board_clock_source() -- with the step-down added on the far side.
     */
    pcfg_dcdc_set_voltage(HPM_PCFG, 1275);

#if CONFIG_HPM_SOC_CPU_FREQ_MHZ != 600
    /*
     * Divide PLL0CLK0 down with the post-divider. The VCO -- MFI/MFN/MFD --
     * is left exactly as sysctl_clock_set_preset(HPM_SYSCTL, 2) established
     * it, and only CLK_TOP_CPU0 / CLK_TOP_CPU1 hang off PLL0CLK0 (UM V0.8
     * default clock source table), so nothing outside the two cores moves.
     *
     * An earlier version called pllctlv2_init_pll_with_freq() to move the VCO
     * (800 MHz for a 400 MHz core, 960 MHz for 480) and that BROKE UART0 --
     * measured, not theorised: with the VCO retuned, both MCUboot and the app
     * transmitted at roughly 1/30 of the programmed 115200 baud, on a UART
     * that is muxed to PLL1CLK0 and therefore should not have been affected
     * at all. The frequency getters read live registers with no caching
     * (soc/HPM6E00/HPM6E80/hpm_clock_drv.c: get_frequency_for_ip_in_common_
     * group -> get_frequency_for_source -> pllctlv2_get_pll_postdiv_freq_in_hz),
     * so software and hardware can only disagree if the PLL stops matching
     * its own MFI/MFN. Root cause is not established; what IS established is
     * that HPM's own board support never touches MFI/MFN on this part --
     * boards/hpm6e00evk/clock.c init_board_clock_source() only points the
     * cores at clk_src_pll0_clk0 and lets the preset own every PLL. Staying
     * inside the post-divider keeps us on that supported path.
     *
     * Post-divider index n divides by (1.0 + 0.2n) -- see pllctlv2_div_t and
     * pllctlv2_get_pll_postdiv_freq_in_hz(), which computes
     * pll_freq / (100 + n*20) * 100. From the preset's 600 MHz VCO that
     * yields 500 / 428.6 / 375 / 333.3 / 300 MHz for n = 1..5.
     *
     * The index is picked at runtime from the VCO the preset actually left,
     * rather than hardcoded, because the achievable set moves with the VCO
     * and a stale table would silently pick the wrong divider. Selection
     * rounds the core frequency DOWN to the first step at or below the
     * request, so CONFIG_HPM_SOC_CPU_FREQ_MHZ reads as a ceiling: 400 lands
     * on 375, 480 lands on 428.6. Rounding down also keeps the voltage bin
     * chosen above valid by construction -- it was selected for the request,
     * and the part ends up slower than that.
     */
    {
        uint64_t vco = pllctlv2_get_pll_freq_in_hz(HPM_PLLCTLV2, PLLCTLV2_PLL_PLL0);
        uint64_t want = (uint64_t)CONFIG_HPM_SOC_CPU_FREQ_MHZ * 1000000ULL;
        uint32_t idx = 0U;

        /* vco / (1 + idx/5) > want  <=>  vco * 5 > want * (5 + idx) */
        while ((idx < (uint32_t)pllctlv2_div_13p6) &&
               ((vco * 5ULL) > (want * (5ULL + idx)))) {
            idx++;
        }
        pllctlv2_set_postdiv(HPM_PLLCTLV2, PLLCTLV2_PLL_PLL0, pllctlv2_clk0,
                             (pllctlv2_div_t)idx);
    }
#endif

    /* Set CPU clock to CONFIG_HPM_SOC_CPU_FREQ_MHZ (default 600 MHz) */
    clock_set_source_divider(clock_cpu0, clk_src_pll0_clk0, 1);
    clock_set_source_divider(clock_cpu1, clk_src_pll0_clk0, 1);

    /* Far side of the ordering rule above: the cores are on the target
     * frequency now, so the voltage may come down to its bin. No-op when the
     * target bin is already 1275 mV.
     */
#if CONFIG_HPM_SOC_CPU_FREQ_MHZ <= 400
    pcfg_dcdc_set_voltage(HPM_PCFG, 1075);
#elif CONFIG_HPM_SOC_CPU_FREQ_MHZ <= 480
    pcfg_dcdc_set_voltage(HPM_PCFG, 1175);
#endif

    /*
     * hpm_core_clock -- which backs clock_cpu_delay_us/_ms -- needs no
     * explicit refresh here: sysctl_config_clock() calls
     * clock_update_core_clock() itself whenever the node being configured is
     * clock_node_cpu0 or clock_node_cpu1, and the two calls above go through
     * it after the post-divider has already moved.
     */

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
