/*
 * Copyright (c) 2026 HPMicro
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
#ifdef CONFIG_XIP
#include "hpm_bootheader.h"
#endif

#ifdef CONFIG_XIP
/*
 * Temporary XIP option values for HPM5151 early bring-up.
 * Replace with final board flash parameters once HPM5151 SDK resources land.
 */
__attribute__ ((section(".nor_cfg_option"))) const uint32_t option[4] = {0xfcf90002, 0x00000005, 0x1000, 0x0};
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
		pllctlv2_xtal_set_rampup_time(HPM_PLLCTLV2, 32UL * 1000UL * 9U);

		/* Select clock setting preset1 */
		sysctl_clock_set_preset(HPM_SYSCTL, 2);
	}

	/* Minimal bring-up clocks: keep aligned with HPM53 single-core setup for now. */
	clock_add_to_group(clock_cpu0, 0);
	clock_add_to_group(clock_ahb, 0);
	clock_add_to_group(clock_lmm0, 0);
	clock_add_to_group(clock_mchtmr0, 0);
	clock_add_to_group(clock_rom, 0);
	clock_add_to_group(clock_gpio, 0);
	clock_add_to_group(clock_hdma0, 0);
	clock_add_to_group(clock_hdma1, 0);
	clock_add_to_group(clock_xpi0, 0);
	clock_add_to_group(clock_ptpc, 0);
	clock_add_to_group(clock_emds, 0);
	clock_add_to_group(clock_pwm0, 0);
	clock_add_to_group(clock_pwm1, 0);
	clock_add_to_group(clock_pwm2, 0);
	clock_add_to_group(clock_pwm3, 0);

	/* Connect Group0 to CPU0 */
	clock_connect_group_to_cpu(0, 0);

#ifndef CONFIG_SOC_SERIES_HPM5100
	/* Bump up DCDC voltage to 1275mv */
	pcfg_dcdc_set_voltage(HPM_PCFG, 1275);
#endif

	/*
	 * Keep BootROM default PLL preset for HPM5151:
	 * - CLK_TOP_CPU0 default is 240MHz
	 * - Functional clocks default to values from UM Table 18
	 */
	sysctl_config_cpu0_domain_clock(HPM_SYSCTL, clock_source_pll0_clk0, 2, 3);

	clock_update_core_clock();

	/* Configure mchtmr to 24MHz */
	clock_set_source_divider(clock_mchtmr0, clk_src_osc24m, 1);
}

/*
 * No soc_init_pma() here, unlike the other HPM SoCs: HPM5151 has no
 * configurable Andes PMA (hpm_soc_feature.h: PMP_SUPPORT_PMA (0); the
 * HPM5100 UM's non-standard CSR list has PMPCFG0-3 but no PMACFG/PMAADDR),
 * so pma_config_attributes() does not even exist in hpm_pmp_drv.c on this
 * part. SOC_ANDES_V5_PMA is therefore permanently n in this SoC's Kconfig
 * and CONFIG_NOCACHE_MEMORY cannot be selected -- see the comment there.
 */

static int hpmicro_soc_init(void)
{
	uint32_t key;

	key = irq_lock();
	soc_init_clock();
	irq_unlock(key);

	return 0;
}

SYS_INIT(hpmicro_soc_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
