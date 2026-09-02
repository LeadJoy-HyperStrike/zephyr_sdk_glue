/*
 * Copyright (c) 2026 LeadJoy
 * SPDX-License-Identifier: Apache-2.0
 *
 * The one list of peripheral clocks CONFIG_HPM_SOC_GATE_UNUSED_CLOCKS turns
 * off, as an X-macro so every consumer generates from the same source.
 *
 * It is a header and not just an array in soc.c because the list had already
 * been transcribed a second time -- into `hs2clk`'s expected_off[] table in
 * the application -- and the two copies drifted the moment the list grew:
 * hs2clk was still checking 23 entries against a 83-entry reality, and was
 * still expecting EMDS to be off after EMDS became unconditionally ON. A
 * diagnostic that lies about the thing it exists to diagnose is worse than no
 * diagnostic, and this list will keep growing, so it gets one home.
 *
 * ---------------------------------------------------------------------------
 * ADMISSION CRITERIA. Three checks, all of them, because the obvious one is
 * exactly the one that failed on TRGM:
 *
 *   1. No status="okay" node at the IP's base address in either board's
 *      generated zephyr.dts. Match on ADDRESS, not on name -- clock_canN maps
 *      to HPM_MCANn_BASE, clock_mbxN to HPM_MBXnA/B_BASE, clock_watchdogN to
 *      HPM_EWDGn_BASE. Name matching silently misses whole families.
 *   2. No reference anywhere in the firmware tree.
 *   3. Not reachable through a raw base address from a driver. The overlays
 *      contain exactly three such addresses: TRGM 0xF047C000, PPI 0xF00CC000
 *      and GPTMR1 0xF0004000. None may ever appear below.
 *
 * "Nothing in the devicetree uses it" ALONE is not a criterion. TRGM has no
 * node, no clock binding, and no declared dependency of any kind; every check
 * passed, four board configurations built clean, `hs2clk` reported "0
 * unexpected", and the analog sticks were dead. See the EMDS block in soc.c.
 *
 * ---------------------------------------------------------------------------
 * IN USE -- these must never appear here:
 *   adc0-3, i2c0 (touchpad), spi1 (WS2812), spi7 (BMI423 IMU), uart0
 *   (console/shell), usb0, pwm0 (ADC TRGO), pwm1 (hs2prod left motor, DRV8833
 *   IN1 on PWM1_P_5 -- added 2026-08-26 by module 10, AFTER this list was
 *   written; it sat here gated until the 2026-09-02 review), emds
 *   (motor-island master gate that clocks TRGM), gptmr0 (hs2_stream), gptmr1
 *   (AD7606 CONVST -- added by that driver through a variable, so grepping
 *   for clock_add_to_group misses it; it was nearly misfiled as unowned).
 *
 * The list is now also checked mechanically: hs2_hpm_firmware
 * tests/static/test_gated_clock_contract.py fails when any entry below has a
 * status="okay" node in a board dts / app overlay, and the pwm glue driver
 * clock_add_to_group()s its own instance in init as a second line of defence
 * (the uart/spi/i2c/adc drivers always did; pwm did not).
 *
 * DELIBERATELY WITHHELD -- each for its own reason, check it before adding:
 *   femc        the AD7606 PPI runs on the pad group shared with FEMC. The
 *               clocks are separate, but this is the shape of the EMDS trap.
 *   ptmr, puart PMIC-domain; may touch the wake path the sleep mode needs.
 *   watchdog0-3 no node and no wdt_* caller, but CONFIG_WATCHDOG=y is still
 *               set, and gating a watchdog's clock turns a missed feed into a
 *               reset. Bad trade for four blocks.
 *   kman, rng, sdp, pka   crypto. MCUboot verifies with tinycrypt (software)
 *               and the app never asks for entropy, so these look free -- but
 *               the failure mode is a boot that cannot verify.
 *   ana0-3, aud0-1, ref0-1   clock SOURCES feeding ADC/I2S/pins, not
 *               peripherals. The ADC tree is downstream of them.
 *
 * ---------------------------------------------------------------------------
 * MEASURED: 0.9 W -> 0.7 W on hs2prod once this list grew past the motor
 * island. The motor entries alone had measured as no change at all -- they are
 * idle logic with nothing toggling. The blocks that moved the needle were the
 * ordinary unused peripherals, 8x MCAN first.
 */

#ifndef HPM_GATED_CLOCKS_H_
#define HPM_GATED_CLOCKS_H_

/* Motor-control island, minus EMDS (its master gate -- see soc.c) and minus
 * PWM1 (hs2prod left motor -- see IN USE above). */
#define HPM_GATED_CLOCK_LIST_MOTOR(X) \
	X(ptpc) \
	X(qei0) X(qei1) X(qei2) X(qei3) \
	X(qeo0) X(qeo1) X(qeo2) X(qeo3) \
	X(pwm2) X(pwm3) \
	X(rdc0) X(rdc1) \
	X(plb0) X(sei0) \
	X(mtg0) X(mtg1) \
	X(vsc0) X(vsc1) \
	X(clc0) X(clc1)

/* Standalone blocks with no consumer on either board. Cross-core traffic uses
 * the ALS-mapped shared xblock, never a mailbox.
 */
#define HPM_GATED_CLOCK_LIST_STANDALONE(X) \
	X(can0) X(can1) X(can2) X(can3) \
	X(can4) X(can5) X(can6) X(can7) \
	X(mbx0) X(mbx1) \
	X(tsns) X(lobs) X(ntmr0)

/* Same-family siblings of blocks we do use. CRC is here because
 * CONFIG_CRC=y selects Zephyr's SOFTWARE crc library (an mcumgr dependency);
 * nothing touches the CRC IP.
 */
#define HPM_GATED_CLOCK_LIST_SIBLINGS(X) \
	X(uart1) X(uart2) X(uart3) X(uart4) X(uart5) \
	X(uart6) X(uart7) X(uart8) X(uart9) X(uart10) \
	X(uart11) X(uart12) X(uart13) X(uart14) X(uart15) \
	X(i2c1) X(i2c2) X(i2c3) X(i2c4) X(i2c5) X(i2c6) X(i2c7) \
	X(spi0) X(spi2) X(spi3) X(spi4) X(spi5) X(spi6) \
	X(gptmr2) X(gptmr3) X(gptmr4) X(gptmr5) X(gptmr6) X(gptmr7) \
	X(acmp0) X(acmp1) X(acmp2) X(acmp3) \
	X(i2s0) X(i2s1) X(dao) X(pdm) \
	X(esc0) X(eth0) X(ffa0) \
	X(sdm0) X(sdm1) \
	X(crc0)

#define HPM_GATED_CLOCK_LIST(X) \
	HPM_GATED_CLOCK_LIST_MOTOR(X) \
	HPM_GATED_CLOCK_LIST_STANDALONE(X) \
	HPM_GATED_CLOCK_LIST_SIBLINGS(X)

#endif /* HPM_GATED_CLOCKS_H_ */
