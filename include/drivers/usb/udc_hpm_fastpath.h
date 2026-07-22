/*
 * Copyright (c) 2026 HyperStrike
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SDK_GLUE_INCLUDE_DRIVERS_USB_UDC_HPM_FASTPATH_H_
#define SDK_GLUE_INCLUDE_DRIVERS_USB_UDC_HPM_FASTPATH_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

/*
 * Single-endpoint IN fast path for the HS2 8 kHz gamepad stream
 * (CONFIG_UDC_HPM_FASTPATH).
 *
 * While an endpoint is claimed:
 *  - its IN completions are routed directly from udc_hpm_isr() to the
 *    registered callback (mcycle timestamp taken at ISR entry), bypassing
 *    the usbd-thread event queue and the class layer;
 *  - the class layer must not feed it (udc ep_enqueue returns -EACCES);
 *  - the owner re-arms it with udc_hpm_fastpath_arm() (ISR-safe: static
 *    dTD rewrite + ENDPTPRIME via usb_device_edpt_xfer, no allocation).
 *
 * Bus reset / port change set the suppressed flag synchronously inside the
 * UDC ISR (the hardware endpoint state is torn down there; the deferred
 * Zephyr events reach the app only later via the usbd thread). A suppressed
 * fast path refuses to arm until the owner re-claims after the interface
 * comes back up.
 */

typedef void (*udc_hpm_fastpath_cb_t)(uint8_t ep_addr, uint32_t isr_cycle,
				      void *ctx);

/*
 * Take ownership of one IN endpoint. Fails with -EBUSY if a class-layer
 * transfer is still in flight on it (retry after the next completion) and
 * -EALREADY if a different endpoint is currently claimed.
 */
int udc_hpm_fastpath_claim(const struct device *dev, uint8_t ep_addr,
			   udc_hpm_fastpath_cb_t cb, void *ctx);

void udc_hpm_fastpath_release(const struct device *dev, uint8_t ep_addr);

int udc_hpm_fastpath_arm(const struct device *dev, uint8_t ep_addr,
			 uint8_t *buf, uint16_t len);

bool udc_hpm_fastpath_suppressed(const struct device *dev);

/*
 * Invoked from the UDC ISR on bus-death edges (bus reset, VBUS removed),
 * BEFORE the hardware endpoint flush. Runs in ISR context: keep it to a
 * few stores. Purpose: let the app revoke the CPU1 engine's stream
 * ownership synchronously instead of waiting for the usbd-thread class
 * disable - the ms-wide gap in between is the window where the engine's
 * cold-start branch re-primes a flushed, disabled endpoint and plants a
 * permanent zombie ENDPTSTAT/ENDPTPRIME bit (2026-07-22 replug wedge).
 */
typedef void (*udc_hpm_fastpath_bus_cb_t)(void *ctx);
void udc_hpm_fastpath_set_bus_cb(const struct device *dev,
				 udc_hpm_fastpath_bus_cb_t cb, void *ctx);

/* True while a transfer is primed/latching on the endpoint. The owner uses
 * this to keep the single-in-flight invariant: never arm on a primed EP.
 */
bool udc_hpm_fastpath_ep_primed(const struct device *dev, uint8_t ep_addr);

/* Flush stale prime/buffer-ready state off the endpoint (thread context;
 * also runs implicitly inside udc_hpm_fastpath_claim). Returns 0 when the
 * endpoint reads clean, -EIO if a prime/stat bit survived the flush dance.
 */
int udc_hpm_fastpath_reconcile(const struct device *dev, uint8_t ep_addr);

/*
 * Cross-core handover plumbing (HS2 CPU1 engine).
 *
 * udc_hpm_fastpath_export: system addresses of the claimed EP's dQH, its
 * first qTD slot and the USB controller base - everything the second hart
 * needs to run the register-level arm flow itself.
 *
 * udc_hpm_fastpath_mask_complete: while masked, this driver's ISR neither
 * processes nor W1C-clears the EP's ENDPTCOMPLETE IN bit; the CPU1 engine
 * polls and consumes it as the exclusive owner. Unmask before falling back
 * to the class/fastpath completion path on this core.
 */
int udc_hpm_fastpath_export(const struct device *dev, uint8_t ep_addr,
			    uint32_t *qhd_addr, uint32_t *qtd_addr,
			    uint32_t *regs_addr);
void udc_hpm_fastpath_mask_complete(const struct device *dev, uint8_t ep_addr,
				    bool mask);

#endif /* SDK_GLUE_INCLUDE_DRIVERS_USB_UDC_HPM_FASTPATH_H_ */
