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

/* True while a transfer is primed/latching on the endpoint. The owner uses
 * this to keep the single-in-flight invariant: never arm on a primed EP.
 */
bool udc_hpm_fastpath_ep_primed(const struct device *dev, uint8_t ep_addr);

#endif /* SDK_GLUE_INCLUDE_DRIVERS_USB_UDC_HPM_FASTPATH_H_ */
