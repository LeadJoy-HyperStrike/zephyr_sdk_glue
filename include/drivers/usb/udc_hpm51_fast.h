#ifndef UDC_HPM51_FAST_H
#define UDC_HPM51_FAST_H
#include <zephyr/device.h>
#include <stdbool.h>
#include <stdint.h>
struct udc_hpm51_fast_view {uintptr_t qhd,qtd,regs;};
/* callback is in USB IRQ context, BEFORE the standard stack clears COMPLETE.
 * dead revokes permission to arm; callback must stop its hardware timer. */
typedef void (*udc_hpm51_fast_cb)(void *ctx,bool dead,uint32_t irq_entry_tick);
int udc_hpm51_fast_claim(const struct device *dev,uint8_t ep,
	udc_hpm51_fast_cb cb,void *ctx,struct udc_hpm51_fast_view *view);
/* Caller first stops its timer and successfully flushes the endpoint. */
void udc_hpm51_fast_release(const struct device *dev);
bool udc_hpm51_fast_allowed(void);
void udc_hpm51_fast_nak_enable(bool enable);
#if defined(CONFIG_HS2_HPM51_IMU_PRIORITY)
/* Cold policy edge only; preserves endpoint ownership and queued packet. */
void udc_hpm51_fast_set_callback(udc_hpm51_fast_cb callback);
#endif
#endif
