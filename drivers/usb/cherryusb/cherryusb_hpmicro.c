/*
 * Copyright (c) 2025 hpmicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#define DT_DRV_COMPAT hpmicro_hpm_cherryusb

#include <soc.h>
#include <string.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/pm/device.h>
#ifdef CONFIG_PM
#include <zephyr/pm/policy.h>
#endif

#include "hpm_soc.h"
#include "hpm_clock_drv.h"
#include "hpm_usb_drv.h"
#include "usb_glue_hpm.h"

struct cherryusb_hpm_config {
	void (*irq_enable_func)(void);
	void (*irq_disable_func)(void);
	uint32_t reg_base;
	uint32_t clock_name;
	const struct pinctrl_dev_config *pincfg;
};

struct cherryusb_hpm_data {
	bool pm_deep_locked;
	bool pm_suspended;
};

static const struct device *cherryusb_dev_from_base(uint32_t base)
{
	const struct device *dev0 = DEVICE_DT_GET(DT_NODELABEL(cherryusb_usb0));
	const struct cherryusb_hpm_config *config0 = dev0->config;

	if (base == config0->reg_base) {
		return dev0;
	}

#if DT_NODE_EXISTS(DT_NODELABEL(cherryusb_usb1))
	const struct device *dev1 = DEVICE_DT_GET(DT_NODELABEL(cherryusb_usb1));
	const struct cherryusb_hpm_config *config1 = dev1->config;

	if (base == config1->reg_base) {
		return dev1;
	}
#endif

	return NULL;
}

#ifdef CONFIG_PM
static void cherryusb_hpm_lock_deep_sleep(const struct device *dev)
{
	struct cherryusb_hpm_data *priv = dev->data;

	if (priv->pm_deep_locked) {
		return;
	}

	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	pm_policy_state_lock_get(PM_STATE_SOFT_OFF, PM_ALL_SUBSTATES);
	priv->pm_deep_locked = true;
#ifdef CONFIG_PM_DEVICE
	pm_device_busy_set(dev);
#endif
}

static void cherryusb_hpm_unlock_deep_sleep(const struct device *dev)
{
	struct cherryusb_hpm_data *priv = dev->data;

	if (!priv->pm_deep_locked) {
		return;
	}

#ifdef CONFIG_PM_DEVICE
	pm_device_busy_clear(dev);
#endif
	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	pm_policy_state_lock_put(PM_STATE_SOFT_OFF, PM_ALL_SUBSTATES);
	priv->pm_deep_locked = false;
}
#endif

void hpm_usb_pm_lock(uint32_t base)
{
#ifdef CONFIG_PM
	const struct device *dev = cherryusb_dev_from_base(base);

	if (dev != NULL) {
		cherryusb_hpm_lock_deep_sleep(dev);
	}
#else
	ARG_UNUSED(base);
#endif
}

void hpm_usb_pm_unlock(uint32_t base)
{
#ifdef CONFIG_PM
	const struct device *dev = cherryusb_dev_from_base(base);

	if (dev != NULL) {
		cherryusb_hpm_unlock_deep_sleep(dev);
	}
#else
	ARG_UNUSED(base);
#endif
}

static void cherryusb_hpm_isr(uint32_t base)
{
	if (HPM_USB0_BASE == base) {
		hpm_isr_usb0();
	} else {
#if DT_NODE_EXISTS(DT_NODELABEL(cherryusb_usb1))
		hpm_isr_usb1();
#endif
	}
}

static int cherryusb_hpm_driver_preinit(const struct device *dev)
{
	const struct cherryusb_hpm_config *config = dev->config;

	clock_add_to_group((clock_name_t)(config->clock_name), 0);

	pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);

	usb_hcd_set_power_ctrl_polarity((USB_Type *)config->reg_base, true);
#if defined(CONFIG_SOC_SERIES_HPM5100)
	/*
	 * Match HPM5100EVK board_init_usb(): QFN/LQFP parts have no VBUS
	 * pin, so the PHY must use internal VBUS after PWR polarity settles.
	 */
	k_busy_wait(100 * 1000);
	usb_phy_using_internal_vbus((USB_Type *)config->reg_base);
#else
	/*
	 * Other SoCs keep the pre-merge sequence. Internal VBUS makes the PHY
	 * ignore the VBUS pin, i.e. no disconnect detection; udc_hpmicro.c
	 * gates the same call to HPM5100 (c0b0b7f).
	 */
	k_sleep(K_MSEC(100));
#endif

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int cherryusb_hpm_pm_action(const struct device *dev,
				   enum pm_device_action action)
{
	const struct cherryusb_hpm_config *config = dev->config;
	struct cherryusb_hpm_data *priv = dev->data;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		/*
		 * EHCI PHY/QH/QTD does not survive SoC Stop. While the stack
		 * is live the policy lock already blocks idle Stop; if Stop
		 * is still requested, fail suspend rather than resume a
		 * corrupted controller.
		 */
		if (priv->pm_deep_locked) {
			return -EBUSY;
		}

		if (priv->pm_suspended) {
			return 0;
		}

		clock_remove_from_group((clock_name_t)config->clock_name, 0);
		priv->pm_suspended = true;
		return 0;

	case PM_DEVICE_ACTION_RESUME:
		if (!priv->pm_suspended) {
			return 0;
		}

		clock_add_to_group((clock_name_t)config->clock_name, 0);
		if (pinctrl_apply_state(config->pincfg,
					PINCTRL_STATE_DEFAULT) < 0) {
			return -EIO;
		}

		usb_hcd_set_power_ctrl_polarity((USB_Type *)config->reg_base,
						true);
#if defined(CONFIG_SOC_SERIES_HPM5100)
		usb_phy_using_internal_vbus((USB_Type *)config->reg_base);
#endif

		priv->pm_suspended = false;
		return 0;

	default:
		return -ENOTSUP;
	}
}
#endif

void hpm_usb_isr_enable(uint32_t base)
{
	const struct device *dev = cherryusb_dev_from_base(base);
	const struct cherryusb_hpm_config *config;

	if (dev == NULL) {
		return;
	}

	config = dev->config;
	config->irq_enable_func();
}

void hpm_usb_isr_disable(uint32_t base)
{
	const struct device *dev = cherryusb_dev_from_base(base);
	const struct cherryusb_hpm_config *config;

	if (dev == NULL) {
		return;
	}

	config = dev->config;
	config->irq_disable_func();
}

#define CHERRYUSB_HPM_DEFINE(n)								\
	static void cherryusb_irq_enable_func##n(void)			\
	{														\
		IRQ_CONNECT(DT_INST_IRQN(n),						\
					DT_INST_IRQ(n, priority),				\
					cherryusb_hpm_isr,						\
					DT_INST_REG_ADDR(n), 0);				\
															\
		irq_enable(DT_INST_IRQN(n));						\
	}														\
															\
	static void cherryusb_irq_disable_func##n(void)			\
	{														\
		irq_disable(DT_INST_IRQN(n));						\
	}														\
															\
	PINCTRL_DT_INST_DEFINE(n);								\
															\
	static struct cherryusb_hpm_config priv_config_##n = {	\
		.reg_base = DT_INST_REG_ADDR(n),					\
		.clock_name = DT_INST_PROP(n, clk_name),			\
		.irq_enable_func = cherryusb_irq_enable_func##n,			\
		.irq_disable_func = cherryusb_irq_disable_func##n,			\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),				\
	};																\
	static struct cherryusb_hpm_data priv_data_##n;					\
	PM_DEVICE_DT_INST_DEFINE(n, cherryusb_hpm_pm_action);			\
																	\
	DEVICE_DT_INST_DEFINE(n, cherryusb_hpm_driver_preinit,			\
						PM_DEVICE_DT_INST_GET(n),					\
						&priv_data_##n, &priv_config_##n,			\
						POST_KERNEL, 80,							\
						NULL);

DT_INST_FOREACH_STATUS_OKAY(CHERRYUSB_HPM_DEFINE)
