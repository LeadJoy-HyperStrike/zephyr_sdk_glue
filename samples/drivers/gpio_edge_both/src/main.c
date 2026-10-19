/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 *
 * USER KEY PA15: GPIO_INT_EDGE_BOTH (HPM5151).
 * Optional overlay: PC01/PC02/PC04 inputs; PC03 output high (jumper to PC01).
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define SW0_NODE DT_ALIAS(sw0)

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});
static struct gpio_callback button_cb;
static bool led_ok;
static struct k_sem irq_sem;
static volatile uint32_t irq_count;
static volatile int last_raw;
static volatile gpio_pin_t last_pin;

#if DT_NODE_EXISTS(DT_ALIAS(sw1))
static const struct gpio_dt_spec pc_keys[] = {
	GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios),
	GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios),
	GPIO_DT_SPEC_GET(DT_ALIAS(sw4), gpios),
};
static const char *const pc_names[] = { "PC01", "PC02", "PC04" };
static struct gpio_callback pc_cb;
#endif

#if DT_NODE_EXISTS(DT_ALIAS(pc03))
static const struct gpio_dt_spec pc03 = GPIO_DT_SPEC_GET(DT_ALIAS(pc03), gpios);
#endif

static void button_isr(const struct device *dev, struct gpio_callback *cb,
		       uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);

	last_pin = button.pin;
	last_raw = gpio_pin_get_raw(button.port, button.pin);
	irq_count++;
	k_sem_give(&irq_sem);
}

#if DT_NODE_EXISTS(DT_ALIAS(sw1))
static void pc_isr(const struct device *dev, struct gpio_callback *cb,
		   uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);

	for (size_t i = 0; i < ARRAY_SIZE(pc_keys); i++) {
		if ((pins & BIT(pc_keys[i].pin)) != 0U) {
			last_pin = pc_keys[i].pin;
			last_raw = gpio_pin_get_raw(pc_keys[i].port, pc_keys[i].pin);
			irq_count++;
			k_sem_give(&irq_sem);
		}
	}
}
#endif

int main(void)
{
	int ret;

	k_sem_init(&irq_sem, 0, K_SEM_MAX_LIMIT);

	if (!gpio_is_ready_dt(&button)) {
		printk("button not ready\n");
		return 0;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret) {
		printk("cfg button %d\n", ret);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_BOTH);
	if (ret) {
		printk("EDGE_BOTH unsupported (%d)\n", ret);
		return 0;
	}

	gpio_init_callback(&button_cb, button_isr, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb);

	led_ok = led.port && gpio_is_ready_dt(&led) &&
		 (gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE) == 0);

	printk("GPIO EDGE_BOTH OK: press/release USER KEY (PA15)\n");
	printk("PA15 idle raw=%d get=%d\n",
	       gpio_pin_get_raw(button.port, button.pin),
	       gpio_pin_get_dt(&button));

#if DT_NODE_EXISTS(DT_ALIAS(pc03))
	if (!gpio_is_ready_dt(&pc03)) {
		printk("PC03 not ready\n");
		return 0;
	}
	ret = gpio_pin_configure_dt(&pc03, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		printk("cfg PC03 %d\n", ret);
		return 0;
	}
	printk("PC03 output high raw=%d (jumper PC03 -> PC01)\n",
	       gpio_pin_get_raw(pc03.port, pc03.pin));
#endif

#if DT_NODE_EXISTS(DT_ALIAS(sw1))
	{
		uint32_t pc_mask = 0;

		for (size_t i = 0; i < ARRAY_SIZE(pc_keys); i++) {
			if (!gpio_is_ready_dt(&pc_keys[i])) {
				printk("%s not ready\n", pc_names[i]);
				return 0;
			}
			ret = gpio_pin_configure_dt(&pc_keys[i], GPIO_INPUT);
			if (ret) {
				printk("cfg %s %d\n", pc_names[i], ret);
				return 0;
			}
			ret = gpio_pin_interrupt_configure_dt(&pc_keys[i],
							      GPIO_INT_EDGE_BOTH);
			if (ret) {
				printk("%s EDGE_BOTH %d\n", pc_names[i], ret);
				return 0;
			}
			pc_mask |= BIT(pc_keys[i].pin);
			printk("%s idle raw=%d get=%d\n",
			       pc_names[i],
			       gpio_pin_get_raw(pc_keys[i].port, pc_keys[i].pin),
			       gpio_pin_get_dt(&pc_keys[i]));
		}

		gpio_init_callback(&pc_cb, pc_isr, pc_mask);
		gpio_add_callback(pc_keys[0].port, &pc_cb);
		printk("jumper PC03->PC01: expect PC01 raw=1 / EDGE_BOTH pin=1\n");
	}
#endif

	while (1) {
		if (k_sem_take(&irq_sem, K_SECONDS(2)) == 0) {
			printk("EDGE_BOTH pin=%u raw=%d count=%u\n",
			       last_pin, last_raw, irq_count);
			if (led_ok && (last_pin == button.pin)) {
				gpio_pin_set_dt(&led, last_raw == 0);
			}
		} else {
#if DT_NODE_EXISTS(DT_ALIAS(sw1))
			printk("poll PC01 raw=%d get=%d",
			       gpio_pin_get_raw(pc_keys[0].port, pc_keys[0].pin),
			       gpio_pin_get_dt(&pc_keys[0]));
#if DT_NODE_EXISTS(DT_ALIAS(pc03))
			printk("  PC03 raw=%d",
			       gpio_pin_get_raw(pc03.port, pc03.pin));
#endif
			printk("\n");
#endif
		}
	}

	return 0;
}
