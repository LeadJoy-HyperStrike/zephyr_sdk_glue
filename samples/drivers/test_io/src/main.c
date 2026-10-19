/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 *
 * Poll xT3U keys from boards/hpm5100evk.overlay (Zephyr GPIO0) plus EVK sw0.
 */

#include <errno.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#define TEST_IO_LONG_PRESS_MS 1000U
#define TEST_IO_POLL_MS       10U
#define TEST_IO_DUMP_MS       2000U

#define BUTTON_ENTRY(_alias, _name) \
	{ \
		.spec = GPIO_DT_SPEC_GET(DT_ALIAS(_alias), gpios), \
		.name = _name, \
	}

struct button_desc {
	struct gpio_dt_spec spec;
	const char *name;
};

struct button_state {
	bool stable_pressed;
	bool long_press_sent;
	bool raw_level_valid;
	bool raw_level;
	uint32_t pressed_since_ms;
};

static const struct button_desc button_descs[] = {
	BUTTON_ENTRY(xt3u_button_up, "dpad-up/PB01"),
	BUTTON_ENTRY(xt3u_button_down, "dpad-down/PA25"),
	BUTTON_ENTRY(xt3u_button_left, "dpad-left/PA26"),
	BUTTON_ENTRY(xt3u_button_right, "dpad-right/PA27"),
	BUTTON_ENTRY(xt3u_button_a, "A/PC01"),
	BUTTON_ENTRY(xt3u_button_b, "B/PC02"),
	BUTTON_ENTRY(xt3u_button_x, "X/PC03"),
	BUTTON_ENTRY(xt3u_button_y, "Y/PC04"),
	BUTTON_ENTRY(sw0, "sw0/PA15"),
};

static struct button_state button_states[ARRAY_SIZE(button_descs)];

static int configure_button_pin(const struct button_desc *desc)
{
	if (!gpio_is_ready_dt(&desc->spec)) {
		printk("GPIO device for %s is not ready\n", desc->name);
		return -ENODEV;
	}

	return gpio_pin_configure_dt(&desc->spec, GPIO_INPUT);
}

static int read_button(const struct button_desc *desc, uint8_t *raw, bool *pressed)
{
	int active = gpio_pin_get_dt(&desc->spec);

	if (active < 0) {
		return active;
	}

	*pressed = active != 0;
	*raw = gpio_pin_get_raw(desc->spec.port, desc->spec.pin);
	return 0;
}

static void emit_event(const char *type, size_t index)
{
	printk("button %s: %s\n", button_descs[index].name, type);
}

static void apply_button_state(size_t index, bool pressed, uint32_t now_ms)
{
	struct button_state *state = &button_states[index];
	bool was_long_press = state->long_press_sent;

	state->stable_pressed = pressed;
	if (pressed) {
		state->long_press_sent = false;
		state->pressed_since_ms = now_ms;
		emit_event("press", index);
	} else {
		emit_event("release", index);
		if (!was_long_press) {
			emit_event("click", index);
		}
		state->long_press_sent = false;
	}
}

static void dump_all(void)
{
	size_t i;

	printk("poll");
	for (i = 0; i < ARRAY_SIZE(button_descs); i++) {
		uint8_t raw;
		bool pressed;

		if (read_button(&button_descs[i], &raw, &pressed) < 0) {
			printk(" %s=err", button_descs[i].name);
			continue;
		}
		printk(" %s raw=%u p=%u", button_descs[i].name,
		       (unsigned int)raw, pressed ? 1U : 0U);
	}
	printk("\n");
}

static int buttons_init(void)
{
	size_t i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(button_descs); i++) {
		ret = configure_button_pin(&button_descs[i]);
		if (ret < 0) {
			printk("Failed to configure %s: %d\n",
			       button_descs[i].name, ret);
			return ret;
		}
	}

	printk("test_io ready: %u keys (Zephyr GPIO0)\n",
	       (unsigned int)ARRAY_SIZE(button_descs));
	return 0;
}

static void buttons_process(uint32_t now_ms)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(button_descs); i++) {
		const struct button_desc *desc = &button_descs[i];
		struct button_state *state = &button_states[i];
		uint8_t raw;
		bool pressed;
		int ret;

		ret = read_button(desc, &raw, &pressed);
		if (ret < 0) {
			printk("Failed to read %s: %d\n", desc->name, ret);
			continue;
		}

		if (!state->raw_level_valid || state->raw_level != (raw != 0)) {
			state->raw_level_valid = true;
			state->raw_level = raw != 0;
			printk("button %s: raw=%u (%s)\n", desc->name,
			       (unsigned int)raw,
			       pressed ? "pressed" : "released");
		}

		if (pressed != state->stable_pressed) {
			apply_button_state(i, pressed, now_ms);
		}

		if (state->stable_pressed && !state->long_press_sent &&
		    (uint32_t)(now_ms - state->pressed_since_ms) >=
			    TEST_IO_LONG_PRESS_MS) {
			state->long_press_sent = true;
			emit_event("long-press", i);
		}
	}
}

int main(void)
{
	uint32_t last_dump_ms;

	if (buttons_init() < 0) {
		return 0;
	}

	last_dump_ms = k_uptime_get_32();
	dump_all();

	while (1) {
		uint32_t now_ms = k_uptime_get_32();

		buttons_process(now_ms);
		if ((uint32_t)(now_ms - last_dump_ms) >= TEST_IO_DUMP_MS) {
			dump_all();
			last_dump_ms = now_ms;
		}
		k_msleep(TEST_IO_POLL_MS);
	}

	return 0;
}
