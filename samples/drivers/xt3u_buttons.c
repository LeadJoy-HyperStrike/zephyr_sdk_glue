/* GPIO button input for xT3U. */
/* SPDX-License-Identifier: Apache-2.0 */

#include "xt3u_buttons.h"

#include <errno.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "board.h"
#include "hpm_gpio_drv.h"
#include "hpm_gpiom_drv.h"
#include "hpm_gpiom_soc_drv.h"
#include "hpm_soc.h"

#include "xt3u_keymap_engine.h"
#include "usb_protocol.h"

LOG_MODULE_REGISTER(xt3u_buttons, LOG_LEVEL_INF);

#define XT3U_BUTTON_A_NODE DT_ALIAS(xt3u_button_a)

#if !DT_NODE_HAS_STATUS(XT3U_BUTTON_A_NODE, okay)
#error "xT3U A button requires a ready xt3u-button-a devicetree alias"
#endif

static const struct gpio_dt_spec button_a_gpio =
	GPIO_DT_SPEC_GET(XT3U_BUTTON_A_NODE, gpios);

struct xt3u_button_desc {
	uint32_t ioc_pad;
	uint32_t gpio_port;
	uint8_t gpio_pin;
	uint32_t gpiom_assignment;
	uint32_t gpio_func_ctl;
	const struct gpio_dt_spec *dt_gpio;
	const char *name;
	enum xt3u_button_logical logical;
};

struct xt3u_button_state {
	bool stable_pressed;
	bool long_press_sent;
	bool raw_level_valid;
	bool raw_level;
	uint32_t pressed_since_ms;
};

/*
 * 映射表中已填写的产品按键由 HPM SDK 直接配置为内部上拉、低有效；sw0 仍保留为
 * EVK 级全按诊断/USB 模式切换键，不作为产品 A 键使用。
 */
static const struct xt3u_button_desc button_descs[] = {
	{
		.ioc_pad = IOC_PAD_PB01,
		.gpio_port = GPIO_DI_GPIOB,
		.gpio_pin = 1U,
		.gpiom_assignment = GPIOM_ASSIGN_GPIOB,
		.gpio_func_ctl = IOC_PB01_FUNC_CTL_GPIO_B_01,
		.name = "dpad-up/PB01",
		.logical = XT3U_BUTTON_UP,
	},
	{
		.ioc_pad = IOC_PAD_PA25,
		.gpio_port = GPIO_DI_GPIOA,
		.gpio_pin = 25U,
		.gpiom_assignment = GPIOM_ASSIGN_GPIOA,
		.gpio_func_ctl = IOC_PA25_FUNC_CTL_GPIO_A_25,
		.name = "dpad-down/PA25",
		.logical = XT3U_BUTTON_DOWN,
	},
	{
		.ioc_pad = IOC_PAD_PA26,
		.gpio_port = GPIO_DI_GPIOA,
		.gpio_pin = 26U,
		.gpiom_assignment = GPIOM_ASSIGN_GPIOA,
		.gpio_func_ctl = IOC_PA26_FUNC_CTL_GPIO_A_26,
		.name = "dpad-left/PA26",
		.logical = XT3U_BUTTON_LEFT,
	},
	{
		.ioc_pad = IOC_PAD_PA27,
		.gpio_port = GPIO_DI_GPIOA,
		.gpio_pin = 27U,
		.gpiom_assignment = GPIOM_ASSIGN_GPIOA,
		.gpio_func_ctl = IOC_PA27_FUNC_CTL_GPIO_A_27,
		.name = "dpad-right/PA27",
		.logical = XT3U_BUTTON_RIGHT,
	},
	{
		.dt_gpio = &button_a_gpio,
		.name = "A/PC01",
		.logical = XT3U_BUTTON_A,
	},
	{
		.ioc_pad = IOC_PAD_PC02,
		.gpio_port = GPIO_DI_GPIOC,
		.gpio_pin = 2U,
		.gpiom_assignment = GPIOM_ASSIGN_GPIOC,
		.gpio_func_ctl = IOC_PC02_FUNC_CTL_GPIO_C_02,
		.name = "B/PC02",
		.logical = XT3U_BUTTON_B,
	},
	{
		.ioc_pad = IOC_PAD_PC03,
		.gpio_port = GPIO_DI_GPIOC,
		.gpio_pin = 3U,
		.gpiom_assignment = GPIOM_ASSIGN_GPIOC,
		.gpio_func_ctl = IOC_PC03_FUNC_CTL_GPIO_C_03,
		.name = "X/PC03",
		.logical = XT3U_BUTTON_X,
	},
	{
		.ioc_pad = IOC_PAD_PC04,
		.gpio_port = GPIO_DI_GPIOC,
		.gpio_pin = 4U,
		.gpiom_assignment = GPIOM_ASSIGN_GPIOC,
		.gpio_func_ctl = IOC_PC04_FUNC_CTL_GPIO_C_04,
		.name = "Y/PC04",
		.logical = XT3U_BUTTON_Y,
	},
	{
		.ioc_pad = IOC_PAD_PA15,
		.gpio_port = GPIO_DI_GPIOA,
		.gpio_pin = 15U,
		.gpiom_assignment = GPIOM_ASSIGN_GPIOA,
		.gpio_func_ctl = IOC_PA15_FUNC_CTL_GPIO_A_15,
		.name = "sw0/all-buttons",
		.logical = XT3U_BUTTON_HOME,
	},
};

static struct xt3u_button_state button_states[ARRAY_SIZE(button_descs)];
static uint32_t logical_buttons;
static uint32_t generation;
static bool initialized;
static xt3u_button_event_callback_t event_callback;
static void *event_callback_user_data;

static int configure_button_pin(const struct xt3u_button_desc *desc)
{
	if (desc->dt_gpio != NULL) {
		if (!gpio_is_ready_dt(desc->dt_gpio)) {
			LOG_ERR("GPIO device for %s is not ready", desc->name);
			return -ENODEV;
		}

		/* Pull-up and active-low flags come from the DT gpio spec. */
		return gpio_pin_configure_dt(desc->dt_gpio, GPIO_INPUT);
	}

	/*
	 * Direct HPM SDK setup: select GPIO, enable the internal pull-up and
	 * Schmitt input, route the pad to the FGPIO controller, then set input.
	 */
	HPM_IOC->PAD[desc->ioc_pad].FUNC_CTL = desc->gpio_func_ctl;
	HPM_IOC->PAD[desc->ioc_pad].PAD_CTL = IOC_PAD_PAD_CTL_HYS_SET(1) |
		IOC_PAD_PAD_CTL_PRS_SET(2) | IOC_PAD_PAD_CTL_PS_SET(1) |
		IOC_PAD_PAD_CTL_PE_SET(1);
	gpiom_set_pin_controller(BOARD_APP_GPIOM_BASE, desc->gpiom_assignment,
				 desc->gpio_pin, BOARD_APP_GPIOM_USING_CTRL_NAME);
	gpio_set_pin_input(BOARD_APP_GPIOM_USING_CTRL, desc->gpio_port,
			   desc->gpio_pin);
	return 0;
}

static const char *event_name(enum xt3u_button_event_type type)
{
	switch (type) {
	case XT3U_BUTTON_EVENT_PRESS:
		return "press";
	case XT3U_BUTTON_EVENT_RELEASE:
		return "release";
	case XT3U_BUTTON_EVENT_CLICK:
		return "click";
	case XT3U_BUTTON_EVENT_LONG_PRESS:
		return "long-press";
	default:
		return "unknown";
	}
}

static void emit_event(enum xt3u_button_event_type type, size_t index)
{
	const struct xt3u_button_desc *desc = &button_descs[index];
	struct xt3u_button_event event = {
		.type = type,
		.logical = desc->logical,
		.name = desc->name,
	};

	LOG_INF("button %s: %s", desc->name, event_name(type));
	if (event_callback != NULL) {
		event_callback(&event, event_callback_user_data);
	}
}

static void apply_button_state(size_t index, bool pressed, uint32_t now_ms)
{
	const struct xt3u_button_desc *desc = &button_descs[index];
	struct xt3u_button_state *state = &button_states[index];
	bool was_long_press = state->long_press_sent;
	int keymap_ret;

	state->stable_pressed = pressed;
	keymap_ret = xt3u_keymap_engine_handle_event(desc->logical, pressed);

	if (keymap_ret < 0) {
		LOG_WRN("button %s keymap dispatch failed: %d", desc->name,
			keymap_ret);
	}
	if (pressed) {
		state->long_press_sent = false;
		state->pressed_since_ms = now_ms;
		logical_buttons |= XT3U_BUTTON_LOGICAL_MASK(desc->logical);
		emit_event(XT3U_BUTTON_EVENT_PRESS, index);
	} else {
		logical_buttons &= ~XT3U_BUTTON_LOGICAL_MASK(desc->logical);
		emit_event(XT3U_BUTTON_EVENT_RELEASE, index);
		if (!was_long_press) {
			emit_event(XT3U_BUTTON_EVENT_CLICK, index);
		}
		state->long_press_sent = false;
	}
	generation++;
}

int xt3u_buttons_init(void)
{
	size_t index;
	int ret;

	if (initialized) {
		return 0;
	}

	ret = xt3u_keymap_engine_init();
	if (ret < 0) {
		return ret;
	}

	for (index = 0; index < ARRAY_SIZE(button_descs); index++) {
		ret = configure_button_pin(&button_descs[index]);
		if (ret < 0) {
			LOG_ERR("Failed to configure button %s: %d",
				button_descs[index].name, ret);
			return ret;
		}
	}

	initialized = true;
	LOG_INF("HPM SDK direct GPIO buttons ready: %u (8 product keys + sw0 diagnostic)",
		(unsigned int)ARRAY_SIZE(button_descs));
	return 0;
}

void xt3u_buttons_process(void)
{
	uint32_t now_ms;
	size_t index;

	if (!initialized) {
		return;
	}

	now_ms = k_uptime_get_32();
	for (index = 0; index < ARRAY_SIZE(button_descs); index++) {
		const struct xt3u_button_desc *desc = &button_descs[index];
		struct xt3u_button_state *state = &button_states[index];
		uint8_t raw;
		bool pressed;

		if (desc->dt_gpio != NULL) {
			int active = gpio_pin_get_dt(desc->dt_gpio);

			if (active < 0) {
				LOG_ERR("Failed to read button %s: %d", desc->name,
					active);
				continue;
			}

			/* Keep the legacy raw-level convention for shared debounce code. */
			pressed = active != 0;
			raw = pressed ? 0U : 1U;
		} else {
			raw = gpio_read_pin(BOARD_APP_GPIOM_USING_CTRL,
					    desc->gpio_port, desc->gpio_pin);
			pressed = raw == 0U;
		}

		if (!state->raw_level_valid || state->raw_level != (raw != 0)) {
			state->raw_level_valid = true;
			state->raw_level = raw != 0;
			LOG_INF("button %s: raw=%u (%s)", desc->name,
				(unsigned int)raw, pressed ? "pressed" : "released");
		}

		if (pressed != state->stable_pressed) {
			apply_button_state(index, pressed, now_ms);
		}

		if (state->stable_pressed && !state->long_press_sent &&
		    (uint32_t)(now_ms - state->pressed_since_ms) >=
			    CONFIG_XT3U_BUTTON_LONG_PRESS_MS) {
			state->long_press_sent = true;
			emit_event(XT3U_BUTTON_EVENT_LONG_PRESS, index);
		}
	}
}

bool xt3u_buttons_is_ready(void)
{
	return initialized;
}

uint32_t xt3u_buttons_get_gamepad_buttons(void)
{
	struct gamepad_state state = { 0 };

	xt3u_keymap_engine_apply(logical_buttons, &state);
	return state.buttons;
}

uint32_t xt3u_buttons_get_generation(void)
{
	return generation;
}

bool xt3u_buttons_is_logical_pressed(enum xt3u_button_logical logical)
{
	if (logical >= XT3U_BUTTON_LOGICAL_COUNT) {
		return false;
	}

	return (logical_buttons & XT3U_BUTTON_LOGICAL_MASK(logical)) != 0U;
}

void xt3u_buttons_apply_to_gamepad_state(struct gamepad_state *state)
{
	if (state == NULL) {
		return;
	}

	xt3u_keymap_engine_apply(logical_buttons, state);
}

void xt3u_buttons_set_event_callback(xt3u_button_event_callback_t callback,
				     void *user_data)
{
	event_callback = callback;
	event_callback_user_data = user_data;
}
