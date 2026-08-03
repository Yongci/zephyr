/*
 * Copyright (c) 2026 Yongci
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <string.h>

LOG_MODULE_REGISTER(envdaq, LOG_LEVEL_INF);

/* ============================================================
 * GPIO Definitions
 * ============================================================ */

static const struct gpio_dt_spec relay_a = GPIO_DT_SPEC_GET(DT_NODELABEL(relay_a), gpios);
static const struct gpio_dt_spec relay_b = GPIO_DT_SPEC_GET(DT_NODELABEL(relay_b), gpios);
static bool relay_state_a;
static bool relay_state_b;

/* ============================================================
 * ADC Definitions - Battery Voltage Measurement
 * ============================================================ */

#define ADC_DEVICE_NODE DT_NODELABEL(adc0)
static const struct device *adc_dev = DEVICE_DT_GET(ADC_DEVICE_NODE);

#define ADC_CHANNEL_ID 2 /* ADC1_CH2 on GPIO2 */
#define ADC_GAIN_SETTING ADC_GAIN_1_4
#define ADC_RESOLUTION 12

/* ADC sample count for oversampling */
#define ADC_SAMPLE_COUNT 16

/* ============================================================
 * Two-Point Linear Calibration (integer mV-based)
 *
 * Measured with a multimeter:
 *   Point 1: ADC = 1620 mV, Battery = 9990 mV
 *   Point 2: ADC = 1720 mV, Battery = 12715 mV
 *
 * Formula: battery_mv = (adc_mv * slope_num) / slope_den + offset_mv
 *          slope_num / slope_den = 27.25 = 109 / 4
 *          offset_mv = -34155
 * ============================================================ */

#define CAL_POINT1_ADC_MV      1620
#define CAL_POINT1_BATTERY_MV 9990
#define CAL_POINT2_ADC_MV      1720
#define CAL_POINT2_BATTERY_MV 12715

#define CAL_SLOPE_NUM 109
#define CAL_SLOPE_DEN 4
#define CAL_OFFSET_MV (-34155)

/* ============================================================
 * Battery Voltage Reading Functions (with oversampling)
 * ============================================================ */

static int read_battery_voltage(int32_t *voltage_mv, int32_t *adc_input_mv)
{
	int ret;
	uint16_t sample_buffer = 0;
	int32_t adc_mv;
	int32_t sum_raw = 0;
	int num_samples = ADC_SAMPLE_COUNT;

	if (!device_is_ready(adc_dev)) {
		LOG_ERR("ADC device not ready");
		return -ENODEV;
	}

	struct adc_channel_cfg ch_cfg = {
		.gain = ADC_GAIN_SETTING,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.channel_id = ADC_CHANNEL_ID,
		.differential = 0,
	};

	ret = adc_channel_setup(adc_dev, &ch_cfg);
	if (ret < 0) {
		LOG_ERR("Failed to setup ADC channel: %d", ret);
		return ret;
	}

	/* multiple samples for oversampling to improve stability */
	for (int i = 0; i < num_samples; i++) {
		struct adc_sequence sequence = {
			.channels = BIT(ADC_CHANNEL_ID),
			.buffer = &sample_buffer,
			.buffer_size = sizeof(sample_buffer),
			.resolution = ADC_RESOLUTION,
			.calibrate = false,
		};

		ret = adc_read(adc_dev, &sequence);
		if (ret < 0) {
			LOG_ERR("ADC read failed: %d", ret);
			return ret;
		}
		sum_raw += sample_buffer;
		k_busy_wait(50);
	}

	int32_t avg_raw = sum_raw / num_samples;

	adc_mv = avg_raw;
	ret = adc_raw_to_millivolts(adc_ref_internal(adc_dev), ADC_GAIN_SETTING,
				    ADC_RESOLUTION, &adc_mv);
	if (ret < 0) {
		LOG_ERR("ADC conversion failed: %d", ret);
		return ret;
	}

	*adc_input_mv = adc_mv;

	/* Apply the linear calibration in millivolts using integer math */
	int64_t battery_mv = ((int64_t)adc_mv * CAL_SLOPE_NUM) / CAL_SLOPE_DEN +
			     CAL_OFFSET_MV;

	*voltage_mv = (int32_t)battery_mv;

	LOG_DBG("ADC raw avg: %d, ADC mV: %d, Battery mV: %d",
		avg_raw, adc_mv, *voltage_mv);

	return 0;
}

/* ============================================================
 * Relay Control Functions (unchanged)
 * ============================================================ */

static int relay_init(void)
{
	int ret;

	if (!gpio_is_ready_dt(&relay_a)) {
		LOG_ERR("relay_a GPIO not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&relay_b)) {
		LOG_ERR("relay_b GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&relay_a, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure relay_a: %d", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&relay_b, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure relay_b: %d", ret);
		return ret;
	}

	relay_state_a = false;
	relay_state_b = false;

	LOG_INF("Relays initialized (OFF)");
	return 0;
}

static int relay_set_one(const struct gpio_dt_spec *relay, bool enabled)
{
	if (!gpio_is_ready_dt(relay)) {
		return -ENODEV;
	}

	return gpio_pin_set_dt(relay, enabled ? 1 : 0);
}

static int relay_toggle_one(const struct gpio_dt_spec *relay)
{
	if (!gpio_is_ready_dt(relay)) {
		return -ENODEV;
	}

	return gpio_pin_toggle_dt(relay);
}

static void relay_print_status(const char *name, bool relay_state,
			       const struct shell *sh)
{
	shell_print(sh, "%s: %s", name, relay_state ? "on" : "off");
}

/* ============================================================
 * Shell Commands (unchanged)
 * ============================================================ */

enum relay_target {
	RELAY_TARGET_ALL,
	RELAY_TARGET_A,
	RELAY_TARGET_B,
};

enum relay_action {
	RELAY_ACTION_ON,
	RELAY_ACTION_OFF,
	RELAY_ACTION_TOGGLE,
	RELAY_ACTION_STATUS,
};

static bool relay_parse_target(const char *arg, enum relay_target *target)
{
	if (strcmp(arg, "a") == 0 || strcmp(arg, "relay_a") == 0) {
		*target = RELAY_TARGET_A;
		return true;
	}

	if (strcmp(arg, "b") == 0 || strcmp(arg, "relay_b") == 0) {
		*target = RELAY_TARGET_B;
		return true;
	}

	if (strcmp(arg, "all") == 0) {
		*target = RELAY_TARGET_ALL;
		return true;
	}

	return false;
}

static bool relay_parse_action(const char *arg, enum relay_action *action)
{
	if (strcmp(arg, "on") == 0) {
		*action = RELAY_ACTION_ON;
		return true;
	}

	if (strcmp(arg, "off") == 0) {
		*action = RELAY_ACTION_OFF;
		return true;
	}

	if (strcmp(arg, "toggle") == 0) {
		*action = RELAY_ACTION_TOGGLE;
		return true;
	}

	if (strcmp(arg, "status") == 0) {
		*action = RELAY_ACTION_STATUS;
		return true;
	}

	return false;
}

static int cmd_envdaq_uptime(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Uptime: %lld ms", k_uptime_get());
	return 0;
}

static int cmd_envdaq_version(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "EnvDAQ Shell v1.0.0");
	shell_print(sh, "Build: %s %s", __DATE__, __TIME__);
	return 0;
}

static int cmd_envdaq_echo(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_help(sh);
		return -EINVAL;
	}

	for (size_t i = 1; i < argc; i++) {
		shell_print(sh, "%s", argv[i]);
	}

	return 0;
}

static int cmd_envdaq_help(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Available commands:");
	shell_print(sh, "  envdaq uptime   - Show system uptime");
	shell_print(sh, "  envdaq version  - Show version info");
	shell_print(sh, "  envdaq echo     - Echo back arguments");
	shell_print(sh, "  envdaq relay    - Control relay_a and relay_b");
	shell_print(sh, "  envdaq battery  - Read battery voltage");
	shell_print(sh, "  envdaq help     - Show this help");
	shell_print(sh, "");
	shell_print(sh, "Type 'help' for Zephyr shell built-in commands.");
	return 0;
}

static int cmd_envdaq_relay(const struct shell *sh, size_t argc, char **argv)
{
	enum relay_action action;
	enum relay_target target = RELAY_TARGET_ALL;
	const struct gpio_dt_spec *relays[] = { &relay_a, &relay_b };
	const char *names[] = { "relay_a", "relay_b" };
	int ret;

	if (argc < 2) {
		shell_help(sh);
		return -EINVAL;
	}

	if (!relay_parse_action(argv[1], &action)) {
		shell_error(sh, "Usage: envdaq relay <on|off|toggle|status> [a|b|all]");
		return -EINVAL;
	}

	if (argc == 3 && !relay_parse_target(argv[2], &target)) {
		shell_error(sh, "Usage: envdaq relay <on|off|toggle|status> [a|b|all]");
		return -EINVAL;
	}

	if (argc > 3) {
		shell_error(sh, "Usage: envdaq relay <on|off|toggle|status> [a|b|all]");
		return -EINVAL;
	}

	if (action == RELAY_ACTION_STATUS) {
		if (target == RELAY_TARGET_A || target == RELAY_TARGET_ALL) {
			relay_print_status(names[0], relay_state_a, sh);
		}

		if (target == RELAY_TARGET_B || target == RELAY_TARGET_ALL) {
			relay_print_status(names[1], relay_state_b, sh);
		}
		return 0;
	}

	if (target == RELAY_TARGET_A || target == RELAY_TARGET_ALL) {
		ret = (action == RELAY_ACTION_TOGGLE) ?
		      relay_toggle_one(relays[0]) :
		      relay_set_one(relays[0], action == RELAY_ACTION_ON);
		if (ret < 0) {
			shell_error(sh, "%s control failed (%d)", names[0], ret);
			return ret;
		}
		relay_state_a = (action == RELAY_ACTION_TOGGLE) ? !relay_state_a :
			action == RELAY_ACTION_ON;
		shell_print(sh, "%s -> %s", names[0],
			    action == RELAY_ACTION_ON ? "on" :
			    action == RELAY_ACTION_OFF ? "off" : "toggled");
	}

	if (target == RELAY_TARGET_B || target == RELAY_TARGET_ALL) {
		ret = (action == RELAY_ACTION_TOGGLE) ?
		      relay_toggle_one(relays[1]) :
		      relay_set_one(relays[1], action == RELAY_ACTION_ON);
		if (ret < 0) {
			shell_error(sh, "%s control failed (%d)", names[1], ret);
			return ret;
		}
		relay_state_b = (action == RELAY_ACTION_TOGGLE) ? !relay_state_b :
			action == RELAY_ACTION_ON;
		shell_print(sh, "%s -> %s", names[1],
			    action == RELAY_ACTION_ON ? "on" :
			    action == RELAY_ACTION_OFF ? "off" : "toggled");
	}

	return 0;
}

static int cmd_envdaq_battery(const struct shell *sh, size_t argc, char **argv)
{
	int ret;
	int32_t voltage_mv;
	int32_t adc_input_mv;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ret = read_battery_voltage(&voltage_mv, &adc_input_mv);
	if (ret < 0) {
		shell_error(sh, "Failed to read battery voltage: %d", ret);
		return ret;
	}

	shell_print(sh, "Battery voltage: %d mV", voltage_mv);
	shell_print(sh, "ADC input: %d mV", adc_input_mv);
	shell_print(sh, "Cal point 1: ADC=%d mV, Battery=%d mV",
		CAL_POINT1_ADC_MV, CAL_POINT1_BATTERY_MV);
	shell_print(sh, "Cal point 2: ADC=%d mV, Battery=%d mV",
		CAL_POINT2_ADC_MV, CAL_POINT2_BATTERY_MV);
	shell_print(sh, "Slope: %d/%d", CAL_SLOPE_NUM, CAL_SLOPE_DEN);
	shell_print(sh, "Offset: %d mV", CAL_OFFSET_MV);
	shell_print(sh, "Samples: %d (oversampling)", ADC_SAMPLE_COUNT);
	shell_print(sh, "Formula: battery_mv = (adc_mv * %d) / %d + %d",
		CAL_SLOPE_NUM, CAL_SLOPE_DEN, CAL_OFFSET_MV);

	return 0;
}

/* ============================================================
 * Shell Command Registration
 * ============================================================ */

SHELL_STATIC_SUBCMD_SET_CREATE(sub_envdaq,
	SHELL_CMD_ARG(uptime, NULL, "Show system uptime", cmd_envdaq_uptime, 1, 0),
	SHELL_CMD_ARG(version, NULL, "Show version info", cmd_envdaq_version, 1, 0),
	SHELL_CMD_ARG(echo, NULL, "Echo back arguments", cmd_envdaq_echo, 2, 9),
	SHELL_CMD_ARG(relay, NULL, "Control relay outputs", cmd_envdaq_relay, 2, 3),
	SHELL_CMD_ARG(battery, NULL, "Read battery voltage", cmd_envdaq_battery, 1, 0),
	SHELL_CMD_ARG(help, NULL, "Show this help", cmd_envdaq_help, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(envdaq, &sub_envdaq, "EnvDAQ Shell commands", NULL);

/* ============================================================
 * Main
 * ============================================================ */

static int envdaq_post_boot_init(void)
{
	int ret;

	printk("========================================\n");
	printk("  EnvDAQ Shell\n");
	printk("  Type 'envdaq help' for available commands\n");
	printk("========================================\n");

	ret = relay_init();
	if (ret < 0) {
		printk("ERROR: Relay initialization failed: %d\n", ret);
	} else {
		printk("Relays initialized successfully\n");
	}

	if (device_is_ready(adc_dev)) {
		printk("ADC initialized successfully\n");
	} else {
		printk("WARNING: ADC device not ready\n");
	}

	return 0;
}

SYS_INIT(envdaq_post_boot_init, POST_KERNEL, 99);

int main(void)
{
	while (1) {
		k_sleep(K_SECONDS(10));
	}

	return 0;
}