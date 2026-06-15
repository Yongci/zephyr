/*
 * Copyright (c) 2026 Yongci
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#if defined(CONFIG_MCUBOOT)

#define ENVDAQ_GPIO_NODE DT_NODELABEL(gpio0)
#define BUCK_EN_PIN 19
#define USB_MUX_SEL_PIN 20


static int envdaq_mcuboot_power_sequence_init(void)
{
	const struct device *gpio = DEVICE_DT_GET(ENVDAQ_GPIO_NODE);
	int ret;

	if (!device_is_ready(gpio)) {
		return -ENODEV;
	}

	ret = gpio_pin_configure(gpio, BUCK_EN_PIN, GPIO_OUTPUT_ACTIVE);
	if (ret != 0) {
		return ret;
	}

	k_busy_wait(5000);

	ret = gpio_pin_configure(gpio, USB_MUX_SEL_PIN, GPIO_OUTPUT_ACTIVE);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

SYS_INIT(envdaq_mcuboot_power_sequence_init, PRE_KERNEL_2,
	 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

#endif /* CONFIG_MCUBOOT */
