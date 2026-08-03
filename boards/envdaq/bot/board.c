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
#define BUCK_PG_PIN 18
#define USB_MUX_SEL_PIN 20
#define USB_MUX_EN_PIN 21

static int envdaq_mcuboot_power_sequence_init(void)
{
	const struct device *gpio = DEVICE_DT_GET(ENVDAQ_GPIO_NODE);
	int ret;

	if (!device_is_ready(gpio)) {
		return -ENODEV;
	}

	/*
	 * Step 1: Enable the BUCK converter
	 * The BUCK must be enabled before the USB mux to ensure stable power delivery
	 */
	ret = gpio_pin_configure(gpio, BUCK_EN_PIN, GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH);
	if (ret != 0) {
		return ret;
	}

	/*
	 * Step 2: Configure PG (Power Good) pin as input
	 * PG is an open-drain output from the BUCK IC with an external pull-up resistor
	 * Active-low: LOW = power fault, HIGH = power good
	 * Internal pull-up is enabled as a redundancy
	 */
	ret = gpio_pin_configure(gpio, BUCK_PG_PIN, GPIO_INPUT | GPIO_PULL_UP);
	if (ret != 0) {
		return ret;
	}

	/*
	 * Step 3: Wait for power to stabilize
	 * PG will go HIGH when the BUCK output is within the normal range
	 * Note: k_msleep() is not available in PRE_KERNEL phase, use k_busy_wait() instead
	 */
	int timeout = 1000; /* 1 second timeout */
	while (timeout-- > 0) {
		if (gpio_pin_get(gpio, BUCK_PG_PIN) == 1) {
			break; /* Power good */
		}
		k_busy_wait(1000); /* Busy wait 1ms */
	}

	if (timeout <= 0) {
		/* Timeout occurred, but continue anyway (power may be unstable) */
	}

	/* Additional delay to ensure power is fully stable */
	k_busy_wait(500);

	/*
	 * Step 4: Configure USB MUX
	 * EN=0: USB connected, EN=1: USB disconnected
	 * SEL=0: select ch224q (default), SEL=1: select esp32c6-mini-1u
	 * Initialize with EN=0, SEL=0 to keep USB connected and select default channel
	 */
	ret = gpio_pin_configure(gpio, USB_MUX_EN_PIN, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
	if (ret != 0) {
		return ret;
	}

	ret = gpio_pin_configure(gpio, USB_MUX_SEL_PIN, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

/*
 * Initialize the power sequence during PRE_KERNEL_2 phase
 * This runs after MCUBoot and before the kernel starts
 */
SYS_INIT(envdaq_mcuboot_power_sequence_init, PRE_KERNEL_2,
	 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

#endif /* CONFIG_MCUBOOT */