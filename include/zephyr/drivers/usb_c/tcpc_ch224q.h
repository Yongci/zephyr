/*
 * Copyright (c) 2026 Yongci
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_USB_C_TCPC_CH224Q_H_
#define ZEPHYR_INCLUDE_DRIVERS_USB_C_TCPC_CH224Q_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get current Power Good state of CH224Q
 *
 * @param dev Pointer to the device structure for the CH224Q instance
 * @return true if power is good (PG pin low), false otherwise
 */
bool ch224q_get_power_good(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_USB_C_TCPC_CH224Q_H_ */
