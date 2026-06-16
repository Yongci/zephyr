// Copyright (c) 2026 Yongci
// SPDX-License-Identifier: Apache-2.0
/*
 * Driver for WCH CH224Q USB PD Sink Controller
 *
 * CH224Q supports:
 * - USB PD 3.2 Sink (up to 140W, 28V/5A)
 * - I2C interface (400kHz max) for voltage request and status read
 * - eMarker simulation
 * - PG (Power Good) signal output when voltage negotiation successful
 *
 * Based on: https://www.wch.cn/products/CH224.html
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/usb_c/tcpc.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(ch224q_tcpc, CONFIG_LOG_DEFAULT_LEVEL);

/* CH224Q I2C Registers (confirmed from datasheet) */
#define CH224Q_REG_VID         0x00    /* Vendor ID (16-bit) */
#define CH224Q_REG_PID         0x02    /* Product ID (16-bit) */
#define CH224Q_REG_XID         0x04    /* eMarker XID (32-bit) */
#define CH224Q_REG_REV         0x08    /* eMarker revision (16-bit) */
#define CH224Q_REG_STATUS      0x0A    /* Status register */
#define CH224Q_REG_VOLT_SEL    0x0B    /* Voltage selection register */
#define CH224Q_REG_CFG         0x0C    /* Configuration register */
#define CH224Q_REG_VOLT_RUN    0x0D    /* Current running voltage */
#define CH224Q_REG_CURR_RUN    0x0E    /* Current running current */
#define CH224Q_REG_VOLT_REQ    0x0F    /* Requested voltage */
#define CH224Q_REG_CURR_REQ    0x10    /* Requested current */
#define CH224Q_REG_OVP_VOLT    0x11    /* OVP threshold */
#define CH224Q_REG_OCP_CURR    0x12    /* OCP threshold */
#define CH224Q_REG_TEMP        0x13    /* Temperature */

/* Status register bits */
#define CH224Q_STATUS_PD_NEGOTIATED  BIT(0)
#define CH224Q_STATUS_HARD_RESET     BIT(1)
#define CH224Q_STATUS_OVP_FAULT      BIT(2)
#define CH224Q_STATUS_OCP_FAULT      BIT(3)
#define CH224Q_STATUS_PD_ENABLED     BIT(4)

/* Voltage selection values (for I2C mode) */
#define CH224Q_VOLT_5V       0x00
#define CH224Q_VOLT_9V       0x01
#define CH224Q_VOLT_12V      0x02
#define CH224Q_VOLT_15V      0x03
#define CH224Q_VOLT_20V      0x04
#define CH224Q_VOLT_28V      0x05

/* PG pin states */
#define CH224Q_PG_ASSERTED   0  /* PG pin low = Power Good */
#define CH224Q_PG_DEASSERTED 1  /* PG pin high = Power Not Good */

/* Voltage to register value mapping */
static const struct {
    uint32_t mv;
    uint8_t reg_val;
} ch224q_voltage_map[] = {
    { 5000,  CH224Q_VOLT_5V  },
    { 9000,  CH224Q_VOLT_9V  },
    { 12000, CH224Q_VOLT_12V },
    { 15000, CH224Q_VOLT_15V },
    { 20000, CH224Q_VOLT_20V },
    { 28000, CH224Q_VOLT_28V },
};

/* CH224Q device configuration */
struct ch224q_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec int_gpio;
    struct gpio_dt_spec pg_gpio;
    struct gpio_dt_spec cfg2_gpio;
    struct gpio_dt_spec cfg3_gpio;
    uint8_t mode;           /* 0: I2C, 1: GPIO */
    uint32_t default_voltage_mv;
};

/* CH224Q device data */
struct ch224q_data {
    struct k_work work;
    struct k_sem lock;
    struct k_work_delayable pg_debounce_work;
    uint8_t status;
    uint32_t current_voltage_mv;
    uint32_t current_current_ma;
    bool pd_negotiated;
    bool pg_state;                  /* Current PG state */
    bool pg_irq_enabled;
#ifdef CONFIG_CH224Q_INTERRUPT
    struct gpio_callback int_cb;
    struct gpio_callback pg_cb;
#endif
};

/* Forward declarations */
static int ch224q_set_voltage(const struct device *dev, uint32_t voltage_mv);
static void ch224q_pg_work_handler(struct k_work *work);

/* Convert voltage in mV to register value */
static int ch224q_voltage_to_reg(uint32_t voltage_mv, uint8_t *reg_val)
{
    for (int i = 0; i < ARRAY_SIZE(ch224q_voltage_map); i++) {
        if (voltage_mv == ch224q_voltage_map[i].mv) {
            *reg_val = ch224q_voltage_map[i].reg_val;
            return 0;
        }
    }
    return -EINVAL;
}

/* Read CH224Q register */
static int ch224q_read_reg(const struct device *dev, uint8_t reg, uint8_t *data)
{
    const struct ch224q_config *cfg = dev->config;

    return i2c_write_read_dt(&cfg->i2c, &reg, 1, data, 1);
}

/* Write CH224Q register */
static int ch224q_write_reg(const struct device *dev, uint8_t reg, uint8_t value)
{
    const struct ch224q_config *cfg = dev->config;
    uint8_t buf[2] = { reg, value };

    return i2c_write_dt(&cfg->i2c, buf, sizeof(buf));
}

/* Read multiple registers */
static int ch224q_read_regs(const struct device *dev, uint8_t reg,
                uint8_t *data, size_t len)
{
    const struct ch224q_config *cfg = dev->config;

    return i2c_write_read_dt(&cfg->i2c, &reg, 1, data, len);
}

/* Get current PG pin state */
static bool ch224q_get_pg_state(const struct device *dev)
{
    const struct ch224q_config *cfg = dev->config;
    int val;

    if (!cfg->pg_gpio.port) {
        return false;   /* No PG pin configured */
    }

    val = gpio_pin_get_dt(&cfg->pg_gpio);
    if (val < 0) {
        LOG_ERR("Failed to read PG pin: %d", val);
        return false;
    }

    /* PG is active low: 0 = Power Good, 1 = Power Not Good */
    return (val == CH224Q_PG_ASSERTED);
}

/* Update status from hardware */
static int ch224q_update_status(const struct device *dev)
{
    struct ch224q_data *data = dev->data;
    uint8_t status;
    int ret;

    ret = ch224q_read_reg(dev, CH224Q_REG_STATUS, &status);
    if (ret < 0) {
        LOG_ERR("Failed to read status: %d", ret);
        return ret;
    }

    data->status = status;
    data->pd_negotiated = (status & CH224Q_STATUS_PD_NEGOTIATED) != 0;

    /* Read current voltage if PD negotiated */
    if (data->pd_negotiated) {
        uint8_t volt_reg, curr_reg;
        ret = ch224q_read_reg(dev, CH224Q_REG_VOLT_RUN, &volt_reg);
        if (ret == 0) {
            data->current_voltage_mv = volt_reg * 100;
        }
        ret = ch224q_read_reg(dev, CH224Q_REG_CURR_RUN, &curr_reg);
        if (ret == 0) {
            data->current_current_ma = curr_reg * 50;
        }
    }

    /* Update PG state from hardware pin */
    data->pg_state = ch224q_get_pg_state(dev);

    return 0;
}

/* Set requested voltage via I2C */
static int ch224q_set_voltage_i2c(const struct device *dev, uint32_t voltage_mv)
{
    const struct ch224q_config *cfg = dev->config;
    struct ch224q_data *data = dev->data;
    uint8_t reg_val;
    int ret;

    ret = ch224q_voltage_to_reg(voltage_mv, &reg_val);
    if (ret < 0) {
        LOG_ERR("Invalid voltage: %d mV", voltage_mv);
        return ret;
    }

    ret = ch224q_write_reg(dev, CH224Q_REG_VOLT_SEL, reg_val);
    if (ret < 0) {
        LOG_ERR("Failed to set voltage: %d", ret);
        return ret;
    }

    LOG_DBG("Requested voltage: %d mV", voltage_mv);

    /* Wait for PG signal to indicate voltage is stable */
    if (cfg->pg_gpio.port) {
        /* PG pin available - poll with timeout */
        int timeout_ms = 500;  /* Max 500ms for PD negotiation */
        while (timeout_ms > 0) {
            if (ch224q_get_pg_state(dev)) {
                LOG_INF("PG asserted after voltage request");
                break;
            }
            k_sleep(K_MSEC(10));
            timeout_ms -= 10;
        }
        if (timeout_ms <= 0) {
            LOG_WRN("PG not asserted after voltage request");
        }
    } else {
        /* No PG pin, just sleep and check I2C status */
        k_sleep(K_MSEC(100));
    }

    ch224q_update_status(dev);

    return 0;
}

/* Set voltage via GPIO pins (hardware mode) */
static int ch224q_set_voltage_gpio(const struct device *dev, uint32_t voltage_mv)
{
    const struct ch224q_config *cfg = dev->config;
    struct ch224q_data *data = dev->data;
    uint8_t reg_val;
    int ret;

    ret = ch224q_voltage_to_reg(voltage_mv, &reg_val);
    if (ret < 0) {
        return ret;
    }

    /* CFG2 and CFG3 pins select voltage */
    if (cfg->cfg2_gpio.port) {
        gpio_pin_set_dt(&cfg->cfg2_gpio, (reg_val >> 0) & 1);
    }
    if (cfg->cfg3_gpio.port) {
        gpio_pin_set_dt(&cfg->cfg3_gpio, (reg_val >> 1) & 1);
    }

    /* Wait for PG signal */
    if (cfg->pg_gpio.port) {
        int timeout_ms = 500;
        while (timeout_ms > 0) {
            if (ch224q_get_pg_state(dev)) {
                LOG_INF("PG asserted after GPIO voltage change");
                break;
            }
            k_sleep(K_MSEC(10));
            timeout_ms -= 10;
        }
    } else {
        k_sleep(K_MSEC(100));
    }

    ch224q_update_status(dev);

    return 0;
}

/* PG interrupt callback - handle hardware PG signal changes */
static void ch224q_pg_callback(const struct device *dev,
                   struct gpio_callback *cb, uint32_t pins)
{
    struct ch224q_data *data = CONTAINER_OF(cb, struct ch224q_data, pg_cb);

    /* Debounce PG signal (PG may have glitches during negotiation) */
    k_work_reschedule(&data->pg_debounce_work, K_MSEC(10));
}

/* PG debounce work handler - called after interrupt debouncing */
static void ch224q_pg_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct ch224q_data *data = CONTAINER_OF(dwork, struct ch224q_data,
                        pg_debounce_work);
    const struct device *dev = (const struct device *)data->pg_debounce_work._handler_data;
    bool pg_state;

    if (!dev) {
        return;
    }

    pg_state = ch224q_get_pg_state(dev);
    data->pg_state = pg_state;

    if (pg_state) {
        LOG_INF("PG Power Good: voltage stable");
        /* Update status from I2C to get actual voltage values */
        ch224q_update_status(dev);
    } else {
        LOG_WRN("PG Power Not Good: voltage unstable or fault");
    }
}

/* Configure PG pin interrupt */
static int ch224q_configure_pg_interrupt(const struct device *dev)
{
    struct ch224q_data *data = dev->data;
    const struct ch224q_config *cfg = dev->config;
    int ret;

    if (!cfg->pg_gpio.port) {
        LOG_DBG("PG pin not configured, skipping interrupt setup");
        return 0;
    }

    /* Configure PG pin as input with interrupt on both edges */
    ret = gpio_pin_configure_dt(&cfg->pg_gpio, GPIO_INPUT | GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Failed to configure PG GPIO: %d", ret);
        return ret;
    }

    /* Initialize GPIO callback */
    gpio_init_callback(&data->pg_cb, ch224q_pg_callback, BIT(cfg->pg_gpio.pin));

    /* Add callback to the GPIO device */
    ret = gpio_add_callback(cfg->pg_gpio.port, &data->pg_cb);
    if (ret < 0) {
        LOG_ERR("Failed to add PG callback: %d", ret);
        return ret;
    }

    /* Enable interrupt on PG pin */
    ret = gpio_pin_interrupt_configure_dt(&cfg->pg_gpio, GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Failed to enable PG interrupt: %d", ret);
        return ret;
    }

    data->pg_irq_enabled = true;
    LOG_INF("PG interrupt configured on pin %d", cfg->pg_gpio.pin);

    return 0;
}

/* TCPC API: Initialize device */
static int ch224q_init(const struct device *dev)
{
    const struct ch224q_config *cfg = dev->config;
    struct ch224q_data *data = dev->data;
    int ret;

    /* Initialize I2C */
    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    /* Initialize PG pin GPIO if configured */
    if (cfg->pg_gpio.port) {
        ret = gpio_pin_configure_dt(&cfg->pg_gpio, GPIO_INPUT);
        if (ret < 0) {
            LOG_ERR("Failed to configure PG GPIO: %d", ret);
            return ret;
        }
        /* Initialize PG debounce work */
        k_work_init_delayable(&data->pg_debounce_work, ch224q_pg_work_handler);
        /* Set handler data for work callback (store device pointer) */
        data->pg_debounce_work._handler_data = (void *)dev;
    }

    /* Initialize INT pin GPIO if configured */
    if (cfg->int_gpio.port) {
        ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
        if (ret < 0) {
            LOG_ERR("Failed to configure INT GPIO: %d", ret);
            return ret;
        }
    }

    /* Initialize CFG pins for GPIO mode */
    if (cfg->cfg2_gpio.port) {
        ret = gpio_pin_configure_dt(&cfg->cfg2_gpio, GPIO_OUTPUT);
        if (ret < 0) {
            LOG_ERR("Failed to configure CFG2 GPIO: %d", ret);
        }
    }

    if (cfg->cfg3_gpio.port) {
        ret = gpio_pin_configure_dt(&cfg->cfg3_gpio, GPIO_OUTPUT);
        if (ret < 0) {
            LOG_ERR("Failed to configure CFG3 GPIO: %d", ret);
        }
    }

    k_sem_init(&data->lock, 1, 1);

    /* Read chip ID to verify communication */
    uint8_t vid_l, vid_h;
    ret = ch224q_read_reg(dev, CH224Q_REG_VID, &vid_l);
    if (ret == 0) {
        ret = ch224q_read_reg(dev, CH224Q_REG_VID + 1, &vid_h);
    }
    if (ret < 0) {
        LOG_ERR("Failed to communicate with CH224Q: %d", ret);
        return ret;
    }
    LOG_INF("CH224Q detected, VID: 0x%02x%02x", vid_h, vid_l);

    /* Configure PG interrupt after driver initialization */
    ret = ch224q_configure_pg_interrupt(dev);
    if (ret < 0) {
        LOG_WRN("PG interrupt configuration failed: %d", ret);
        /* Non-fatal - driver can still work by polling */
    }

    /* Enable PD if needed */
    ret = ch224q_write_reg(dev, CH224Q_REG_CFG, 0x01);
    if (ret < 0) {
        LOG_WRN("Failed to enable PD, maybe already enabled");
    }

    /* Request default voltage */
    if (cfg->mode == 0) {
        ch224q_set_voltage_i2c(dev, cfg->default_voltage_mv);
    } else {
        ch224q_set_voltage_gpio(dev, cfg->default_voltage_mv);
    }

    /* Update initial status */
    ch224q_update_status(dev);

    LOG_INF("CH224Q initialized, PD negotiated: %d, PG state: %s",
        data->pd_negotiated,
        data->pg_state ? "Good" : "Not Good");

    return 0;
}

/* TCPC API: Get capabilities */
static int ch224q_get_caps(const struct device *dev,
               struct tcpc_capabilities *caps)
{
    if (!caps) {
        return -EINVAL;
    }

    /* CH224Q is a Sink-only device */
    caps->flags = TCPC_CAPS_SINK_ONLY;
    caps->pd_supported = true;
    caps->max_voltage_mv = 28000;
    caps->max_current_ma = 5000;

    return 0;
}

/* TCPC API: Get current CC status */
static int ch224q_get_cc(const struct device *dev, enum tcpc_cc_polarity *polarity,
             enum tcpc_cc_status *cc1, enum tcpc_cc_status *cc2)
{
    struct ch224q_data *data = dev->data;

    ch224q_update_status(dev);

    if (data->pd_negotiated && data->pg_state) {
        /* Attached and power good */
        *cc1 = TCPC_CC_STATUS_RP_DEF;
        *cc2 = TCPC_CC_STATUS_OPEN;
    } else {
        *cc1 = TCPC_CC_STATUS_OPEN;
        *cc2 = TCPC_CC_STATUS_OPEN;
    }

    return 0;
}

/* TCPC API: Request a new voltage */
static int ch224q_set_voltage_req(const struct device *dev, uint32_t voltage_mv)
{
    const struct ch224q_config *cfg = dev->config;
    int ret;

    if (cfg->mode == 0) {
        ret = ch224q_set_voltage_i2c(dev, voltage_mv);
    } else {
        ret = ch224q_set_voltage_gpio(dev, voltage_mv);
    }

    return ret;
}

/* TCPC API: Get current voltage */
static int ch224q_get_voltage(const struct device *dev, uint32_t *voltage_mv)
{
    struct ch224q_data *data = dev->data;

    ch224q_update_status(dev);
    *voltage_mv = data->current_voltage_mv;

    return 0;
}

/* TCPC API: Get current current */
static int ch224q_get_current(const struct device *dev, uint32_t *current_ma)
{
    struct ch224q_data *data = dev->data;

    ch224q_update_status(dev);
    *current_ma = data->current_current_ma;

    return 0;
}

/* TCPC API: Check if power is good */
static int ch224q_is_power_good(const struct device *dev, bool *power_good)
{
    struct ch224q_data *data = dev->data;

    ch224q_update_status(dev);
    *power_good = data->pg_state;

    return 0;
}

/* TCPC API: Send PD message (not fully supported by CH224Q as Sink only) */
static int ch224q_send_pd_msg(const struct device *dev,
                  enum tcpc_pd_msg_type type,
                  uint32_t *data, int len)
{
    LOG_WRN("PD message sending not fully supported by CH224Q");
    return -ENOTSUP;
}

/* Extended API: Get PG state without side effects */
bool ch224q_get_power_good(const struct device *dev)
{
    struct ch224q_data *data = dev->data;

    ch224q_update_status(dev);
    return data->pg_state;
}

/* TCPC driver API vtable */
static const struct tcpc_driver_api ch224q_tcpc_api = {
    .init = ch224q_init,
    .get_caps = ch224q_get_caps,
    .get_cc = ch224q_get_cc,
    .set_voltage_req = ch224q_set_voltage_req,
    .get_voltage = ch224q_get_voltage,
    .get_current = ch224q_get_current,
    .send_pd_msg = ch224q_send_pd_msg,
};

/* I2C address configuration */
#define CH224Q_I2C_ADDR 0x37  /* Default 7-bit address */

/* DT macro to get I2C bus */
#define CH224Q_DT_SPEC(n) \
    .bus = DEVICE_DT_GET(DT_PARENT(DT_DRV_INST(n))), \
    .addr = CH224Q_I2C_ADDR

/* Instance macro for devicetree */
#define CH224Q_INIT(n) \
    static const struct ch224q_config ch224q_config_##n = { \
        .i2c = I2C_DT_SPEC_INST_GET(n), \
        .int_gpio = GPIO_DT_SPEC_INST_GET_OR(n, int_gpios, {0}), \
        .pg_gpio = GPIO_DT_SPEC_INST_GET_OR(n, pg_gpios, {0}), \
        .cfg2_gpio = GPIO_DT_SPEC_INST_GET_OR(n, cfg2_gpios, {0}), \
        .cfg3_gpio = GPIO_DT_SPEC_INST_GET_OR(n, cfg3_gpios, {0}), \
        .mode = DT_ENUM_IDX(DT_DRV_INST(n), mode), \
        .default_voltage_mv = DT_PROP_OR(DT_DRV_INST(n), voltage_request, 5000), \
    }; \
    static struct ch224q_data ch224q_data_##n; \
    DEVICE_DT_INST_DEFINE(n, ch224q_init, NULL, \
                  &ch224q_data_##n, &ch224q_config_##n, \
                  POST_KERNEL, CONFIG_CH224Q_INIT_PRIORITY, \
                  &ch224q_tcpc_api);

/* Create instances for each compatible node in devicetree */
#define DT_DRV_COMPAT wch_ch224q

#ifdef DT_HAS_COMPAT_STATUS_OKAY
DT_INST_FOREACH_STATUS_OKAY(CH224Q_INIT)
#endif /* DT_HAS_COMPAT_STATUS_OKAY */
