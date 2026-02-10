#/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
# * SVEA Note.
# *
# * Part of the SVEA Low‑Level Interface (Zephyr) application.
# * Author: Nils Kiefer
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "control.h"
#include "imu_sensor.h"
#include "ina3221_sensor.h"
#include "mcp9600.h"
#include "ina2xx_common.h"
#include "rc_input.h"
#include "ros_iface.h"
#include "wheel_enc.h"
#include <stdio.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define WDT_TIMEOUT_MS 2000

static const struct device *wdt;
static int wdt_channel_id = -1;

static void app_wdt_setup(void) {
    wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));
    if (!device_is_ready(wdt)) {
        LOG_ERR("Watchdog device not ready!");
        return;
    }

    struct wdt_timeout_cfg cfg = {
        .window = {
            .min = 0,
            .max = WDT_TIMEOUT_MS,
        },
        .callback = NULL, // NULL = reset on timeout
        .flags = WDT_FLAG_RESET_SOC};

    wdt_channel_id = wdt_install_timeout(wdt, &cfg);
    if (wdt_channel_id < 0) {
        LOG_ERR("Failed to install watchdog timeout");
        return;
    }
    if (wdt_setup(wdt, 0) < 0) { // This line is correct: calls Zephyr's wdt_setup
        LOG_ERR("Failed to setup watchdog");
    } else {
        LOG_INF("Watchdog started (%d ms timeout)", WDT_TIMEOUT_MS);
    }
}

int main(void) {
    LOG_INF("SVEA LLI starting");
    k_sleep(K_SECONDS(1)); // Allow time for logging to initialize
    app_wdt_setup();
    k_sleep(K_SECONDS(1)); // Allow time for watchdog setup

    rc_input_init();
    servo_init();

    if (ina3221_sensor_init() != 0) {
        LOG_WRN("INA3221 sensor init failed");
    }
    imu_sensor_start();
    wheel_enc_init();

    ros_iface_init();

    while (1) {
        if (wdt_channel_id >= 0) {
            wdt_feed(wdt, wdt_channel_id);
        }
        k_sleep(K_MSEC(500)); // Feed often enough to avoid reset
    }

    return 0;
}
