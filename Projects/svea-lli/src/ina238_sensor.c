/*
 * Part of the SVEA Low-Level Interface (Zephyr) application.
 */

#include "ina238_sensor.h"

#include "ros_iface.h"

#include <math.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ina238_sensor, LOG_LEVEL_INF);

#define INA238_THREAD_STACK_SIZE 2048
#define INA238_THREAD_PRIORITY   6
#define INA238_PUBLISH_PERIOD_MS 250

struct ina238_sensor_ctx {
    const struct device *dev;
    rcl_publisher_t *pub;
    const char *thread_name;
    std_msgs__msg__Float32MultiArray msg;
    bool msg_ready;
};

static int ina238_prepare_msg(struct ina238_sensor_ctx *ctx)
{
    if (ctx->msg_ready) {
        return 0;
    }

    if (!std_msgs__msg__Float32MultiArray__init(&ctx->msg)) {
        LOG_ERR("Failed to init INA238 message (%s)", ctx->thread_name);
        return -ENOMEM;
    }
    if (!rosidl_runtime_c__float32__Sequence__init(&ctx->msg.data, 5)) {
        LOG_ERR("Failed to init INA238 sequence (%s)", ctx->thread_name);
        std_msgs__msg__Float32MultiArray__fini(&ctx->msg);
        return -ENOMEM;
    }
    memset(ctx->msg.data.data, 0, sizeof(float) * ctx->msg.data.size);
    ctx->msg_ready = true;
    return 0;
}

static void ina238_publish_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct ina238_sensor_ctx *ctx = p1;
    if (ctx == NULL) {
        return;
    }

    if (ina238_prepare_msg(ctx) != 0) {
        LOG_ERR("INA238 %s publisher disabled", ctx->thread_name);
        return;
    }

    if (!device_is_ready(ctx->dev)) {
        LOG_WRN("INA238 %s device not ready", ctx->thread_name);
    }

    while (true) {
        float voltage = NAN;
        float current = NAN;
        float power = NAN;

        if (ctx->dev && device_is_ready(ctx->dev)) {
            if (sensor_sample_fetch(ctx->dev) == 0) {
                struct sensor_value val;
                if (sensor_channel_get(ctx->dev, SENSOR_CHAN_VOLTAGE, &val) == 0) {
                    voltage = (float)sensor_value_to_double(&val);
                }
                if (sensor_channel_get(ctx->dev, SENSOR_CHAN_CURRENT, &val) == 0) {
                    current = (float)sensor_value_to_double(&val);
                }
                if (sensor_channel_get(ctx->dev, SENSOR_CHAN_POWER, &val) == 0) {
                    power = (float)sensor_value_to_double(&val);
                }
            } else {
                LOG_DBG("INA238 %s sample fetch failed", ctx->thread_name);
            }
        }

        uint64_t now_ms = ros_iface_epoch_millis();
        uint32_t now_ns = (uint32_t)(ros_iface_epoch_nanos() % 1000000000ULL);

        ctx->msg.data.data[0] = (float)(now_ms / 1000ULL);
        ctx->msg.data.data[1] = (float)now_ns;
        ctx->msg.data.data[2] = voltage;
        ctx->msg.data.data[3] = current;
        ctx->msg.data.data[4] = power;

        if (ros_initialized) {
            (void)ros_publish_try(ctx->pub, &ctx->msg);
        }

        k_sleep(K_MSEC(INA238_PUBLISH_PERIOD_MS));
    }
}

#if DT_NODE_HAS_STATUS(DT_NODELABEL(ina238_aux), okay)
#define INA238_AUX_ENABLED 1
static struct ina238_sensor_ctx ina238_aux_ctx = {
    .dev = DEVICE_DT_GET(DT_NODELABEL(ina238_aux)),
    .pub = &ina238_aux_pub,
    .thread_name = "ina238_aux",
};
static K_THREAD_STACK_DEFINE(ina238_aux_stack, INA238_THREAD_STACK_SIZE);
static struct k_thread ina238_aux_thread;
#else
#define INA238_AUX_ENABLED 0
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(ina238_fan), okay)
#define INA238_FAN_ENABLED 1
static struct ina238_sensor_ctx ina238_fan_ctx = {
    .dev = DEVICE_DT_GET(DT_NODELABEL(ina238_fan)),
    .pub = &ina238_fan_pub,
    .thread_name = "ina238_fan",
};
static K_THREAD_STACK_DEFINE(ina238_fan_stack, INA238_THREAD_STACK_SIZE);
static struct k_thread ina238_fan_thread;
#else
#define INA238_FAN_ENABLED 0
#endif

int ina238_sensor_init(void)
{
    int started = 0;

#if INA238_AUX_ENABLED
    k_thread_create(&ina238_aux_thread, ina238_aux_stack, INA238_THREAD_STACK_SIZE,
                    ina238_publish_thread, &ina238_aux_ctx, NULL, NULL,
                    INA238_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&ina238_aux_thread, ina238_aux_ctx.thread_name);
    started++;
#endif

#if INA238_FAN_ENABLED
    k_thread_create(&ina238_fan_thread, ina238_fan_stack, INA238_THREAD_STACK_SIZE,
                    ina238_publish_thread, &ina238_fan_ctx, NULL, NULL,
                    INA238_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&ina238_fan_thread, ina238_fan_ctx.thread_name);
    started++;
#endif

    if (started == 0) {
        LOG_INF("No INA238 nodes enabled");
    }

    return 0;
}
