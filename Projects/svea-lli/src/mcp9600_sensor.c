/*
 * Part of the SVEA Low-Level Interface (Zephyr) application.
 */

#include "mcp9600_sensor.h"

#include "mcp9600.h"
#include "ros_iface.h"

#include <math.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mcp9600_sensor, LOG_LEVEL_INF);

#define MCP9600_THREAD_STACK_SIZE 1536
#define MCP9600_THREAD_PRIORITY   6
#define MCP9600_PUBLISH_PERIOD_MS 200

#define MCP9600_REG_TEMP_HOT 0x00

#if DT_NODE_HAS_STATUS(DT_NODELABEL(mcp9600), okay)
#define MCP9600_HAS_NODE 1
#define MCP9600_NODE DT_NODELABEL(mcp9600)
static const struct device *const mcp9600_dev = DEVICE_DT_GET(MCP9600_NODE);
static const struct i2c_dt_spec mcp9600_bus = I2C_DT_SPEC_GET(MCP9600_NODE);
BUILD_ASSERT(DT_REG_ADDR(MCP9600_NODE) == 0x60, "MCP9600 must use default address 0x60");
#else
#define MCP9600_HAS_NODE 0
#endif

static K_THREAD_STACK_DEFINE(mcp9600_stack, MCP9600_THREAD_STACK_SIZE);
static struct k_thread mcp9600_thread_data;
static std_msgs__msg__Float32MultiArray mcp9600_msg;
static bool mcp9600_msg_ready;
static uint32_t timestamp_anchor_ms32;

static int mcp9600_msg_init(void)
{
    if (mcp9600_msg_ready) {
        return 0;
    }

    if (!std_msgs__msg__Float32MultiArray__init(&mcp9600_msg)) {
        LOG_ERR("Failed to init MCP9600 message");
        return -ENOMEM;
    }

    if (!rosidl_runtime_c__float32__Sequence__init(&mcp9600_msg.data, 2)) {
        LOG_ERR("Failed to init MCP9600 data sequence");
        std_msgs__msg__Float32MultiArray__fini(&mcp9600_msg);
        return -ENOMEM;
    }

    memset(mcp9600_msg.data.data, 0, sizeof(float) * mcp9600_msg.data.size);
    mcp9600_msg_ready = true;
    return 0;
}

static int mcp9600_read_hot_direct(float *value)
{
#if MCP9600_HAS_NODE
    if (!device_is_ready(mcp9600_bus.bus)) {
        return -ENODEV;
    }

    uint8_t buf[2];
    int ret = i2c_burst_read_dt(&mcp9600_bus, MCP9600_REG_TEMP_HOT, buf, sizeof(buf));
    if (ret < 0) {
        return ret;
    }

    int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
    *value = (float)raw * 0.0625f;
    return 0;
#else
    ARG_UNUSED(value);
    return -ENODEV;
#endif
}

static void mcp9600_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    if (mcp9600_msg_init() != 0) {
        LOG_ERR("MCP9600 publisher disabled (message init failed)");
        return;
    }

    float last_logged_temp = NAN;

#if MCP9600_HAS_NODE
    const struct device *const mcp9600_bus_dev = DEVICE_DT_GET(DT_BUS(MCP9600_NODE));
    if (!device_is_ready(mcp9600_dev)) {
        LOG_WRN("MCP9600 device %s not ready; publishing fallback values", mcp9600_dev->name);
    } else {
        LOG_INF("MCP9600 using I2C bus %s at address 0x%02x", mcp9600_bus_dev->name,
            (unsigned int)DT_REG_ADDR(MCP9600_NODE));
    }
#else
    LOG_WRN("No MCP9600 node defined; publishing NaN values");
#endif

    while (true) {
        float hot_c = NAN;
        bool hot_valid = false;

#if MCP9600_HAS_NODE
        if (device_is_ready(mcp9600_dev)) {
            int fetch_rc = sensor_sample_fetch(mcp9600_dev);
            if (fetch_rc == 0) {
                struct sensor_value hot_sv;
                if (sensor_channel_get(mcp9600_dev, SENSOR_CHAN_MCP9600_HOT_JUNCTION_TEMP,
                                       &hot_sv) == 0) {
                    hot_c = (float)sensor_value_to_double(&hot_sv);
                    hot_valid = true;
                } else {
                    LOG_DBG("Failed to read MCP9600 hot junction channel");
                }
            } else {
                LOG_DBG("Failed to fetch MCP9600 sample (%d)", fetch_rc);
            }
        }

        if (!hot_valid) {
            if (mcp9600_read_hot_direct(&hot_c) == 0) {
                hot_valid = true;
            } else {
                LOG_DBG("Falling back direct read failed");
            }
        }
#endif

        uint32_t now_ms32 = k_uptime_get_32();
        if (timestamp_anchor_ms32 == 0U) {
            timestamp_anchor_ms32 = now_ms32;
        }
        float timestamp_s = (float)(now_ms32 - timestamp_anchor_ms32) / 1000.0f;

        mcp9600_msg.data.data[0] = timestamp_s;
        mcp9600_msg.data.data[1] = hot_c;

        if (hot_valid &&
            (isnan(last_logged_temp) || fabsf(hot_c - last_logged_temp) > 5.0f)) {
            LOG_INF("MCP9600 hot junction temperature %.2f C", hot_c);
            last_logged_temp = hot_c;
        }

        if (ros_initialized) {
            (void)ros_publish_try(&mcp9600_pub, &mcp9600_msg);
        }

        k_sleep(K_MSEC(MCP9600_PUBLISH_PERIOD_MS));
    }
}

void mcp9600_sensor_start(void)
{
    k_thread_create(&mcp9600_thread_data, mcp9600_stack, MCP9600_THREAD_STACK_SIZE,
                    mcp9600_thread, NULL, NULL, NULL,
                    MCP9600_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&mcp9600_thread_data, "mcp9600_pub");
}
