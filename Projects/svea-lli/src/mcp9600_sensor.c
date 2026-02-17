/*
 * Part of the SVEA Low-Level Interface (Zephyr) application.
 */

#include "mcp9600_sensor.h"

#include "mcp9600.h"
#include "ros_iface.h"

#include <math.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/temperature.h>
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
#define MCP9600_REG_ID_REVISION 0x20

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
static sensor_msgs__msg__Temperature mcp9600_msg;
static bool mcp9600_msg_ready;

static int mcp9600_msg_init(void)
{
    if (mcp9600_msg_ready) {
        return 0;
    }

    if (!sensor_msgs__msg__Temperature__init(&mcp9600_msg)) {
        LOG_ERR("Failed to init MCP9600 message");
        return -ENOMEM;
    }

    if (!rosidl_runtime_c__String__assign(&mcp9600_msg.header.frame_id, "mcp9600_hot")) {
        sensor_msgs__msg__Temperature__fini(&mcp9600_msg);
        LOG_ERR("Failed to set MCP9600 frame id");
        return -ENOMEM;
    }

    mcp9600_msg.temperature = NAN;
    mcp9600_msg.variance = NAN;
    mcp9600_msg_ready = true;
    return 0;
}

static void mcp9600_log_device_id(void)
{
#if MCP9600_HAS_NODE
    uint8_t buf[2];
    int ret = i2c_burst_read_dt(&mcp9600_bus, MCP9600_REG_ID_REVISION, buf, sizeof(buf));
    if (ret == 0) {
        LOG_INF("MCP9600 device id 0x%02x rev 0x%02x", buf[0], buf[1]);
    } else {
        LOG_WRN("MCP9600: failed to read device id (%d)", ret);
    }
#endif
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
        mcp9600_log_device_id();
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
                if (sensor_channel_get(mcp9600_dev, SENSOR_CHAN_AMBIENT_TEMP,
                                       &hot_sv) == 0) {
                    hot_c = (float)sensor_value_to_double(&hot_sv);
                    hot_valid = true;
                } else {
                    LOG_WRN("MCP9600: sensor_channel_get failed");
                }
            } else {
                LOG_WRN("MCP9600: sensor_sample_fetch failed (%d)", fetch_rc);
            }
        }

        if (!hot_valid) {
            int direct_rc = mcp9600_read_hot_direct(&hot_c);
            if (direct_rc == 0) {
                hot_valid = true;
            } else {
                LOG_WRN("MCP9600: direct register read failed (%d)", direct_rc);
            }
        }
#endif

        uint64_t now_ms = ros_iface_epoch_millis();
        uint64_t now_ns = ros_iface_epoch_nanos();
        mcp9600_msg.header.stamp.sec = (int32_t)(now_ms / 1000ULL);
        mcp9600_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000ULL);
        mcp9600_msg.temperature = hot_valid ? hot_c : NAN;

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
