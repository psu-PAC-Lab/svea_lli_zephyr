/*
 * Part of the SVEA Low-Level Interface (Zephyr) application.
 */

#include "mcp9600_sensor.h"

#include "ros_iface.h"

#include <math.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mcp9600_sensor, LOG_LEVEL_INF);

#define MCP9600_THREAD_STACK_SIZE 1536
#define MCP9600_THREAD_PRIORITY   6
#define MCP9600_PUBLISH_PERIOD_MS 200

#define MCP9600_REG_TEMP_HOT   0x00
#define MCP9600_REG_TEMP_DELTA 0x01
#define MCP9600_REG_TEMP_COLD  0x02
#define MCP9600_REG_RAW_ADC    0x03

#if DT_NODE_HAS_STATUS(DT_NODELABEL(mcp9600), okay)
#define MCP9600_HAS_NODE 1
#define MCP9600_NODE DT_NODELABEL(mcp9600)
static const struct i2c_dt_spec mcp9600_bus = I2C_DT_SPEC_GET(MCP9600_NODE);
#else
#define MCP9600_HAS_NODE 0
#endif

static K_THREAD_STACK_DEFINE(mcp9600_stack, MCP9600_THREAD_STACK_SIZE);
static struct k_thread mcp9600_thread_data;
static std_msgs__msg__Float32MultiArray mcp9600_msg;
static bool mcp9600_msg_ready;

static int mcp9600_msg_init(void)
{
    if (mcp9600_msg_ready) {
        return 0;
    }

    if (!std_msgs__msg__Float32MultiArray__init(&mcp9600_msg)) {
        LOG_ERR("Failed to init MCP9600 message");
        return -ENOMEM;
    }

    if (!rosidl_runtime_c__float32__Sequence__init(&mcp9600_msg.data, 6)) {
        LOG_ERR("Failed to init MCP9600 data sequence");
        std_msgs__msg__Float32MultiArray__fini(&mcp9600_msg);
        return -ENOMEM;
    }

    memset(mcp9600_msg.data.data, 0, sizeof(float) * mcp9600_msg.data.size);
    mcp9600_msg_ready = true;
    return 0;
}

static int mcp9600_read_signed16(uint8_t reg, float *value)
{
#if MCP9600_HAS_NODE
    uint8_t buf[2];
    int ret = i2c_burst_read_dt(&mcp9600_bus, reg, buf, sizeof(buf));
    if (ret < 0) {
        return ret;
    }

    int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
    *value = (float)raw * 0.0625f;
    return 0;
#else
    ARG_UNUSED(reg);
    *value = NAN;
    return -ENODEV;
#endif
}

static int mcp9600_read_raw_adc(float *value)
{
#if MCP9600_HAS_NODE
    uint8_t buf[3];
    int ret = i2c_burst_read_dt(&mcp9600_bus, MCP9600_REG_RAW_ADC, buf, sizeof(buf));
    if (ret < 0) {
        return ret;
    }

    int32_t raw = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
    if (raw & 0x800000) {
        raw |= 0xFF000000;
    }
    *value = (float)raw;
    return 0;
#else
    *value = NAN;
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

#if MCP9600_HAS_NODE
    bool bus_ready = device_is_ready(mcp9600_bus.bus);
    if (!bus_ready) {
        LOG_WRN("MCP9600 I2C bus %s not ready; publishing NaN values", mcp9600_bus.bus->name);
    }
#else
    LOG_WRN("No MCP9600 node defined; publishing NaN values");
#endif

    while (true) {
        float hot = NAN, cold = NAN, delta = NAN, adc_counts = NAN;
#if MCP9600_HAS_NODE
        if (device_is_ready(mcp9600_bus.bus)) {
            if (mcp9600_read_signed16(MCP9600_REG_TEMP_HOT, &hot) < 0) {
                LOG_DBG("Failed to read MCP9600 hot junction");
            }
            if (mcp9600_read_signed16(MCP9600_REG_TEMP_COLD, &cold) < 0) {
                LOG_DBG("Failed to read MCP9600 cold junction");
            }
            if (mcp9600_read_signed16(MCP9600_REG_TEMP_DELTA, &delta) < 0) {
                LOG_DBG("Failed to read MCP9600 delta");
            }
            if (mcp9600_read_raw_adc(&adc_counts) < 0) {
                LOG_DBG("Failed to read MCP9600 ADC");
            }
        }
#endif
        uint64_t now_ms = ros_iface_epoch_millis();
        uint32_t now_ns = (uint32_t)(ros_iface_epoch_nanos() % 1000000000ULL);

        mcp9600_msg.data.data[0] = (float)(now_ms / 1000ULL);
        mcp9600_msg.data.data[1] = (float)now_ns;
        mcp9600_msg.data.data[2] = hot;
        mcp9600_msg.data.data[3] = cold;
        mcp9600_msg.data.data[4] = delta;
        mcp9600_msg.data.data[5] = adc_counts;

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
