/*
 * Part of the SVEA Low-Level Interface (Zephyr) application.
 */

#include "mcp4725_monitor.h"

#include "ros_iface.h"

#include <errno.h>
#include <math.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(mcp4725_monitor, LOG_LEVEL_INF);

#define MCP4725_THREAD_STACK_SIZE 1024
#define MCP4725_THREAD_PRIORITY   7
#define MCP4725_PUBLISH_PERIOD_MS 40  // ~25 Hz updates without hammering I2C

#if DT_NODE_HAS_STATUS(DT_NODELABEL(mcp4725), okay)
#define MCP4725_HAS_NODE 1
#define MCP4725_NODE DT_NODELABEL(mcp4725)
static const struct i2c_dt_spec mcp4725_bus = I2C_DT_SPEC_GET(MCP4725_NODE);
#else
#define MCP4725_HAS_NODE 0
#endif

static K_THREAD_STACK_DEFINE(mcp4725_stack, MCP4725_THREAD_STACK_SIZE);
static struct k_thread mcp4725_thread_data;
static std_msgs__msg__Float32MultiArray mcp4725_msg;
static bool mcp4725_msg_ready;

#define MCP4725_MAX_RAW_VALUE 4095U

struct mcp4725_command_state {
    struct k_mutex lock;
    bool pending;
    uint16_t raw_value;
};

static struct mcp4725_command_state mcp4725_cmd_state;

static uint16_t mcp4725_normalized_to_raw(float normalized)
{
    if (!isfinite(normalized)) {
        return 0U;
    }

    if (normalized <= 0.0f) {
        return 0U;
    }

    if (normalized >= 1.0f) {
        return MCP4725_MAX_RAW_VALUE;
    }

    return (uint16_t)lrintf(normalized * (float)MCP4725_MAX_RAW_VALUE);
}

static int mcp4725_write_raw(uint16_t raw)
{
#if MCP4725_HAS_NODE
    uint8_t tx[2];

    tx[0] = (raw >> 8) & 0x0F;
    tx[1] = raw & 0xFF;

    return i2c_write_dt(&mcp4725_bus, tx, sizeof(tx));
#else
    ARG_UNUSED(raw);
    return -ENODEV;
#endif
}

static bool mcp4725_consume_pending_command(uint16_t *raw_out)
{
    bool has_command = false;

    k_mutex_lock(&mcp4725_cmd_state.lock, K_FOREVER);
    if (mcp4725_cmd_state.pending) {
        has_command = true;
        mcp4725_cmd_state.pending = false;
        *raw_out = mcp4725_cmd_state.raw_value;
    }
    k_mutex_unlock(&mcp4725_cmd_state.lock);

    return has_command;
}

int mcp4725_set_target(float normalized)
{
    if (!isfinite(normalized)) {
        return -EINVAL;
    }

    uint16_t raw = mcp4725_normalized_to_raw(normalized);

    k_mutex_lock(&mcp4725_cmd_state.lock, K_FOREVER);
    mcp4725_cmd_state.raw_value = raw;
    mcp4725_cmd_state.pending = true;
    k_mutex_unlock(&mcp4725_cmd_state.lock);

    return 0;
}

static int mcp4725_msg_init(void)
{
    if (mcp4725_msg_ready) {
        return 0;
    }

    if (!std_msgs__msg__Float32MultiArray__init(&mcp4725_msg)) {
        LOG_ERR("Failed to init MCP4725 message");
        return -ENOMEM;
    }

    if (!rosidl_runtime_c__float32__Sequence__init(&mcp4725_msg.data, 2)) {
        LOG_ERR("Failed to init MCP4725 data sequence");
        std_msgs__msg__Float32MultiArray__fini(&mcp4725_msg);
        return -ENOMEM;
    }

    memset(mcp4725_msg.data.data, 0, sizeof(float) * mcp4725_msg.data.size);
    mcp4725_msg_ready = true;
    return 0;
}

static int mcp4725_snapshot(uint16_t *raw_value, bool *ready)
{
#if MCP4725_HAS_NODE
    uint8_t buf[5];
    int ret = i2c_read_dt(&mcp4725_bus, buf, sizeof(buf));
    if (ret < 0) {
        return ret;
    }

    *ready = (buf[0] & BIT(7)) != 0;
    *raw_value = ((uint16_t)buf[1] << 4) | (buf[2] >> 4);
    *raw_value &= 0x0FFF;
    return 0;
#else
    ARG_UNUSED(raw_value);
    ARG_UNUSED(ready);
    return -ENODEV;
#endif
}

static void mcp4725_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    if (mcp4725_msg_init() != 0) {
        LOG_ERR("MCP4725 publisher disabled (message init failed)");
        return;
    }

#if MCP4725_HAS_NODE
    if (!device_is_ready(mcp4725_bus.bus)) {
        LOG_WRN("MCP4725 I2C bus %s not ready; publishing zeros", mcp4725_bus.bus->name);
    }
#else
    LOG_WRN("No MCP4725 node defined; publishing zeros");
#endif

    while (true) {
        uint16_t raw = 0;
        bool ready = false;
#if MCP4725_HAS_NODE
        if (device_is_ready(mcp4725_bus.bus)) {
            uint16_t command_raw = 0;
            if (mcp4725_consume_pending_command(&command_raw)) {
                int write_rc = mcp4725_write_raw(command_raw);
                if (write_rc < 0) {
                    LOG_WRN("Failed to command MCP4725 value (%d)", write_rc);
                }
            }
            if (mcp4725_snapshot(&raw, &ready) < 0) {
                LOG_DBG("Failed to read MCP4725 snapshot");
            }
        }
#endif
        float normalized = ready ? ((float)raw / 4095.0f) : NAN;
        mcp4725_msg.data.data[0] = (float)raw;
        mcp4725_msg.data.data[1] = normalized;

        if (ros_initialized) {
            (void)ros_publish_try(&mcp4725_pub, &mcp4725_msg);
        }

        k_sleep(K_MSEC(MCP4725_PUBLISH_PERIOD_MS));
    }
}

void mcp4725_monitor_start(void)
{
    k_mutex_init(&mcp4725_cmd_state.lock);
    k_thread_create(&mcp4725_thread_data, mcp4725_stack, MCP4725_THREAD_STACK_SIZE,
                    mcp4725_thread, NULL, NULL, NULL,
                    MCP4725_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&mcp4725_thread_data, "mcp4725_pub");
}
