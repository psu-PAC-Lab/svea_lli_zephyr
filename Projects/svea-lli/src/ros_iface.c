#/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
# * SVEA Note.
# *
# * Part of the SVEA Low‑Level Interface (Zephyr) application.
# * Author: Nils Kiefer
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "ros_iface.h"
#include "control.h"
#include "loop_delays.h"
#include "rc_input.h"
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/twist_with_covariance_stamped.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/battery_state.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int8.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/posix/time.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int8.h>

#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <microros_transports.h>
#include <rmw_microros/rmw_microros.h>

LOG_MODULE_REGISTER(ros_iface, LOG_LEVEL_INF);

#define ROS_STACK_SIZE 4096
#define RCCHECK(fn)                                                                     \
    {                                                                                   \
        rcl_ret_t temp_rc = fn;                                                         \
        if ((temp_rc != RCL_RET_OK)) {                                                  \
            LOG_ERR("Failed status on line %d: %d. Aborting.", __LINE__, (int)temp_rc); \
            return false;                                                               \
        }                                                                               \
    }
#define RCSOFTCHECK(fn)                                                                   \
    {                                                                                     \
        rcl_ret_t temp_rc = fn;                                                           \
        if ((temp_rc != RCL_RET_OK)) {                                                    \
            LOG_ERR("Failed status on line %d: %d. Continuing.", __LINE__, (int)temp_rc); \
        }                                                                                 \
    }

ros_ctrl_t g_ros_ctrl = {0};
bool ros_initialized = false;

static K_THREAD_STACK_DEFINE(ros_stack, ROS_STACK_SIZE);
static struct k_thread ros_thread;
// Global mutex to serialize all micro-ROS transport I/O (publish, spin, sync, epoch reads)
static struct k_mutex uros_io_mutex;

// Node support
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;

// Publishers
rcl_publisher_t pub_remote_steer, pub_remote_throttle, pub_remote_gear, pub_remote_override, pub_remote_connected;
rcl_publisher_t imu_pub, encoders_pub, ina3221_pub, battery_pub, mcp9600_pub, mcp4725_pub, ina2xx_common_pub;

// Subscriptions - ensure proper alignment
static rcl_subscription_t sub_steer, sub_throttle, sub_gear, sub_diff;
static std_msgs__msg__Int8 submsg_steer __aligned(4);
static std_msgs__msg__Int8 submsg_throttle __aligned(4);
static std_msgs__msg__Bool submsg_gear __aligned(4);
static std_msgs__msg__Bool submsg_diff __aligned(4);

// Time synchronization variables
static uint64_t epoch_off_ns;
static atomic_t epoch_locked = ATOMIC_INIT(0);
static struct k_thread time_sync_thread_data;
K_THREAD_STACK_DEFINE(time_sync_stack, 2048);

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

// Forward declarations for locked helpers used below
static void ros_sync_session_locked(int timeout_ms);
static bool ros_epoch_synchronized_locked(void);
static uint64_t ros_epoch_nanos_locked(void);
static rcl_ret_t ros_executor_spin_some_locked(uint64_t timeout_ns);

// Subscription callbacks
static void steer_cb(const void *msg) {
    int8_t value = ((std_msgs__msg__Int8 *)msg)->data;
#if LOG_LEVEL >= LOG_LEVEL_DBG
    LOG_DBG("Received steering command: %d", value);
#endif
    g_ros_ctrl.steering = value;
}

static void throttle_cb(const void *msg) {
    int8_t value = ((std_msgs__msg__Int8 *)msg)->data;
#if LOG_LEVEL >= LOG_LEVEL_DBG
    LOG_DBG("Received throttle command: %d", value);
#endif
    g_ros_ctrl.throttle = value;
    g_ros_ctrl.timestamp = k_uptime_get_32();
}

static void gear_cb(const void *msg) {
    bool value = ((std_msgs__msg__Bool *)msg)->data;
#if LOG_LEVEL >= LOG_LEVEL_DBG
    LOG_DBG("Received high gear command: %s", value ? "true" : "false");
#endif
    g_ros_ctrl.high_gear = value;
}

static void diff_cb(const void *msg) {
    bool value = ((std_msgs__msg__Bool *)msg)->data;
#if LOG_LEVEL >= LOG_LEVEL_DBG
    LOG_DBG("Received diff command: %s", value ? "true" : "false");
#endif
    g_ros_ctrl.diff = value;
}

// Create allocator, support, pub, sub, executor for node if agent connection is successful
bool create_entities() {
    allocator = rcl_get_default_allocator();

    // Init support - this will fail if no agent is available
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        LOG_DBG("Support init failed: %d (no agent available)", ret);
        return false;
    }

    // Configure transport timeouts for better disconnection detection
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    if (rmw_context != NULL) {
        // Set session timeout (how long to wait for agent responses)
        rmw_uros_set_context_entity_creation_session_timeout(rmw_context, 1000); // 1 second
        rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 500);   // 0.5 seconds
    }

    // Node
    ret = rclc_node_init_default(&node, "svea_lli_node", "", &support);
    if (ret != RCL_RET_OK) {
        LOG_ERR("Node init failed: %d", ret);
        rclc_support_fini(&support);
        return false;
    }

    // Publishers
    RCCHECK(rclc_publisher_init_best_effort(&pub_remote_steer, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "/lli/remote/steering"));
    RCCHECK(rclc_publisher_init_best_effort(&pub_remote_throttle, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "/lli/remote/throttle"));
    RCCHECK(rclc_publisher_init_best_effort(&pub_remote_gear, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/lli/remote/high_gear"));
    RCCHECK(rclc_publisher_init_best_effort(&pub_remote_override, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/lli/remote/override"));
    RCCHECK(rclc_publisher_init_default(&pub_remote_connected, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/lli/remote/connected"));

    // Sensor publishers
    RCCHECK(rclc_publisher_init_best_effort(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/lli/sensor/imu"));
    RCCHECK(rclc_publisher_init_best_effort(&encoders_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistWithCovarianceStamped), "/lli/sensor/encoders"));
    RCCHECK(rclc_publisher_init_best_effort(&ina3221_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/lli/sensor/ina3221"));
    RCCHECK(rclc_publisher_init_best_effort(&battery_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "/lli/battery/state"));
    RCCHECK(rclc_publisher_init_best_effort(&mcp9600_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/lli/sensor/mcp9600"));
    RCCHECK(rclc_publisher_init_best_effort(&mcp4725_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/lli/sensor/mcp4725"));
    RCCHECK(rclc_publisher_init_best_effort(&ina2xx_common_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "/lli/battery/fan"));

    // Subscriptions
    RCCHECK(rclc_subscription_init_best_effort(&sub_steer, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "/lli/ctrl/steering"));
    RCCHECK(rclc_subscription_init_best_effort(&sub_throttle, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "/lli/ctrl/throttle"));
    RCCHECK(rclc_subscription_init_best_effort(&sub_gear, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/lli/ctrl/high_gear"));
    RCCHECK(rclc_subscription_init_best_effort(&sub_diff, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/lli/ctrl/diff"));

    // Executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_steer, &submsg_steer, steer_cb, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_throttle, &submsg_throttle, throttle_cb, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_gear, &submsg_gear, gear_cb, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_diff, &submsg_diff, diff_cb, ON_NEW_DATA));

    return true;
}

// Destroy allocator, support, pub, sub, executor for node if agent connection is lost or unsuccessful
bool destroy_entities() {

    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFTCHECK(rcl_publisher_fini(&pub_remote_steer, &node));
    RCSOFTCHECK(rcl_publisher_fini(&pub_remote_throttle, &node));
    RCSOFTCHECK(rcl_publisher_fini(&pub_remote_gear, &node));
    RCSOFTCHECK(rcl_publisher_fini(&pub_remote_override, &node));
    RCSOFTCHECK(rcl_publisher_fini(&pub_remote_connected, &node));
    RCSOFTCHECK(rcl_publisher_fini(&imu_pub, &node));
    RCSOFTCHECK(rcl_publisher_fini(&encoders_pub, &node));
    RCSOFTCHECK(rcl_publisher_fini(&ina3221_pub, &node));
    RCSOFTCHECK(rcl_publisher_fini(&mcp9600_pub, &node));
    RCSOFTCHECK(rcl_publisher_fini(&mcp4725_pub, &node));
    RCSOFTCHECK(rcl_publisher_fini(&ina2xx_common_pub, &node));
    RCSOFTCHECK(rcl_publisher_fini(&battery_pub, &node));
    RCSOFTCHECK(rcl_subscription_fini(&sub_steer, &node));
    RCSOFTCHECK(rcl_subscription_fini(&sub_throttle, &node));
    RCSOFTCHECK(rcl_subscription_fini(&sub_gear, &node));
    RCSOFTCHECK(rcl_subscription_fini(&sub_diff, &node));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));

    return true;
}

void ros_iface_handle_remote_publish_error(void) {
    LOG_WRN("Remote publish error, forcing reconnect");
    state = AGENT_DISCONNECTED;
    ros_initialized = false;
}

// Sync time thread: updates virtual clock every second
static void time_sync_thread(void *a, void *b, void *c) {
    while (state != AGENT_CONNECTED) {
        k_sleep(K_MSEC(100));
    }

    while (1) {
        if (state == AGENT_CONNECTED) {
            ros_sync_session_locked(1);

            if (ros_epoch_synchronized_locked() && !atomic_test_and_set_bit(&epoch_locked, 0)) {
                uint64_t agent_ns = ros_epoch_nanos_locked();
                uint64_t up_ns = k_uptime_get() * 1000000ULL;

                unsigned int key = irq_lock();
                epoch_off_ns = agent_ns - up_ns;
                irq_unlock(key);

                LOG_INF("Time locked to agent, offset = %lld ns", (long long)epoch_off_ns);
            }
        }
        k_sleep(K_MSEC(ROS_TIME_SYNC_LOOP_DELAY_MS));
    }
}

// Main transport task for transmitting and receiving data
static void ros_iface_thread(void *a, void *b, void *c) {
    LOG_INF("ros_iface_thread: started");
    k_msleep(1000); // Allow time for other subsystems to initialize

    uint64_t last_publish_check = 0;
    uint32_t last_retry_log_ms = 0;

    while (1) {
        // k_msleep(1);
        switch (state) {
        case WAITING_AGENT: {
            LOG_DBG("Attempting to create micro-ROS entities");
            bool created = create_entities();
            state = created ? AGENT_CONNECTED : WAITING_AGENT;
            if (!created) {
                uint32_t now_ms = k_uptime_get_32();
                if ((uint32_t)(now_ms - last_retry_log_ms) >= 10000U) {
                    LOG_WRN("Failed to create micro-ROS entities, retrying...");
                    last_retry_log_ms = now_ms;
                }
                k_msleep(2000);
            } else {
                LOG_INF("Connected to uROS agent and created entities.");
                ros_initialized = true;
                last_publish_check = k_uptime_get();
            }
            break;
        }

        case AGENT_AVAILABLE:
            // Not using ping from example
            state = WAITING_AGENT;
            break;

        case AGENT_CONNECTED:
            // Try to spin the executor
            rcl_ret_t spin_ret = ros_executor_spin_some_locked(0);
            if (spin_ret != RCL_RET_OK) {
                LOG_WRN("Executor spin failed (%d), assuming disconnection", spin_ret);
                state = AGENT_DISCONNECTED;
                ros_initialized = false;
                break;
            }
            ros_initialized = true;
            k_msleep(5);
            break;

        case AGENT_DISCONNECTED:
            LOG_WRN("Disconnected from agent. Cleaning up and retrying...");
            // Guarantees that control commands are zeroed on disconnect
            g_ros_ctrl.steering = 0;
            g_ros_ctrl.throttle = 0;

            destroy_entities();
            ros_initialized = false;
            state = WAITING_AGENT;
            k_msleep(1000);
            break;

        default:
            state = WAITING_AGENT;
            break;
        }
    }
}

uint64_t ros_iface_epoch_millis(void) {
    unsigned int key = irq_lock();
    uint64_t off = epoch_off_ns;
    irq_unlock(key);

    return k_uptime_get() + off / 1000000ULL;
}

uint64_t ros_iface_epoch_nanos(void) {
    unsigned int key = irq_lock();
    uint64_t off = epoch_off_ns;
    irq_unlock(key);

    return k_uptime_get() * 1000000ULL + off;
}

void ros_iface_init(void) {
    // Serialize all USB/micro-ROS access across threads
    k_mutex_init(&uros_io_mutex);
    // Setup custom transports for microros
    rmw_uros_set_custom_transport(
        MICRO_ROS_FRAMING_REQUIRED,
        (void *)&default_params,
        zephyr_transport_open,
        zephyr_transport_close,
        zephyr_transport_write,
        zephyr_transport_read);

    state = WAITING_AGENT;

    k_thread_create(&ros_thread, ros_stack, K_THREAD_STACK_SIZEOF(ros_stack),
                    ros_iface_thread, NULL, NULL, NULL,
                    5, 0, K_NO_WAIT);
    k_thread_name_set(&ros_thread, "ros_iface");

    k_thread_create(&time_sync_thread_data, time_sync_stack, K_THREAD_STACK_SIZEOF(time_sync_stack),
                    time_sync_thread, NULL, NULL, NULL,
                    4, 0, K_NO_WAIT);
    k_thread_name_set(&time_sync_thread_data, "time_sync");
}

// Publish with mutex lock, blocking if necessary, making sure we only load the usb port if it is free
rcl_ret_t ros_publish_try(rcl_publisher_t *pub, const void *msg) {
    int lock_rc = k_mutex_lock(&uros_io_mutex, K_FOREVER); // wait a tiny bit at least
    if (lock_rc != 0) {
        // Transport is busy; skip publish without blocking
        return RCL_RET_ERROR;
    }

    rcl_ret_t rc = rcl_publish(pub, msg, NULL);
    k_mutex_unlock(&uros_io_mutex);
    return rc;
}

rcl_ret_t ros_publish_locked(rcl_publisher_t *pub, const void *msg) {
    k_mutex_lock(&uros_io_mutex, K_FOREVER);
    rcl_ret_t rc = rcl_publish(pub, msg, NULL);
    k_mutex_unlock(&uros_io_mutex);
    return rc;
}

static rcl_ret_t ros_executor_spin_some_locked(uint64_t timeout_ns) {
    k_mutex_lock(&uros_io_mutex, K_FOREVER);
    rcl_ret_t rc = rclc_executor_spin_some(&executor, timeout_ns);
    k_mutex_unlock(&uros_io_mutex);
    return rc;
}

static void ros_sync_session_locked(int timeout_ms) {
    k_mutex_lock(&uros_io_mutex, K_FOREVER);
    (void)rmw_uros_sync_session(timeout_ms);
    k_mutex_unlock(&uros_io_mutex);
}

static bool ros_epoch_synchronized_locked(void) {
    bool synced;
    k_mutex_lock(&uros_io_mutex, K_FOREVER);
    synced = rmw_uros_epoch_synchronized();
    k_mutex_unlock(&uros_io_mutex);
    return synced;
}

static uint64_t ros_epoch_nanos_locked(void) {
    uint64_t ns;
    k_mutex_lock(&uros_io_mutex, K_FOREVER);
    ns = rmw_uros_epoch_nanos();
    k_mutex_unlock(&uros_io_mutex);
    return ns;
}
