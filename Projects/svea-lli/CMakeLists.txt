# SPDX-License-Identifier: Apache-2.0


message(WARNING 
"========================================================================
MICRO-ROS BUILD NOTICE:
If you have changed the target board/port and are doing a pristine build,
you may need to manually remove micro-ROS build artifacts:
  rm -rf modules/lib/micro_ros_zephyr_module/modules/libmicroros/libmicroros.a
  rm -rf modules/lib/micro_ros_zephyr_module/modules/libmicroros/micro_ros_src/build
This ensures micro-ROS rebuilds for the correct target architecture.
========================================================================")


cmake_minimum_required(VERSION 3.13.1)

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/lib/micro_ros_zephyr_module/modules/libmicroros)
list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/../../external/modules/debug/segger
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(svea_lli_control C CXX)

# Include directories
target_include_directories(app PRIVATE include)
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/lib/svea-bms/app/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/lib/svea-bms/app/src/bq769x0
)

# Source files
target_sources(app PRIVATE
    src/main.c
    src/control.c
    src/rc_input.c
    src/ros_iface.c
    src/imu_sensor.c
    src/ina3221_sensor.c
    src/wheel_enc.c
    src/ina2xx_common.c
    
    # BMS integration
    src/bms.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/lib/svea-bms/app/src/bms_common.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/lib/svea-bms/app/src/bms_soc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/lib/svea-bms/app/src/wake_chip.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/lib/svea-bms/app/src/helper.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/lib/svea-bms/app/src/bq769x0/bms_bq769x0.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/lib/svea-bms/app/src/bq769x0/interface.c
)
