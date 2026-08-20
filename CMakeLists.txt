cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME GETFUN_F722_FreeRTOS)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
    APP/Src/algorithms/accel_calibration.c
    APP/Src/algorithms/angle_outer_loop.c
    APP/Src/algorithms/attitude_estimator.c
    APP/Src/algorithms/flight_arming.c
    APP/Src/algorithms/gyro_calibration.c
    APP/Src/algorithms/imu_filter.c
    APP/Src/algorithms/power_monitor.c
    APP/Src/algorithms/quad_x_mixer.c
    APP/Src/algorithms/rc_input.c
    APP/Src/algorithms/rc_setpoint.c
    APP/Src/algorithms/rate_pid.c
    APP/Src/app_state.c
    APP/Src/bsp/crsf_uart.c
    APP/Src/bsp/dshot_motor.c
    APP/Src/bsp/imu_bus.c
    APP/Src/bsp/linux_rc_monitor.c
    APP/Src/bsp/power_adc.c
    APP/Src/bsp/usb_cdc_transport.c
    APP/Src/drivers/icm42688p.c
    APP/Src/platform/platform_diag.c
    APP/Src/platform/platform_time.c
    APP/Src/protocol/crsf.c
    APP/Src/protocol/msp_transport.c
    APP/Src/protocol/msp_server.c
    APP/Src/rtos/app_task.c
    APP/Src/rtos/battery_task.c
    APP/Src/rtos/flight_task.c
    APP/Src/rtos/imu_task.c
    APP/Src/rtos/rc_task.c
    APP/Src/storage/parameter_store.c
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    APP/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Generate flashable Intel HEX and raw BIN images whenever the ELF is linked.
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.hex and ${CMAKE_PROJECT_NAME}.bin"
)
