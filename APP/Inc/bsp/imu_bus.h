/*
 * imu_bus.h - Blocking SPI transaction adapter for the board ICM42688P.
 *
 * Purpose:
 *   Isolates HAL SPI1 and PA4 chip-select handling from the sensor register
 *   driver. The sensor layer sees bounded read/write transactions only.
 *
 * Core interfaces:
 *   - imu_bus_init(): validates SPI1 and leaves chip select inactive high.
 *   - imu_bus_read()/imu_bus_write(): execute one complete CS-low transaction.
 *   - imu_bus_clock_hz(): reports the configured bring-up bus rate.
 *
 * Data flow and constraints:
 *   ImuTask is the only caller after the scheduler starts. Calls are blocking
 *   with a 5 ms HAL timeout, allow at most 32 payload bytes, and always restore
 *   CS high before returning, including HAL error paths.
 */
#ifndef IMU_BUS_H
#define IMU_BUS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    IMU_BUS_STATUS_OK = 0,
    IMU_BUS_STATUS_BAD_ARGUMENT,
    IMU_BUS_STATUS_NOT_READY,
    IMU_BUS_STATUS_HAL_ERROR,
    IMU_BUS_STATUS_HAL_BUSY,
    IMU_BUS_STATUS_HAL_TIMEOUT
} imu_bus_status_t;

imu_bus_status_t imu_bus_init(void);
imu_bus_status_t imu_bus_read(uint8_t register_address,
                              uint8_t *data,
                              size_t length);
imu_bus_status_t imu_bus_write(uint8_t register_address,
                               const uint8_t *data,
                               size_t length);
uint32_t imu_bus_clock_hz(void);

#ifdef __cplusplus
}
#endif

#endif
