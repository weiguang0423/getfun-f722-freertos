/*
 * icm42688p.h - Register-level driver interface for the ICM42688P IMU.
 *
 * Purpose:
 *   Defines the deterministic reset/configuration sequence and raw 14-byte
 *   sample interface without depending on FreeRTOS or application state.
 *
 * Core interfaces and types:
 *   - icm42688p_initialize(): reset, identify, configure, wait, and read back.
 *   - icm42688p_data_ready(): polls the Bank 0 data-ready status bit.
 *   - icm42688p_read_sample(): reads temperature, accel, and gyro atomically.
 *   - icm42688p_raw_sample_t: signed big-endian register values.
 *
 * Constraints:
 *   The caller supplies a millisecond delay callback and is responsible for
 *   serialization. Values remain in the sensor frame and raw ADC units here.
 */
#ifndef ICM42688P_H
#define ICM42688P_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp/imu_bus.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ICM42688P_WHO_AM_I_EXPECTED 0x47U

typedef enum
{
    ICM42688P_STATUS_OK = 0,
    ICM42688P_STATUS_BAD_ARGUMENT,
    ICM42688P_STATUS_BUS_ERROR,
    ICM42688P_STATUS_WHO_AM_I_MISMATCH,
    ICM42688P_STATUS_CONFIG_MISMATCH
} icm42688p_status_t;

typedef void (*icm42688p_delay_ms_fn)(uint32_t delay_ms);

typedef struct
{
    int16_t temperature;
    int16_t acceleration[3];
    int16_t angular_rate[3];
} icm42688p_raw_sample_t;

typedef struct
{
    uint8_t who_am_i;
    uint8_t gyro_config0;
    uint8_t accel_config0;
    uint8_t pwr_mgmt0;
    imu_bus_status_t bus_status;
} icm42688p_diagnostics_t;

icm42688p_status_t icm42688p_initialize(
    icm42688p_delay_ms_fn delay_ms,
    icm42688p_diagnostics_t *diagnostics);
icm42688p_status_t icm42688p_data_ready(bool *ready,
                                        imu_bus_status_t *bus_status);
icm42688p_status_t icm42688p_read_sample(
    icm42688p_raw_sample_t *sample,
    imu_bus_status_t *bus_status);

#ifdef __cplusplus
}
#endif

#endif
