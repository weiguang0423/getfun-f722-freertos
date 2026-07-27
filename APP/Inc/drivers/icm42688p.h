/*
 * icm42688p.h —— ICM42688P IMU 寄存器级驱动接口
 *
 * 作用:
 *   定义确定性的复位/配置序列以及原始 14 字节采样接口,不依赖 FreeRTOS 或
 *   应用状态。
 *
 * 核心接口与类型:
 *   - icm42688p_initialize(): 复位、识别、配置、等待并回读校验。
 *   - icm42688p_data_ready(): 轮询 Bank 0 的数据就绪状态位。
 *   - icm42688p_read_sample(): 原子地读取温度、加速度和陀螺数据。
 *   - icm42688p_start_sample_dma()/finish_sample_dma(): 异步采样事务,字节
 *     解码仍保留在器件层内部。
 *   - icm42688p_raw_sample_t: 有符号大端寄存器数值。
 *
 * 约束:
 *   调用方需提供毫秒级延时回调,并负责事务串行化。本接口中的数值保持传感器
 *   坐标系和原始 ADC 单位,不做单位换算。
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
icm42688p_status_t icm42688p_start_sample_dma(
    imu_bus_status_t *bus_status);
icm42688p_status_t icm42688p_finish_sample_dma(
    icm42688p_raw_sample_t *sample,
    imu_bus_status_t *bus_status);

#ifdef __cplusplus
}
#endif

#endif
