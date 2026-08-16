/*
 * icm42688p.c —— 板载 ICM42688P 的 Bank 0 寄存器驱动
 *
 * 作用:
 *   实现器件复位、身份校验、固定 1 kHz 配置、配置回读、数据就绪轮询和
 *   连续突发采样。
 *
 * 核心函数:
 *   - icm42688p_initialize(): 应用轮询基线配置。
 *   - icm42688p_data_ready(): 读取 INT_STATUS.DATA_RDY_INT。
 *   - icm42688p_read_sample(): 解码 TEMP_DATA1..GYRO_DATA_Z0。
 *   - icm42688p_start_sample_dma()/finish_sample_dma(): 把异步采样传输与同样
 *     确定性的大端解码拆分开。
 *
 * 数据流与约束:
 *   寄存器事务只通过 imu_bus 进行。本驱动不涉及 RTOS、日志、应用状态、单位
 *   换算或板级对准。
 */
#include "drivers/icm42688p.h"

#include <stddef.h>
#include <string.h>

#define ICM42688P_REG_DEVICE_CONFIG 0x11U
#define ICM42688P_REG_TEMP_DATA1 0x1DU
#define ICM42688P_REG_INT_STATUS 0x2DU
#define ICM42688P_REG_PWR_MGMT0 0x4EU
#define ICM42688P_REG_GYRO_CONFIG0 0x4FU
#define ICM42688P_REG_ACCEL_CONFIG0 0x50U
#define ICM42688P_REG_WHO_AM_I 0x75U
#define ICM42688P_REG_BANK_SEL 0x76U

#define ICM42688P_DEVICE_SOFT_RESET 0x01U
#define ICM42688P_BANK_0 0x00U
#define ICM42688P_GYRO_CONFIG_2000DPS_1KHZ 0x06U
#define ICM42688P_ACCEL_CONFIG_16G_1KHZ 0x06U
#define ICM42688P_PWR_ACCEL_GYRO_LOW_NOISE 0x0FU
#define ICM42688P_INT_STATUS_DATA_READY 0x08U
#define ICM42688P_BURST_SAMPLE_SIZE 14U

static int16_t decode_be_i16(const uint8_t *bytes)
{
    return (int16_t)(((uint16_t)bytes[0] << 8U) | bytes[1]);
}

static void decode_sample_bytes(const uint8_t bytes[ICM42688P_BURST_SAMPLE_SIZE],
                                icm42688p_raw_sample_t *sample)
{
    sample->temperature = decode_be_i16(&bytes[0]);
    sample->acceleration[0] = decode_be_i16(&bytes[2]);
    sample->acceleration[1] = decode_be_i16(&bytes[4]);
    sample->acceleration[2] = decode_be_i16(&bytes[6]);
    sample->angular_rate[0] = decode_be_i16(&bytes[8]);
    sample->angular_rate[1] = decode_be_i16(&bytes[10]);
    sample->angular_rate[2] = decode_be_i16(&bytes[12]);
}

static icm42688p_status_t read_register(uint8_t address,
                                        uint8_t *value,
                                        imu_bus_status_t *bus_status)
{
    const imu_bus_status_t status = imu_bus_read(address, value, 1U);

    if (bus_status != NULL) {
        *bus_status = status;
    }
    return status == IMU_BUS_STATUS_OK
               ? ICM42688P_STATUS_OK
               : ICM42688P_STATUS_BUS_ERROR;
}

static icm42688p_status_t write_register(uint8_t address,
                                         uint8_t value,
                                         imu_bus_status_t *bus_status)
{
    const imu_bus_status_t status = imu_bus_write(address, &value, 1U);

    if (bus_status != NULL) {
        *bus_status = status;
    }
    return status == IMU_BUS_STATUS_OK
               ? ICM42688P_STATUS_OK
               : ICM42688P_STATUS_BUS_ERROR;
}

icm42688p_status_t icm42688p_initialize(
    icm42688p_delay_ms_fn delay_ms,
    icm42688p_diagnostics_t *diagnostics)
{
    icm42688p_status_t status;
    icm42688p_raw_sample_t warmup_sample;

    if ((delay_ms == NULL) || (diagnostics == NULL)) {
        return ICM42688P_STATUS_BAD_ARGUMENT;
    }

    memset(diagnostics, 0, sizeof(*diagnostics));
    diagnostics->bus_status = imu_bus_init();
    if (diagnostics->bus_status != IMU_BUS_STATUS_OK) {
        return ICM42688P_STATUS_BUS_ERROR;
    }

    status = write_register(ICM42688P_REG_BANK_SEL,
                            ICM42688P_BANK_0,
                            &diagnostics->bus_status);
    if (status != ICM42688P_STATUS_OK) {
        return status;
    }

    status = write_register(ICM42688P_REG_DEVICE_CONFIG,
                            ICM42688P_DEVICE_SOFT_RESET,
                            &diagnostics->bus_status);
    if (status != ICM42688P_STATUS_OK) {
        return status;
    }
    delay_ms(2U);

    status = write_register(ICM42688P_REG_BANK_SEL,
                            ICM42688P_BANK_0,
                            &diagnostics->bus_status);
    if (status != ICM42688P_STATUS_OK) {
        return status;
    }

    status = read_register(ICM42688P_REG_WHO_AM_I,
                           &diagnostics->who_am_i,
                           &diagnostics->bus_status);
    if (status != ICM42688P_STATUS_OK) {
        return status;
    }
    if (diagnostics->who_am_i != ICM42688P_WHO_AM_I_EXPECTED) {
        return ICM42688P_STATUS_WHO_AM_I_MISMATCH;
    }

    status = write_register(ICM42688P_REG_GYRO_CONFIG0,
                            ICM42688P_GYRO_CONFIG_2000DPS_1KHZ,
                            &diagnostics->bus_status);
    if (status != ICM42688P_STATUS_OK) {
        return status;
    }
    status = write_register(ICM42688P_REG_ACCEL_CONFIG0,
                            ICM42688P_ACCEL_CONFIG_16G_1KHZ,
                            &diagnostics->bus_status);
    if (status != ICM42688P_STATUS_OK) {
        return status;
    }
    status = write_register(ICM42688P_REG_PWR_MGMT0,
                            ICM42688P_PWR_ACCEL_GYRO_LOW_NOISE,
                            &diagnostics->bus_status);
    if (status != ICM42688P_STATUS_OK) {
        return status;
    }

    delay_ms(45U);

    status = read_register(ICM42688P_REG_GYRO_CONFIG0,
                           &diagnostics->gyro_config0,
                           &diagnostics->bus_status);
    if (status != ICM42688P_STATUS_OK) {
        return status;
    }
    status = read_register(ICM42688P_REG_ACCEL_CONFIG0,
                           &diagnostics->accel_config0,
                           &diagnostics->bus_status);
    if (status != ICM42688P_STATUS_OK) {
        return status;
    }
    status = read_register(ICM42688P_REG_PWR_MGMT0,
                           &diagnostics->pwr_mgmt0,
                           &diagnostics->bus_status);
    if (status != ICM42688P_STATUS_OK) {
        return status;
    }

    if ((diagnostics->gyro_config0 !=
         ICM42688P_GYRO_CONFIG_2000DPS_1KHZ) ||
        (diagnostics->accel_config0 !=
         ICM42688P_ACCEL_CONFIG_16G_1KHZ) ||
        (diagnostics->pwr_mgmt0 !=
         ICM42688P_PWR_ACCEL_GYRO_LOW_NOISE)) {
        return ICM42688P_STATUS_CONFIG_MISMATCH;
    }

    return icm42688p_read_sample(&warmup_sample,
                                 &diagnostics->bus_status);
}

icm42688p_status_t icm42688p_data_ready(bool *ready,
                                        imu_bus_status_t *bus_status)
{
    uint8_t int_status;
    icm42688p_status_t status;

    if (ready == NULL) {
        return ICM42688P_STATUS_BAD_ARGUMENT;
    }
    status = read_register(ICM42688P_REG_INT_STATUS,
                           &int_status,
                           bus_status);
    if (status != ICM42688P_STATUS_OK) {
        *ready = false;
        return status;
    }

    *ready = (int_status & ICM42688P_INT_STATUS_DATA_READY) != 0U;
    return ICM42688P_STATUS_OK;
}

icm42688p_status_t icm42688p_read_sample(
    icm42688p_raw_sample_t *sample,
    imu_bus_status_t *bus_status)
{
    uint8_t bytes[ICM42688P_BURST_SAMPLE_SIZE];
    imu_bus_status_t status;

    if (sample == NULL) {
        return ICM42688P_STATUS_BAD_ARGUMENT;
    }

    status = imu_bus_read(ICM42688P_REG_TEMP_DATA1,
                          bytes,
                          sizeof(bytes));
    if (bus_status != NULL) {
        *bus_status = status;
    }
    if (status != IMU_BUS_STATUS_OK) {
        return ICM42688P_STATUS_BUS_ERROR;
    }

    decode_sample_bytes(bytes, sample);
    return ICM42688P_STATUS_OK;
}

icm42688p_status_t icm42688p_start_sample_dma(
    imu_bus_status_t *bus_status)
{
    const imu_bus_status_t status =
        imu_bus_read_dma_start(ICM42688P_REG_TEMP_DATA1,
                               ICM42688P_BURST_SAMPLE_SIZE);

    if (bus_status != NULL) {
        *bus_status = status;
    }
    return status == IMU_BUS_STATUS_OK
               ? ICM42688P_STATUS_OK
               : ICM42688P_STATUS_BUS_ERROR;
}

icm42688p_status_t icm42688p_finish_sample_dma(
    icm42688p_raw_sample_t *sample,
    imu_bus_status_t *bus_status)
{
    uint8_t bytes[ICM42688P_BURST_SAMPLE_SIZE];
    imu_bus_status_t status;

    if (sample == NULL) {
        return ICM42688P_STATUS_BAD_ARGUMENT;
    }

    status = imu_bus_read_dma_finish(bytes, sizeof(bytes));
    if (bus_status != NULL) {
        *bus_status = status;
    }
    if (status != IMU_BUS_STATUS_OK) {
        return ICM42688P_STATUS_BUS_ERROR;
    }

    decode_sample_bytes(bytes, sample);
    return ICM42688P_STATUS_OK;
}
