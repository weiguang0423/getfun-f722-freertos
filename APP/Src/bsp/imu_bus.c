/*
 * imu_bus.c - Blocking SPI1/PA4 transaction implementation for ICM42688P.
 *
 * Purpose:
 *   Converts bounded register reads and writes into one HAL full-duplex SPI
 *   transfer while owning chip-select timing and error cleanup.
 *
 * Core functions:
 *   - imu_bus_init(): checks the generated SPI1 handle and deasserts CS.
 *   - imu_bus_read(): sends address|0x80 plus dummy bytes and returns payload.
 *   - imu_bus_write(): sends address&0x7f followed by the supplied payload.
 *
 * Data flow and constraints:
 *   Each call uses a small stack buffer and HAL_SPI_TransmitReceive(). No
 *   dynamic memory, RTOS primitive, UART output, or nested transaction is used.
 *   ImuTask must remain the single owner of this adapter.
 */
#include "bsp/imu_bus.h"

#include <string.h>

#include "main.h"
#include "spi.h"

#define IMU_BUS_MAX_PAYLOAD_SIZE 32U
#define IMU_BUS_SPI_TIMEOUT_MS 5U
#define IMU_BUS_READ_BIT 0x80U
#define IMU_BUS_CONFIGURED_CLOCK_HZ 1687500UL

static imu_bus_status_t imu_bus_status_from_hal(HAL_StatusTypeDef status)
{
    switch (status) {
    case HAL_OK:
        return IMU_BUS_STATUS_OK;
    case HAL_BUSY:
        return IMU_BUS_STATUS_HAL_BUSY;
    case HAL_TIMEOUT:
        return IMU_BUS_STATUS_HAL_TIMEOUT;
    case HAL_ERROR:
    default:
        return IMU_BUS_STATUS_HAL_ERROR;
    }
}

static imu_bus_status_t imu_bus_transfer(const uint8_t *transmit,
                                         uint8_t *receive,
                                         size_t length)
{
    HAL_StatusTypeDef hal_status;

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    hal_status = HAL_SPI_TransmitReceive(&hspi1,
                                        (uint8_t *)transmit,
                                        receive,
                                        (uint16_t)length,
                                        IMU_BUS_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    return imu_bus_status_from_hal(hal_status);
}

imu_bus_status_t imu_bus_init(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    if (hspi1.Instance != SPI1) {
        return IMU_BUS_STATUS_NOT_READY;
    }
    if (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {
        return IMU_BUS_STATUS_NOT_READY;
    }
    return IMU_BUS_STATUS_OK;
}

imu_bus_status_t imu_bus_read(uint8_t register_address,
                              uint8_t *data,
                              size_t length)
{
    uint8_t transmit[IMU_BUS_MAX_PAYLOAD_SIZE + 1U] = {0};
    uint8_t receive[IMU_BUS_MAX_PAYLOAD_SIZE + 1U] = {0};
    imu_bus_status_t status;

    if ((data == NULL) || (length == 0U) ||
        (length > IMU_BUS_MAX_PAYLOAD_SIZE)) {
        return IMU_BUS_STATUS_BAD_ARGUMENT;
    }

    transmit[0] = register_address | IMU_BUS_READ_BIT;
    status = imu_bus_transfer(transmit, receive, length + 1U);
    if (status == IMU_BUS_STATUS_OK) {
        memcpy(data, &receive[1], length);
    }
    return status;
}

imu_bus_status_t imu_bus_write(uint8_t register_address,
                               const uint8_t *data,
                               size_t length)
{
    uint8_t transmit[IMU_BUS_MAX_PAYLOAD_SIZE + 1U] = {0};
    uint8_t receive[IMU_BUS_MAX_PAYLOAD_SIZE + 1U] = {0};

    if ((data == NULL) || (length == 0U) ||
        (length > IMU_BUS_MAX_PAYLOAD_SIZE)) {
        return IMU_BUS_STATUS_BAD_ARGUMENT;
    }

    transmit[0] = register_address & (uint8_t)~IMU_BUS_READ_BIT;
    memcpy(&transmit[1], data, length);
    return imu_bus_transfer(transmit, receive, length + 1U);
}

uint32_t imu_bus_clock_hz(void)
{
    return IMU_BUS_CONFIGURED_CLOCK_HZ;
}
