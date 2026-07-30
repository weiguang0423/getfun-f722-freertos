/*
 * platform_diag.c —— 平台基线诊断、心跳监控与故障安全停机
 *
 * 作用：
 *   通过 UART4 提供启动和运行状态的可观测性，并在 HAL、CPU 或 FreeRTOS
 *   故障发生时立即把 Motor 1～8 切回低电平，并保留供 SWD 读取的故障信息。
 *
 * 核心函数：
 *   - platform_motor_outputs_force_safe()：直接配置 GPIO 寄存器并拉低所有 Motor。
 *   - platform_diag_startup()：输出固件身份、构建时间和运行时时钟。
 *   - platform_diag_heartbeat()：输出平台心跳，以及 1 Hz IMU 在线状态、采样率、
 *       DRDY/DMA 计数、错误计数、陀螺校准进度/零偏、SI 物理量和 ImuTask
 *       栈余量。
 *   - platform_fault_halt()：记录故障码、关闭中断、强制安全输出并停止运行。
 *   - platform_freertos_assert_failed()：记录 FreeRTOS 断言位置后进入安全停机。
 *
 * 关键约束：
 *   UART 输出只用于启动和 InitTask 的 1 Hz 维护路径；ImuTask 和中断不得调用。
 */
#include "platform/platform_diag.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "algorithms/gyro_calibration.h"
#include "app_state.h"
#include "bsp/imu_bus.h"
#include "main.h"
#include "rtos/imu_task.h"
#include "task.h"
#include "usart.h"

#define DIAG_TX_TIMEOUT_MS 100U

#define MOTOR_GPIOA_PINS \
    (MOTOR1_Pin | MOTOR2_Pin | MOTOR3_Pin | MOTOR4_Pin)
#define MOTOR_GPIOB_PINS (MOTOR7_Pin | MOTOR8_Pin)
#define MOTOR_GPIOC_PINS (MOTOR5_Pin | MOTOR6_Pin)

volatile uint32_t g_platform_fault_code;
volatile uint32_t g_platform_fault_line;
const char *volatile g_platform_fault_file;

static void motor_port_force_output_low(GPIO_TypeDef *port, uint32_t pins)
{
    uint32_t bit;
    uint32_t mode = port->MODER;
    uint32_t pull = port->PUPDR;
    uint32_t speed = port->OSPEEDR;

    /* 将输出锁存器拉低后，再把引脚切换为复用功能模式或输入模式。 */
    port->BSRR = pins << 16U;

    for (bit = 0U; bit < 16U; ++bit) {
        const uint32_t pin = 1UL << bit;
        const uint32_t field_shift = bit * 2U;
        const uint32_t field_mask = 3UL << field_shift;

        if ((pins & pin) != 0U) {
            mode = (mode & ~field_mask) | (1UL << field_shift);
            pull &= ~field_mask;
            speed &= ~field_mask;
        }
    }

    port->OTYPER &= ~pins;
    port->PUPDR = pull;
    port->OSPEEDR = speed;
    port->MODER = mode;
    port->BSRR = pins << 16U;
    __DSB();
}

void platform_motor_outputs_force_safe(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN;
    __DSB();

    motor_port_force_output_low(GPIOA, MOTOR_GPIOA_PINS);
    motor_port_force_output_low(GPIOB, MOTOR_GPIOB_PINS);
    motor_port_force_output_low(GPIOC, MOTOR_GPIOC_PINS);
}

static void diag_write(const char *text, size_t length)
{
    if ((text == NULL) || (length == 0U) || (length > UINT16_MAX)) {
        return;
    }

    if ((huart4.Instance != UART4) ||
        (huart4.gState != HAL_UART_STATE_READY)) {
        return;
    }

    (void)HAL_UART_Transmit(&huart4,
                            (const uint8_t *)text,
                            (uint16_t)length,
                            DIAG_TX_TIMEOUT_MS);
}

static void diag_write_string(const char *text)
{
    if (text != NULL) {
        diag_write(text, strlen(text));
    }
}

static void diag_write_formatted(char *buffer,
                                 size_t capacity,
                                 int length)
{
    if ((buffer == NULL) || (capacity == 0U) || (length <= 0)) {
        return;
    }

    if ((size_t)length >= capacity) {
        length = (int)(capacity - 1U);
    }
    diag_write(buffer, (size_t)length);
}

static int32_t diag_float_to_milli(float value)
{
    const float scaled = value * 1000.0f;

    if (!isfinite(scaled)) {
        return 0;
    }
    if (scaled >= (float)INT32_MAX) {
        return INT32_MAX;
    }
    if (scaled <= (float)INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)lroundf(scaled);
}

void platform_diag_startup(void)
{
    char line[192];
    int length;

    diag_write_string("\r\nGETFUN F722 FreeRTOS\r\n");
    diag_write_string("MCU: STM32F722RET6\r\n");

    length = snprintf(line,
                      sizeof(line),
                      "BUILD: %s %s\r\n"
                      "SYSCLK: %lu\r\n"
                      "HCLK: %lu\r\n"
                      "PCLK1: %lu\r\n"
                      "PCLK2: %lu\r\n",
                      __DATE__,
                      __TIME__,
                      (unsigned long)HAL_RCC_GetSysClockFreq(),
                      (unsigned long)HAL_RCC_GetHCLKFreq(),
                      (unsigned long)HAL_RCC_GetPCLK1Freq(),
                      (unsigned long)HAL_RCC_GetPCLK2Freq());
    diag_write_formatted(line, sizeof(line), length);
    diag_write_string("MOTORS: safe GPIO low\r\n");
    length = snprintf(line,
                      sizeof(line),
                      "IMU: SPI1 %lu Hz mode=0 dma2 rx=s0 tx=s3 "
                      "trigger=drdy_poll\r\n",
                      (unsigned long)imu_bus_clock_hz());
    diag_write_formatted(line, sizeof(line), length);
    diag_write_string("RTOS: starting\r\n");
}

void platform_diag_rtos_started(void)
{
    diag_write_string("RTOS: started\r\n");
}

void platform_diag_heartbeat(void)
{
    static bool have_previous_sample;
    static TickType_t previous_tick;
    static uint32_t previous_sample_count;
    app_state_snapshot_t snapshot;
    const TickType_t current_tick = xTaskGetTickCount();
    const TickType_t elapsed_ticks = current_tick - previous_tick;
    uint32_t sample_rate_hz = 0U;
    char line[256];
    int length;

    app_state_get_snapshot(&snapshot);
    if (have_previous_sample && (elapsed_ticks != 0U)) {
        const uint32_t sample_delta =
            snapshot.imu.sample_count - previous_sample_count;
        sample_rate_hz =
            (uint32_t)(((uint64_t)sample_delta * configTICK_RATE_HZ) /
                       elapsed_ticks);
    }

    length = snprintf(
        line,
        sizeof(line),
        "heartbeat tick=%lu heap=%lu min_heap=%lu init_stack_min=%lu\r\n",
        (unsigned long)xTaskGetTickCount(),
        (unsigned long)xPortGetFreeHeapSize(),
        (unsigned long)xPortGetMinimumEverFreeHeapSize(),
        (unsigned long)uxTaskGetStackHighWaterMark(NULL));
    diag_write_formatted(line, sizeof(line), length);

    length = snprintf(
        line,
        sizeof(line),
        "imu dma_ok=%lu drdy_poll=%lu dma_start_err=%lu "
        "dma_cplt_err=%lu dma_timeout=%lu dma_abort=%lu\r\n",
        (unsigned long)snapshot.imu.dma_transfer_count,
        (unsigned long)snapshot.imu.drdy_poll_count,
        (unsigned long)snapshot.imu.dma_start_error_count,
        (unsigned long)snapshot.imu.dma_completion_error_count,
        (unsigned long)snapshot.imu.dma_timeout_count,
        (unsigned long)snapshot.imu.dma_abort_count);
    diag_write_formatted(line, sizeof(line), length);

    length = snprintf(
        line,
        sizeof(line),
        "imu present=%u who=0x%02X status=%u bus=%u hz=%lu samples=%lu "
        "init_errors=%lu read_errors=%lu not_ready=%lu missed=%lu\r\n",
        snapshot.imu.present ? 1U : 0U,
        snapshot.imu.who_am_i,
        snapshot.imu.last_error,
        snapshot.imu.last_bus_error,
        (unsigned long)sample_rate_hz,
        (unsigned long)snapshot.imu.sample_count,
        (unsigned long)snapshot.imu.initialization_error_count,
        (unsigned long)snapshot.imu.read_error_count,
        (unsigned long)snapshot.imu.data_not_ready_count,
        (unsigned long)snapshot.imu.missed_deadline_count);
    diag_write_formatted(line, sizeof(line), length);

    length = snprintf(
        line,
        sizeof(line),
        "imu cal=%u progress=%u/%u restart=%lu motion=%lu invalid=%lu "
        "inhibit=0x%08lX bias_mrad_s=[%ld,%ld,%ld]\r\n",
        (unsigned int)snapshot.imu.gyro_calibration_state,
        (unsigned int)snapshot.imu.gyro_calibration_sample_count,
        (unsigned int)GYRO_CALIBRATION_REQUIRED_SAMPLES,
        (unsigned long)snapshot.imu.gyro_calibration_restart_count,
        (unsigned long)snapshot.imu.gyro_calibration_motion_reject_count,
        (unsigned long)snapshot.imu.gyro_calibration_invalid_sample_count,
        (unsigned long)snapshot.arming_inhibit_flags,
        (long)diag_float_to_milli(snapshot.imu.gyro_bias_rad_s[0]),
        (long)diag_float_to_milli(snapshot.imu.gyro_bias_rad_s[1]),
        (long)diag_float_to_milli(snapshot.imu.gyro_bias_rad_s[2]));
    diag_write_formatted(line, sizeof(line), length);

    length = snprintf(
        line,
        sizeof(line),
        "imu acc_mm_s2=[%ld,%ld,%ld] gyro_mrad_s=[%ld,%ld,%ld] "
        "temp_mc=%ld imu_stack_min=%lu cfg=[%02X,%02X,%02X]\r\n",
        (long)diag_float_to_milli(snapshot.imu.acceleration_m_s2[0]),
        (long)diag_float_to_milli(snapshot.imu.acceleration_m_s2[1]),
        (long)diag_float_to_milli(snapshot.imu.acceleration_m_s2[2]),
        (long)diag_float_to_milli(snapshot.imu.angular_rate_rad_s[0]),
        (long)diag_float_to_milli(snapshot.imu.angular_rate_rad_s[1]),
        (long)diag_float_to_milli(snapshot.imu.angular_rate_rad_s[2]),
        (long)diag_float_to_milli(snapshot.imu.temperature_c),
        (unsigned long)imu_task_stack_high_water_mark(),
        snapshot.imu.gyro_config0,
        snapshot.imu.accel_config0,
        snapshot.imu.pwr_mgmt0);
    diag_write_formatted(line, sizeof(line), length);

    previous_tick = current_tick;
    previous_sample_count = snapshot.imu.sample_count;
    have_previous_sample = true;
}

void platform_fault_halt(platform_fault_code_t code)
{
    if (code != PLATFORM_FAULT_FREERTOS_ASSERT) {
        g_platform_fault_file = NULL;
        g_platform_fault_line = 0U;
    }
    g_platform_fault_code = (uint32_t)code;
    __DMB();

    __disable_irq();
    platform_motor_outputs_force_safe();

    for (;;) {
        __NOP();
    }
}

void platform_freertos_assert_failed(const char *file, uint32_t line)
{
    g_platform_fault_file = file;
    g_platform_fault_line = line;
    __DMB();
    platform_fault_halt(PLATFORM_FAULT_FREERTOS_ASSERT);
}
