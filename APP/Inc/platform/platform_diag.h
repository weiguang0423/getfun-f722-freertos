/*
 * platform_diag.h —— 平台诊断与安全停机公共接口
 *
 * 作用：
 *   为启动日志、RTOS 心跳、Motor 安全电平和统一故障停机提供应用层接口。
 *
 * 核心内容：
 *   - platform_fault_code_t：可通过 SWD 读取的平台故障码。
 *   - platform_motor_outputs_force_safe()：强制 Motor 1～8 为 GPIO 低电平。
 *   - platform_diag_startup()/rtos_started()/heartbeat()：输出平台运行信息和
 *       InitTask 维护的 1 Hz IMU 摘要。
 *   - platform_fault_halt()：记录故障、关闭中断并停在安全循环。
 *   - platform_freertos_assert_failed()：保存断言文件和行号后安全停机。
 */
#ifndef PLATFORM_DIAG_H
#define PLATFORM_DIAG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    PLATFORM_FAULT_NONE = 0,
    PLATFORM_FAULT_HAL_ERROR = 1,
    PLATFORM_FAULT_STACK_OVERFLOW = 2,
    PLATFORM_FAULT_MALLOC_FAILED = 3,
    PLATFORM_FAULT_FREERTOS_ASSERT = 4,
    PLATFORM_FAULT_HARD = 5,
    PLATFORM_FAULT_MEMORY_MANAGEMENT = 6,
    PLATFORM_FAULT_BUS = 7,
    PLATFORM_FAULT_USAGE = 8,
    PLATFORM_FAULT_SCHEDULER_RETURNED = 9
} platform_fault_code_t;

/*
 * These values intentionally remain globally visible so they can be inspected
 * over SWD after the CPU enters the fault halt loop.
 */
extern volatile uint32_t g_platform_fault_code;
extern volatile uint32_t g_platform_fault_line;
extern const char *volatile g_platform_fault_file;

void platform_motor_outputs_force_safe(void);
void platform_diag_startup(void);
void platform_diag_rtos_started(void);
void platform_diag_heartbeat(void);

void platform_fault_halt(platform_fault_code_t code)
    __attribute__((noreturn));
void platform_freertos_assert_failed(const char *file, uint32_t line)
    __attribute__((noreturn));

#ifdef __cplusplus
}
#endif

#endif
