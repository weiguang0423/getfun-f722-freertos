/*
 * platform_time.h —— Cortex-M7 DWT 微秒时间源接口
 *
 * 作用：
 *   使用DWT CYCCNT提供不占用Timer、DMA或中断的微秒时间。DMA ISR只调用
 *   platform_time_capture_cycles()捕获原始周期计数，ImuTask作为唯一状态写者
 *   调用resolve/maintain完成32位回绕扩展和周期到微秒的余数累计。
 *
 * 关键约束：
 *   216 MHz下原始CYCCNT约19.88秒回绕；resolve或maintain的相邻调用间隔必须
 *   严格短于该周期。对外uint32微秒时间戳允许约71.58分钟自然回绕，间隔必须
 *   使用无符号减法。不使用动态内存，不依赖FreeRTOS或HAL tick。
 */
#ifndef PLATFORM_TIME_H
#define PLATFORM_TIME_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool platform_time_initialize(void);
bool platform_time_ready(void);
uint32_t platform_time_capture_cycles(void);
bool platform_time_resolve_cycles(uint32_t cycle_count,
                                  uint32_t *timestamp_us);
bool platform_time_maintain(uint32_t *timestamp_us);

#ifdef __cplusplus
}
#endif

#endif
