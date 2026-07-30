/*
 * imu_task.h —— DMA IMU 采样的静态 FreeRTOS 任务接口
 *
 * 作用:
 *   对外暴露唯一 IMU 所有者任务的创建接口、加速度校准请求入口和低频栈诊断
 *   接口。DMA完成通知与陀螺/加速度校准上下文保持为imu_task.c内部私有。
 *
 * 核心接口:
 *   - imu_task_create(): 创建 1 kHz DRDY 门控的 DMA ImuTask。
 *   - imu_task_request_accel_calibration(): 排队一次水平加速度校准请求。
 *   - imu_task_stack_high_water_mark(): 返回最小空闲栈字数。
 *
 * 约束:
 *   任务在调度器启动前由 app_tasks_init() 一次性创建。
 *   只有低频诊断任务可调用栈查询函数。
 */
#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void imu_task_create(void);
bool imu_task_request_accel_calibration(void);
uint32_t imu_task_stack_high_water_mark(void);

#ifdef __cplusplus
}
#endif

#endif
