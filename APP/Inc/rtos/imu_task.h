/*
 * imu_task.h —— DMA IMU 采样的静态 FreeRTOS 任务接口
 *
 * 作用:
 *   对外暴露唯一 IMU 所有者任务的创建接口和低频栈诊断接口。DMA 完成通知
 *   保持为 imu_task.c 内部私有。
 *
 * 核心接口:
 *   - imu_task_create(): 创建 1 kHz DRDY 门控的 DMA ImuTask。
 *   - imu_task_stack_high_water_mark(): 返回最小空闲栈字数。
 *
 * 约束:
 *   任务在调度器启动前由 app_tasks_init() 一次性创建。
 *   只有低频诊断任务可调用栈查询函数。
 */
#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void imu_task_create(void);
uint32_t imu_task_stack_high_water_mark(void);

#ifdef __cplusplus
}
#endif

#endif
