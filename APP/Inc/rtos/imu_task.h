/*
 * imu_task.h - Static FreeRTOS task interface for ICM42688P sampling.
 *
 * Purpose:
 *   Exposes creation and low-rate stack diagnostics for the single IMU owner.
 *
 * Core interfaces:
 *   - imu_task_create(): creates the 1 kHz statically allocated ImuTask.
 *   - imu_task_stack_high_water_mark(): returns the minimum free stack words.
 *
 * Constraints:
 *   Creation occurs once from app_tasks_init() before the scheduler starts.
 *   Only the low-rate diagnostic task may call the stack query function.
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
