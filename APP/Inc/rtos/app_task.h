/*
 * app_task.h —— 应用层 RTOS 任务的初始化接口
 *
 * 本头文件只暴露一个入口：app_tasks_init()。
 * 它由 CubeMX 的 MX_FREERTOS_Init()（Core/Src/freertos.c）调用，负责初始化
 * app_state、USB CDC transport，并静态创建 ImuTask 与 MspTask。详见 app_task.c。
 */
#ifndef APP_TASK_H
#define APP_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

void app_tasks_init(void);

#ifdef __cplusplus
}
#endif

#endif
