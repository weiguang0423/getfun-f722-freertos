/*
 * app_task.h —— 应用层 RTOS 任务的初始化接口
 *
 * 本头文件只暴露一个入口：app_tasks_init()。
 * 它由 CubeMX 的 MX_FREERTOS_Init()（Core/Src/freertos.c）调用，负责创建应用层任务
 * 并初始化任务所需的子模块（app_state、USB CDC transport 等）。详见 app_task.c。
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
