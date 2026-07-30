/*
 * imu_filter.h —— 校准后IMU三轴PT1低通接口
 *
 * 作用：
 *   对机体系、SI单位、已经完成零偏校准的Gyro/Accel数据执行按真实dt更新的
 *   一阶低通。模块不依赖HAL、RTOS、日志或全局状态。
 *
 * 数据语义：
 *   第一个有限样本只播种状态并返回false；后续有效dt样本返回true。输入、dt或
 *   计算结果无效时用当前有限输入重新播种并返回false，调用者不得把该样本交给
 *   后续姿态积分。
 */
#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_FILTER_AXIS_COUNT 3U
#define IMU_FILTER_GYRO_CUTOFF_HZ 100.0f
#define IMU_FILTER_ACCEL_CUTOFF_HZ 30.0f
#define IMU_FILTER_MIN_INTERVAL_US 500U
#define IMU_FILTER_MAX_INTERVAL_US 2000U
#define IMU_FILTER_MIN_DT_S \
    ((float)IMU_FILTER_MIN_INTERVAL_US * 0.000001f)
#define IMU_FILTER_MAX_DT_S \
    ((float)IMU_FILTER_MAX_INTERVAL_US * 0.000001f)

typedef struct
{
    bool initialized;
    float acceleration_state_m_s2[IMU_FILTER_AXIS_COUNT];
    float angular_rate_state_rad_s[IMU_FILTER_AXIS_COUNT];
} imu_filter_t;

void imu_filter_reset(imu_filter_t *filter);
bool imu_filter_process(
    imu_filter_t *filter,
    const float acceleration_m_s2[IMU_FILTER_AXIS_COUNT],
    const float angular_rate_rad_s[IMU_FILTER_AXIS_COUNT],
    float dt_s,
    float filtered_acceleration_m_s2[IMU_FILTER_AXIS_COUNT],
    float filtered_angular_rate_rad_s[IMU_FILTER_AXIS_COUNT]);

#ifdef __cplusplus
}
#endif

#endif
