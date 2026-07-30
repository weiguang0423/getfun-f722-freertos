/*
 * gyro_calibration.h —— 上电陀螺静态零偏校准状态机接口
 *
 * 作用：
 *   对已经换算为机体系 rad/s 的三轴角速度执行确定性的上电静态校准，并在校准
 *   完成后提供零偏扣除。模块不依赖 RTOS、HAL、日志或全局应用状态。
 *
 * 核心接口与数据流：
 *   gyro_calibration_start() -> gyro_calibration_process() ->
 *   gyro_calibration_apply()。调用者每得到一个新的 IMU 样本就处理一次；状态机先
 *   等待连续静止预热，再累计稳定窗口并用方差复核，最后冻结三轴零偏。
 *
 * 关键约束：
 *   输入角速度必须已经完成板级轴向变换，单位为 rad/s；加速度单位为 m/s^2。
 *   校准只在传感器初始化或故障恢复后重新开始，READY 后不会在飞行中自动改写
 *   零偏；不使用动态内存。
 */
#ifndef GYRO_CALIBRATION_H
#define GYRO_CALIBRATION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GYRO_CALIBRATION_AXIS_COUNT 3U
#define GYRO_CALIBRATION_WARMUP_SAMPLES 250U
#define GYRO_CALIBRATION_REQUIRED_SAMPLES 2000U

typedef enum
{
    GYRO_CALIBRATION_NOT_STARTED = 0,
    GYRO_CALIBRATION_CALIBRATING,
    GYRO_CALIBRATION_READY
} gyro_calibration_state_t;

typedef struct
{
    gyro_calibration_state_t state;
    uint16_t warmup_sample_count;
    uint16_t stable_sample_count;
    uint32_t restart_count;
    uint32_t motion_reject_count;
    uint32_t invalid_sample_count;
    float bias_rad_s[GYRO_CALIBRATION_AXIS_COUNT];
    float mean_rad_s[GYRO_CALIBRATION_AXIS_COUNT];
    float m2_rad_s2[GYRO_CALIBRATION_AXIS_COUNT];
} gyro_calibration_t;

void gyro_calibration_reset(gyro_calibration_t *calibration);
void gyro_calibration_start(gyro_calibration_t *calibration);
gyro_calibration_state_t gyro_calibration_process(
    gyro_calibration_t *calibration,
    const float angular_rate_rad_s[GYRO_CALIBRATION_AXIS_COUNT],
    const float acceleration_m_s2[GYRO_CALIBRATION_AXIS_COUNT]);
void gyro_calibration_apply(
    const gyro_calibration_t *calibration,
    const float angular_rate_rad_s[GYRO_CALIBRATION_AXIS_COUNT],
    float corrected_rad_s[GYRO_CALIBRATION_AXIS_COUNT]);

#ifdef __cplusplus
}
#endif

#endif
