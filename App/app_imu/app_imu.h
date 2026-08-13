/**
 * @file app_imu.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief IMU 应用模块接口 -- BMI088 采样与 EKF 姿态解算
 * @version 1.0
 * @date 2026-08-12
 * @copyright Copyright (c) 2026
 *
 * @note 定义 IMU 配置与实例结构体，管理 BMI088 采样、温度控制和 EKF 姿态解算。
 */

#ifndef APP_IMU_H
#define APP_IMU_H

#include "alg_imu_ekf.h"
#include "app_types.h"
#include "bsp_common.h"
#include "module_bmi088.h"
#include <stdbool.h>

/** @brief 加热器 PWM 输出回调。 */
typedef void (*app_imu_set_heater_t)(void *context, float duty_ratio);

/** @brief IMU 运行状态。 */
typedef enum
{
    APP_IMU_STATE_UNCALIBRATED = 0,  /**< 未标定。 */
    APP_IMU_STATE_WARMING,           /**< 预热中（温度未稳定）。 */
    APP_IMU_STATE_READY,             /**< 就绪，姿态有效。 */
    APP_IMU_STATE_FAULT              /**< 故障。 */
} app_imu_state_t;

/** @brief IMU 模块的静态配置。 */
typedef struct
{
    module_bmi088_t *sensor;                       /**< BMI088 传感器实例（需已初始化）。 */
    const alg_imu_ekf_config_t *ekf_config;        /**< EKF 调参配置，NULL 时使用默认参数。 */
    uint32_t calibration_sample_count;             /**< 标定采样次数。 */
    uint32_t calibration_sample_interval_ms;       /**< 标定采样间隔 [ms]。 */
    float calibration_max_gyro_deviation_rad_per_s;        /**< 标定允许的最大陀螺偏差 [rad/s]。 */
    float calibration_max_acceleration_deviation_m_per_s2; /**< 标定允许的最大加速度偏差 [m/s^2]。 */
    float target_temperature_c;                    /**< 目标温度 [°C]。 */
    float temperature_tolerance_c;                 /**< 温度稳定容差 [°C]。 */
    float temperature_stable_time_s;               /**< 判定温度稳定的持续时间 [s]。 */
    float heater_kp;                               /**< 加热器 PID 比例系数。 */
    float heater_ki;                               /**< 加热器 PID 积分系数。 */
    app_imu_set_heater_t set_heater;               /**< 加热器输出回调，可为 NULL。 */
    void *heater_context;                          /**< 加热器回调上下文。 */
} app_imu_config_t;

/** @brief IMU 运行时实例。 */
typedef struct
{
    app_imu_config_t config;                 /**< 静态配置的副本。 */
    alg_imu_ekf_t ekf;                       /**< EKF 滤波器状态。 */
    app_imu_snapshot_t snapshot;             /**< 最近一次姿态快照。 */
    float heater_integral;                   /**< 加热器 PID 积分项。 */
    float temperature_stable_elapsed_s;      /**< 温度已稳定持续时间 [s]。 */
    app_imu_state_t state;                   /**< 当前运行状态。 */
    bool attitude_initialized;               /**< 俯仰/横滚已由加速度计初始化。 */
    bool initialized;                        /**< 初始化阶段已成功完成。 */
} app_imu_t;

/**
 * @brief  初始化 IMU 模块及 EKF 滤波器。
 * @param  me      指向调用方分配的实例。
 * @param  config  静态配置（内部拷贝）。
 * @return 成功返回 BSP_STATUS_OK，参数无效返回 BSP_STATUS_INVALID_ARGUMENT。
 *
 * 若未提供 EKF 配置，则使用默认参数初始化滤波器。
 */
bsp_status_t app_imu_init(app_imu_t *me, const app_imu_config_t *config);

/**
 * @brief  执行 IMU 标定。
 * @param  me  已初始化的 IMU 实例。
 * @return 成功返回 BSP_STATUS_OK，标定失败返回 BSP_STATUS_IO_ERROR。
 */
bsp_status_t app_imu_calibrate(app_imu_t *me);

/**
 * @brief  执行一个 IMU 姿态估计周期。
 * @param  me            已初始化的 IMU 实例。
 * @param  delta_time_s  距上次调用的经过时间 [s]（必须 > 0）。
 * @return 成功返回 BSP_STATUS_OK，传感器读取失败返回 BSP_STATUS_IO_ERROR。
 */
bsp_status_t app_imu_update(app_imu_t *me, float delta_time_s);

/**
 * @brief  读取最近一次姿态快照。
 * @param  me  已初始化的 IMU 实例。
 * @return 只读快照指针，实例无效时返回 NULL。
 */
const app_imu_snapshot_t *app_imu_get_snapshot(const app_imu_t *me);

/**
 * @brief  读取当前运行状态。
 * @param  me  已初始化的 IMU 实例。
 * @return 当前状态，实例无效时返回 APP_IMU_STATE_FAULT。
 */
app_imu_state_t app_imu_get_state(const app_imu_t *me);

#endif
