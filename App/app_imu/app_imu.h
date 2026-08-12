/**
 * @file app_imu.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief IMU 应用模块接口 -- 基于扩展卡尔曼滤波的姿态估计
 * @version 1.0
 * @date 2026-08-12
 * @copyright Copyright (c) 2026
 *
 * @note 定义 IMU 配置结构体及姿态估计接口，通过 EKF 融合 BMI088 加速度计/陀螺仪数据发布姿态快照。
 */

#ifndef APP_IMU_H
#define APP_IMU_H

#include "bsp_common.h"
#include "alg_imu_ekf.h"
#include "app_types.h"
#include "module_bmi088.h"

#include <stdbool.h>

/** @brief IMU 模块的静态配置。 */
typedef struct
{
    module_bmi088_t *sensor;                /**< BMI088 传感器实例。 */
    const alg_imu_ekf_config_t *ekf_config; /**< EKF 调参配置（NULL 则使用默认值）。 */
} app_imu_config_t;

/** @brief IMU 运行时实例。 */
typedef struct
{
    module_bmi088_t *sensor;             /**< BMI088 传感器句柄。 */
    alg_imu_ekf_t ekf;                   /**< EKF 状态。 */
    app_imu_snapshot_t snapshot;         /**< 最新姿态快照。 */
    bool attitude_initialized;           /**< 俯仰/横滚已由加速度计初始化。 */
    bool initialized;                    /**< 初始化阶段已成功完成。 */
} app_imu_t;

/**
 * @brief  初始化 IMU 模块。
 * @param  me      指向调用方分配的实例。
 * @param  config  静态配置（内部拷贝）。
 * @return 成功返回 BSP_STATUS_OK。
 */
bsp_status_t app_imu_init(app_imu_t *me, const app_imu_config_t *config);

/**
 * @brief  执行一个 IMU 姿态估计周期。
 * @param  me            已初始化的 IMU 实例。
 * @param  delta_time_s  距上次调用的经过时间 [s]（必须 > 0）。
 */
void app_imu_update(app_imu_t *me, float delta_time_s);

#endif
