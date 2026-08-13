/**
 * @file app_imu.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief IMU 应用模块实现
 * @version 1.0
 * @date 2026-08-12
 * @copyright Copyright (c) 2026
 *
 * @note 读取 BMI088 传感器数据，通过 EKF 融合陀螺仪/加速度计更新姿态快照。
 */

#include "app_imu.h"

#include <math.h>

/* ======================== 公共 API ======================== */

/**
 * @brief  初始化 IMU 模块。
 * @param  me      指向调用方分配的实例。
 * @param  config  静态配置（内部拷贝）。
 * @return 成功返回 BSP_STATUS_OK，参数无效返回 BSP_STATUS_INVALID_ARGUMENT。
 *
 * 若未提供 EKF 配置，则使用默认参数初始化滤波器。
 */
bsp_status_t app_imu_init(app_imu_t *me, const app_imu_config_t *config)
{
    alg_imu_ekf_config_t default_ekf_config;
    const alg_imu_ekf_config_t *ekf_config;

    if ((me == NULL) || (config == NULL) || (config->sensor == NULL) ||
        !config->sensor->is_initialized)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    /* 未提供 EKF 配置时，使用默认参数。 */
    if (config->ekf_config == NULL)
    {
        if (alg_imu_ekf_config_init(&default_ekf_config) != ALG_IMU_EKF_STATUS_OK)
        {
            return BSP_STATUS_INVALID_ARGUMENT;
        }
        ekf_config = &default_ekf_config;
    }
    else
    {
        ekf_config = config->ekf_config;
    }

    *me = (app_imu_t){.sensor = config->sensor};
    if (alg_imu_ekf_init(&me->ekf, ekf_config) != ALG_IMU_EKF_STATUS_OK)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    me->initialized = true;
    return BSP_STATUS_OK;
}

/**
 * @brief  执行一个 IMU 姿态估计周期。
 * @param  me            已初始化的 IMU 实例。
 * @param  delta_time_s  距上次调用的经过时间 [s]（必须 > 0）。
 *
 * 传感器读取失败时快照置为无效；首次加速度计采样用于初始化
 * 姿态，之后每周期通过 EKF 更新姿态估计。
 */
bsp_status_t app_imu_update(app_imu_t *me, float delta_time_s)
{
    const module_bmi088_process_data_t *data;
    alg_imu_ekf_euler_t euler;
    float continuous_yaw_rad;
    float corrected_gyroscope[3];
    alg_imu_ekf_status_t status;

    if ((me == NULL) || !isfinite(delta_time_s) || (delta_time_s <= 0.0F))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!me->initialized)
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    if (module_bmi088_read(me->sensor) != MODULE_BMI088_STATUS_OK)
    {
        me->snapshot.valid = false;
        return BSP_STATUS_IO_ERROR;
    }
    data = module_bmi088_get_data(me->sensor);
    if ((data == NULL) || !data->is_valid)
    {
        me->snapshot.valid = false;
        return BSP_STATUS_IO_ERROR;
    }

    /* 首次有效加速度计数据用于初始化俯仰/横滚姿态。 */
    if (!me->attitude_initialized)
    {
        status = alg_imu_ekf_reset_from_accelerometer(&me->ekf, data->acceleration_m_per_s2);
        me->attitude_initialized = (status == ALG_IMU_EKF_STATUS_OK);
    }
    else
    {
        /* 正常 EKF 更新：融合陀螺仪与加速度计数据。 */
        status = alg_imu_ekf_update(&me->ekf, data->angular_velocity_rad_per_s,
                                    data->acceleration_m_per_s2, delta_time_s, NULL);
    }
    /* 提取欧拉角与修正后的陀螺仪数据。 */
    if ((status != ALG_IMU_EKF_STATUS_OK) ||
        (alg_imu_ekf_get_euler(&me->ekf, &euler) != ALG_IMU_EKF_STATUS_OK) ||
        (alg_imu_ekf_get_continuous_yaw(&me->ekf, &continuous_yaw_rad) !=
         ALG_IMU_EKF_STATUS_OK) ||
        (alg_imu_ekf_get_corrected_gyroscope(&me->ekf, data->angular_velocity_rad_per_s,
                                             corrected_gyroscope) != ALG_IMU_EKF_STATUS_OK))
    {
        me->snapshot.valid = false;
        return BSP_STATUS_IO_ERROR;
    }

    me->snapshot.roll_rad = euler.roll_rad;
    me->snapshot.pitch_rad = euler.pitch_rad;
    me->snapshot.yaw_rad = euler.yaw_rad;
    me->snapshot.continuous_yaw_rad = continuous_yaw_rad;
    me->snapshot.angular_velocity_rad_per_s[0] = corrected_gyroscope[0];
    me->snapshot.angular_velocity_rad_per_s[1] = corrected_gyroscope[1];
    me->snapshot.angular_velocity_rad_per_s[2] = corrected_gyroscope[2];
    me->snapshot.sample_count = data->sample_count;
    me->snapshot.valid = true;
    return BSP_STATUS_OK;
}

const app_imu_snapshot_t *app_imu_get_snapshot(const app_imu_t *me)
{
    return ((me != NULL) && me->initialized) ? &me->snapshot : NULL;
}
