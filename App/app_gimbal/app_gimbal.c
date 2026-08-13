/**
 * @file app_gimbal.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 云台应用模块实现
 * @version 1.0
 * @date 2026-08-12
 * @copyright Copyright (c) 2026
 *
 * @note 根据反馈模式选择编码器或 IMU 姿态驱动俯仰/偏航电机到目标角度，并发布通用云台反馈。
 */

#include "app_gimbal.h"

#include <math.h>

/* ======================== 公共 API ======================== */

/**
 * @brief  初始化云台模块。
 * @param  me      指向调用方分配的实例。
 * @param  config  静态配置（内部拷贝）。
 * @return 成功返回 BSP_STATUS_OK，参数无效返回 BSP_STATUS_INVALID_ARGUMENT。
 */
bsp_status_t app_gimbal_init(app_gimbal_t *me, const app_gimbal_config_t *config)
{
    if ((me == NULL) || (config == NULL) || (config->pitch_motor == NULL) ||
        (config->yaw_motor == NULL) ||
        (config->target_tolerance_rad < 0.0F))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *me = (app_gimbal_t){
        .config = *config,
        .initialized = true,
    };
    return BSP_STATUS_OK;
}

/**
 * @brief  执行一个云台控制周期。
 * @param  me            已初始化的云台实例。
 * @param  delta_time_s  距上次调用的经过时间 [s]。
 *
 * 从交换层读取云台指令和 IMU 快照，根据反馈模式选择编码器或 IMU
 * 姿态作为位置反馈，驱动俯仰和偏航电机到目标角度，并将反馈
 * 通过交换层发布。
 */
bsp_status_t app_gimbal_update(app_gimbal_t *me,
                               const app_gimbal_command_t *command,
                               const app_imu_snapshot_t *imu,
                               float delta_time_s)
{
    app_gimbal_feedback_t feedback = {0};
    const module_motor_feedback_t *pitch_feedback;
    const module_motor_feedback_t *yaw_feedback;
    float pitch_position;
    float yaw_position;

    bool had_motor_error = false;

    if ((me == NULL) || (command == NULL) || !isfinite(delta_time_s) ||
        (delta_time_s <= 0.0F))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!me->initialized)
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    pitch_feedback = module_motor_get_feedback(me->config.pitch_motor);
    yaw_feedback = module_motor_get_feedback(me->config.yaw_motor);
    if (!command->enabled || (pitch_feedback == NULL) || (yaw_feedback == NULL))
    {
        (void)module_motor_disable(me->config.pitch_motor);
        (void)module_motor_disable(me->config.yaw_motor);
        me->feedback = feedback;
        return command->enabled ? BSP_STATUS_IO_ERROR : BSP_STATUS_OK;
    }

    /* 根据配置的反馈模式选择位置反馈源：IMU 姿态或电机编码器。 */
    pitch_position = (command->feedback_mode == APP_GIMBAL_FEEDBACK_IMU) &&
                             (imu != NULL) && imu->valid
                         ? imu->pitch_rad
                         : pitch_feedback->position_rad;
    yaw_position = (command->feedback_mode == APP_GIMBAL_FEEDBACK_IMU) &&
                           (imu != NULL) && imu->valid
                       ? imu->yaw_rad
                       : yaw_feedback->position_rad;
    had_motor_error = (module_motor_enable(me->config.pitch_motor) != MODULE_MOTOR_STATUS_OK) ||
                      (module_motor_enable(me->config.yaw_motor) != MODULE_MOTOR_STATUS_OK);
    /* 云台电机必须配置为角度模式，目标值的单位明确为 rad。 */
    had_motor_error =
        (module_motor_set_target(me->config.pitch_motor, command->pitch_target_rad) !=
         MODULE_MOTOR_STATUS_OK) ||
        (module_motor_set_target(me->config.yaw_motor, command->yaw_target_rad) !=
         MODULE_MOTOR_STATUS_OK) || had_motor_error;

    feedback.pitch_rad = pitch_position;
    feedback.yaw_rad = yaw_position;
    feedback.pitch_velocity_rad_per_s = pitch_feedback->velocity_rad_per_s;
    feedback.yaw_velocity_rad_per_s = yaw_feedback->velocity_rad_per_s;
    feedback.motors_online = pitch_feedback->is_online && yaw_feedback->is_online;
    /* 当俯仰和偏航误差均在容差范围内时，判定目标已锁定。 */
    feedback.target_locked =
        (fabsf(command->pitch_target_rad - pitch_position) <=
         me->config.target_tolerance_rad) &&
        (fabsf(command->yaw_target_rad - yaw_position) <= me->config.target_tolerance_rad);
    feedback.imu_valid = (imu != NULL) && imu->valid;
    me->feedback = feedback;
    return had_motor_error ? BSP_STATUS_IO_ERROR : BSP_STATUS_OK;
}

const app_gimbal_feedback_t *app_gimbal_get_feedback(const app_gimbal_t *me)
{
    return ((me != NULL) && me->initialized) ? &me->feedback : NULL;
}
