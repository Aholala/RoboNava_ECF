/**
 * @file app_chassis.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 底盘应用模块实现
 * @version 1.0
 * @date 2026-08-13
 * @copyright Copyright (c) 2026
 *
 * @note 同一套接口支持麦轮、全向轮和舵轮三种底盘，周期计算并设置电机目标，发布底盘反馈。
 */
#include "app_chassis.h"

#include "alg_chassis_motion.h"

#include <math.h>

/* ======================== 内部辅助 ======================== */

/** @brief 返回固定轮式底盘的执行器数量（麦轮固定 4，全向轮取配置轮数）。 */
static size_t app_chassis_actuator_count(const app_chassis_t *me)
{
    if (me->config.type == APP_CHASSIS_TYPE_OMNI)
    {
        return me->config.drive.omni.wheel_count;
    }
    return APP_CHASSIS_MAX_WHEEL_COUNT;
}

/** @brief 按索引返回固定轮式底盘的电机指针（麦轮或全向轮）。 */
static module_motor_t *app_chassis_fixed_motor(app_chassis_t *me, size_t index)
{
    return (me->config.type == APP_CHASSIS_TYPE_MECANUM)
               ? me->config.drive.mecanum.motors[index]
               : me->config.drive.omni.motors[index];
}

/** @brief 禁用全部执行器（舵轮调用模块禁用，固定轮式调用电机禁用）。 */
static void app_chassis_disable_all(app_chassis_t *me)
{
    size_t index;
    if (me->config.type == APP_CHASSIS_TYPE_SWERVE)
    {
        for (index = 0U; index < ALG_SWERVE_RECTANGULAR_MODULE_COUNT; ++index)
        {
            (void)module_swerve_disable(me->config.drive.swerve.modules[index]);
        }
        return;
    }
    for (index = 0U; index < app_chassis_actuator_count(me); ++index)
    {
        (void)module_motor_disable(app_chassis_fixed_motor(me, index));
    }
}

/** @brief 校验底盘配置：增益有限且非负，运动学与执行器指针齐全。 */
static bool app_chassis_config_is_valid(const app_chassis_config_t *config)
{
    size_t index;
    size_t count;
    if ((config == NULL) || !isfinite(config->follow_gain) ||
        !isfinite(config->stop_deadband) || (config->follow_gain < 0.0F) ||
        (config->stop_deadband < 0.0F))
    {
        return false;
    }
    switch (config->type)
    {
        case APP_CHASSIS_TYPE_MECANUM:
            if (config->drive.mecanum.kinematics == NULL)
            {
                return false;
            }
            count = ALG_MECANUM_WHEEL_COUNT;
            break;
        case APP_CHASSIS_TYPE_OMNI:
            count = config->drive.omni.wheel_count;
            if ((config->drive.omni.kinematics == NULL) ||
                (count < ALG_OMNI_THREE_WHEEL_COUNT) || (count > APP_CHASSIS_MAX_WHEEL_COUNT) ||
                (config->drive.omni.kinematics->wheel_count != count))
            {
                return false;
            }
            break;
        case APP_CHASSIS_TYPE_SWERVE:
            if (config->drive.swerve.kinematics == NULL)
            {
                return false;
            }
            count = ALG_SWERVE_RECTANGULAR_MODULE_COUNT;
            break;
        default:
            return false;
    }
    for (index = 0U; index < count; ++index)
    {
        if ((config->type == APP_CHASSIS_TYPE_SWERVE)
                ? (config->drive.swerve.modules[index] == NULL)
                : ((config->type == APP_CHASSIS_TYPE_MECANUM)
                       ? (config->drive.mecanum.motors[index] == NULL)
                       : (config->drive.omni.motors[index] == NULL)))
        {
            return false;
        }
    }
    return true;
}

/* ======================== 公共 API ======================== */

/**
 * @brief  初始化底盘模块。
 * @param  me      指向调用方分配的实例。
 * @param  config  静态配置（内部拷贝）。
 * @return 成功返回 BSP_STATUS_OK，参数无效返回 BSP_STATUS_INVALID_ARGUMENT。
 */
bsp_status_t app_chassis_init(app_chassis_t *me, const app_chassis_config_t *config)
{
    if ((me == NULL) || !app_chassis_config_is_valid(config))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *me = (app_chassis_t){.config = *config, .initialized = true};
    return BSP_STATUS_OK;
}

/** @brief 使能固定轮式电机并设置目标，全部成功才返回 true。 */
static bool app_chassis_apply_fixed_targets(app_chassis_t *me,
                                             const float *targets,
                                             size_t count)
{
    bool online = true;
    size_t index;
    for (index = 0U; index < count; ++index)
    {
        module_motor_t *motor = app_chassis_fixed_motor(me, index);
        if ((module_motor_enable(motor) != MODULE_MOTOR_STATUS_OK) ||
            (module_motor_set_target(motor, targets[index]) != MODULE_MOTOR_STATUS_OK))
        {
            online = false;
        }
    }
    return online;
}

/** @brief 更新固定轮式（麦轮/全向轮）底盘：逆解算并下发轮速目标。 */
static bool app_chassis_update_fixed(app_chassis_t *me,
                                     const alg_chassis_velocity_t *velocity)
{
    bool available[APP_CHASSIS_MAX_WHEEL_COUNT] = {true, true, true, true};
    float targets[APP_CHASSIS_MAX_WHEEL_COUNT] = {0};
    float scale;
    size_t count = app_chassis_actuator_count(me);
    alg_chassis_status_t status;

    if (me->config.type == APP_CHASSIS_TYPE_MECANUM)
    {
        status = alg_mecanum_inverse(me->config.drive.mecanum.kinematics, velocity,
                                     available, targets, &scale);
    }
    else
    {
        status = alg_omni_inverse(me->config.drive.omni.kinematics, velocity,
                                  available, targets, count, &scale);
    }
    return (status == ALG_CHASSIS_STATUS_OK) &&
           app_chassis_apply_fixed_targets(me, targets, count);
}

/** @brief 更新舵轮底盘：解算各模块转向/驱动目标并下发，支持自锁。 */
static bool app_chassis_update_swerve(app_chassis_t *me,
                                      const alg_chassis_velocity_t *velocity,
                                      float reference_heading_rad,
                                      bool self_lock)
{
    alg_swerve_module_target_t targets[ALG_SWERVE_RECTANGULAR_MODULE_COUNT];
    alg_swerve_command_t command = {
        .velocity_x_m_per_s = velocity->velocity_x_m_per_s,
        .velocity_y_m_per_s = velocity->velocity_y_m_per_s,
        .angular_velocity_rad_per_s = velocity->angular_velocity_rad_per_s,
        .reference_heading_rad = reference_heading_rad,
        .command_is_reference_relative = true,
    };
    size_t index;
    bool online = true;
    alg_swerve_status_t status = self_lock
        ? alg_swerve_calculate_self_lock(me->config.drive.swerve.kinematics, targets,
                                         ALG_SWERVE_RECTANGULAR_MODULE_COUNT)
        : alg_swerve_calculate(me->config.drive.swerve.kinematics, &command, targets,
                               ALG_SWERVE_RECTANGULAR_MODULE_COUNT);
    if (status != ALG_SWERVE_STATUS_OK)
    {
        return false;
    }
    for (index = 0U; index < ALG_SWERVE_RECTANGULAR_MODULE_COUNT; ++index)
    {
        if ((module_swerve_enable(me->config.drive.swerve.modules[index]) !=
             MODULE_SWERVE_STATUS_OK) ||
            (module_swerve_apply_target(me->config.drive.swerve.modules[index], &targets[index]) !=
             MODULE_SWERVE_STATUS_OK))
        {
            online = false;
        }
    }
    return online;
}

/**
 * @brief  执行一个底盘控制周期。
 * @param  me            已初始化的底盘实例。
 * @param  input         命令层发布的底盘运动指令。
 * @param  delta_time_s  距上次调用的经过时间 [s]（必须 > 0）。
 * @return 成功返回 BSP_STATUS_OK，参数无效/未初始化/电机离线返回对应错误码。
 */
bsp_status_t app_chassis_update(app_chassis_t *me,
                                const app_chassis_command_t *input,
                                float delta_time_s)
{
    app_chassis_feedback_t feedback = {0};
    alg_chassis_velocity_t reference_velocity;
    alg_chassis_velocity_t body_velocity;
    bool stopped;
    bool online;

    if ((me == NULL) || (input == NULL) || !isfinite(delta_time_s) ||
        (delta_time_s <= 0.0F))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!me->initialized)
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    /* 指令无效或无动力模式：禁用全部执行器并发布无力反馈。 */
    if (!input->enabled || (input->mode == APP_CHASSIS_MODE_NO_FORCE))
    {
        app_chassis_disable_all(me);
        feedback.mode = APP_CHASSIS_MODE_NO_FORCE;
        me->feedback = feedback;
        return BSP_STATUS_OK;
    }

    reference_velocity = (alg_chassis_velocity_t){input->velocity_x_m_per_s,
                                                   input->velocity_y_m_per_s,
                                                   input->angular_velocity_rad_per_s};
    /* 跟随云台模式：用云台偏航角按比例增益生成偏航角速率。 */
    if (input->mode == APP_CHASSIS_MODE_FOLLOW_GIMBAL)
    {
        reference_velocity.angular_velocity_rad_per_s =
            me->config.follow_gain * alg_swerve_wrap_angle_rad(input->gimbal_yaw_rad);
    }
    /* 三轴速度均低于死区时判定为停车。 */
    stopped = (fabsf(reference_velocity.velocity_x_m_per_s) < me->config.stop_deadband) &&
              (fabsf(reference_velocity.velocity_y_m_per_s) < me->config.stop_deadband) &&
              (fabsf(reference_velocity.angular_velocity_rad_per_s) < me->config.stop_deadband);
    if (alg_chassis_transform_reference_to_body(&reference_velocity, input->gimbal_yaw_rad,
                                                &body_velocity) != ALG_CHASSIS_STATUS_OK)
    {
        app_chassis_disable_all(me);
        return BSP_STATUS_IO_ERROR;
    }

    feedback.self_lock_active = stopped && input->self_lock_when_stopped;
    online = (me->config.type == APP_CHASSIS_TYPE_SWERVE)
                 ? app_chassis_update_swerve(me, &reference_velocity, input->gimbal_yaw_rad,
                                              feedback.self_lock_active)
                 : app_chassis_update_fixed(me, feedback.self_lock_active
                                                    ? &(alg_chassis_velocity_t){0}
                                                    : &body_velocity);
    if (!online)
    {
        app_chassis_disable_all(me);
    }
    feedback.velocity_x_m_per_s = reference_velocity.velocity_x_m_per_s;
    feedback.velocity_y_m_per_s = reference_velocity.velocity_y_m_per_s;
    feedback.angular_velocity_rad_per_s = reference_velocity.angular_velocity_rad_per_s;
    feedback.mode = input->mode;
    feedback.motors_online = online;
    me->feedback = feedback;
    return online ? BSP_STATUS_OK : BSP_STATUS_IO_ERROR;
}

/**
 * @brief  读取最近一次底盘反馈。
 * @param  me  已初始化的底盘实例。
 * @return 只读反馈指针，实例无效时返回 NULL。
 */
const app_chassis_feedback_t *app_chassis_get_feedback(const app_chassis_t *me)
{
    return ((me != NULL) && me->initialized) ? &me->feedback : NULL;
}
