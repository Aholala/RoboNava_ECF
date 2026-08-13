/**
 * @file app_chassis.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 底盘应用模块实现
 * @version 1.0
 * @date 2026-08-12
 * @copyright Copyright (c) 2026
 *
 * @note 负责底盘运动控制、舵轮模块协调和底盘模式管理，包含逆运动学解算与自锁逻辑。
 */

#include "app_chassis.h"

#include <math.h>

/* ======================== 内部辅助函数 ======================== */

/**
 * @brief  禁用（断电）全部四个舵轮模块。
 * @param  me  已初始化的底盘实例。
 */
static void app_chassis_disable_all(app_chassis_t *me)
{
    size_t index;
    for (index = 0U; index < ALG_SWERVE_RECTANGULAR_MODULE_COUNT; ++index)
    {
        (void)module_swerve_disable(me->config.modules[index]);
    }
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
    size_t index;
    if ((me == NULL) || (config == NULL) || (config->kinematics == NULL) ||
        !isfinite(config->follow_gain) || !isfinite(config->stop_deadband) ||
        (config->follow_gain < 0.0F) || (config->stop_deadband < 0.0F))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    for (index = 0U; index < ALG_SWERVE_RECTANGULAR_MODULE_COUNT; ++index)
    {
        if (config->modules[index] == NULL)
        {
            return BSP_STATUS_INVALID_ARGUMENT;
        }
    }
    *me = (app_chassis_t){
        .config = *config,
        .initialized = true,
    };
    return BSP_STATUS_OK;
}

/**
 * @brief  执行一个底盘控制周期。
 * @param  me            已初始化的底盘实例。
 * @param  delta_time_s  距上次调用的经过时间 [s]。
 *
 * 从交换层读取底盘指令，按所选驱动模式（普通/自旋/跟随云台/自锁）
 * 运行逆运动学解算，并驱动舵轮模块。最新反馈保存在实例中，
 * 通过 app_chassis_get_feedback() 读取。
 */
bsp_status_t app_chassis_update(app_chassis_t *me,
                                const app_chassis_command_t *input,
                                float delta_time_s)
{
    app_chassis_feedback_t feedback = {0};
    alg_swerve_module_target_t targets[ALG_SWERVE_RECTANGULAR_MODULE_COUNT];
    alg_swerve_command_t command = {0};
    bool stopped;
    size_t index;

    if ((me == NULL) || (input == NULL) || !isfinite(delta_time_s) ||
        (delta_time_s <= 0.0F))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!me->initialized)
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    if (!input->enabled || (input->mode == APP_CHASSIS_MODE_NO_FORCE))
    {
        app_chassis_disable_all(me);
        feedback.mode = APP_CHASSIS_MODE_NO_FORCE;
        me->feedback = feedback;
        return BSP_STATUS_OK;
    }

    /* 将交换层输入填充至运动学算法指令结构体。 */
    command.velocity_x_m_per_s = input->velocity_x_m_per_s;
    command.velocity_y_m_per_s = input->velocity_y_m_per_s;
    command.angular_velocity_rad_per_s = input->angular_velocity_rad_per_s;
    command.reference_heading_rad = input->gimbal_yaw_rad;
    command.command_is_reference_relative = true;
    if (input->mode == APP_CHASSIS_MODE_FOLLOW_GIMBAL)
    {
        /* 覆盖偏航角速率：以增益系数向云台朝向收敛。 */
        command.angular_velocity_rad_per_s =
            me->config.follow_gain * alg_swerve_wrap_angle_rad(input->gimbal_yaw_rad);
    }

    /* 检测零速状态以判断是否进入自锁。 */
    stopped = (fabsf(command.velocity_x_m_per_s) < me->config.stop_deadband) &&
              (fabsf(command.velocity_y_m_per_s) < me->config.stop_deadband) &&
              (fabsf(command.angular_velocity_rad_per_s) < me->config.stop_deadband);
    if (stopped && input->self_lock_when_stopped)
    {
        if (alg_swerve_calculate_self_lock(me->config.kinematics, targets,
                                           ALG_SWERVE_RECTANGULAR_MODULE_COUNT) !=
            ALG_SWERVE_STATUS_OK)
        {
            app_chassis_disable_all(me);
            me->feedback = feedback;
            return BSP_STATUS_IO_ERROR;
        }
        feedback.self_lock_active = true;
    }
    else if (alg_swerve_calculate(me->config.kinematics, &command, targets,
                                  ALG_SWERVE_RECTANGULAR_MODULE_COUNT) != ALG_SWERVE_STATUS_OK)
    {
        app_chassis_disable_all(me);
        me->feedback = feedback;
        return BSP_STATUS_IO_ERROR;
    }

    /* 使能各模块并下发解算后的目标值。 */
    feedback.motors_online = true;
    for (index = 0U; index < ALG_SWERVE_RECTANGULAR_MODULE_COUNT; ++index)
    {
        if (module_swerve_enable(me->config.modules[index]) != MODULE_SWERVE_STATUS_OK)
        {
            feedback.motors_online = false;
        }
        if (module_swerve_apply_target(me->config.modules[index], &targets[index],
                                       delta_time_s) != MODULE_SWERVE_STATUS_OK)
        {
            feedback.motors_online = false;
        }
    }
    feedback.velocity_x_m_per_s = command.velocity_x_m_per_s;
    feedback.velocity_y_m_per_s = command.velocity_y_m_per_s;
    feedback.angular_velocity_rad_per_s = command.angular_velocity_rad_per_s;
    feedback.mode = input->mode;

    me->feedback = feedback;
    return feedback.motors_online ? BSP_STATUS_OK : BSP_STATUS_IO_ERROR;
}

const app_chassis_feedback_t *app_chassis_get_feedback(const app_chassis_t *me)
{
    return ((me != NULL) && me->initialized) ? &me->feedback : NULL;
}
