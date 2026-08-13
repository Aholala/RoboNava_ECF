/**
 * @file app_chassis.c
 * @brief Unified Mecanum, omni-wheel and swerve chassis application.
 */
#include "app_chassis.h"

#include "alg_chassis_motion.h"

#include <math.h>

static size_t app_chassis_actuator_count(const app_chassis_t *me)
{
    if (me->config.type == APP_CHASSIS_TYPE_OMNI)
    {
        return me->config.drive.omni.wheel_count;
    }
    return APP_CHASSIS_MAX_WHEEL_COUNT;
}

static module_motor_t *app_chassis_fixed_motor(app_chassis_t *me, size_t index)
{
    return (me->config.type == APP_CHASSIS_TYPE_MECANUM)
               ? me->config.drive.mecanum.motors[index]
               : me->config.drive.omni.motors[index];
}

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

bsp_status_t app_chassis_init(app_chassis_t *me, const app_chassis_config_t *config)
{
    if ((me == NULL) || !app_chassis_config_is_valid(config))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *me = (app_chassis_t){.config = *config, .initialized = true};
    return BSP_STATUS_OK;
}

static bool app_chassis_apply_fixed_targets(app_chassis_t *me,
                                             const float *targets,
                                             size_t count,
                                             float delta_time_s)
{
    bool online = true;
    size_t index;
    for (index = 0U; index < count; ++index)
    {
        module_motor_t *motor = app_chassis_fixed_motor(me, index);
        if ((module_motor_enable(motor) != MODULE_MOTOR_STATUS_OK) ||
            (module_motor_set_target(motor, targets[index]) != MODULE_MOTOR_STATUS_OK) ||
            (module_motor_update(motor, delta_time_s) != MODULE_MOTOR_STATUS_OK))
        {
            online = false;
        }
    }
    return online;
}

static bool app_chassis_update_fixed(app_chassis_t *me,
                                     const alg_chassis_velocity_t *velocity,
                                     float delta_time_s)
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
           app_chassis_apply_fixed_targets(me, targets, count, delta_time_s);
}

static bool app_chassis_update_swerve(app_chassis_t *me,
                                      const alg_chassis_velocity_t *velocity,
                                      float reference_heading_rad,
                                      bool self_lock,
                                      float delta_time_s)
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
            (module_swerve_apply_target(me->config.drive.swerve.modules[index], &targets[index],
                                        delta_time_s) != MODULE_SWERVE_STATUS_OK))
        {
            online = false;
        }
    }
    return online;
}

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
    if (input->mode == APP_CHASSIS_MODE_FOLLOW_GIMBAL)
    {
        reference_velocity.angular_velocity_rad_per_s =
            me->config.follow_gain * alg_swerve_wrap_angle_rad(input->gimbal_yaw_rad);
    }
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
                                              feedback.self_lock_active, delta_time_s)
                 : app_chassis_update_fixed(me, feedback.self_lock_active
                                                    ? &(alg_chassis_velocity_t){0}
                                                    : &body_velocity,
                                             delta_time_s);
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

const app_chassis_feedback_t *app_chassis_get_feedback(const app_chassis_t *me)
{
    return ((me != NULL) && me->initialized) ? &me->feedback : NULL;
}
