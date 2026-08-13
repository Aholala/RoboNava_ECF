/**
 * @file app_command.c
 * @brief Convert remote, gimbal and vision snapshots into robot commands.
 */
#include "app_command.h"

#include <math.h>

static float app_command_scale_channel(const app_command_t *me, int16_t value)
{
    const int16_t maximum = me->config.channel_maximum_offset;
    if (value > maximum)
    {
        value = maximum;
    }
    else if (value < -maximum)
    {
        value = (int16_t)-maximum;
    }
    return (float)value / (float)maximum;
}

bsp_status_t app_command_init(app_command_t *me, const app_command_config_t *config)
{
    if ((me == NULL) || (config == NULL) || (config->channel_maximum_offset <= 0) ||
        !isfinite(config->maximum_yaw_rate_rad_per_s) ||
        !isfinite(config->maximum_pitch_rate_rad_per_s) ||
        !isfinite(config->minimum_pitch_rad) || !isfinite(config->maximum_pitch_rad) ||
        !isfinite(config->maximum_chassis_velocity_m_per_s) ||
        !isfinite(config->maximum_chassis_spin_rad_per_s) ||
        !isfinite(config->friction_velocity_rad_per_s) ||
        (config->maximum_yaw_rate_rad_per_s <= 0.0F) ||
        (config->maximum_pitch_rate_rad_per_s <= 0.0F) ||
        (config->minimum_pitch_rad >= config->maximum_pitch_rad) ||
        (config->maximum_chassis_velocity_m_per_s <= 0.0F) ||
        (config->maximum_chassis_spin_rad_per_s <= 0.0F) ||
        (config->friction_velocity_rad_per_s < 0.0F))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *me = (app_command_t){.config = *config, .initialized = true};
    return BSP_STATUS_OK;
}

bsp_status_t app_command_update(app_command_t *me,
                                const app_remote_input_t *remote,
                                const app_gimbal_feedback_t *gimbal_feedback,
                                const app_vision_target_t *vision_target,
                                float delta_time_s)
{
    app_command_output_t output = {0};
    float channel[4] = {0};
    const bool online = (remote != NULL) && remote->online;
    size_t index;

    if ((me == NULL) || !isfinite(delta_time_s) || (delta_time_s <= 0.0F))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!me->initialized)
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    for (index = 0U; index < 4U; ++index)
    {
        channel[index] = online ? app_command_scale_channel(me, remote->channel[index]) : 0.0F;
    }

    me->yaw_target_rad += channel[0] * me->config.maximum_yaw_rate_rad_per_s * delta_time_s;
    me->pitch_target_rad += channel[1] * me->config.maximum_pitch_rate_rad_per_s * delta_time_s;
    if (me->pitch_target_rad > me->config.maximum_pitch_rad)
    {
        me->pitch_target_rad = me->config.maximum_pitch_rad;
    }
    else if (me->pitch_target_rad < me->config.minimum_pitch_rad)
    {
        me->pitch_target_rad = me->config.minimum_pitch_rad;
    }

    output.chassis.enabled = online;
    output.chassis.velocity_x_m_per_s =
        channel[3] * me->config.maximum_chassis_velocity_m_per_s;
    output.chassis.velocity_y_m_per_s =
        channel[2] * me->config.maximum_chassis_velocity_m_per_s;
    output.chassis.self_lock_when_stopped = true;
    output.chassis.gimbal_yaw_rad = (gimbal_feedback != NULL) ? gimbal_feedback->yaw_rad : 0.0F;
    if (!online || (remote->left_switch == APP_SWITCH_DOWN))
    {
        output.chassis.mode = APP_CHASSIS_MODE_NO_FORCE;
        output.chassis.enabled = false;
    }
    else if (remote->left_switch == APP_SWITCH_UP)
    {
        output.chassis.mode = APP_CHASSIS_MODE_SPIN;
        output.chassis.angular_velocity_rad_per_s = me->config.maximum_chassis_spin_rad_per_s;
    }
    else if (remote->right_switch == APP_SWITCH_DOWN)
    {
        output.chassis.mode = APP_CHASSIS_MODE_FOLLOW_GIMBAL;
    }
    else
    {
        output.chassis.mode = APP_CHASSIS_MODE_NORMAL;
    }

    output.gimbal.enabled = online;
    output.gimbal.yaw_target_rad = me->yaw_target_rad;
    output.gimbal.pitch_target_rad = me->pitch_target_rad;
    output.gimbal.feedback_mode = (online && (remote->right_switch == APP_SWITCH_MIDDLE))
                                      ? APP_GIMBAL_FEEDBACK_IMU
                                      : APP_GIMBAL_FEEDBACK_ENCODER;
    output.automatic_vision_requested = online && remote->mouse_right_pressed;
    if (output.automatic_vision_requested && (vision_target != NULL) && vision_target->target_valid)
    {
        output.gimbal.yaw_target_rad = vision_target->target_yaw_rad;
        output.gimbal.pitch_target_rad = vision_target->target_pitch_rad;
        output.gimbal.feedback_mode = APP_GIMBAL_FEEDBACK_IMU;
    }

    output.shooter.friction_enabled = online && (remote->dial > 100);
    output.shooter.fire_requested = output.shooter.friction_enabled && (remote->dial > 500);
    output.shooter.automatic_fire_enabled =
        output.automatic_vision_requested && (vision_target != NULL) && vision_target->tracking_ready;
    output.shooter.friction_velocity_rad_per_s = me->config.friction_velocity_rad_per_s;

    ++me->sequence;
    output.chassis.sequence = me->sequence;
    output.gimbal.sequence = me->sequence;
    output.shooter.sequence = me->sequence;
    me->output = output;
    return BSP_STATUS_OK;
}

const app_command_output_t *app_command_get_output(const app_command_t *me)
{
    return ((me != NULL) && me->initialized) ? &me->output : NULL;
}
