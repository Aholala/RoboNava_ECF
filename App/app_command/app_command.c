/**
 * @file app_command.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 命令应用模块实现
 * @version 1.0
 * @date 2026-08-13
 * @copyright Copyright (c) 2026
 *
 * @note 把与设备无关的遥控输入转换为底盘、云台和发射命令，不依赖 DR16、板间通信或视觉模块。
 */
#include "app_command.h"

#include <math.h>

/* ======================== 内部辅助 ======================== */

/** @brief 将通道整数限幅后归一化到 [-1, 1]。 */
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

/* ======================== 公共 API ======================== */

/**
 * @brief  初始化命令模块。
 * @param  me      指向调用方分配的实例。
 * @param  config  静态配置（内部拷贝）。
 * @return 成功返回 BSP_STATUS_OK，参数无效返回 BSP_STATUS_INVALID_ARGUMENT。
 */
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

/**
 * @brief  执行一个命令转换周期。
 * @param  me               已初始化的命令实例。
 * @param  remote           通用遥控输入，可为 NULL（视为失联）。
 * @param  gimbal_feedback  云台反馈，可为 NULL。
 * @param  vision_target    视觉目标，可为 NULL。
 * @param  delta_time_s     距上次调用的经过时间 [s]（必须 > 0）。
 * @return 成功返回 BSP_STATUS_OK，参数无效/未初始化返回对应错误码。
 */
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
    /* 失联时通道置零，否则归一化到 [-1, 1]。 */
    for (index = 0U; index < 4U; ++index)
    {
        channel[index] = online ? app_command_scale_channel(me, remote->channel[index]) : 0.0F;
    }

    /* 按通道速率累积云台目标角，并对俯仰角限幅。 */
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
    /* 依据左右拨杆选择底盘模式：左拨杆下 = 无力，上 = 自旋，右拨杆下 = 跟随云台。 */
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
    /* 视觉自动跟踪：右键按下且目标有效时，用视觉目标覆盖云台目标并切换 IMU 反馈。 */
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

/**
 * @brief  读取最近一次发布的输出指令。
 * @param  me  已初始化的命令实例。
 * @return 只读输出指针，实例无效时返回 NULL。
 */
const app_command_output_t *app_command_get_output(const app_command_t *me)
{
    return ((me != NULL) && me->initialized) ? &me->output : NULL;
}
