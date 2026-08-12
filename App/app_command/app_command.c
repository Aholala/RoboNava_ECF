/**
 * @file app_command.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 命令应用模块实现
 * @version 1.0
 * @date 2026-08-12
 * @copyright Copyright (c) 2026
 *
 * @note 解码遥控器输入，对云台目标进行速率限制和幅值钳位，根据拨杆开关切换底盘模式并处理视觉辅助超控。
 */

#include "app_command.h"

#include "app_exchange.h"
#include "app_types.h"
#include "app_vision.h"

#include <string.h>
#include <math.h>

/* ======================== 模块状态（单例） ======================== */

static app_command_config_t app_command_config;  /**< 已保存的配置。 */
static float app_command_yaw_target_rad;         /**< 累积偏航目标角。 */
static float app_command_pitch_target_rad;       /**< 累积俯仰目标角。 */
static uint32_t app_command_sequence;            /**< 单调递增帧计数器。 */
static bool app_command_initialized;             /**< 初始化守护标志。 */

/* ======================== 内部辅助函数 ======================== */

/**
 * @brief  从本地 DR16 或远端板间通信链路获取遥控数据，填充到 @p remote。
 * @param  remote  输出的遥控数据结构体。
 * @return 成功获取到有效遥控数据时返回 true。
 *
 * 当 DR16 为本地连接时，同时将数据通过板间通信转发给其他单板使用。
 */
static bool app_command_get_remote(module_board_comm_remote_process_data_t *remote)
{
    if (app_command_config.dr16_is_local)
    {
        const module_dr16_process_data_t *const data =
            module_dr16_get_data(app_command_config.dr16);
        if ((data == NULL) || !data->is_online)
        {
            return false;
        }
        /* 将 DR16 字段映射到板间通信通用的遥控结构体。 */
        memset(remote, 0, sizeof(*remote));
        memcpy(remote->channel, data->channel, sizeof(remote->channel));
        remote->left_switch = (module_board_comm_switch_t)data->left_switch;
        remote->right_switch = (module_board_comm_switch_t)data->right_switch;
        remote->mouse_x = data->mouse_x;
        remote->mouse_y = data->mouse_y;
        remote->mouse_z = data->mouse_z;
        remote->mouse_left_pressed = data->mouse_left_pressed;
        remote->mouse_right_pressed = data->mouse_right_pressed;
        remote->keyboard = data->keyboard;
        remote->dial = data->dial;
        remote->is_online = true;
        remote->update_count = data->valid_frame_count;
        /* 通过板间通信转发给其他单板。 */
        if (app_command_config.board_comm != NULL)
        {
            if (module_board_comm_send_remote(app_command_config.board_comm, remote) !=
                MODULE_BOARD_COMM_STATUS_OK)
            {
                bsp_error_record(BSP_STATUS_IO_ERROR, "send_remote", 0);
            }
        }
        return true;
    }
    /* 回退：尝试通过板间通信从其他单板获取遥控数据。 */
    if (app_command_config.board_comm != NULL)
    {
        const module_board_comm_remote_process_data_t *const data =
            module_board_comm_get_remote(app_command_config.board_comm);
        if ((data != NULL) && data->is_online)
        {
            *remote = *data;
            return true;
        }
    }
    return false;
}

/* ======================== 公共 API ======================== */

/**
 * @brief  初始化命令模块（单例）。
 * @param  config  静态配置（内部拷贝）。
 * @return 成功返回 BSP_STATUS_OK，参数无效返回 BSP_STATUS_INVALID_ARGUMENT。
 */
bsp_status_t app_command_init(const app_command_config_t *config)
{
    if ((config == NULL) || (config->dr16_is_local && (config->dr16 == NULL)) ||
        !isfinite(config->maximum_yaw_rate_rad_per_s) ||
        !isfinite(config->maximum_pitch_rate_rad_per_s) ||
        !isfinite(config->minimum_pitch_rad) || !isfinite(config->maximum_pitch_rad) ||
        !isfinite(config->maximum_chassis_velocity_m_per_s) ||
        !isfinite(config->maximum_chassis_spin_rad_per_s) ||
        (config->maximum_yaw_rate_rad_per_s <= 0.0F) ||
        (config->maximum_pitch_rate_rad_per_s <= 0.0F) ||
        (config->minimum_pitch_rad >= config->maximum_pitch_rad) ||
        (config->maximum_chassis_velocity_m_per_s <= 0.0F) ||
        (config->maximum_chassis_spin_rad_per_s <= 0.0F))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    app_command_config = *config;
    app_command_yaw_target_rad = 0.0F;
    app_command_pitch_target_rad = 0.0F;
    app_command_sequence = 0U;
    app_command_initialized = true;
    return BSP_STATUS_OK;
}

/**
 * @brief  执行一个命令解码周期。
 * @param  delta_time_s  距上次调用的经过时间 [s]。
 *
 * 读取遥控器状态，以速率限制方式累积云台目标值并进行幅值钳位，
 * 根据拨杆位置选择底盘驱动模式，根据拨轮/鼠标状态构建射击器指令，
 * 最后将三类指令统一编号后发布到交换层。
 */
void app_command_update(float delta_time_s)
{
    module_board_comm_remote_process_data_t remote = {0};
    app_chassis_command_t chassis = {0};
    app_gimbal_command_t gimbal = {0};
    app_shooter_command_t shooter = {0};
    app_gimbal_feedback_t gimbal_feedback;
    app_vision_target_t vision_target;
    float channel[4];
    bool online;
    size_t index;

    if (!app_command_initialized)
    {
        return;
    }
    online = app_command_get_remote(&remote);
    app_exchange_read_gimbal_feedback(&gimbal_feedback);
    app_exchange_read_vision_target(&vision_target);

    /* 将四个摇杆通道归一化到 [-1, 1] 区间。 */
    for (index = 0U; index < 4U; ++index)
    {
        channel[index] = online ? module_dr16_normalize_channel_value(remote.channel[index]) : 0.0F;
    }

    /* 以速率限制方式累积云台目标值。 */
    app_command_yaw_target_rad +=
        channel[0] * app_command_config.maximum_yaw_rate_rad_per_s * delta_time_s;
    app_command_pitch_target_rad +=
        channel[1] * app_command_config.maximum_pitch_rate_rad_per_s * delta_time_s;
    /* 将俯仰角钳位在配置的机械限位内。 */
    if (app_command_pitch_target_rad > app_command_config.maximum_pitch_rad)
    {
        app_command_pitch_target_rad = app_command_config.maximum_pitch_rad;
    }
    else if (app_command_pitch_target_rad < app_command_config.minimum_pitch_rad)
    {
        app_command_pitch_target_rad = app_command_config.minimum_pitch_rad;
    }

    /* ===== 底盘指令 ===== */
    chassis.enabled = online;
    chassis.velocity_x_m_per_s = channel[3] * app_command_config.maximum_chassis_velocity_m_per_s;
    chassis.velocity_y_m_per_s = channel[2] * app_command_config.maximum_chassis_velocity_m_per_s;
    chassis.self_lock_when_stopped = true;
    chassis.gimbal_yaw_rad = gimbal_feedback.yaw_rad;

    /* 驱动模式优先级：禁用 -> 自旋 -> 跟随云台 -> 普通。 */
    if (!online || (remote.left_switch == MODULE_BOARD_COMM_SWITCH_DOWN))
    {
        chassis.mode = APP_CHASSIS_MODE_NO_FORCE;
        chassis.enabled = false;
    }
    else if (remote.left_switch == MODULE_BOARD_COMM_SWITCH_UP)
    {
        chassis.mode = APP_CHASSIS_MODE_SPIN;
        chassis.angular_velocity_rad_per_s = app_command_config.maximum_chassis_spin_rad_per_s;
    }
    else if (remote.right_switch == MODULE_BOARD_COMM_SWITCH_DOWN)
    {
        chassis.mode = APP_CHASSIS_MODE_FOLLOW_GIMBAL;
    }
    else
    {
        chassis.mode = APP_CHASSIS_MODE_NORMAL;
    }

    /* ===== 云台指令 ===== */
    gimbal.enabled = online;
    gimbal.yaw_target_rad = app_command_yaw_target_rad;
    gimbal.pitch_target_rad = app_command_pitch_target_rad;
    gimbal.feedback_mode = (remote.right_switch == MODULE_BOARD_COMM_SWITCH_MIDDLE)
                               ? APP_GIMBAL_FEEDBACK_IMU
                               : APP_GIMBAL_FEEDBACK_ENCODER;

    /* 视觉辅助超控：按住鼠标右键且有有效目标时，直接锁定视觉目标。 */
    if (vision_target.target_valid && remote.mouse_right_pressed)
    {
        gimbal.yaw_target_rad = vision_target.target_yaw_rad;
        gimbal.pitch_target_rad = vision_target.target_pitch_rad;
        gimbal.feedback_mode = APP_GIMBAL_FEEDBACK_IMU;
    }

    /* ===== 射击器指令 ===== */
    shooter.friction_enabled = online && (remote.dial > 100);
    shooter.fire_requested = shooter.friction_enabled && (remote.dial > 500);
    shooter.automatic_fire_enabled = remote.mouse_right_pressed && vision_target.tracking_ready;
    shooter.friction_velocity_rad_per_s = 500.0F;
    app_vision_set_mode(remote.mouse_right_pressed ? APP_VISION_MODE_AUTOMATIC
                                                   : APP_VISION_MODE_MANUAL);

    /* 将统一递增的序号写入各指令，然后发布到交换层。 */
    ++app_command_sequence;
    chassis.sequence = app_command_sequence;
    gimbal.sequence = app_command_sequence;
    shooter.sequence = app_command_sequence;
    app_exchange_publish_chassis_command(&chassis);
    app_exchange_publish_gimbal_command(&gimbal);
    app_exchange_publish_shooter_command(&shooter);
}
