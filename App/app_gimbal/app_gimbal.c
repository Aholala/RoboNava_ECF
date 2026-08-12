/**
 * @file app_gimbal.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 云台应用模块实现
 * @version 1.0
 * @date 2026-08-12
 * @copyright Copyright (c) 2026
 *
 * @note 根据反馈模式选择编码器或 IMU 姿态驱动俯仰/偏航电机到目标角度，发布云台反馈并转发板间通信。
 */

#include "app_gimbal.h"

#include "app_exchange.h"
#include "app_types.h"

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
 * 通过交换层发布，同时可选地通过板间通信转发。
 */
void app_gimbal_update(app_gimbal_t *me, float delta_time_s)
{
    app_gimbal_command_t command;
    app_imu_snapshot_t imu;
    app_gimbal_feedback_t feedback = {0};
    const module_motor_feedback_t *pitch_feedback;
    const module_motor_feedback_t *yaw_feedback;
    float pitch_position;
    float yaw_position;

    if ((me == NULL) || !me->initialized)
    {
        return;
    }
    app_exchange_read_gimbal_command(&command);
    app_exchange_read_imu(&imu);
    pitch_feedback = module_motor_get_feedback(me->config.pitch_motor);
    yaw_feedback = module_motor_get_feedback(me->config.yaw_motor);
    if (!command.enabled || (pitch_feedback == NULL) || (yaw_feedback == NULL))
    {
        (void)module_motor_disable(me->config.pitch_motor);
        (void)module_motor_disable(me->config.yaw_motor);
        app_exchange_publish_gimbal_feedback(&feedback);
        return;
    }

    /* 根据配置的反馈模式选择位置反馈源：IMU 姿态或电机编码器。 */
    pitch_position = (command.feedback_mode == APP_GIMBAL_FEEDBACK_IMU) && imu.valid
                         ? imu.pitch_rad
                         : pitch_feedback->position_rad;
    yaw_position = (command.feedback_mode == APP_GIMBAL_FEEDBACK_IMU) && imu.valid
                       ? imu.yaw_rad
                       : yaw_feedback->position_rad;
    (void)module_motor_enable(me->config.pitch_motor);
    (void)module_motor_enable(me->config.yaw_motor);
    /* 云台电机必须配置为角度模式，目标值的单位明确为 rad。 */
    (void)module_motor_set_target(me->config.pitch_motor, command.pitch_target_rad);
    (void)module_motor_set_target(me->config.yaw_motor, command.yaw_target_rad);
    (void)module_motor_update(me->config.pitch_motor, delta_time_s);
    (void)module_motor_update(me->config.yaw_motor, delta_time_s);

    feedback.pitch_rad = pitch_position;
    feedback.yaw_rad = yaw_position;
    feedback.pitch_velocity_rad_per_s = pitch_feedback->velocity_rad_per_s;
    feedback.yaw_velocity_rad_per_s = yaw_feedback->velocity_rad_per_s;
    feedback.motors_online = pitch_feedback->is_online && yaw_feedback->is_online;
    /* 当俯仰和偏航误差均在容差范围内时，判定目标已锁定。 */
    feedback.target_locked =
        (fabsf(command.pitch_target_rad - pitch_position) <=
         me->config.target_tolerance_rad) &&
        (fabsf(command.yaw_target_rad - yaw_position) <= me->config.target_tolerance_rad);
    app_exchange_publish_gimbal_feedback(&feedback);

    /* 可选：将云台反馈转发给裁判系统/UI 板。 */
    if (me->config.board_comm != NULL)
    {
        const module_board_comm_gimbal_process_data_t board_data = {
            .yaw_rad = feedback.yaw_rad,
            .pitch_rad = feedback.pitch_rad,
            .yaw_velocity_rad_per_s = feedback.yaw_velocity_rad_per_s,
            .pitch_velocity_rad_per_s = feedback.pitch_velocity_rad_per_s,
            .imu_valid = imu.valid,
            .motors_online = feedback.motors_online,
        };
        if (module_board_comm_send_gimbal(me->config.board_comm, &board_data) !=
            MODULE_BOARD_COMM_STATUS_OK)
        {
            bsp_error_record(BSP_STATUS_IO_ERROR, "send_gimbal", 0);
        }
    }
}
