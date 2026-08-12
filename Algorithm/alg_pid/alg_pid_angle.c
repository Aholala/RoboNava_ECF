/**
 * @file alg_pid_angle.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 角度串级 PID 封装实现
 * @version 1.0
 * @date 2026-08-01
 * @copyright Copyright (c) 2026
 *
 * @note 对串级 PID 的薄封装，将角度域输入转换为通用串级 PID 输入。
 *       所有物理量使用弧度制。
 */

#include "alg_pid.h"

#include <stddef.h>

/* ======================== 角度串级 PID API ======================== */

/**
 * @brief 初始化角度串级 PID
 * @param me 角度串级 PID 对象
 * @param config 角度串级 PID 配置
 * @return 执行状态
 * @note 直接委托给 alg_pid_cascade_init，传入内嵌的 cascade_config
 */
alg_pid_status_t alg_pid_angle_init(alg_pid_angle_t *me, const alg_pid_angle_config_t *config)
{
    // 参数校验
    if ((me == NULL) || (config == NULL))
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }

    // 委托给底层串级 PID 初始化
    return alg_pid_cascade_init(&me->cascade, &config->cascade_config);
}

/**
 * @brief 重置角度串级 PID
 * @param me 角度串级 PID 对象
 * @param measured_position_rad 当前测量角度（rad）
 * @param measured_velocity_rad_per_s 当前测量角速度（rad/s）
 * @param initial_output 初始输出值
 * @return 执行状态
 * @note 委托给 alg_pid_cascade_reset，清除位置环/速度环历史
 */
alg_pid_status_t alg_pid_angle_reset(alg_pid_angle_t *me, float measured_position_rad,
                                     float measured_velocity_rad_per_s, float initial_output)
{
    // 参数校验
    if (me == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }

    // 委托给底层串级 PID 重置
    return alg_pid_cascade_reset(&me->cascade, measured_position_rad,
                                 measured_velocity_rad_per_s, initial_output);
}

/**
 * @brief 更新角度串级 PID
 * @param me 角度串级 PID 对象
 * @param input 角度输入结构体（目标位置/速度、测量位置/速度）
 * @param control_output 输出控制量（输出参数）
 * @return 执行状态
 * @note 将角度域字段映射到串级 PID 输入字段：
 *       target_position_rad -> position_setpoint,
 *       measured_position_rad -> position_measurement,
 *       measured_velocity_rad_per_s -> velocity_measurement,
 *       target_velocity_rad_per_s -> velocity_feedforward,
 *       actuator_feedforward -> actuator_feedforward
 */
alg_pid_status_t alg_pid_angle_update(alg_pid_angle_t *me, const alg_pid_angle_input_t *input,
                                      float *control_output)
{
    alg_pid_cascade_input_t cascade_input;

    // 参数校验
    if ((me == NULL) || (input == NULL) || (control_output == NULL))
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }

    // 将角度域输入映射为串级 PID 的通用输入
    cascade_input = (alg_pid_cascade_input_t){
        .position_setpoint = input->target_position_rad,
        .position_measurement = input->measured_position_rad,
        .velocity_measurement = input->measured_velocity_rad_per_s,
        .velocity_feedforward = input->target_velocity_rad_per_s,
        .actuator_feedforward = input->actuator_feedforward,
        .delta_time_s = input->delta_time_s,
    };

    // 委托给底层串级 PID 更新
    return alg_pid_cascade_update(&me->cascade, &cascade_input, control_output);
}

/**
 * @brief 获取当前角速度设定值
 * @param me 角度串级 PID 对象
 * @return 角速度设定值（rad/s），未初始化时返回 0.0F
 * @note 返回最近一次位置环产出的速度指令
 */
float alg_pid_angle_get_velocity_setpoint(const alg_pid_angle_t *me)
{
    // 空指针保护
    if (me == NULL)
    {
        return 0.0F;
    }

    // 委托给底层串级 PID
    return alg_pid_cascade_get_velocity_setpoint(&me->cascade);
}
