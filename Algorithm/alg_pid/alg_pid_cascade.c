/**
 * @file alg_pid_cascade.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 位置—速度串级 PID 控制器实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 外环（位置环）输出作为内环（速度环）的设定值。
 *       位置环可按降频运行，速度环每周期执行。
 *       支持速度设定值限幅和反饱和积分。
 */

#include "alg_pid.h"
#include <math.h>
#include <stddef.h>

/* ======================== 内部辅助函数 ======================== */

/**
 * @brief 值限幅（内部函数）
 * @param value 输入值
 * @param minimum 下限
 * @param maximum 上限
 * @return clamp(minimum, maximum, value)
 * @note 与 alg_pid_core.c 中的 alg_pid_clamp 功能相同（文件级 static 避免符号冲突）
 */
static float alg_pid_cascade_clamp(float value, float minimum, float maximum)
{
    // 下溢出
    if (value < minimum)
    {
        return minimum;
    }
    // 上溢出
    if (value > maximum)
    {
        return maximum;
    }
    // 在范围内：直通
    return value;
}

/* ======================== 串级 PID API ======================== */

/**
 * @brief 初始化串级 PID
 * @param me 串级 PID 对象
 * @param config 串级 PID 配置
 * @return 执行状态
 * @note 同时初始化位置环和速度环两个子 PID 控制器。
 *       位置环降频计数器初始化为 divider-1，使首次更新时立即执行。
 */
alg_pid_status_t alg_pid_cascade_init(alg_pid_cascade_t *me, const alg_pid_cascade_config_t *config)
{
    alg_pid_status_t status;

    // 参数校验
    if ((me == NULL) || (config == NULL)) {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }

    // 先标记未初始化，防止中途失败留下半成品
    me->is_initialized = false;
    if ((config->position_loop_divider == 0U) || !isfinite(config->velocity_setpoint_min) ||
        !isfinite(config->velocity_setpoint_max) ||
        (config->velocity_setpoint_min >= config->velocity_setpoint_max)) {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }

    // 初始化位置环子控制器
    status = alg_pid_init(&me->position_controller, &config->position_config);
    if (status != ALG_PID_STATUS_OK) {
        return status;
    }

    // 初始化速度环子控制器
    status = alg_pid_init(&me->velocity_controller, &config->velocity_config);
    if (status != ALG_PID_STATUS_OK) {
        return status;
    }

    // 装载配置参数
    me->position_loop_divider = config->position_loop_divider;
    me->position_loop_counter = config->position_loop_divider - 1U; // 首次即触发
    me->position_elapsed_time_s = 0.0F;
    me->velocity_setpoint_min = config->velocity_setpoint_min;
    me->velocity_setpoint_max = config->velocity_setpoint_max;
    me->velocity_setpoint = 0.0F;
    me->is_initialized = true;
    return ALG_PID_STATUS_OK;
}

/**
 * @brief 重置串级 PID
 * @param me 串级 PID 对象
 * @param position_measurement 当前位置测量值
 * @param velocity_measurement 当前速度测量值
 * @param initial_output 初始输出值（用于速度环积分项无扰切换）
 * @return 执行状态
 * @note 同时重置位置环和速度环，清除降频计数器和累计时间
 */
alg_pid_status_t alg_pid_cascade_reset(alg_pid_cascade_t *me, float position_measurement,
                                       float velocity_measurement, float initial_output)
{
    alg_pid_status_t status;

    // 参数校验
    if (me == NULL) {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized) {
        return ALG_PID_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(position_measurement) || !isfinite(velocity_measurement) ||
        !isfinite(initial_output)) {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }

    // 重置位置环（输出 0，让位置环从零开始）
    status = alg_pid_reset(&me->position_controller, position_measurement, 0.0F);
    if (status != ALG_PID_STATUS_OK) {
        return status;
    }

    // 重置速度环（初始输出写入积分项）
    status = alg_pid_reset(&me->velocity_controller, velocity_measurement, initial_output);
    if (status != ALG_PID_STATUS_OK) {
        return status;
    }

    // 重置降频状态
    me->position_loop_counter = me->position_loop_divider - 1U;
    me->position_elapsed_time_s = 0.0F;
    me->velocity_setpoint = 0.0F;
    return ALG_PID_STATUS_OK;
}

/**
 * @brief 更新串级 PID
 * @param me 串级 PID 对象
 * @param input 包含位置/速度目标值和测量值的输入结构体
 * @param output 输出控制量（输出参数）
 * @return 执行状态
 * @note 控制流程：
 *       1. 累积时间并递增降频计数器
 *       2. 若计数器 >= divider：运行位置环 → 输出经限幅后作为速度设定值
 *       3. 每周期运行速度环 → 输出最终控制量
 *       位置环使用累积时间 dt，速度环使用单周期 dt
 */
alg_pid_status_t alg_pid_cascade_update(alg_pid_cascade_t *me, const alg_pid_cascade_input_t *input,
                                        float *output)
{
    alg_pid_input_t pos_input, vel_input;
    alg_pid_status_t status;
    float pos_output;

    // ---- 参数校验 ----
    if ((me == NULL) || (input == NULL) || (output == NULL)) {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized) {
        return ALG_PID_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(input->position_setpoint) || !isfinite(input->position_measurement) ||
        !isfinite(input->velocity_measurement) || !isfinite(input->velocity_feedforward) ||
        !isfinite(input->actuator_feedforward) || !isfinite(input->delta_time_s) ||
        (input->delta_time_s <= 0.0F)) {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }

    // ---- 累积位置环时间和帧计数 ----
    me->position_elapsed_time_s += input->delta_time_s;
    ++me->position_loop_counter;

    // ---- 位置环更新（按 divider 降频执行） ----
    if (me->position_loop_counter >= me->position_loop_divider)
    {
        // 组装位置环输入：使用累积的 dt 而非单周期 dt
        pos_input = (alg_pid_input_t){.setpoint = input->position_setpoint,
                                      .measurement = input->position_measurement,
                                      .velocity_feedforward = input->velocity_feedforward,
                                      .acceleration_feedforward = 0.0F,
                                      .additional_feedforward = 0.0F,
                                      .delta_time_s = me->position_elapsed_time_s};
        status = alg_pid_update_advanced(&me->position_controller, &pos_input, &pos_output);
        if (status != ALG_PID_STATUS_OK) {
            return status;
        }

        // 位置环输出经限幅后成为速度环设定值
        me->velocity_setpoint = alg_pid_cascade_clamp(pos_output, me->velocity_setpoint_min,
                                                      me->velocity_setpoint_max);
        // 重置降频计数器
        me->position_loop_counter = 0U;
        me->position_elapsed_time_s = 0.0F;
    }

    // ---- 速度环更新（每周期执行） ----
    vel_input = (alg_pid_input_t){.setpoint = me->velocity_setpoint,
                                  .measurement = input->velocity_measurement,
                                  .velocity_feedforward = 0.0F,
                                  .acceleration_feedforward = 0.0F,
                                  .additional_feedforward = input->actuator_feedforward,
                                  .delta_time_s = input->delta_time_s};
    return alg_pid_update_advanced(&me->velocity_controller, &vel_input, output);
}

/**
 * @brief 获取当前速度环设定值
 * @param me 串级 PID 对象
 * @return 速度设定值，未初始化时返回 0.0F
 * @note 返回最近一次位置环更新产出的速度指令（未经速度环处理）
 */
float alg_pid_cascade_get_velocity_setpoint(const alg_pid_cascade_t *me)
{
    return ((me != NULL) && me->is_initialized) ? me->velocity_setpoint : 0.0F;
}
