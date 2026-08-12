/**
 * @file alg_pid_core.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 单环 PID 控制器核心实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 实现并行式 PID 控制器，包含：
 *       - 反饱和积分（conditional integration）
 *       - 微分项低通滤波（一阶 RC）
 *       - 微分模式可配置（作用于误差或测量值）
 *       - 前馈分量（速度、加速度、附加前馈）
 */

#include "alg_pid.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

/** @brief 2π 常量，用于微分滤波器系数计算 */
#define ALG_PID_TWO_PI_F 6.28318530717958647692F

/* ======================== 内部辅助函数 ======================== */

/**
 * @brief 值限幅（内部函数）
 * @param value 输入值
 * @param minimum 下限
 * @param maximum 上限
 * @return clamp(minimum, maximum, value)
 * @note 简单饱和限幅，不施加额外处理
 */
static float alg_pid_clamp(float value, float minimum, float maximum)
{
    // 下溢出：取下限
    if (value < minimum)
    {
        return minimum;
    }
    // 上溢出：取上限
    if (value > maximum)
    {
        return maximum;
    }
    // 在范围内：直通
    return value;
}

/**
 * @brief 校验 PID 配置有效性（内部函数）
 * @param config PID 配置指针
 * @return 执行状态
 * @note 检查所有增益和限幅值是否为有限数，并验证限幅区间有效性：
 *       integral_min <= integral_max,
 *       output_min < output_max,
 *       derivative_filter_cutoff_hz >= 0
 */
static alg_pid_status_t alg_pid_validate_config(const alg_pid_config_t *config)
{
    // 空指针检查
    if (config == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }

    // 所有字段必须为有限数；限幅区间必须有效
    if (!isfinite(config->proportional_gain) || !isfinite(config->integral_gain) ||
        !isfinite(config->derivative_gain) ||
        !isfinite(config->derivative_filter_cutoff_hz) ||
        (config->derivative_filter_cutoff_hz < 0.0F) || !isfinite(config->integral_min) ||
        !isfinite(config->integral_max) || (config->integral_min > config->integral_max) ||
        !isfinite(config->output_min) || !isfinite(config->output_max) ||
        (config->output_min >= config->output_max))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }
    return ALG_PID_STATUS_OK;
}

/* ======================== 单环 PID API ======================== */

/**
 * @brief 初始化 PID 配置为安全默认值
 * @param config 配置结构体指针
 * @return 执行状态
 * @note 默认值：
 *       - 所有增益为 0（输出恒为前馈分量）
 *       - 积分/输出限幅为 ±FLT_MAX（无实际限幅）
 *       - 微分滤波器截止频率为 0（无滤波）
 *       - 微分作用于测量值（避免设定值突变引起的微分冲击）
 */
alg_pid_status_t alg_pid_config_init(alg_pid_config_t *config)
{
    // 空指针检查
    if (config == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }

    // 写入安全默认值
    *config = (alg_pid_config_t){
        .proportional_gain = 0.0F,
        .integral_gain = 0.0F,
        .derivative_gain = 0.0F,
        .derivative_filter_cutoff_hz = 0.0F,
        .integral_min = -FLT_MAX,
        .integral_max = FLT_MAX,
        .output_min = -FLT_MAX,
        .output_max = FLT_MAX,
        .derivative_on_measurement = true, // 推荐模式：微分不响应对设定值突变
    };
    return ALG_PID_STATUS_OK;
}

/**
 * @brief 初始化 PID 控制器
 * @param me PID 控制器对象
 * @param config PID 配置
 * @return 执行状态
 * @note 校验配置后零初始化对象并写入配置。初始化后无历史采样数据，
 *       首次 update 时微分项为 0。
 */
alg_pid_status_t alg_pid_init(alg_pid_t *me, const alg_pid_config_t *config)
{
    alg_pid_status_t status;

    // 空指针检查
    if (me == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }

    // 先标记未初始化，防止中途失败留下半成品
    me->is_initialized = false;

    // 配置校验
    status = alg_pid_validate_config(config);
    if (status != ALG_PID_STATUS_OK)
    {
        return status;
    }

    // 零初始化整个对象，再写入配置
    *me = (alg_pid_t){0};
    me->config = *config;
    me->is_initialized = true;
    return ALG_PID_STATUS_OK;
}

/**
 * @brief 重置 PID 控制器状态
 * @param me PID 控制器对象
 * @param measurement 当前测量值（用于初始化微分历史）
 * @param initial_output 初始输出值
 * @return 执行状态
 * @note 将积分项设为 initial_output（经积分限幅），实现无扰切换。
 *       清除微分滤波器和误差历史。
 */
alg_pid_status_t alg_pid_reset(alg_pid_t *me, float measurement, float initial_output)
{
    // 参数校验
    if (me == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_PID_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(measurement) || !isfinite(initial_output))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }

    // 清零所有分量
    me->terms = (alg_pid_terms_t){0};

    // 积分项 = initial_output（经积分限幅），实现无扰切换
    me->terms.integral = alg_pid_clamp(initial_output, me->config.integral_min,
                                       me->config.integral_max);
    // 未饱和输出 = 积分项（P、D 均为 0，无前馈）
    me->terms.unsaturated_output = me->terms.integral;
    // 实际输出 = 未饱和输出经输出限幅
    me->terms.output = alg_pid_clamp(me->terms.integral, me->config.output_min,
                                     me->config.output_max);

    // 清除历史和滤波器状态
    me->previous_error = 0.0F;
    me->previous_measurement = measurement;
    me->filtered_derivative = 0.0F;
    me->has_previous_sample = true;
    return ALG_PID_STATUS_OK;
}

/**
 * @brief 简化 PID 更新（无前馈）
 * @param me PID 控制器对象
 * @param setpoint 目标值
 * @param measurement 当前测量值
 * @param delta_time_s 时间步长（秒）
 * @param output 输出控制量（输出参数）
 * @return 执行状态
 * @note 便捷接口，内部组装全零前馈的 input 后调用 alg_pid_update_advanced
 */
alg_pid_status_t alg_pid_update(alg_pid_t *me, float setpoint, float measurement,
                                float delta_time_s, float *output)
{
    // 组装输入：前馈分量均为 0
    const alg_pid_input_t input = {
        .setpoint = setpoint,
        .measurement = measurement,
        .velocity_feedforward = 0.0F,
        .acceleration_feedforward = 0.0F,
        .additional_feedforward = 0.0F,
        .delta_time_s = delta_time_s,
    };
    return alg_pid_update_advanced(me, &input, output);
}

/**
 * @brief 高级 PID 更新（含前馈和反饱和积分）
 * @param me PID 控制器对象
 * @param input 包含设定值、测量值、前馈和 dt 的输入结构体
 * @param output 输出控制量（输出参数）
 * @return 执行状态
 * @note 算法步骤：
 *       1. 计算误差 e = setpoint - measurement
 *       2. P 项 = Kp * e
 *       3. I 项 = clamp(I_prev + Ki * e * dt, I_min, I_max)
 *       4. D 项 = Kd * filtered(de/dt) 或 Kd * filtered(-d_measurement/dt)
 *       5. 前馈合计 = velocity_ff + acceleration_ff + additional_ff
 *       6. 未饱和输出 = P + I + D + 前馈
 *       7. 输出限幅后得到最终输出
 *       8. 反饱和积分（conditional integration）：
 *          若输出饱和且积分项在加大饱和方向，则冻结积分项
 * @note 微分滤波器为一阶低通 RC：
 *       alpha = (2*pi*fc*dt) / (1 + 2*pi*fc*dt)
 *       fd = fd_prev + alpha * (derivative_raw - fd_prev)
 */
alg_pid_status_t alg_pid_update_advanced(alg_pid_t *me, const alg_pid_input_t *input,
                                         float *output)
{
    float error;
    float proportional;
    float integral;
    float derivative_signal = 0.0F;
    float filtered_derivative;
    float derivative;
    float unsaturated_output;
    float saturated_output;
    float smoothing_factor;
    bool saturation_pushes_with_error;

    // ---- 参数校验 ----
    if ((me == NULL) || (input == NULL) || (output == NULL))
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_PID_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(input->setpoint) || !isfinite(input->measurement) ||
        !isfinite(input->velocity_feedforward) ||
        !isfinite(input->acceleration_feedforward) ||
        !isfinite(input->additional_feedforward) || !isfinite(input->delta_time_s) ||
        (input->delta_time_s <= 0.0F))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }

    // ---- 比例项 ----
    error = input->setpoint - input->measurement;
    proportional = me->config.proportional_gain * error;

    // ---- 积分项（含限幅） ----
    // 先计算积分增量再累加，再限幅
    integral = alg_pid_clamp(me->terms.integral +
                                 (me->config.integral_gain * error * input->delta_time_s),
                             me->config.integral_min, me->config.integral_max);

    // ---- 微分项 ----
    // 计算原始微分信号（仅在有历史数据时有效）
    if (me->has_previous_sample)
    {
        // derivative_on_measurement: 对测量值求负变化率，避免设定值突变的微分冲击
        derivative_signal = me->config.derivative_on_measurement
                                ? -(input->measurement - me->previous_measurement) /
                                      input->delta_time_s
                                : (error - me->previous_error) / input->delta_time_s;
    }

    // 微分滤波（一阶低通 RC）
    filtered_derivative = derivative_signal;
    if (me->has_previous_sample && (me->config.derivative_filter_cutoff_hz > 0.0F))
    {
        // 平滑因子 alpha = (2*pi*fc*dt) / (1 + 2*pi*fc*dt)
        smoothing_factor = (ALG_PID_TWO_PI_F * me->config.derivative_filter_cutoff_hz *
                            input->delta_time_s) /
                           (1.0F + (ALG_PID_TWO_PI_F *
                                    me->config.derivative_filter_cutoff_hz *
                                    input->delta_time_s));
        // 一阶低通：fd = fd_prev + alpha * (raw - fd_prev)
        filtered_derivative = me->filtered_derivative +
                              smoothing_factor * (derivative_signal - me->filtered_derivative);
    }
    derivative = me->config.derivative_gain * filtered_derivative;

    // ---- 前馈合计 ----
    me->terms.feedforward = input->velocity_feedforward + input->acceleration_feedforward +
                            input->additional_feedforward;

    // ---- 输出合成与限幅 ----
    // 未饱和输出 = P + I + D + 前馈
    unsaturated_output = proportional + integral + derivative + me->terms.feedforward;
    // 输出限幅
    saturated_output = alg_pid_clamp(unsaturated_output, me->config.output_min,
                                     me->config.output_max);

    // ---- 反饱和积分（conditional integration） ----
    // 条件：输出饱和 且 误差方向与饱和方向一致（积分在加大饱和）
    saturation_pushes_with_error =
        ((unsaturated_output > me->config.output_max) && (error > 0.0F)) ||
        ((unsaturated_output < me->config.output_min) && (error < 0.0F));
    if (saturation_pushes_with_error)
    {
        // 冻结积分项：恢复本次更新前的积分值
        integral = me->terms.integral;
        // 用冻结后的积分项重新计算输出
        unsaturated_output = proportional + integral + derivative + me->terms.feedforward;
        saturated_output = alg_pid_clamp(unsaturated_output, me->config.output_min,
                                         me->config.output_max);
    }

    // ---- 数值检查 ----
    if (!isfinite(unsaturated_output) || !isfinite(saturated_output))
    {
        return ALG_PID_STATUS_NUMERICAL_ERROR;
    }

    // ---- 状态更新 ----
    me->terms.proportional = proportional;
    me->terms.integral = integral;
    me->terms.derivative = derivative;
    me->terms.unsaturated_output = unsaturated_output;
    me->terms.output = saturated_output;
    me->previous_error = error;
    me->previous_measurement = input->measurement;
    me->filtered_derivative = filtered_derivative;
    me->has_previous_sample = true;

    *output = saturated_output;
    return ALG_PID_STATUS_OK;
}

/**
 * @brief 获取 PID 各分量的当前值（用于调试/遥测）
 * @param me PID 控制器对象
 * @return 指向内部 terms 的只读指针，未初始化或空指针时返回 NULL
 * @note 返回的指针指向对象内部内存，调用方不应修改或释放
 */
const alg_pid_terms_t *alg_pid_get_terms(const alg_pid_t *me)
{
    return ((me != NULL) && me->is_initialized) ? &me->terms : NULL;
}
