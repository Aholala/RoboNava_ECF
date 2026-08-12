/**
 * @file alg_lqr.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 固定增益线性二次型状态反馈控制器实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 增益矩阵由离线工具（如 MATLAB、Python）生成，车载端仅执行状态反馈与限幅。
 *       控制律：u = u_ff - K * (x - x_ref)
 */

#include "alg_lqr.h"

#include <math.h>   // isfinite
#include <stddef.h> // NULL, size_t
#include <stdint.h> // SIZE_MAX

/**
 * @brief 检查数组中的所有值是否为有限数（内部函数）
 * @param values 浮点数数组指针
 * @param count 数组元素个数
 * @return true=所有值均为有限数，false=存在非有限值
 */
static bool alg_lqr_is_finite_array(const float *values, size_t count)
{
    size_t index;
    for (index = 0U; index < count; ++index)
    {
        if (!isfinite(values[index]))
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 初始化 LQR 控制器
 * @param me 控制器对象
 * @param config 控制器配置参数
 * @return 执行状态
 * @note 参数校验包括：维度非零、增益矩阵元素有限、控制限幅对称性检查。
 *       control_min 和 control_max 必须同时为 NULL 或同时为非 NULL。
 */
alg_lqr_status_t alg_lqr_init(alg_lqr_t *me, const alg_lqr_config_t *config)
{
    size_t control_index;

    // ---- 参数非空校验 ----
    if ((me == NULL) || (config == NULL) || (config->gain_matrix == NULL))
    {
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
    }

    // 先标记为未初始化，防止中途失败留下半成品
    me->is_initialized = false;

    // ---- 维度与数值合法性校验 ----
    if ((config->state_dimension == 0U) || (config->control_dimension == 0U) ||
        (config->state_dimension > (SIZE_MAX / config->control_dimension)) ||
        ((config->control_min == NULL) != (config->control_max == NULL)) ||
        !alg_lqr_is_finite_array(config->gain_matrix,
                                 config->state_dimension * config->control_dimension))
    {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
    }

    // ---- 控制限幅数组元素校验 ----
    // control_min[i] 必须 < control_max[i]
    if (config->control_min != NULL)
    {
        for (control_index = 0U; control_index < config->control_dimension; ++control_index)
        {
            if (!isfinite(config->control_min[control_index]) ||
                !isfinite(config->control_max[control_index]) ||
                (config->control_min[control_index] >= config->control_max[control_index]))
            {
                return ALG_LQR_STATUS_OUT_OF_RANGE;
            }
        }
    }

    // ---- 存储配置，标记初始化完成 ----
    me->config = *config;
    me->is_initialized = true;
    return ALG_LQR_STATUS_OK;
}

/**
 * @brief 计算控制输出
 * @param me 控制器对象
 * @param reference 参考状态（NULL 表示零参考状态）
 * @param state 当前状态向量
 * @param feedforward 前馈控制量（NULL 表示零前馈）
 * @param output 输出控制向量
 * @return 执行状态
 * @note 控制律：u = u_ff - K * (x - x_ref)
 *       若无前馈，u_ff = 0；若无参考状态，x_ref = 0。
 *       增益矩阵按行优先存储：K[control_index * state_dimension + state_index]
 *       输出经 control_min / control_max 限幅后写入 output。
 */
alg_lqr_status_t alg_lqr_update(const alg_lqr_t *me, const float *reference,
                                const float *state, const float *feedforward, float *output)
{
    size_t control_index;
    size_t state_index;
    float control;
    float state_error;

    // ---- 参数非空校验 ----
    if ((me == NULL) || (state == NULL) || (output == NULL))
    {
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
    }

    // ---- 初始化状态校验 ----
    if (!me->is_initialized)
    {
        return ALG_LQR_STATUS_NOT_INITIALIZED;
    }

    // ---- 输入值有限性校验 ----
    if (!alg_lqr_is_finite_array(state, me->config.state_dimension) ||
        ((reference != NULL) &&
         !alg_lqr_is_finite_array(reference, me->config.state_dimension)) ||
        ((feedforward != NULL) &&
         !alg_lqr_is_finite_array(feedforward, me->config.control_dimension)))
    {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
    }

    // ---- 计算每个控制通道的输出 ----
    for (control_index = 0U; control_index < me->config.control_dimension; ++control_index)
    {
        // 起始值 = 前馈项（若无则为 0）
        control = (feedforward != NULL) ? feedforward[control_index] : 0.0F;

        // 累加负反馈项：Σ K[ci][si] * (x[si] - x_ref[si])
        for (state_index = 0U; state_index < me->config.state_dimension; ++state_index)
        {
            state_error = state[state_index] -
                          ((reference != NULL) ? reference[state_index] : 0.0F);
            control -= me->config.gain_matrix[(control_index * me->config.state_dimension) +
                                              state_index] *
                       state_error;
        }

        // 数值安全检查
        if (!isfinite(control))
        {
            return ALG_LQR_STATUS_NUMERICAL_ERROR;
        }

        // ---- 控制量限幅 ----
        if (me->config.control_min != NULL)
        {
            if (control < me->config.control_min[control_index])
            {
                control = me->config.control_min[control_index];
            }
            else if (control > me->config.control_max[control_index])
            {
                control = me->config.control_max[control_index];
            }
        }

        // 写入输出
        output[control_index] = control;
    }

    return ALG_LQR_STATUS_OK;
}
