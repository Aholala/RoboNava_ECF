/**
 * @file alg_lqr.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 固定增益线性二次型状态反馈控制器
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 增益矩阵由离线工具（如 MATLAB、Python）生成；车载端只执行状态反馈与限幅。
 */
#ifndef ALG_LQR_H
#define ALG_LQR_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LQR 控制器状态码
 */
typedef enum
{
    ALG_LQR_STATUS_OK = 0,              /**< 执行成功 */
    ALG_LQR_STATUS_INVALID_ARGUMENT,     /**< 无效参数（NULL 指针等） */
    ALG_LQR_STATUS_OUT_OF_RANGE,         /**< 参数超出合法范围 */
    ALG_LQR_STATUS_NOT_INITIALIZED,      /**< 对象未初始化 */
    ALG_LQR_STATUS_NUMERICAL_ERROR       /**< 数值计算错误（NaN 或 Inf） */
} alg_lqr_status_t;

/**
 * @brief LQR 控制器配置参数
 * @note gain_matrix 按行优先存储，大小为 control_dimension * state_dimension
 */
typedef struct
{
    size_t state_dimension;     /**< 状态维度 */
    size_t control_dimension;   /**< 控制维度 */
    const float *gain_matrix;   /**< 增益矩阵 K */
    const float *control_min;   /**< 控制下限（NULL 表示无限幅） */
    const float *control_max;   /**< 控制上限（NULL 表示无限幅） */
} alg_lqr_config_t;

/**
 * @brief LQR 控制器实例
 */
typedef struct
{
    alg_lqr_config_t config;    /**< 配置副本 */
    bool is_initialized;        /**< 是否已初始化 */
} alg_lqr_t;

/**
 * @brief 初始化 LQR 控制器
 * @param me 控制器对象
 * @param config 控制器配置参数
 * @return 执行状态
 */
alg_lqr_status_t alg_lqr_init(alg_lqr_t *me, const alg_lqr_config_t *config);

/**
 * @brief 计算控制输出
 * @param me 控制器对象
 * @param reference 参考状态（可为 NULL，表示零参考状态）
 * @param state 当前状态向量
 * @param feedforward 前馈控制量（可为 NULL，表示零前馈）
 * @param output 输出控制向量
 * @return 执行状态
 * @note 控制律：output = feedforward - K * (state - reference)
 */
alg_lqr_status_t alg_lqr_update(const alg_lqr_t *me, const float *reference,
                                const float *state, const float *feedforward,
                                float *output);

#ifdef __cplusplus
}
#endif

#endif
