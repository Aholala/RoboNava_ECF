/**
 * @file alg_math_statistics.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief Welford 在线统计算法实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 采用 Welford 算法，仅需保存均值、离差平方和、计数及极值。
 *       无需存储历史样本，适合嵌入式实时监控。
 */

#include "alg_math.h"
#include <float.h>
#include <math.h>

/* ======================== Welford 在线统计 ======================== */

/**
 * @brief 初始化统计结构体
 * @param me 统计对象指针
 * @return 执行状态
 * @note 将均值、离差平方和置零，极值置为 FLT_MAX / -FLT_MAX
 */
alg_math_status_t alg_math_statistics_init(alg_math_statistics_t *me)
{
    if (me == NULL) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    me->sample_count = 0U;
    me->mean = 0.0F;
    me->sum_of_squared_deviations = 0.0F;
    me->minimum = FLT_MAX;
    me->maximum = -FLT_MAX;
    return ALG_MATH_STATUS_OK;
}

/**
 * @brief 更新统计（Welford 在线算法）
 * @param me 统计对象指针
 * @param sample 新样本值
 * @return 执行状态
 * @note 算法步骤（单次遍历，无需存储历史数据）：
 *       1. N = N + 1
 *       2. delta = x - mean_old
 *       3. mean_new = mean_old + delta / N
 *       4. M2 = M2 + delta * (x - mean_new)
 *       5. 更新 min / max
 */
alg_math_status_t alg_math_statistics_update(alg_math_statistics_t *me, float sample)
{
    float delta;
    float updated_mean;

    if (me == NULL) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if (!isfinite(sample) || (me->sample_count == UINT32_MAX)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}

    ++me->sample_count;
    delta = sample - me->mean;
    updated_mean = me->mean + (delta / (float)me->sample_count);
    me->sum_of_squared_deviations += delta * (sample - updated_mean);
    me->mean = updated_mean;
    me->minimum = fminf(me->minimum, sample);
    me->maximum = fmaxf(me->maximum, sample);

    return (isfinite(me->mean) && isfinite(me->sum_of_squared_deviations))
               ? ALG_MATH_STATUS_OK
               : ALG_MATH_STATUS_NUMERICAL_ERROR;
}

/**
 * @brief 获取总体方差：sigma^2 = M2 / N
 * @param me 统计对象指针
 * @param variance 输出总体方差（>= 0）
 * @return 执行状态，样本数为 0 时返回 OUT_OF_RANGE
 */
alg_math_status_t alg_math_statistics_get_population_variance(const alg_math_statistics_t *me,
                                                              float *variance)
{
    if ((me == NULL) || (variance == NULL)) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if (me->sample_count == 0U) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}
    *variance = me->sum_of_squared_deviations / (float)me->sample_count;
    *variance = fmaxf(*variance, 0.0F);
    return isfinite(*variance) ? ALG_MATH_STATUS_OK : ALG_MATH_STATUS_NUMERICAL_ERROR;
}

/**
 * @brief 获取样本方差（无偏估计）：s^2 = M2 / (N - 1)
 * @param me 统计对象指针
 * @param variance 输出样本方差（>= 0）
 * @return 执行状态，样本数 < 2 时返回 OUT_OF_RANGE
 */
alg_math_status_t alg_math_statistics_get_sample_variance(const alg_math_statistics_t *me,
                                                          float *variance)
{
    if ((me == NULL) || (variance == NULL)) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if (me->sample_count < 2U) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}
    *variance = me->sum_of_squared_deviations / (float)(me->sample_count - 1U);
    *variance = fmaxf(*variance, 0.0F);
    return isfinite(*variance) ? ALG_MATH_STATUS_OK : ALG_MATH_STATUS_NUMERICAL_ERROR;
}

/**
 * @brief 获取标准差
 * @param me 统计对象指针
 * @param sample_standard_deviation true=样本标准差（除以 N-1），false=总体标准差（除以 N）
 * @param standard_deviation 输出标准差
 * @return 执行状态
 * @note 先获取对应方差，再开平方
 */
alg_math_status_t alg_math_statistics_get_standard_deviation(const alg_math_statistics_t *me,
                                                             bool sample_standard_deviation,
                                                             float *standard_deviation)
{
    float variance;
    alg_math_status_t status;
    if (standard_deviation == NULL) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}

    status = sample_standard_deviation ? alg_math_statistics_get_sample_variance(me, &variance)
                                       : alg_math_statistics_get_population_variance(me, &variance);
    if (status != ALG_MATH_STATUS_OK) {
        return status;
}
    *standard_deviation = sqrtf(variance);
    return ALG_MATH_STATUS_OK;
}

/* ======================== 数组统计 ======================== */

/**
 * @brief 数组算术均值：mean = sum(values) / N
 * @param values 数组指针
 * @param value_count 数组长度（必须 > 0）
 * @param mean 输出均值
 * @return 执行状态
 */
alg_math_status_t alg_math_array_mean(const float *values, size_t value_count, float *mean)
{
    size_t index;
    float accumulator = 0.0F;

    if ((values == NULL) || (mean == NULL)) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if ((value_count == 0U) || !alg_math_is_finite_array(values, value_count)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}

    for (index = 0U; index < value_count; ++index) {
        accumulator += values[index];
}
    *mean = accumulator / (float)value_count;
    return isfinite(*mean) ? ALG_MATH_STATUS_OK : ALG_MATH_STATUS_NUMERICAL_ERROR;
}

/**
 * @brief 数组均方根（RMS）：rms = sqrt( sum(values^2) / N )
 * @param values 数组指针
 * @param value_count 数组长度（必须 > 0）
 * @param rms 输出均方根值
 * @return 执行状态
 */
alg_math_status_t alg_math_array_rms(const float *values, size_t value_count, float *rms)
{
    size_t index;
    float accumulator = 0.0F;

    if ((values == NULL) || (rms == NULL)) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if ((value_count == 0U) || !alg_math_is_finite_array(values, value_count)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}

    for (index = 0U; index < value_count; ++index) {
        accumulator += values[index] * values[index];
}
    *rms = sqrtf(accumulator / (float)value_count);
    return isfinite(*rms) ? ALG_MATH_STATUS_OK : ALG_MATH_STATUS_NUMERICAL_ERROR;
}