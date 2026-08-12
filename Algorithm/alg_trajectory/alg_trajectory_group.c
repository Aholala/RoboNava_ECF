/**
 * @file alg_trajectory_group.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 多轴同步轨迹生成器实现（五次多项式时间缩放）
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 采用五次多项式插值，保证位置、速度、加速度连续且边界条件为零。
 *       每轴独立限制，共同持续时间取最大所需时间。
 *       计算时间使用近似解析公式，避免迭代。
 */

#include "alg_trajectory_group.h"
#include <math.h>

/* ======================== 内部辅助函数 ======================== */

/**
 * @brief 取两个浮点数较大值
 * @param left  左操作数
 * @param right 右操作数
 * @return 较大值
 */
static float alg_trajectory_group_maximum(float left, float right)
{
    return (left > right) ? left : right;
}

/**
 * @brief 计算单轴完成指定位移所需的最小时间（近似解析）
 * @param distance  位移（可正可负）
 * @param config    该轴配置
 * @return 所需时间（秒）
 * @note 公式基于以下假设：
 *       速度限制占优时：T = 1.875 * |d| / Vmax
 *       加速度限制占优时：T = sqrt(5.7735 * |d| / Amax)
 *       加加速度限制占优时：T = cbrt(60 * |d| / Jmax)
 *       取三者最大值作为估计。
 *       这些常数来自五次多项式时间缩放理论。
 */
static float alg_trajectory_group_axis_duration(float distance,
                                                const alg_trajectory_config_t *config)
{
    const float abs_dist = fabsf(distance);
    const float vel_duration = 1.875F * abs_dist / config->maximum_velocity_per_s;
    const float acc_duration = sqrtf(5.773503F * abs_dist / config->maximum_acceleration_per_s2);
    const float jerk_duration = cbrtf(60.0F * abs_dist / config->maximum_jerk_per_s3);
    return alg_trajectory_group_maximum(vel_duration,
                                        alg_trajectory_group_maximum(acc_duration, jerk_duration));
}

/* ======================== 初始化与目标设置 ======================== */

/**
 * @brief 初始化多轴同步轨迹组
 * @param me                    轨迹组对象
 * @param axis_config_storage   每轴配置存储（外部数组）
 * @param axis_state_storage    每轴状态存储（外部数组）
 * @param start_position_storage 起始位置存储（外部数组）
 * @param target_position_storage 目标位置存储（外部数组）
 * @param axis_count            轴数量
 * @param initial_states        初始状态数组（每轴位置、速度、加速度）
 * @param axis_configs          每轴配置数组
 * @return 执行状态
 */
alg_trajectory_status_t alg_trajectory_group_init(alg_trajectory_group_t *me,
                                                  alg_trajectory_config_t *axis_config_storage,
                                                  alg_trajectory_state_t *axis_state_storage,
                                                  float *start_position_storage,
                                                  float *target_position_storage, size_t axis_count,
                                                  const alg_trajectory_state_t *initial_states,
                                                  const alg_trajectory_config_t *axis_configs)
{
    size_t i;
    if ((me == NULL) || (axis_config_storage == NULL) || (axis_state_storage == NULL) ||
        (start_position_storage == NULL) || (target_position_storage == NULL) ||
        (axis_count == 0U) || (initial_states == NULL) || (axis_configs == NULL)) {
        return ALG_TRAJECTORY_STATUS_INVALID_ARGUMENT;
}

    for (i = 0U; i < axis_count; ++i)
    {
        if (!isfinite(initial_states[i].position) ||
            (axis_configs[i].maximum_velocity_per_s <= 0.0F) ||
            (axis_configs[i].maximum_acceleration_per_s2 <= 0.0F) ||
            (axis_configs[i].maximum_jerk_per_s3 <= 0.0F)) {
            return ALG_TRAJECTORY_STATUS_INVALID_ARGUMENT;
}
        axis_config_storage[i] = axis_configs[i];
        axis_state_storage[i] = initial_states[i];
        start_position_storage[i] = initial_states[i].position;
        target_position_storage[i] = initial_states[i].position;
    }

    *me = (alg_trajectory_group_t){
        .axis_configs = axis_config_storage,
        .axis_states = axis_state_storage,
        .start_positions = start_position_storage,
        .target_positions = target_position_storage,
        .axis_count = axis_count,
        .is_finished = true,
        .is_initialized = true,
    };
    return ALG_TRAJECTORY_STATUS_OK;
}

/**
 * @brief 设置目标位置（所有轴同步运动）
 * @param me               轨迹组对象
 * @param target_positions 每轴目标位置数组
 * @return 执行状态（位移为零则立即返回 FINISHED）
 * @note 根据每轴位移和限制计算共同的持续时间，保证同步起停。
 */
alg_trajectory_status_t alg_trajectory_group_set_target(alg_trajectory_group_t *me,
                                                        const float *target_positions)
{
    size_t i;
    float duration = 0.0F;
    if ((me == NULL) || (target_positions == NULL)) {
        return ALG_TRAJECTORY_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_TRAJECTORY_STATUS_NOT_INITIALIZED;
}

    for (i = 0U; i < me->axis_count; ++i)
    {
        float axis_dur;
        if (!isfinite(target_positions[i])) {
            return ALG_TRAJECTORY_STATUS_INVALID_ARGUMENT;
}
        me->start_positions[i] = me->axis_states[i].position;
        me->target_positions[i] = target_positions[i];
        axis_dur = alg_trajectory_group_axis_duration(target_positions[i] - me->start_positions[i],
                                                      &me->axis_configs[i]);
        duration = alg_trajectory_group_maximum(duration, axis_dur);
    }

    me->elapsed_time_s = 0.0F;
    me->duration_s = duration;
    me->is_finished = (duration <= 1.0e-6F);
    return me->is_finished ? ALG_TRAJECTORY_STATUS_FINISHED : ALG_TRAJECTORY_STATUS_OK;
}

/* ======================== 更新与查询 ======================== */

/**
 * @brief 更新轨迹（单步推进）
 * @param me            轨迹组对象
 * @param delta_time_s  时间步长（>0）
 * @return 执行状态（完成则返回 FINISHED）
 * @note 使用五次多项式对归一化时间进行插值，生成位置、速度、加速度。
 *       所有轴同步推进，共同持续时间保证同时起停。
 */
alg_trajectory_status_t alg_trajectory_group_update(alg_trajectory_group_t *me, float delta_time_s)
{
    float t, t2, t3, t4, t5;
    float pos_scale, vel_scale, acc_scale;
    size_t i;

    if ((me == NULL) || !isfinite(delta_time_s) || (delta_time_s <= 0.0F)) {
        return ALG_TRAJECTORY_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_TRAJECTORY_STATUS_NOT_INITIALIZED;
}
    if (me->is_finished) {
        return ALG_TRAJECTORY_STATUS_FINISHED;
}

    // ---- 更新已用时间（不超过总时长） ----
    me->elapsed_time_s = fminf(me->elapsed_time_s + delta_time_s, me->duration_s);

    // ---- 归一化时间及幂 ----
    t = me->elapsed_time_s / me->duration_s;
    t2 = t * t;
    t3 = t2 * t;
    t4 = t3 * t;
    t5 = t4 * t;

    // ---- 五次多项式缩放系数（边界条件：位置、速度、加速度在两端均为零） ----
    // 位置：s(t) = 10*t^3 - 15*t^4 + 6*t^5
    // 速度：v(t) = (30*t^2 - 60*t^3 + 30*t^4) / T
    // 加速度：a(t) = (60*t - 180*t^2 + 120*t^3) / T^2
    pos_scale = 10.0F * t3 - 15.0F * t4 + 6.0F * t5;
    vel_scale = (30.0F * t2 - 60.0F * t3 + 30.0F * t4) / me->duration_s;
    acc_scale = (60.0F * t - 180.0F * t2 + 120.0F * t3) / (me->duration_s * me->duration_s);

    // ---- 更新每轴状态 ----
    for (i = 0U; i < me->axis_count; ++i)
    {
        const float dist = me->target_positions[i] - me->start_positions[i];
        me->axis_states[i].position = me->start_positions[i] + dist * pos_scale;
        me->axis_states[i].velocity_per_s = dist * vel_scale;
        me->axis_states[i].acceleration_per_s2 = dist * acc_scale;
    }

    me->is_finished = (me->elapsed_time_s >= me->duration_s);
    return me->is_finished ? ALG_TRAJECTORY_STATUS_FINISHED : ALG_TRAJECTORY_STATUS_OK;
}

/**
 * @brief 获取指定轴当前状态
 * @param me          轨迹组对象
 * @param axis_index  轴索引
 * @return 状态指针（若索引无效或对象未初始化则返回 NULL）
 */
const alg_trajectory_state_t *alg_trajectory_group_get_state(const alg_trajectory_group_t *me,
                                                             size_t axis_index)
{
    return ((me != NULL) && me->is_initialized && (axis_index < me->axis_count))
               ? &me->axis_states[axis_index]
               : NULL;
}

/**
 * @brief 查询轨迹组是否已完成
 * @param me 轨迹组对象
 * @return true 表示已完成
 */
bool alg_trajectory_group_is_finished(const alg_trajectory_group_t *me)
{
    return (me != NULL) && me->is_initialized && me->is_finished;
}