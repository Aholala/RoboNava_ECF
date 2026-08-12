/**
 * @file module_motor.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 通用电机基类实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供统一的注册、使能、失能、目标设置、周期更新和反馈管理。
 *       反馈超时检测和故障状态管理。
 */

#include "module_motor.h"

#include <math.h>   // isfinite
#include <stddef.h> // NULL

/**
 * @brief 校验电机是否已注册且有效
 * @param me 电机对象
 * @return 执行状态
 */
static module_motor_status_t module_motor_validate_registered(const module_motor_t *const me)
{
    if (me == NULL)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized || (me->vptr == NULL))
    {
        return MODULE_MOTOR_STATUS_NOT_INITIALIZED;
    }
    return me->is_registered ? MODULE_MOTOR_STATUS_OK : MODULE_MOTOR_STATUS_NOT_REGISTERED;
}

static module_motor_status_t module_motor_enter_feedback_fault(module_motor_t *const me)
{
    const module_motor_status_t status = me->vptr->disable(me);
    me->state = MODULE_MOTOR_STATE_FAULT;
    return (status == MODULE_MOTOR_STATUS_OK) ? MODULE_MOTOR_STATUS_FEEDBACK_UNAVAILABLE : status;
}

/* ======================== 基类初始化 ======================== */

/**
 * @brief 初始化电机基类
 * @param me 电机对象
 * @param vptr 虚表指针
 * @param motor_name 调试可见的电机名称
 * @param registration_key 注册键值
 * @param motor_identifier 电机协议 ID 或主机 ID
 * @return 执行状态
 * @note 检查所有虚函数是否非空
 */
module_motor_status_t module_motor_init_base(module_motor_t *const me,
                                             const module_motor_ops_t *const vptr,
                                             const char *const motor_name,
                                             uint32_t registration_key,
                                             uint32_t motor_identifier)
{
    // ---- 参数校验 ----
    if ((me == NULL) || (vptr == NULL) || (motor_name == NULL) || (vptr->enable == NULL) ||
        (vptr->disable == NULL) || (vptr->set_target == NULL) || (vptr->update == NULL))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (me->is_initialized)
    {
        return MODULE_MOTOR_STATUS_ALREADY_INITIALIZED;
    }

    // ---- 初始化基类字段 ----
    me->vptr = vptr;
    me->motor_name = motor_name;
    me->registration_key = registration_key;
    me->motor_identifier = motor_identifier;
    me->state = MODULE_MOTOR_STATE_DISABLED;
    me->feedback = (module_motor_feedback_t){0};
    me->delta_time_s = 0.0F;
    me->total_runtime_us = 0U;
    me->enabled_runtime_us = 0U;
    me->control_update_count = 0U;
    me->last_update_status = MODULE_MOTOR_STATUS_OK;
    me->feedback_timeout_ms = 0U;
    me->is_registered = false;
    me->is_initialized = true;
    return MODULE_MOTOR_STATUS_OK;
}

/* ======================== 电机操作 ======================== */

/**
 * @brief 使能电机
 * @param me 电机对象
 * @return 执行状态
 * @note 检查注册、故障状态和反馈在线状态
 */
module_motor_status_t module_motor_enable(module_motor_t *const me)
{
    module_motor_status_t status = module_motor_validate_registered(me);

    // 故障状态不能使能
    if ((status == MODULE_MOTOR_STATUS_OK) && (me->state == MODULE_MOTOR_STATE_FAULT))
    {
        return MODULE_MOTOR_STATUS_FEEDBACK_UNAVAILABLE;
    }
    // 反馈离线不能使能
    if ((status == MODULE_MOTOR_STATUS_OK) && !me->feedback.is_online)
    {
        return MODULE_MOTOR_STATUS_FEEDBACK_UNAVAILABLE;
    }
    return (status == MODULE_MOTOR_STATUS_OK) ? me->vptr->enable(me) : status;
}

/**
 * @brief 禁用电机
 */
module_motor_status_t module_motor_disable(module_motor_t *const me)
{
    module_motor_status_t status = module_motor_validate_registered(me);
    return (status == MODULE_MOTOR_STATUS_OK) ? me->vptr->disable(me) : status;
}

/**
 * @brief 清除故障状态
 * @param me 电机对象
 * @return 执行状态
 * @note 只有反馈在线时才能清除故障
 */
module_motor_status_t module_motor_clear_fault(module_motor_t *const me)
{
    module_motor_status_t status = module_motor_validate_registered(me);

    if (status != MODULE_MOTOR_STATUS_OK)
    {
        return status;
    }
    // 反馈离线不能清除故障
    if (!me->feedback.is_online)
    {
        return MODULE_MOTOR_STATUS_FEEDBACK_UNAVAILABLE;
    }
    if ((me->vptr->can_clear_fault != NULL) &&
        (me->vptr->can_clear_fault(me) != MODULE_MOTOR_STATUS_OK))
    {
        return MODULE_MOTOR_STATUS_FEEDBACK_UNAVAILABLE;
    }
    if (me->state == MODULE_MOTOR_STATE_FAULT)
    {
        me->state = MODULE_MOTOR_STATE_DISABLED; // 恢复到禁用状态
    }
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 设置目标值
 */
module_motor_status_t module_motor_set_target(module_motor_t *const me, float target_value)
{
    module_motor_status_t status = module_motor_validate_registered(me);
    return (status == MODULE_MOTOR_STATUS_OK) ? me->vptr->set_target(me, target_value) : status;
}

/**
 * @brief 周期更新电机
 * @param me 电机对象
 * @param delta_time_s 时间步长（秒）
 * @return 执行状态
 * @note 若反馈离线且使能，自动进入故障状态
 */
module_motor_status_t module_motor_update(module_motor_t *const me, float delta_time_s)
{
    module_motor_status_t status = module_motor_validate_registered(me);
    uint64_t elapsed_time_us;

    // 使能状态下反馈离线 → 进入故障
    if ((status == MODULE_MOTOR_STATUS_OK) && (me->state == MODULE_MOTOR_STATE_ENABLED) &&
        !me->feedback.is_online)
    {
        status = module_motor_enter_feedback_fault(me);
        me->last_update_status = status;
        return status;
    }
    // 检查时间步长有效性
    if ((status == MODULE_MOTOR_STATUS_OK) && (!isfinite(delta_time_s) || (delta_time_s <= 0.0F)))
    {
        me->last_update_status = MODULE_MOTOR_STATUS_OUT_OF_RANGE;
        return me->last_update_status;
    }
    if (status != MODULE_MOTOR_STATUS_OK)
    {
        return status;
    }

    status = me->vptr->update(me, delta_time_s);
    me->last_update_status = status;
    if (status == MODULE_MOTOR_STATUS_OK)
    {
        me->delta_time_s = delta_time_s;
        elapsed_time_us = (delta_time_s >= ((float)UINT64_MAX / 1000000.0F))
                              ? UINT64_MAX
                              : (uint64_t)(delta_time_s * 1000000.0F + 0.5F);
        if (UINT64_MAX - me->total_runtime_us < elapsed_time_us)
        {
            me->total_runtime_us = UINT64_MAX;
        }
        else
        {
            me->total_runtime_us += elapsed_time_us;
        }
        if (me->control_update_count != UINT32_MAX)
        {
            ++me->control_update_count;
        }
        if (me->state == MODULE_MOTOR_STATE_ENABLED)
        {
            if (UINT64_MAX - me->enabled_runtime_us < elapsed_time_us)
            {
                me->enabled_runtime_us = UINT64_MAX;
            }
            else
            {
                me->enabled_runtime_us += elapsed_time_us;
            }
        }
    }
    return status;
}

/* ======================== 反馈管理 ======================== */

/**
 * @brief 设置反馈超时时间
 * @param me 电机对象
 * @param feedback_timeout_ms 超时时间（毫秒），0 表示禁用
 */
module_motor_status_t module_motor_set_feedback_timeout(module_motor_t *const me,
                                                        uint32_t feedback_timeout_ms)
{
    if (me == NULL)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_MOTOR_STATUS_NOT_INITIALIZED;
    }
    me->feedback_timeout_ms = feedback_timeout_ms;
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 更新反馈超时计时
 * @param me 电机对象
 * @param elapsed_time_ms 距上次调用的时间（毫秒）
 * @return 执行状态
 * @note 需周期性调用（由任务或健康管理器驱动）
 */
module_motor_status_t module_motor_update_feedback_time(module_motor_t *const me,
                                                        uint32_t elapsed_time_ms)
{
    if (me == NULL)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_MOTOR_STATUS_NOT_INITIALIZED;
    }
    if (!me->feedback.is_online)
    {
        return MODULE_MOTOR_STATUS_OK;
    }

    // 累加时间（防溢出）
    if (elapsed_time_ms > (UINT32_MAX - me->feedback.elapsed_time_since_update_ms))
    {
        me->feedback.elapsed_time_since_update_ms = UINT32_MAX;
    }
    else
    {
        me->feedback.elapsed_time_since_update_ms += elapsed_time_ms;
    }

    // 检查超时
    if ((me->feedback_timeout_ms > 0U) &&
        (me->feedback.elapsed_time_since_update_ms >= me->feedback_timeout_ms))
    {
        me->feedback.is_online = false;
        // 若使能状态，进入故障
        if (me->state == MODULE_MOTOR_STATE_ENABLED)
        {
            return module_motor_enter_feedback_fault(me);
        }
    }
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 通知反馈已更新（由派生类在解析反馈后调用）
 * @param me 电机对象
 * @return 执行状态
 * @note 重置超时计时，标记在线，递增更新计数
 */
module_motor_status_t module_motor_notify_feedback(module_motor_t *const me)
{
    if (me == NULL)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_MOTOR_STATUS_NOT_INITIALIZED;
    }
    // 重置超时计时
    me->feedback.elapsed_time_since_update_ms = 0U;
    me->feedback.is_online = true;

    // 递增更新计数（防溢出）
    if (me->feedback.update_count != UINT32_MAX)
    {
        ++me->feedback.update_count;
    }
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 获取反馈数据指针
 * @param me 电机对象
 * @return 反馈指针，未注册或未初始化则返回 NULL
 */
const module_motor_feedback_t *module_motor_get_feedback(const module_motor_t *const me)
{
    return (module_motor_validate_registered(me) == MODULE_MOTOR_STATUS_OK) ? &me->feedback : NULL;
}
