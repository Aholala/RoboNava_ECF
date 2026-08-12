/**
 * @file bsp_pwm.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BSP PWM 外设抽象层实现
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供平台无关的 PWM（脉宽调制）输出控制，支持频率、
 *       脉宽（计数值）和占空比管理。通过 bsp_pwm_bind_platform()
 *       注册全局单例平台驱动，所有通道操作均校验共享的平台操作指针。
 */

#include "bsp_pwm.h"

#include <math.h>

/** @brief 已注册的平台 PWM 驱动操作虚表（全局单例指针） */
static const bsp_pwm_driver_ops_t *bsp_pwm_platform_ops;

/* ======================== 内部验证函数 ======================== */

/**
 * @brief 校验驱动操作虚表是否包含所有必需的回调
 * @param ops 待校验的驱动操作虚表指针
 * @return 所有必需操作均非空返回 true
 */
static bool bsp_pwm_ops_valid(const bsp_pwm_driver_ops_t *ops)
{
    return (ops != NULL) && (ops->start != NULL) && (ops->stop != NULL) &&
           (ops->set_frequency != NULL) && (ops->get_frequency != NULL) &&
           (ops->set_pulse != NULL) && (ops->get_pulse != NULL) && (ops->get_period != NULL);
}

/* ======================== 公共 API - 平台绑定 ======================== */

/**
 * @brief 注册平台特定的 PWM 驱动
 * @param driver_ops 平台 PWM 驱动操作虚表
 * @return 成功返回 BSP_STATUS_OK，已绑定不同驱动返回 BSP_STATUS_BUSY
 * @note 必须在任何 PWM 实例初始化前调用。
 *       每个应用只能注册一个平台驱动。
 */
bsp_status_t bsp_pwm_bind_platform(const bsp_pwm_driver_ops_t *driver_ops)
{
    if (!bsp_pwm_ops_valid(driver_ops)) return BSP_STATUS_INVALID_ARGUMENT;
    if ((bsp_pwm_platform_ops != NULL) && (bsp_pwm_platform_ops != driver_ops)) return BSP_STATUS_BUSY;
    bsp_pwm_platform_ops = driver_ops;
    return BSP_STATUS_OK;
}

/* ======================== 公共 API - 生命周期 ======================== */

/**
 * @brief 初始化 PWM 通道实例
 * @param me PWM 对象指针
 * @param config PWM 配置（设备句柄、驱动操作、通道号）
 * @return 成功返回 BSP_STATUS_OK，或平台/驱动错误码
 * @note 内部自动调用 bsp_pwm_bind_platform() 注册驱动。
 */
bsp_status_t bsp_pwm_init(bsp_pwm_t *me, const bsp_pwm_config_t *config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL)) return BSP_STATUS_INVALID_ARGUMENT;
    status = bsp_pwm_bind_platform(config->driver_ops);
    if (status != BSP_STATUS_OK) return status;
    if (bsp_pwm_platform_ops->init != NULL)
    {
        status = bsp_pwm_platform_ops->init(config->device_handle, config->channel);
        if (status != BSP_STATUS_OK) return status;
    }
    me->device_handle = config->device_handle; me->channel = config->channel; me->is_initialized = true;
    return BSP_STATUS_OK;
}

/**
 * @brief 反初始化 PWM 通道并停止输出
 * @param me PWM 对象指针
 * @return 成功返回 BSP_STATUS_OK
 * @note 反初始化前先停止 PWM 输出。
 */
bsp_status_t bsp_pwm_deinit(bsp_pwm_t *me)
{
    bsp_status_t status = BSP_STATUS_OK;
    if ((me == NULL) || !me->is_initialized || (bsp_pwm_platform_ops == NULL))
        return (me == NULL) ? BSP_STATUS_INVALID_ARGUMENT : BSP_STATUS_NOT_INITIALIZED;
    (void)bsp_pwm_platform_ops->stop(me->device_handle, me->channel);
    if (bsp_pwm_platform_ops->deinit != NULL) status = bsp_pwm_platform_ops->deinit(me->device_handle, me->channel);
    if (status == BSP_STATUS_OK) { me->device_handle = NULL; me->channel = 0U; me->is_initialized = false; }
    return status;
}

/**
 * @brief 检查 PWM 实例是否已完全初始化并就绪
 * @param me PWM 对象指针（只读）
 * @return 已初始化且平台驱动有效返回 true
 */
bool bsp_pwm_is_initialized(const bsp_pwm_t *me)
{ return (me != NULL) && me->is_initialized && (bsp_pwm_platform_ops != NULL); }

/* ======================== 公共 API - 输出控制 ======================== */

/**
 * @brief 简易 PWM 操作的便捷宏，自动完成实例校验
 * @param me PWM 对象指针
 * @param member 要调用的驱动操作成员（如 start、stop）
 * @note me 为空返回 BSP_STATUS_INVALID_ARGUMENT，
 *       me 未初始化返回 BSP_STATUS_NOT_INITIALIZED，
 *       否则调用指定的驱动函数。
 */
#define BSP_PWM_CALL(me, member) \
    (((me) == NULL) ? BSP_STATUS_INVALID_ARGUMENT : \
     (!bsp_pwm_is_initialized(me) ? BSP_STATUS_NOT_INITIALIZED : \
      bsp_pwm_platform_ops->member((me)->device_handle, (me)->channel)))

/**
 * @brief 启动指定通道的 PWM 输出
 * @param me PWM 对象指针
 * @return 成功返回 BSP_STATUS_OK
 */
bsp_status_t bsp_pwm_start(bsp_pwm_t *me) { return BSP_PWM_CALL(me, start); }

/**
 * @brief 停止指定通道的 PWM 输出
 * @param me PWM 对象指针
 * @return 成功返回 BSP_STATUS_OK
 */
bsp_status_t bsp_pwm_stop(bsp_pwm_t *me) { return BSP_PWM_CALL(me, stop); }

/* ======================== 公共 API - 频率 ======================== */

/**
 * @brief 设置 PWM 输出频率
 * @param me PWM 对象指针
 * @param frequency_hz 目标频率（Hz），必须大于 0
 * @return 成功返回 BSP_STATUS_OK，频率为 0 返回 BSP_STATUS_OUT_OF_RANGE
 */
bsp_status_t bsp_pwm_set_frequency(bsp_pwm_t *me, uint32_t frequency_hz)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_pwm_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    if (frequency_hz == 0U) return BSP_STATUS_OUT_OF_RANGE;
    return bsp_pwm_platform_ops->set_frequency(me->device_handle, me->channel, frequency_hz);
}

/**
 * @brief 获取当前 PWM 输出频率
 * @param me PWM 对象指针（只读）
 * @param frequency_hz 输出：当前频率（Hz）
 * @return 成功返回 BSP_STATUS_OK
 */
bsp_status_t bsp_pwm_get_frequency(const bsp_pwm_t *me, uint32_t *frequency_hz)
{
    if ((me == NULL) || (frequency_hz == NULL)) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_pwm_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    return bsp_pwm_platform_ops->get_frequency(me->device_handle, me->channel, frequency_hz);
}

/* ======================== 公共 API - 脉宽 ======================== */

/**
 * @brief 设置 PWM 脉宽（定时器计数值）
 * @param me PWM 对象指针
 * @param pulse_ticks 脉宽计数值（不得超出周期值）
 * @return 成功返回 BSP_STATUS_OK，脉宽超出周期返回 BSP_STATUS_OUT_OF_RANGE
 * @note 脉宽被钳位至当前周期值。
 */
bsp_status_t bsp_pwm_set_pulse(bsp_pwm_t *me, uint32_t pulse_ticks)
{
    uint32_t period_ticks; bsp_status_t status;
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_pwm_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    status = bsp_pwm_platform_ops->get_period(me->device_handle, me->channel, &period_ticks);
    if (status != BSP_STATUS_OK) return status;
    if (pulse_ticks > period_ticks) return BSP_STATUS_OUT_OF_RANGE;
    return bsp_pwm_platform_ops->set_pulse(me->device_handle, me->channel, pulse_ticks);
}

/**
 * @brief 获取当前 PWM 脉宽（定时器计数值）
 * @param me PWM 对象指针（只读）
 * @param pulse_ticks 输出：当前脉宽计数值
 * @return 成功返回 BSP_STATUS_OK
 */
bsp_status_t bsp_pwm_get_pulse(const bsp_pwm_t *me, uint32_t *pulse_ticks)
{
    if ((me == NULL) || (pulse_ticks == NULL)) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_pwm_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    return bsp_pwm_platform_ops->get_pulse(me->device_handle, me->channel, pulse_ticks);
}

/* ======================== 公共 API - 占空比 ======================== */

/**
 * @brief 设置 PWM 占空比（0.0 ~ 1.0）
 * @param me PWM 对象指针
 * @param duty_cycle 占空比比例 [0.0, 1.0]
 * @return 成功返回 BSP_STATUS_OK，duty_cycle 无效返回 BSP_STATUS_OUT_OF_RANGE
 * @note 内部根据当前周期将占空比换算为脉宽计数值。
 *       使用四舍五入（加 0.5）进行计数值转换。
 */
bsp_status_t bsp_pwm_set_duty_cycle(bsp_pwm_t *me, float duty_cycle)
{
    uint32_t period_ticks; bsp_status_t status;
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_pwm_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    if (!isfinite(duty_cycle) || (duty_cycle < 0.0F) || (duty_cycle > 1.0F)) return BSP_STATUS_OUT_OF_RANGE;
    status = bsp_pwm_platform_ops->get_period(me->device_handle, me->channel, &period_ticks);
    if (status != BSP_STATUS_OK) return status;
    return bsp_pwm_platform_ops->set_pulse(me->device_handle, me->channel,
                                            (uint32_t)((float)period_ticks * duty_cycle + 0.5F));
}

/**
 * @brief 获取当前 PWM 占空比
 * @param me PWM 对象指针（只读）
 * @param duty_cycle 输出：当前占空比比例 [0.0, 1.0]
 * @return 成功返回 BSP_STATUS_OK，周期为 0 返回 BSP_STATUS_IO_ERROR
 */
bsp_status_t bsp_pwm_get_duty_cycle(const bsp_pwm_t *me, float *duty_cycle)
{
    uint32_t period_ticks, pulse_ticks; bsp_status_t status;
    if ((me == NULL) || (duty_cycle == NULL)) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_pwm_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    status = bsp_pwm_platform_ops->get_period(me->device_handle, me->channel, &period_ticks);
    if ((status != BSP_STATUS_OK) || (period_ticks == 0U)) return (status != BSP_STATUS_OK) ? status : BSP_STATUS_IO_ERROR;
    status = bsp_pwm_platform_ops->get_pulse(me->device_handle, me->channel, &pulse_ticks);
    if (status == BSP_STATUS_OK) *duty_cycle = (float)pulse_ticks / (float)period_ticks;
    return status;
}
