/**
 * @file bsp_exti.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 外部中断抽象层实现
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 单例驱动模式：所有 EXTI 实例共享同一套 bsp_exti_platform_ops。
 *       bind_platform 首次调用绑定驱动，后续调用仅校验一致性。
 */

#include "bsp_exti.h"

/* ======================== 平台驱动绑定（单例） ======================== */

/** @brief 全局平台驱动操作表 — 所有 EXTI 实例共享 */
static const bsp_exti_driver_ops_t *bsp_exti_platform_ops;

/**
 * @brief 绑定平台驱动（单例模式）
 * @param driver_ops 平台实现的驱动操作表
 * @return BSP_STATUS_OK 成功，BSP_STATUS_BUSY 已绑定不同驱动
 * @note 首次调用绑定驱动，后续调用仅校验是否为同一驱动实例
 */
bsp_status_t bsp_exti_bind_platform(const bsp_exti_driver_ops_t *driver_ops)
{
    if ((driver_ops == NULL) || (driver_ops->enable == NULL) || (driver_ops->disable == NULL))
        return BSP_STATUS_INVALID_ARGUMENT;
    if ((bsp_exti_platform_ops != NULL) && (bsp_exti_platform_ops != driver_ops))
        return BSP_STATUS_BUSY;
    bsp_exti_platform_ops = driver_ops;
    return BSP_STATUS_OK;
}

/* ======================== 生命周期 ======================== */

/**
 * @brief 初始化 EXTI 实例
 * @param me EXTI 对象指针
 * @param config 配置参数
 * @return 执行状态
 * @note 内部调用 bind_platform 确保驱动已绑定，再调用驱动 init（若存在）
 */
bsp_status_t bsp_exti_init(bsp_exti_t *me, const bsp_exti_config_t *config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL))
        return BSP_STATUS_INVALID_ARGUMENT;
    status = bsp_exti_bind_platform(config->driver_ops);
    if (status != BSP_STATUS_OK) return status;
    if (bsp_exti_platform_ops->init != NULL)
    {
        status = bsp_exti_platform_ops->init(config->device_handle);
        if (status != BSP_STATUS_OK) return status;
    }
    me->device_handle = config->device_handle;
    me->callback = config->callback;
    me->user_context = config->user_context;
    me->is_initialized = true;
    return BSP_STATUS_OK;
}

/**
 * @brief 反初始化 EXTI 实例
 * @param me EXTI 对象指针
 * @return 执行状态
 * @note 先禁用中断，再调用驱动 deinit（若存在），成功后清除所有字段
 */
bsp_status_t bsp_exti_deinit(bsp_exti_t *me)
{
    bsp_status_t status = BSP_STATUS_OK;
    if ((me == NULL) || !me->is_initialized || (bsp_exti_platform_ops == NULL))
        return (me == NULL) ? BSP_STATUS_INVALID_ARGUMENT : BSP_STATUS_NOT_INITIALIZED;
    (void)bsp_exti_platform_ops->disable(me->device_handle);
    if (bsp_exti_platform_ops->deinit != NULL) status = bsp_exti_platform_ops->deinit(me->device_handle);
    if (status == BSP_STATUS_OK)
    {
        me->device_handle = NULL; me->callback = NULL; me->user_context = NULL;
        me->is_initialized = false;
    }
    return status;
}

/* ======================== 状态查询与配置 ======================== */

/**
 * @brief 查询 EXTI 实例是否已初始化
 * @param me EXTI 对象指针
 * @return true 已初始化且平台驱动已绑定
 */
bool bsp_exti_is_initialized(const bsp_exti_t *me)
{ return (me != NULL) && me->is_initialized && (bsp_exti_platform_ops != NULL); }

/**
 * @brief 设置中断回调
 * @param me EXTI 对象指针
 * @param callback 回调函数指针
 * @param user_context 用户上下文
 * @return 执行状态
 */
bsp_status_t bsp_exti_set_callback(bsp_exti_t *me, bsp_exti_callback_t callback, void *user_context)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_exti_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    me->callback = callback; me->user_context = user_context; return BSP_STATUS_OK;
}

/* ======================== 中断控制 ======================== */

/**
 * @brief 使能外部中断
 * @param me EXTI 对象指针
 * @return 执行状态
 */
bsp_status_t bsp_exti_enable(bsp_exti_t *me)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_exti_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    return bsp_exti_platform_ops->enable(me->device_handle);
}

/**
 * @brief 禁用外部中断
 * @param me EXTI 对象指针
 * @return 执行状态
 */
bsp_status_t bsp_exti_disable(bsp_exti_t *me)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_exti_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    return bsp_exti_platform_ops->disable(me->device_handle);
}

/* ======================== 中断通知 ======================== */

/**
 * @brief ISR 入口 — 将硬件中断事件转发给用户回调
 * @param me EXTI 对象指针
 * @note 内部做非空和初始化检查，安全调用用户回调
 */
void bsp_exti_notify(bsp_exti_t *me)
{
    if (bsp_exti_is_initialized(me) && (me->callback != NULL)) me->callback(me, me->user_context);
}
