/**
 * @file bsp_watchdog.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BSP 独立看门狗抽象层 —— 实现
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 实现 bsp_watchdog.h 中定义的驱动无关看门狗接口。
 *       提供喂狗、超时查询和复位原因检测。
 *       看门狗一旦启动，在 STM32 上通常无法停止 ——
 *       反初始化在大多数硬件后端上是空操作。
 */
#include "bsp_watchdog.h"

/* ======================== 内部辅助函数 ======================== */

/**
 * @brief 验证看门狗对象已初始化且非空
 * @param me 看门狗对象指针
 * @return 有效则返回 BSP_STATUS_OK，为 NULL 则返回 BSP_STATUS_INVALID_ARGUMENT，
 *         未初始化则返回 BSP_STATUS_NOT_INITIALIZED
 */
static bsp_status_t bsp_watchdog_validate(const bsp_watchdog_t *me)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return me->is_initialized ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

/* ======================== 生命周期 ======================== */

/**
 * @brief 使用给定配置初始化看门狗实例
 * @param me 看门狗对象指针（入口时需清零）
 * @param config 配置参数（设备句柄、驱动操作表）
 * @return 成功返回 BSP_STATUS_OK，否则返回驱动特定错误码
 * @note 必需的驱动操作：refresh。首先调用可选的驱动初始化钩子，
 *       该钩子通常配置并使能看门狗硬件。
 */
bsp_status_t bsp_watchdog_init(bsp_watchdog_t *me, const bsp_watchdog_config_t *config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->refresh == NULL))
        return BSP_STATUS_INVALID_ARGUMENT;

    *me = (bsp_watchdog_t){0};
    if (config->driver_ops->init != NULL) {
        status = config->driver_ops->init(config->device_handle);
        if (status != BSP_STATUS_OK) return status;
    }
    me->device_handle = config->device_handle;
    me->driver_ops = config->driver_ops;
    me->is_initialized = true;
    return BSP_STATUS_OK;
}

/**
 * @brief 反初始化看门狗实例
 * @param me 看门狗对象指针
 * @return 成功返回 BSP_STATUS_OK，否则返回驱动特定错误码
 * @note 在大多数 STM32 IWDG/WWDG 外设上，看门狗一旦使能就无法停止；
 *       反初始化仅将对象标记为未初始化状态。
 */
bsp_status_t bsp_watchdog_deinit(bsp_watchdog_t *me)
{
    bsp_status_t status = bsp_watchdog_validate(me);
    if (status != BSP_STATUS_OK) return status;
    status = (me->driver_ops->deinit != NULL) ? me->driver_ops->deinit(me->device_handle)
                                              : BSP_STATUS_OK;
    if (status == BSP_STATUS_OK) me->is_initialized = false;
    return status;
}

/* ======================== 看门狗控制 ======================== */

/**
 * @brief 刷新（喂狗）看门狗以防止系统复位
 * @param me 看门狗对象指针
 * @return 成功返回 BSP_STATUS_OK，否则返回驱动特定错误码
 * @note 必须在看门狗超时窗口内周期性调用。
 *       未及时喂狗将触发硬件复位。
 */
bsp_status_t bsp_watchdog_refresh(bsp_watchdog_t *me)
{
    const bsp_status_t status = bsp_watchdog_validate(me);
    return (status == BSP_STATUS_OK) ? me->driver_ops->refresh(me->device_handle) : status;
}

/* ======================== 状态查询 ======================== */

/**
 * @brief 获取配置的看门狗超时时间（毫秒）
 * @param me 看门狗对象指针（const）
 * @param timeout_ms 超时值（毫秒）的输出指针
 * @return 成功返回 BSP_STATUS_OK，驱动未实现 get_timeout_ms 则返回
 *         BSP_STATUS_UNSUPPORTED
 */
bsp_status_t bsp_watchdog_get_timeout_ms(const bsp_watchdog_t *me, uint32_t *timeout_ms)
{
    const bsp_status_t status = bsp_watchdog_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if (timeout_ms == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return (me->driver_ops->get_timeout_ms != NULL)
               ? me->driver_ops->get_timeout_ms(me->device_handle, timeout_ms)
               : BSP_STATUS_UNSUPPORTED;
}

/**
 * @brief 检查上次系统复位是否由看门狗引起
 * @param me 看门狗对象指针（const）
 * @param reset_detected 输出指针，发生了看门狗复位则设为 true
 * @return 成功返回 BSP_STATUS_OK，驱动未实现 get_reset_detected 则返回
 *         BSP_STATUS_UNSUPPORTED
 * @note 用于启动时诊断，判断系统是否从看门狗超时中恢复。
 */
bsp_status_t bsp_watchdog_get_reset_detected(const bsp_watchdog_t *me, bool *reset_detected)
{
    const bsp_status_t status = bsp_watchdog_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if (reset_detected == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return (me->driver_ops->get_reset_detected != NULL)
               ? me->driver_ops->get_reset_detected(me->device_handle, reset_detected)
               : BSP_STATUS_UNSUPPORTED;
}
