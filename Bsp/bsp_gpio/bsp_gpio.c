/**
 * @file bsp_gpio.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief GPIO 抽象层实现
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 单例驱动模式：所有 GPIO 实例共享同一套 bsp_gpio_platform_ops。
 *       bind_platform 首次调用绑定驱动，后续调用仅校验一致性。
 */

#include "bsp_gpio.h"

/* ======================== 平台驱动绑定（单例） ======================== */

/** @brief 全局平台驱动操作表 — 所有 GPIO 实例共享 */
static const bsp_gpio_driver_ops_t *bsp_gpio_platform_ops;

/**
 * @brief 校验驱动操作表有效性
 * @param ops 驱动操作表指针
 * @return true read/write/toggle 三个必选函数均非空
 */
static bool bsp_gpio_ops_valid(const bsp_gpio_driver_ops_t *ops)
{
    return (ops != NULL) && (ops->read != NULL) && (ops->write != NULL) &&
           (ops->toggle != NULL);
}

/**
 * @brief 绑定平台驱动（单例模式）
 * @param driver_ops 平台实现的驱动操作表
 * @return BSP_STATUS_OK 成功，BSP_STATUS_BUSY 已绑定不同驱动
 * @note 首次调用绑定驱动，后续调用仅校验是否为同一驱动实例
 */
bsp_status_t bsp_gpio_bind_platform(const bsp_gpio_driver_ops_t *driver_ops)
{
    if (!bsp_gpio_ops_valid(driver_ops))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if ((bsp_gpio_platform_ops != NULL) && (bsp_gpio_platform_ops != driver_ops))
    {
        return BSP_STATUS_BUSY;
    }
    bsp_gpio_platform_ops = driver_ops;
    return BSP_STATUS_OK;
}

/* ======================== 生命周期 ======================== */

/**
 * @brief 初始化 GPIO 实例
 * @param me GPIO 对象指针
 * @param config 配置参数
 * @return 执行状态
 * @note 内部调用 bind_platform 确保驱动已绑定，再调用驱动 init（若存在）
 */
bsp_status_t bsp_gpio_init(bsp_gpio_t *me, const bsp_gpio_config_t *config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    status = bsp_gpio_bind_platform(config->driver_ops);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (bsp_gpio_platform_ops->init != NULL)
    {
        status = bsp_gpio_platform_ops->init(config->device_handle);
        if (status != BSP_STATUS_OK)
        {
            return status;
        }
    }
    me->device_handle = config->device_handle;
    me->is_initialized = true;
    return BSP_STATUS_OK;
}

/**
 * @brief 反初始化 GPIO 实例
 * @param me GPIO 对象指针
 * @return 执行状态
 * @note 调用驱动 deinit（若存在），成功后清除 device_handle 和初始化标志
 */
bsp_status_t bsp_gpio_deinit(bsp_gpio_t *me)
{
    bsp_status_t status = BSP_STATUS_OK;
    if ((me == NULL) || !me->is_initialized || (bsp_gpio_platform_ops == NULL))
    {
        return (me == NULL) ? BSP_STATUS_INVALID_ARGUMENT : BSP_STATUS_NOT_INITIALIZED;
    }
    if (bsp_gpio_platform_ops->deinit != NULL)
    {
        status = bsp_gpio_platform_ops->deinit(me->device_handle);
    }
    if (status == BSP_STATUS_OK)
    {
        me->device_handle = NULL;
        me->is_initialized = false;
    }
    return status;
}

/* ======================== 状态查询 ======================== */

/**
 * @brief 查询 GPIO 实例是否已初始化
 * @param me GPIO 对象指针
 * @return true 已初始化且平台驱动已绑定
 */
bool bsp_gpio_is_initialized(const bsp_gpio_t *me)
{
    return (me != NULL) && me->is_initialized && (bsp_gpio_platform_ops != NULL);
}

/* ======================== IO 操作 ======================== */

/**
 * @brief 读取 GPIO 引脚电平
 * @param me GPIO 对象指针
 * @param level 输出参数，true = 高电平，false = 低电平
 * @return 执行状态
 */
bsp_status_t bsp_gpio_read(const bsp_gpio_t *me, bool *level)
{
    if ((me == NULL) || (level == NULL)) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_gpio_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    return bsp_gpio_platform_ops->read(me->device_handle, level);
}

/**
 * @brief 设置 GPIO 引脚电平
 * @param me GPIO 对象指针
 * @param level true = 高电平，false = 低电平
 * @return 执行状态
 */
bsp_status_t bsp_gpio_write(bsp_gpio_t *me, bool level)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_gpio_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    return bsp_gpio_platform_ops->write(me->device_handle, level);
}

/**
 * @brief 翻转 GPIO 引脚电平
 * @param me GPIO 对象指针
 * @return 执行状态
 */
bsp_status_t bsp_gpio_toggle(bsp_gpio_t *me)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_gpio_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    return bsp_gpio_platform_ops->toggle(me->device_handle);
}
