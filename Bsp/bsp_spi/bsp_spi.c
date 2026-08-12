/**
 * @file bsp_spi.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BSP SPI 外设抽象层实现
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供平台无关的 SPI 主设备操作，包括发送、接收和全双工交换。
 *       所有操作先校验实例状态，再委托给初始化时注册的平台特定
 *       驱动操作。
 */

#include "bsp_spi.h"

/* ======================== 内部验证函数 ======================== */

/**
 * @brief 校验 SPI 实例是否已正确初始化
 * @param me SPI 对象指针（只读）
 * @return 有效返回 BSP_STATUS_OK，空指针返回 BSP_STATUS_INVALID_ARGUMENT，
 *         未初始化返回 BSP_STATUS_NOT_INITIALIZED
 */
static bsp_status_t bsp_spi_validate(const bsp_spi_t *me)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return me->is_initialized ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

/* ======================== 公共 API - 生命周期 ======================== */

/**
 * @brief 使用平台特定配置初始化 SPI 实例
 * @param me SPI 对象指针
 * @param config SPI 配置，包含设备句柄和驱动操作
 * @return 成功返回 BSP_STATUS_OK，参数无效返回 BSP_STATUS_INVALID_ARGUMENT
 */
bsp_status_t bsp_spi_init(bsp_spi_t *me, const bsp_spi_config_t *config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->transmit == NULL) ||
        (config->driver_ops->receive == NULL)) return BSP_STATUS_INVALID_ARGUMENT;

    *me = (bsp_spi_t){0};
    if (config->driver_ops->init != NULL) {
        status = config->driver_ops->init(config->device_handle);
        if (status != BSP_STATUS_OK) return status;
    }
    me->device_handle = config->device_handle;
    me->driver_ops = config->driver_ops;
    me->callback = config->callback;
    me->user_context = config->user_context;
    me->is_initialized = true;
    return BSP_STATUS_OK;
}

/**
 * @brief 反初始化 SPI 实例并释放硬件资源
 * @param me SPI 对象指针
 * @return 成功返回 BSP_STATUS_OK，或校验/驱动反初始化返回的错误码
 */
bsp_status_t bsp_spi_deinit(bsp_spi_t *me)
{
    bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK) return status;
    status = (me->driver_ops->deinit != NULL) ? me->driver_ops->deinit(me->device_handle)
                                              : BSP_STATUS_OK;
    if (status == BSP_STATUS_OK) me->is_initialized = false;
    return status;
}

/**
 * @brief 注册 SPI 实例的异步事件回调
 * @param me SPI 对象指针
 * @param callback 事件回调函数指针
 * @param context 传递给回调的用户自定义上下文
 * @return 成功返回 BSP_STATUS_OK，或校验错误码
 */
bsp_status_t bsp_spi_set_callback(bsp_spi_t *me, bsp_event_callback_t callback, void *context)
{
    const bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK) return status;
    me->callback = callback;
    me->user_context = context;
    return BSP_STATUS_OK;
}

/* ======================== 公共 API - 数据传输 ======================== */

/**
 * @brief SPI 发送数据（半双工，仅 MOSI）
 * @param me SPI 对象指针
 * @param data 发送数据缓冲区
 * @param size 发送字节数
 * @param mode 传输模式（阻塞/中断/DMA）
 * @param timeout_ms 操作超时时间（毫秒）
 * @return 成功返回 BSP_STATUS_OK，或校验/驱动错误码
 */
bsp_status_t bsp_spi_transmit(bsp_spi_t *me, const uint8_t *data, size_t size,
                              bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if ((data == NULL) || (size == 0U) || !bsp_transfer_mode_is_valid(mode))
        return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->transmit(me->device_handle, data, size, mode, timeout_ms);
}

/**
 * @brief SPI 接收数据（半双工，仅 MISO）
 * @param me SPI 对象指针
 * @param data 接收数据缓冲区
 * @param size 接收字节数
 * @param mode 传输模式（阻塞/中断/DMA）
 * @param timeout_ms 操作超时时间（毫秒）
 * @return 成功返回 BSP_STATUS_OK，或校验/驱动错误码
 */
bsp_status_t bsp_spi_receive(bsp_spi_t *me, uint8_t *data, size_t size,
                             bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if ((data == NULL) || (size == 0U) || !bsp_transfer_mode_is_valid(mode))
        return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->receive(me->device_handle, data, size, mode, timeout_ms);
}

/**
 * @brief SPI 全双工同时收发数据
 * @param me SPI 对象指针
 * @param tx 发送数据缓冲区
 * @param rx 接收数据缓冲区
 * @param size 收发字节数
 * @param mode 传输模式（阻塞/中断/DMA）
 * @param timeout_ms 操作超时时间（毫秒）
 * @return 成功返回 BSP_STATUS_OK，驱动不支持返回 BSP_STATUS_UNSUPPORTED
 */
bsp_status_t bsp_spi_exchange(bsp_spi_t *me, const uint8_t *tx, uint8_t *rx, size_t size,
                              bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if ((tx == NULL) || (rx == NULL) || (size == 0U) || !bsp_transfer_mode_is_valid(mode))
        return BSP_STATUS_INVALID_ARGUMENT;
    return (me->driver_ops->exchange != NULL)
               ? me->driver_ops->exchange(me->device_handle, tx, rx, size, mode, timeout_ms)
               : BSP_STATUS_UNSUPPORTED;
}

/* ======================== 公共 API - 设备状态 ======================== */

/**
 * @brief 中止正在进行的 SPI 传输
 * @param me SPI 对象指针
 * @return 成功返回 BSP_STATUS_OK，驱动不支持返回 BSP_STATUS_UNSUPPORTED
 */
bsp_status_t bsp_spi_abort(bsp_spi_t *me)
{
    const bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK) return status;
    return (me->driver_ops->abort != NULL) ? me->driver_ops->abort(me->device_handle)
                                           : BSP_STATUS_UNSUPPORTED;
}

/**
 * @brief 查询 SPI 总线当前是否忙碌
 * @param me SPI 对象指针（只读）
 * @param is_busy 输出：true 表示忙碌，false 表示空闲
 * @return 成功返回 BSP_STATUS_OK，驱动不支持返回 BSP_STATUS_UNSUPPORTED
 */
bsp_status_t bsp_spi_get_busy(const bsp_spi_t *me, bool *is_busy)
{
    const bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if (is_busy == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return (me->driver_ops->get_busy != NULL) ? me->driver_ops->get_busy(me->device_handle, is_busy)
                                              : BSP_STATUS_UNSUPPORTED;
}

/* ======================== 内部函数 - 事件通知 ======================== */

/**
 * @brief SPI 传输完成事件的内部通知分发器
 * @param me SPI 对象指针
 * @param event 事件类型（如传输完成）
 * @param status 传输状态
 * @param transferred_size 已传输字节数
 * @note 由平台驱动调用，用于通知上层异步事件。
 *       无效或空实例将被静默忽略。
 */
void bsp_spi_notify(bsp_spi_t *me, bsp_event_t event, bsp_status_t status, size_t transferred_size)
{
    if ((me != NULL) && me->is_initialized && (me->callback != NULL))
        me->callback(event, status, transferred_size, me->user_context);
}
