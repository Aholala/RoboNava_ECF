/**
 * @file bsp_usart.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BSP USART/UART 串行通信抽象层 —— 实现
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 实现 bsp_usart.h 中定义的驱动无关 USART 接口。
 *       支持标准发送/接收、空闲线 DMA 接收和用于高吞吐量流式传输的双缓冲 DMA。
 */
#include "bsp_usart.h"

/* ======================== 内部辅助函数 ======================== */

/**
 * @brief 验证 USART 对象已初始化且非空
 * @param me USART 对象指针
 * @return 有效则返回 BSP_STATUS_OK，为 NULL 则返回 BSP_STATUS_INVALID_ARGUMENT，
 *         未初始化则返回 BSP_STATUS_NOT_INITIALIZED
 */
static bsp_status_t bsp_usart_validate(const bsp_usart_t *me)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return me->is_initialized ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

/* ======================== 生命周期 ======================== */

/**
 * @brief 使用给定配置初始化 USART 实例
 * @param me USART 对象指针（入口时需清零）
 * @param config 配置参数（设备句柄、驱动操作表、可选回调）
 * @return 成功返回 BSP_STATUS_OK，否则返回驱动特定错误码
 * @note 必需的驱动操作：transmit、receive。首先调用可选的驱动初始化钩子，
 *       然后装配对象字段。
 */
bsp_status_t bsp_usart_init(bsp_usart_t *me, const bsp_usart_config_t *config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->transmit == NULL) ||
        (config->driver_ops->receive == NULL)) return BSP_STATUS_INVALID_ARGUMENT;
    *me = (bsp_usart_t){0};
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
 * @brief 反初始化 USART 实例并释放硬件资源
 * @param me USART 对象指针
 * @return 成功返回 BSP_STATUS_OK，否则返回驱动特定错误码
 */
bsp_status_t bsp_usart_deinit(bsp_usart_t *me)
{
    bsp_status_t status = bsp_usart_validate(me);
    if (status != BSP_STATUS_OK) return status;
    status = (me->driver_ops->deinit != NULL) ? me->driver_ops->deinit(me->device_handle)
                                              : BSP_STATUS_OK;
    if (status == BSP_STATUS_OK) me->is_initialized = false;
    return status;
}

/* ======================== 回调配置 ======================== */

/**
 * @brief 设置或替换 USART 事件回调及其用户上下文
 * @param me USART 对象指针
 * @param callback 回调函数指针（可为 NULL 以禁用）
 * @param context 传递给回调的不透明用户上下文
 * @return 成功返回 BSP_STATUS_OK
 */
bsp_status_t bsp_usart_set_callback(bsp_usart_t *me, bsp_event_callback_t callback, void *context)
{
    const bsp_status_t status = bsp_usart_validate(me);
    if (status != BSP_STATUS_OK) return status;
    me->callback = callback;
    me->user_context = context;
    return BSP_STATUS_OK;
}

/* ======================== 传输验证 ======================== */

/**
 * @brief 检查传输请求是否具有有效的数据、大小和模式
 * @param data 缓冲区指针
 * @param size 要传输的字节数
 * @param mode 传输模式（阻塞 / 中断 / DMA）
 * @return 传输参数有效则返回 true
 */
static bool transfer_is_valid(const void *data, size_t size, bsp_transfer_mode_t mode)
{
    return (data != NULL) && (size > 0U) && bsp_transfer_mode_is_valid(mode);
}

/* ======================== 发送 / 接收 ======================== */

/**
 * @brief 通过 USART 发送数据
 * @param me USART 对象指针
 * @param data 指向发送缓冲区的指针（不能为 NULL）
 * @param size 要发送的字节数（必须 > 0）
 * @param mode 传输模式（阻塞、中断或 DMA）
 * @param timeout_ms 超时时间（毫秒，用于阻塞模式）
 * @return 成功返回 BSP_STATUS_OK，否则返回驱动特定错误码
 */
bsp_status_t bsp_usart_transmit(bsp_usart_t *me, const uint8_t *data, size_t size,
                                bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_usart_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if (!transfer_is_valid(data, size, mode)) return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->transmit(me->device_handle, data, size, mode, timeout_ms);
}

/**
 * @brief 从 USART 接收数据
 * @param me USART 对象指针
 * @param data 指向接收缓冲区的指针（不能为 NULL）
 * @param size 要接收的字节数（必须 > 0）
 * @param mode 传输模式（阻塞、中断或 DMA）
 * @param timeout_ms 超时时间（毫秒，用于阻塞模式）
 * @return 成功返回 BSP_STATUS_OK，否则返回驱动特定错误码
 */
bsp_status_t bsp_usart_receive(bsp_usart_t *me, uint8_t *data, size_t size,
                               bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_usart_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if (!transfer_is_valid(data, size, mode)) return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->receive(me->device_handle, data, size, mode, timeout_ms);
}

/* ======================== 空闲线 DMA 接收 ======================== */

/**
 * @brief 接收数据直到检测到空闲线（通常与 DMA 配合使用）
 * @param me USART 对象指针
 * @param data 指向接收缓冲区的指针（不能为 NULL）
 * @param capacity 最大接收字节数（必须 > 0）
 * @param mode 传输模式
 * @param timeout_ms 超时时间（毫秒）
 * @return 成功返回 BSP_STATUS_OK，驱动未实现 receive_to_idle 则返回
 *         BSP_STATUS_UNSUPPORTED
 * @note 当检测到空闲线或缓冲区满时，驱动通过 bsp_usart_notify 通知完成。
 */
bsp_status_t bsp_usart_receive_to_idle(bsp_usart_t *me, uint8_t *data, size_t capacity,
                                       bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_usart_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if (!transfer_is_valid(data, capacity, mode)) return BSP_STATUS_INVALID_ARGUMENT;
    return (me->driver_ops->receive_to_idle != NULL)
               ? me->driver_ops->receive_to_idle(me->device_handle, data, capacity, mode, timeout_ms)
               : BSP_STATUS_UNSUPPORTED;
}

/**
 * @brief 启动双缓冲 DMA 空闲线接收
 * @param me USART 对象指针
 * @param first 指向第一个缓冲区的指针（不能为 NULL）
 * @param second 指向第二个缓冲区的指针（不能为 NULL 且 != first）
 * @param capacity 每个缓冲区的容量（字节，必须 > 0）
 * @param callback 缓冲区填满时调用的回调
 * @param context 传递给回调的不透明用户上下文
 * @return 成功返回 BSP_STATUS_OK，驱动未实现双缓冲接收则返回
 *         BSP_STATUS_UNSUPPORTED
 * @note 两个缓冲区以乒乓方式使用：一个由 DMA 填充时，应用程序处理另一个。
 */
bsp_status_t bsp_usart_receive_to_idle_double_buffer(
    bsp_usart_t *me, uint8_t *first, uint8_t *second, size_t capacity,
    bsp_usart_double_buffer_callback_t callback, void *context)
{
    const bsp_status_t status = bsp_usart_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if ((first == NULL) || (second == NULL) || (first == second) || (capacity == 0U) ||
        (callback == NULL)) return BSP_STATUS_INVALID_ARGUMENT;
    if (me->driver_ops->receive_to_idle_double_buffer == NULL) return BSP_STATUS_UNSUPPORTED;
    me->double_buffer_callback = callback;
    me->double_buffer_user_context = context;
    return me->driver_ops->receive_to_idle_double_buffer(me->device_handle, first, second, capacity);
}

/* ======================== 控制 ======================== */

/**
 * @brief 中止任何进行中的 USART 传输
 * @param me USART 对象指针
 * @return 成功返回 BSP_STATUS_OK，驱动未实现 abort 则返回
 *         BSP_STATUS_UNSUPPORTED
 */
bsp_status_t bsp_usart_abort(bsp_usart_t *me)
{
    const bsp_status_t status = bsp_usart_validate(me);
    if (status != BSP_STATUS_OK) return status;
    return (me->driver_ops->abort != NULL) ? me->driver_ops->abort(me->device_handle)
                                           : BSP_STATUS_UNSUPPORTED;
}

/**
 * @brief 查询 USART 外设当前是否忙碌
 * @param me USART 对象指针（const）
 * @param is_busy 输出指针，传输进行中则设为 true
 * @return 成功返回 BSP_STATUS_OK，驱动未实现 get_busy 则返回
 *         BSP_STATUS_UNSUPPORTED
 */
bsp_status_t bsp_usart_get_busy(const bsp_usart_t *me, bool *is_busy)
{
    const bsp_status_t status = bsp_usart_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if (is_busy == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return (me->driver_ops->get_busy != NULL) ? me->driver_ops->get_busy(me->device_handle, is_busy)
                                              : BSP_STATUS_UNSUPPORTED;
}

/* ======================== 中断服务通知 ======================== */

/**
 * @brief 通知 USART 对象发生了传输事件（由 ISR 调用）
 * @param me USART 对象指针
 * @param event 事件类型（发送完成、接收完成、错误等）
 * @param status 传输状态
 * @param size 传输的字节数
 * @note 由驱动层的中断处理函数调用。如果注册了用户回调则调用之。
 */
void bsp_usart_notify(bsp_usart_t *me, bsp_event_t event, bsp_status_t status, size_t size)
{
    if ((me != NULL) && me->is_initialized && (me->callback != NULL))
        me->callback(event, status, size, me->user_context);
}

/**
 * @brief 通知 USART 对象发生了双缓冲区切换（由 ISR 调用）
 * @param me USART 对象指针
 * @param index 被填满的缓冲区索引（0 或 1）
 * @param size 接收到缓冲区中的字节数
 * @note 由驱动层的 DMA 空闲线中断处理函数调用。
 *       如果注册了双缓冲回调则调用之。
 */
void bsp_usart_notify_double_buffer(bsp_usart_t *me, uint8_t index, size_t size)
{
    if ((me != NULL) && me->is_initialized && (me->double_buffer_callback != NULL))
        me->double_buffer_callback(index, size, me->double_buffer_user_context);
}
