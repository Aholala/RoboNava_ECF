/**
 * @file bsp_can.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief CAN 总线外设抽象层实现
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供 CAN 初始化、收发、滤波器配置和事件回调通知。
 *       内部对帧标识符和帧结构进行合法性校验。
 */

#include "bsp_can.h"

/* ======================== 内部辅助函数 ======================== */

/**
 * @brief 验证 CAN 对象是否有效
 * @param me CAN 对象指针
 * @return BSP_STATUS_OK 有效，否则返回相应错误码
 */
static bsp_status_t bsp_can_validate(const bsp_can_t *me)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return me->is_initialized ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

/**
 * @brief 验证 CAN 标识符是否在合法范围内
 * @param id 标识符值
 * @param type 标准帧 / 扩展帧
 * @return true 合法
 */
static bool bsp_can_is_id_valid(uint32_t id, bsp_can_id_type_t type)
{
    return (type == BSP_CAN_ID_STANDARD) ? (id <= 0x7FFU)
         : (type == BSP_CAN_ID_EXTENDED) ? (id <= 0x1FFFFFFFU)
                                         : false;
}

/**
 * @brief 验证 CAN 帧结构是否合法
 * @param frame 帧指针
 * @return true 合法
 * @note 校验帧指针非空、数据长度 ≤ 8、帧类型和标识符合法
 */
static bool bsp_can_is_frame_valid(const bsp_can_frame_t *frame)
{
    return (frame != NULL) && (frame->data_length <= 8U) &&
           ((frame->frame_type == BSP_CAN_FRAME_DATA) ||
            (frame->frame_type == BSP_CAN_FRAME_REMOTE)) &&
           bsp_can_is_id_valid(frame->identifier, frame->id_type);
}

/* ======================== 公共 API ======================== */

/**
 * @brief 初始化 CAN 实例
 * @param me CAN 对象指针
 * @param config 配置参数
 * @return 执行状态
 */
bsp_status_t bsp_can_init(bsp_can_t *me, const bsp_can_config_t *config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->start == NULL) ||
        (config->driver_ops->stop == NULL) || (config->driver_ops->configure_filter == NULL) ||
        (config->driver_ops->transmit == NULL) || (config->driver_ops->receive == NULL))
        return BSP_STATUS_INVALID_ARGUMENT;
    *me = (bsp_can_t){0};
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
 * @brief 反初始化 CAN 实例
 * @param me CAN 对象指针
 * @return 执行状态
 */
bsp_status_t bsp_can_deinit(bsp_can_t *me)
{
    bsp_status_t status = bsp_can_validate(me);
    if (status != BSP_STATUS_OK) return status;
    status = (me->driver_ops->deinit != NULL) ? me->driver_ops->deinit(me->device_handle)
                                              : BSP_STATUS_OK;
    if (status == BSP_STATUS_OK) me->is_initialized = false;
    return status;
}

/**
 * @brief 设置 CAN 事件回调
 * @param me CAN 对象指针
 * @param callback 回调函数
 * @param context 回调用户上下文
 * @return 执行状态
 */
bsp_status_t bsp_can_set_callback(bsp_can_t *me, bsp_event_callback_t callback, void *context)
{
    const bsp_status_t status = bsp_can_validate(me);
    if (status != BSP_STATUS_OK) return status;
    me->callback = callback;
    me->user_context = context;
    return BSP_STATUS_OK;
}

/**
 * @brief 启动 CAN 通信
 * @param me CAN 对象指针
 * @return 执行状态
 */
bsp_status_t bsp_can_start(bsp_can_t *me)
{
    const bsp_status_t status = bsp_can_validate(me);
    return (status == BSP_STATUS_OK) ? me->driver_ops->start(me->device_handle) : status;
}

/**
 * @brief 停止 CAN 通信
 * @param me CAN 对象指针
 * @return 执行状态
 */
bsp_status_t bsp_can_stop(bsp_can_t *me)
{
    const bsp_status_t status = bsp_can_validate(me);
    return (status == BSP_STATUS_OK) ? me->driver_ops->stop(me->device_handle) : status;
}

/**
 * @brief 配置 CAN 硬件滤波器
 * @param me CAN 对象指针
 * @param filter 滤波器配置
 * @return 执行状态
 */
bsp_status_t bsp_can_configure_filter(bsp_can_t *me, const bsp_can_filter_t *filter)
{
    const bsp_status_t status = bsp_can_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if ((filter == NULL) || !bsp_can_is_id_valid(filter->identifier, filter->id_type) ||
        !bsp_can_is_id_valid(filter->mask, filter->id_type) ||
        ((filter->receive_fifo != BSP_CAN_RX_FIFO_0) &&
         (filter->receive_fifo != BSP_CAN_RX_FIFO_1))) return BSP_STATUS_OUT_OF_RANGE;
    return me->driver_ops->configure_filter(me->device_handle, filter);
}

/**
 * @brief 发送 CAN 帧
 * @param me CAN 对象指针
 * @param frame 待发送的帧
 * @param timeout_ms 超时时间（ms）
 * @return 执行状态
 */
bsp_status_t bsp_can_transmit(bsp_can_t *me, const bsp_can_frame_t *frame, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_can_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if (!bsp_can_is_frame_valid(frame)) return BSP_STATUS_OUT_OF_RANGE;
    return me->driver_ops->transmit(me->device_handle, frame, timeout_ms);
}

/**
 * @brief 接收 CAN 帧
 * @param me CAN 对象指针
 * @param fifo 接收 FIFO 选择
 * @param frame 输出接收到的帧
 * @return 执行状态
 * @note 接收后会二次校验帧合法性
 */
bsp_status_t bsp_can_receive(bsp_can_t *me, bsp_can_receive_fifo_t fifo, bsp_can_frame_t *frame)
{
    bsp_status_t status = bsp_can_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if ((frame == NULL) || ((fifo != BSP_CAN_RX_FIFO_0) && (fifo != BSP_CAN_RX_FIFO_1)))
        return BSP_STATUS_INVALID_ARGUMENT;
    status = me->driver_ops->receive(me->device_handle, fifo, frame);
    if (status != BSP_STATUS_OK) return status;
    return bsp_can_is_frame_valid(frame) ? BSP_STATUS_OK : BSP_STATUS_IO_ERROR;
}

/**
 * @brief 获取 CAN 发送邮箱空闲等级
 * @param me CAN 对象指针
 * @param free_level 输出空闲邮箱数量
 * @return 执行状态
 */
bsp_status_t bsp_can_get_transmit_free_level(const bsp_can_t *me, uint32_t *free_level)
{
    const bsp_status_t status = bsp_can_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if (free_level == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return (me->driver_ops->get_tx_free_level != NULL)
               ? me->driver_ops->get_tx_free_level(me->device_handle, free_level)
               : BSP_STATUS_UNSUPPORTED;
}

/**
 * @brief CAN 事件通知（由底层驱动在中断中调用）
 * @param me CAN 对象指针
 * @param event 事件类型
 * @param status 操作状态
 * @param size 传输数据大小
 * @note 若注册了回调则转发事件给用户回调
 */
void bsp_can_notify(bsp_can_t *me, bsp_event_t event, bsp_status_t status, size_t size)
{
    if ((me != NULL) && me->is_initialized && (me->callback != NULL))
        me->callback(event, status, size, me->user_context);
}
