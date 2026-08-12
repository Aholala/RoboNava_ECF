/**
 * @file bsp_usart.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BSP USART/UART 串行通信抽象层
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供驱动无关的 USART/UART 外设接口。
 *       支持发送、接收、空闲线检测和双缓冲 DMA 接收。
 *       遵循面向对象的 BSP 模式，具备初始化/反初始化生命周期。
 */
#ifndef BSP_USART_H
#define BSP_USART_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 双缓冲回调，在通过 DMA 空闲线检测填满一个缓冲区时调用
 *  @param buffer_index 被填满的缓冲区索引（0 或 1）
 *  @param received_size 接收到缓冲区中的字节数
 *  @param context 用户提供的上下文指针
 */
typedef void (*bsp_usart_double_buffer_callback_t)(uint8_t, size_t, void *);

/* ======================== 驱动操作表 ======================== */
/** @brief USART 硬件后端的虚拟驱动操作表 */
typedef struct {
    bsp_status_t (*init)(void *);                           /**< @brief 初始化 USART 外设 */
    bsp_status_t (*deinit)(void *);                         /**< @brief 反初始化 USART 外设 */
    bsp_status_t (*transmit)(void *, const uint8_t *, size_t, bsp_transfer_mode_t, uint32_t); /**< @brief 发送数据 */
    bsp_status_t (*receive)(void *, uint8_t *, size_t, bsp_transfer_mode_t, uint32_t);        /**< @brief 接收数据 */
    bsp_status_t (*receive_to_idle)(void *, uint8_t *, size_t, bsp_transfer_mode_t, uint32_t);/**< @brief 接收直到检测到空闲线 */
    bsp_status_t (*receive_to_idle_double_buffer)(void *, uint8_t *, uint8_t *, size_t);     /**< @brief DMA 双缓冲空闲接收 */
    bsp_status_t (*abort)(void *);                          /**< @brief 中止任何进行中的传输 */
    bsp_status_t (*get_busy)(const void *, bool *);         /**< @brief 查询外设是否忙碌 */
} bsp_usart_driver_ops_t;

/* ======================== USART 对象 ======================== */
/** @brief USART 实例（不透明成员，仅通过公共 API 访问） */
typedef struct bsp_usart {
    void *device_handle;                                    /**< @brief 指向硬件特定句柄的指针 */
    const bsp_usart_driver_ops_t *driver_ops;               /**< @brief 虚拟驱动操作表 */
    bsp_event_callback_t callback;                          /**< @brief 用户事件回调 */
    void *user_context;                                     /**< @brief 回调的不透明用户上下文 */
    bsp_usart_double_buffer_callback_t double_buffer_callback; /**< @brief 双缓冲完成回调 */
    void *double_buffer_user_context;                       /**< @brief 双缓冲回调的不透明用户上下文 */
    bool is_initialized;                                    /**< @brief 成功初始化后为真 */
} bsp_usart_t;

/* ======================== 配置 ======================== */
/** @brief USART 实例的初始化配置 */
typedef struct {
    void *device_handle;                      /**< @brief 指向硬件特定句柄的指针 */
    const bsp_usart_driver_ops_t *driver_ops; /**< @brief 虚拟驱动操作表 */
    bsp_event_callback_t callback;            /**< @brief 可选回调（可为 NULL） */
    void *user_context;                       /**< @brief 可选的用户上下文 */
} bsp_usart_config_t;

/* ======================== 公共 API ======================== */
bsp_status_t bsp_usart_init(bsp_usart_t *me, const bsp_usart_config_t *config);
bsp_status_t bsp_usart_deinit(bsp_usart_t *me);
bsp_status_t bsp_usart_set_callback(bsp_usart_t *me, bsp_event_callback_t callback, void *context);
bsp_status_t bsp_usart_transmit(bsp_usart_t *me, const uint8_t *data, size_t size,
                                bsp_transfer_mode_t mode, uint32_t timeout_ms);
bsp_status_t bsp_usart_receive(bsp_usart_t *me, uint8_t *data, size_t size,
                               bsp_transfer_mode_t mode, uint32_t timeout_ms);
bsp_status_t bsp_usart_receive_to_idle(bsp_usart_t *me, uint8_t *data, size_t capacity,
                                       bsp_transfer_mode_t mode, uint32_t timeout_ms);
bsp_status_t bsp_usart_receive_to_idle_double_buffer(
    bsp_usart_t *me, uint8_t *first, uint8_t *second, size_t capacity,
    bsp_usart_double_buffer_callback_t callback, void *context);
bsp_status_t bsp_usart_abort(bsp_usart_t *me);
bsp_status_t bsp_usart_get_busy(const bsp_usart_t *me, bool *is_busy);
void bsp_usart_notify(bsp_usart_t *me, bsp_event_t event, bsp_status_t status, size_t size);
void bsp_usart_notify_double_buffer(bsp_usart_t *me, uint8_t buffer_index, size_t received_size);

#ifdef __cplusplus
}
#endif
#endif
