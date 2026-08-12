/**
 * @file bsp_usb_vcp.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BSP USB 虚拟串口抽象层
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供驱动无关的 USB CDC ACM 虚拟串口外设接口。
 *       支持发送、接收、连接状态查询和事件通知。
 *       遵循面向对象的 BSP 模式，具备初始化/反初始化生命周期。
 */
#ifndef BSP_USB_VCP_H
#define BSP_USB_VCP_H
#include "bsp_common.h"
#ifdef __cplusplus
extern "C" {
#endif
/* ======================== 驱动操作表 ======================== */
/** @brief USB VCP 硬件后端的虚拟驱动操作表 */
typedef struct {
    bsp_status_t (*init)(void *);                           /**< @brief 初始化 USB CDC 外设 */
    bsp_status_t (*deinit)(void *);                         /**< @brief 反初始化 USB CDC 外设 */
    bsp_status_t (*transmit)(void *, const uint8_t *, size_t, uint32_t); /**< @brief 带超时的数据发送 */
    bsp_status_t (*receive)(void *, uint8_t *, size_t);     /**< @brief 接收数据到缓冲区 */
    bsp_status_t (*abort)(void *);                          /**< @brief 中止任何进行中的传输 */
    bsp_status_t (*get_connected)(const void *, bool *);    /**< @brief 查询 USB 主机连接状态 */
    bsp_status_t (*get_busy)(const void *, bool *);         /**< @brief 查询外设是否忙碌 */
} bsp_usb_vcp_driver_ops_t;

/* ======================== VCP 对象 ======================== */
/** @brief USB VCP 实例（不透明成员，仅通过公共 API 访问） */
typedef struct bsp_usb_vcp {
    void *device_handle;                          /**< @brief 指向硬件特定句柄的指针 */
    const bsp_usb_vcp_driver_ops_t *driver_ops;   /**< @brief 虚拟驱动操作表 */
    bsp_event_callback_t callback;                /**< @brief 用户事件回调 */
    void *user_context;                           /**< @brief 回调的不透明用户上下文 */
    bool is_initialized;                          /**< @brief 成功初始化后为真 */
} bsp_usb_vcp_t;

/* ======================== 配置 ======================== */
/** @brief USB VCP 实例的初始化配置 */
typedef struct {
    void *device_handle;                          /**< @brief 指向硬件特定句柄的指针 */
    const bsp_usb_vcp_driver_ops_t *driver_ops;   /**< @brief 虚拟驱动操作表 */
    bsp_event_callback_t callback;                /**< @brief 可选回调（可为 NULL） */
    void *user_context;                           /**< @brief 可选的用户上下文 */
} bsp_usb_vcp_config_t;

/* ======================== 公共 API ======================== */
bsp_status_t bsp_usb_vcp_init(bsp_usb_vcp_t *, const bsp_usb_vcp_config_t *);
bsp_status_t bsp_usb_vcp_deinit(bsp_usb_vcp_t *);
bsp_status_t bsp_usb_vcp_set_callback(bsp_usb_vcp_t *, bsp_event_callback_t, void *);
bsp_status_t bsp_usb_vcp_transmit(bsp_usb_vcp_t *, const uint8_t *, size_t, uint32_t);
bsp_status_t bsp_usb_vcp_receive(bsp_usb_vcp_t *, uint8_t *, size_t);
bsp_status_t bsp_usb_vcp_abort(bsp_usb_vcp_t *);
bsp_status_t bsp_usb_vcp_get_connected(const bsp_usb_vcp_t *, bool *);
bsp_status_t bsp_usb_vcp_get_busy(const bsp_usb_vcp_t *, bool *);
void bsp_usb_vcp_notify(bsp_usb_vcp_t *, bsp_event_t, bsp_status_t, size_t);
#ifdef __cplusplus
}
#endif
#endif
