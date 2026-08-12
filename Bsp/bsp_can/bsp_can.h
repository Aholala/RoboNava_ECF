/**
 * @file bsp_can.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief CAN 总线外设抽象层头文件
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供 CAN 总线的硬件无关接口，支持标准帧/扩展帧、数据帧/远程帧、
 *       硬件滤波器和 FIFO 收发。
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief CAN 标识符类型 */
typedef enum { BSP_CAN_ID_STANDARD = 0, BSP_CAN_ID_EXTENDED } bsp_can_id_type_t;

/** @brief CAN 帧类型 */
typedef enum { BSP_CAN_FRAME_DATA = 0, BSP_CAN_FRAME_REMOTE } bsp_can_frame_type_t;

/** @brief CAN 接收 FIFO 选择 */
typedef enum { BSP_CAN_RX_FIFO_0 = 0, BSP_CAN_RX_FIFO_1 } bsp_can_receive_fifo_t;

/** @brief CAN 帧结构 */
typedef struct {
    uint32_t identifier;         /**< 帧标识符 */
    bsp_can_id_type_t id_type;   /**< 标准帧 / 扩展帧 */
    bsp_can_frame_type_t frame_type; /**< 数据帧 / 远程帧 */
    uint8_t data_length;         /**< 数据长度（0~8） */
    uint8_t data[8];             /**< 数据载荷 */
} bsp_can_frame_t;

/** @brief CAN 硬件滤波器配置 */
typedef struct {
    uint32_t identifier;                /**< 滤波器标识符 */
    uint32_t mask;                      /**< 滤波器掩码（1 表示关心该位） */
    bsp_can_id_type_t id_type;          /**< 标准帧 / 扩展帧 */
    bsp_can_receive_fifo_t receive_fifo; /**< 关联的接收 FIFO */
    uint32_t filter_index;              /**< 滤波器硬件索引 */
} bsp_can_filter_t;

/** @brief CAN 底层驱动操作集 */
typedef struct {
    bsp_status_t (*init)(void *);                                      /**< 初始化 CAN 外设 */
    bsp_status_t (*deinit)(void *);                                    /**< 反初始化 CAN 外设 */
    bsp_status_t (*start)(void *);                                     /**< 启动 CAN 通信 */
    bsp_status_t (*stop)(void *);                                      /**< 停止 CAN 通信 */
    bsp_status_t (*configure_filter)(void *, const bsp_can_filter_t *); /**< 配置硬件滤波器 */
    bsp_status_t (*transmit)(void *, const bsp_can_frame_t *, uint32_t); /**< 发送帧 */
    bsp_status_t (*receive)(void *, bsp_can_receive_fifo_t, bsp_can_frame_t *); /**< 接收帧 */
    bsp_status_t (*get_tx_free_level)(const void *, uint32_t *);         /**< 获取发送邮箱空闲等级 */
} bsp_can_driver_ops_t;

/** @brief CAN 实例对象 */
typedef struct bsp_can {
    void *device_handle;                /**< 硬件句柄 */
    const bsp_can_driver_ops_t *driver_ops; /**< 驱动操作集 */
    bsp_event_callback_t callback;      /**< 事件回调 */
    void *user_context;                 /**< 回调用户上下文 */
    bool is_initialized;                /**< 初始化标志 */
} bsp_can_t;

/** @brief CAN 初始化配置 */
typedef struct {
    void *device_handle;                /**< 硬件句柄 */
    const bsp_can_driver_ops_t *driver_ops; /**< 驱动操作集 */
    bsp_event_callback_t callback;      /**< 事件回调 */
    void *user_context;                 /**< 回调用户上下文 */
} bsp_can_config_t;

/* ======================== 公共 API ======================== */

bsp_status_t bsp_can_init(bsp_can_t *me, const bsp_can_config_t *config);
bsp_status_t bsp_can_deinit(bsp_can_t *me);
bsp_status_t bsp_can_set_callback(bsp_can_t *me, bsp_event_callback_t callback, void *context);
bsp_status_t bsp_can_start(bsp_can_t *me);
bsp_status_t bsp_can_stop(bsp_can_t *me);
bsp_status_t bsp_can_configure_filter(bsp_can_t *me, const bsp_can_filter_t *filter);
bsp_status_t bsp_can_transmit(bsp_can_t *me, const bsp_can_frame_t *frame, uint32_t timeout_ms);
bsp_status_t bsp_can_receive(bsp_can_t *me, bsp_can_receive_fifo_t fifo, bsp_can_frame_t *frame);
bsp_status_t bsp_can_get_transmit_free_level(const bsp_can_t *me, uint32_t *free_level);
void bsp_can_notify(bsp_can_t *me, bsp_event_t event, bsp_status_t status, size_t transferred_size);

#ifdef __cplusplus
}
#endif
#endif
