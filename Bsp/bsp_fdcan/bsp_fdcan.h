/**
 * @file bsp_fdcan.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief FDCAN 抽象层头文件 — 支持 Classic CAN 和 CAN FD 的统一接口
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 兼容标准 CAN 2.0 (Classic) 和 CAN FD (可变速率/可变数据长度)。
 *       bsp_fdcan_frame_t 支持最多 64 字节数据载荷与 FD 加速位速率切换。
 *       事件回调通过 bsp_fdcan_notify() 由驱动层触发。
 */

#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H
#include "bsp_can.h"
#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 类型定义 ======================== */

/** @brief FDCAN 帧格式类型 */
typedef enum {BSP_FDCAN_FORMAT_CLASSIC=0,               /**< 经典 CAN 2.0 */
              BSP_FDCAN_FORMAT_FD_NO_BRS,               /**< CAN FD 无比特率切换 */
              BSP_FDCAN_FORMAT_FD_BRS} bsp_fdcan_format_t; /**< CAN FD 带比特率切换 */

/** @brief FDCAN 帧结构 — 兼容 Classic 和 FD 帧 */
typedef struct {uint32_t identifier;                    /**< CAN 标识符 */
    bsp_can_id_type_t id_type;                          /**< 标准帧/扩展帧 */
    bsp_can_frame_type_t frame_type;                    /**< 数据帧/远程帧 */
    bsp_fdcan_format_t format;                          /**< Classic / FD 格式 */
    uint8_t data_length;                                /**< 数据长度（字节） */
    uint8_t data[64];} bsp_fdcan_frame_t;               /**< 数据载荷（FD 最大 64 字节） */

/** @brief FDCAN 协议状态寄存器摘要 */
typedef struct {bool is_bus_off;                        /**< 总线关闭标志 */
    bool is_error_passive;                              /**< 错误被动标志 */
    bool has_warning;                                   /**< 错误警告标志 */
    uint8_t transmit_error_count;                       /**< 发送错误计数器 */
    uint8_t receive_error_count;                        /**< 接收错误计数器 */
    uint32_t last_error_code;} bsp_fdcan_protocol_status_t; /**< 最近错误码 */

/** @brief FDCAN 驱动操作表 — 由平台层实现 */
typedef struct {bsp_status_t(*init)(void*);               /**< 硬件初始化（可选） */
    bsp_status_t(*deinit)(void*);                        /**< 硬件反初始化（可选） */
    bsp_status_t(*start)(void*);                         /**< 启动外设 */
    bsp_status_t(*stop)(void*);                          /**< 停止外设 */
    bsp_status_t(*configure_filter)(void*,const bsp_can_filter_t*); /**< 配置硬件滤波器 */
    bsp_status_t(*transmit)(void*,const bsp_fdcan_frame_t*,uint32_t); /**< 发送帧 */
    bsp_status_t(*receive)(void*,bsp_can_receive_fifo_t,bsp_fdcan_frame_t*); /**< 接收帧 */
    bsp_status_t(*get_protocol_status)(const void*,bsp_fdcan_protocol_status_t*); /**< 读取协议状态（可选） */
    bsp_status_t(*get_transmit_free_level)(const void*,uint32_t*);} bsp_fdcan_driver_ops_t; /**< 读取发送缓冲区空闲级别（可选） */

/** @brief FDCAN 实例对象 */
typedef struct bsp_fdcan {void *device_handle;           /**< 硬件句柄 */
    const bsp_fdcan_driver_ops_t *driver_ops;            /**< 驱动操作表 */
    bsp_event_callback_t callback;                       /**< 事件回调 */
    void *user_context;                                  /**< 用户上下文 */
    bool is_initialized;} bsp_fdcan_t;                   /**< 初始化标志 */

/** @brief FDCAN 初始化配置 */
typedef struct {void *device_handle;                     /**< 硬件句柄 */
    const bsp_fdcan_driver_ops_t *driver_ops;            /**< 驱动操作表 */
    bsp_event_callback_t callback;                       /**< 事件回调 */
    void *user_context;} bsp_fdcan_config_t;             /**< 用户上下文 */

/* ======================== 公共 API ======================== */

bsp_status_t bsp_fdcan_init(bsp_fdcan_t*,const bsp_fdcan_config_t*);
bsp_status_t bsp_fdcan_deinit(bsp_fdcan_t*);
bsp_status_t bsp_fdcan_set_callback(bsp_fdcan_t*,bsp_event_callback_t,void*);
bsp_status_t bsp_fdcan_start(bsp_fdcan_t*); bsp_status_t bsp_fdcan_stop(bsp_fdcan_t*);
bsp_status_t bsp_fdcan_configure_filter(bsp_fdcan_t*,const bsp_can_filter_t*);
bsp_status_t bsp_fdcan_transmit(bsp_fdcan_t*,const bsp_fdcan_frame_t*,uint32_t);
bsp_status_t bsp_fdcan_receive(bsp_fdcan_t*,bsp_can_receive_fifo_t,bsp_fdcan_frame_t*);
bsp_status_t bsp_fdcan_get_protocol_status(const bsp_fdcan_t*,bsp_fdcan_protocol_status_t*);
bsp_status_t bsp_fdcan_get_transmit_free_level(const bsp_fdcan_t*,uint32_t*);

/**
 * @brief 事件通知入口 — 由驱动层 ISR/DMA 回调触发
 * @param m FDCAN 对象指针
 * @param e 事件类型
 * @param s 事件关联状态
 * @param n 传输字节数
 * @note 内部做非空和初始化检查后调用用户回调
 */
void bsp_fdcan_notify(bsp_fdcan_t*,bsp_event_t,bsp_status_t,size_t);

#ifdef __cplusplus
}
#endif
#endif
