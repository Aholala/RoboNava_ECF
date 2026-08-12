/**
 * @file bsp_common.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BSP 层公共类型和通用工具头文件
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 定义 BSP 层所有模块共享的状态码、事件类型、传输模式、
 *       回调函数签名和错误记录结构。
 */

#ifndef BSP_COMMON_H
#define BSP_COMMON_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 状态码枚举 ======================== */

/** @brief BSP 层统一状态码 */
typedef enum {
    BSP_STATUS_OK = 0,               /**< 操作成功 */
    BSP_STATUS_INVALID_ARGUMENT,     /**< 无效参数 */
    BSP_STATUS_OUT_OF_RANGE,         /**< 参数超出范围 */
    BSP_STATUS_NOT_INITIALIZED,      /**< 对象未初始化 */
    BSP_STATUS_BUSY,                 /**< 设备忙 */
    BSP_STATUS_TIMEOUT,              /**< 操作超时 */
    BSP_STATUS_IO_ERROR,             /**< IO 错误 */
    BSP_STATUS_NO_RESOURCE,          /**< 无可用资源 */
    BSP_STATUS_UNSUPPORTED           /**< 不支持的操作 */
} bsp_status_t;

/* ======================== 编译期断言 ======================== */

/** @brief 编译期断言：结构体成员 super 必须在偏移 0 处 */
#define BSP_STATIC_ASSERT_SUPER_FIRST(type) \
    _Static_assert(offsetof(type, super) == 0U, #type " must place super first")

/* ======================== 传输模式 ======================== */

/** @brief 数据传输模式 */
typedef enum {
    BSP_TRANSFER_MODE_BLOCKING = 0,  /**< 阻塞模式 */
    BSP_TRANSFER_MODE_INTERRUPT,     /**< 中断模式 */
    BSP_TRANSFER_MODE_DMA            /**< DMA 模式 */
} bsp_transfer_mode_t;

/* ======================== 事件类型 ======================== */

/** @brief BSP 事件类型 */
typedef enum {
    BSP_EVENT_TRANSMIT_COMPLETE = 0, /**< 发送完成 */
    BSP_EVENT_RECEIVE_COMPLETE,      /**< 接收完成 */
    BSP_EVENT_TRANSFER_COMPLETE,     /**< 传输完成 */
    BSP_EVENT_RECEIVE_PENDING,       /**< 有待接收数据 */
    BSP_EVENT_ABORT_COMPLETE,        /**< 中止完成 */
    BSP_EVENT_ERROR                  /**< 错误事件 */
} bsp_event_t;

/* ======================== 回调函数类型 ======================== */

/** @brief 事件回调函数签名 */
typedef void (*bsp_event_callback_t)(bsp_event_t, bsp_status_t, size_t, void *);

/* ======================== 传输模式校验 ======================== */

/**
 * @brief 校验传输模式是否合法
 * @param mode 传输模式
 * @return true 合法
 */
bool bsp_transfer_mode_is_valid(bsp_transfer_mode_t mode);

/* ======================== 错误记录 ======================== */

/** @brief 错误记录结构 */
typedef struct {
    bsp_status_t code;       /**< 错误码 */
    const char *source;      /**< 错误来源描述 */
    int detail;              /**< 错误详情码 */
    bool is_valid;           /**< 记录是否有效 */
} bsp_error_t;

/**
 * @brief 记录一条错误信息
 * @param code 错误码
 * @param source 错误来源描述
 * @param detail 错误详情
 */
void bsp_error_record(bsp_status_t code, const char *source, int detail);

/**
 * @brief 读取最后记录的错误
 * @return 错误记录指针（只读）
 */
const bsp_error_t *bsp_error_read(void);

#ifdef __cplusplus
}
#endif
#endif
