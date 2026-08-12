/**
 * @file bsp_common.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BSP 公共基础设施头文件
 * @note 包含所有 BSP 模块共享的类型、枚举、宏和基类定义。
 *       BSP 对象与状态码保持在本层，不依赖上层模块。
 * @version 2.0
 * @date 2026-08-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BSP_COMMON_H
#define BSP_COMMON_H

#include <stdbool.h> // 布尔类型
#include <stddef.h>  // size_t 等
#include <stdint.h>  // 固定宽度整数类型

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        BSP_STATUS_OK = 0,
        BSP_STATUS_INVALID_ARGUMENT,
        BSP_STATUS_OUT_OF_RANGE,
        BSP_STATUS_NOT_INITIALIZED,
        BSP_STATUS_BUSY,
        BSP_STATUS_TIMEOUT,
        BSP_STATUS_IO_ERROR,
        BSP_STATUS_NO_RESOURCE,
        BSP_STATUS_UNSUPPORTED
    } bsp_status_t;

#define BSP_CONTAINER_OF(pointer, type, member)                                                    \
    ((type *)((uint8_t *)(pointer) - offsetof(type, member)))
#define BSP_CONTAINER_OF_CONST(pointer, type, member)                                              \
    ((const type *)((const uint8_t *)(pointer) - offsetof(type, member)))
#define BSP_STATIC_ASSERT_SUPER_FIRST(derived_type)                                                \
    _Static_assert(offsetof(derived_type, super) == 0U, #derived_type " must place super first")
#define BSP_DEVICE_OBJECT_MAGIC (0x4253504FU)

    /* ========================================================================
     * 传输模式枚举（BSP 特有）
     * ======================================================================== */

    /**
     * @brief 数据传输模式
     * @note 用于配置外设的读写方式
     */
    typedef enum
    {
        BSP_TRANSFER_MODE_BLOCKING = 0, // 阻塞模式（轮询）
        BSP_TRANSFER_MODE_INTERRUPT,    // 中断模式
        BSP_TRANSFER_MODE_DMA           // DMA 模式
    } bsp_transfer_mode_t;

    /* ========================================================================
     * 事件系统（BSP 特有）
     * ======================================================================== */

    /**
     * @brief BSP 事件类型（用于回调通知）
     */
    typedef enum
    {
        BSP_EVENT_TRANSMIT_COMPLETE = 0, // 发送完成
        BSP_EVENT_RECEIVE_COMPLETE,      // 接收完成
        BSP_EVENT_TRANSFER_COMPLETE,     // 传输完成（通用）
        BSP_EVENT_RECEIVE_PENDING,       // 有数据待接收（如 FIFO 非空）
        BSP_EVENT_ABORT_COMPLETE,        // 中止操作完成
        BSP_EVENT_ERROR                  // 发生错误
    } bsp_event_t;

    /**
     * @brief 事件回调函数原型
     * @param event 事件类型
     * @param status 状态码
     * @param transferred_size 已传输的数据量（如字节数或帧数）
     * @param user_context 用户自定义上下文
     * @note 回调函数必须在非阻塞上下文中执行（不应包含延时或信号量获取）
     */
    typedef void (*bsp_event_callback_t)(bsp_event_t event, bsp_status_t status,
                                         size_t transferred_size, void *user_context);

    /* ========================================================================
     * 设备基类
     * ======================================================================== */

    /** @brief BSP 设备基类 */
    typedef struct bsp_device bsp_device_t;

    /**
     * @brief 设备操作虚表
     * @note 派生类可扩展此表，但必须保持 super 为第一个成员。
     */
    typedef struct
    {
        bsp_status_t (*deinit)(bsp_device_t *const me); // 析构函数（虚）
    } bsp_device_ops_t;

    /**
     * @brief 设备基类定义
     */
    struct bsp_device
    {
        const bsp_device_ops_t *vptr; // 虚表指针（只读）
        void *device_handle;          // 平台相关句柄（不透明）
        uint32_t object_magic;        // 魔数（BSP_DEVICE_OBJECT_MAGIC）
        bool is_initialized;          // 初始化完成标志
    };

    /* ========================================================================
     * 公共 API
     * ======================================================================== */

    bsp_status_t bsp_device_init(bsp_device_t *const me, const bsp_device_ops_t *const vptr,
                                 void *const device_handle);
    bsp_status_t bsp_device_deinit(bsp_device_t *const me);
    bool bsp_device_is_initialized(const bsp_device_t *const me);

    /** @brief 校验传输模式是否合法 */
    bool bsp_transfer_mode_is_valid(bsp_transfer_mode_t transfer_mode);

    /** @brief 获取设备句柄（只读），若无效则返回 NULL */
    void *bsp_device_get_handle(const bsp_device_t *const me);

    /* ========================================================================
     * 全局错误寄存器（BSP 特有）
     * ======================================================================== */

    typedef struct
    {
        bsp_status_t code;  // 错误码
        const char *source; // 来源模块名（如 "can"/"spi"/"dr16_init"）
        int detail;         // 模块自定义补充码
        bool is_valid;      // true 表示至少记录过一次错误
    } bsp_error_t;

    void bsp_error_record(bsp_status_t code, const char *source, int detail);
    const bsp_error_t *bsp_error_read(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_COMMON_H */
