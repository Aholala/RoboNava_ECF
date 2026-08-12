/**
 * @file module_usb_comm.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief USB CDC 视觉双向固定帧协议头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 基于 USB VCP（Virtual COM Port）的点对点协议，用于与上位机视觉系统通信。
 *       帧格式：帧头(0xA5 + 0x5A) + 模式(1B) + ID(1B) + 扩展数据 + CRC8(1B)
 *       当前只定义 mode 和 id 字段，扩展数据区通过宏预留。
 */
#ifndef MODULE_USB_COMM_H
#define MODULE_USB_COMM_H

#include "bsp_usb_vcp.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ======================== 协议常量 ======================== */

/** @brief 扩展数据字节数，默认为 0，可通过编译选项 -D 覆盖 */
#ifndef MODULE_USB_COMM_EXTRA_DATA_SIZE
#define MODULE_USB_COMM_EXTRA_DATA_SIZE (0U)
#endif

#if (MODULE_USB_COMM_EXTRA_DATA_SIZE > 250U)
#error "MODULE_USB_COMM_EXTRA_DATA_SIZE must be in range 0..250"
#endif

/** @brief 帧头首字节，固定为 0xA5 */
#define MODULE_USB_COMM_HEADER_FIRST (0xA5U)
/** @brief 帧头次字节，固定为 0x5A */
#define MODULE_USB_COMM_HEADER_SECOND (0x5AU)
/** @brief ID 字段最小值 */
#define MODULE_USB_COMM_MINIMUM_ID (1U)
/** @brief ID 字段最大值 */
#define MODULE_USB_COMM_MAXIMUM_ID (7U)
/** @brief 完整帧大小 = 帧头(2) + 模式(1) + ID(1) + 扩展数据 + CRC8(1) */
#define MODULE_USB_COMM_FRAME_SIZE (MODULE_USB_COMM_EXTRA_DATA_SIZE + 5U)

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief USB 通信模块状态码
     */
    typedef enum
    {
        MODULE_USB_COMM_STATUS_OK = 0,           // 操作成功
        MODULE_USB_COMM_STATUS_INVALID_ARGUMENT, // 参数非法
        MODULE_USB_COMM_STATUS_NOT_INITIALIZED,  // 对象未初始化
        MODULE_USB_COMM_STATUS_TRANSPORT_ERROR,  // USB 传输错误
        MODULE_USB_COMM_STATUS_BUSY,             // 发送忙（上一帧未发完）
        MODULE_USB_COMM_STATUS_INVALID_FRAME,    // 无效帧（帧头或 ID 非法）
        MODULE_USB_COMM_STATUS_CHECKSUM_ERROR,   // CRC8 校验失败
        MODULE_USB_COMM_STATUS_NO_DATA           // 尚无有效数据
    } module_usb_comm_status_t;

    /* ======================== 数据结构体 ======================== */

    /**
     * @brief USB 协议传输数据（收发共用）
     */
    typedef struct
    {
        uint8_t mode;  // 模式字节
        uint8_t id;    // 标识 ID（范围 1~7）
#if MODULE_USB_COMM_EXTRA_DATA_SIZE > 0U
        uint8_t extra_data[MODULE_USB_COMM_EXTRA_DATA_SIZE]; // 扩展数据区（可选）
#endif
    } module_usb_comm_data_t;

    /**
     * @brief 解析后的处理数据（含有效性标记和更新计数）
     */
    typedef struct
    {
        module_usb_comm_data_t data; // 解析出的协议数据
        uint32_t update_count;       // 数据更新计数器
        bool is_valid;               // 当前数据是否有效
    } module_usb_comm_process_data_t;

    /**
     * @brief USB 通信初始化配置
     */
    typedef struct
    {
        bsp_usb_vcp_t *usb_vcp;        // USB VCP BSP 基类指针（必须已初始化）
        uint32_t transmit_timeout_ms;  // 发送超时时间（毫秒）
    } module_usb_comm_config_t;

    /**
     * @brief USB 通信设备对象
     */
    typedef struct
    {
        bsp_usb_vcp_t *usb_vcp;                             // USB VCP BSP 基类指针
        uint32_t transmit_timeout_ms;                       // 发送超时时间（毫秒）
        uint8_t transmit_frame[MODULE_USB_COMM_FRAME_SIZE]; // 发送帧缓冲区
        uint8_t stream_frame[MODULE_USB_COMM_FRAME_SIZE];   // 流式接收组装缓冲区
        size_t stream_size;                                  // 流缓冲区中已累积字节数
        module_usb_comm_process_data_t received_data;        // 最近一次成功接收的数据
        uint32_t valid_frame_count;                          // 有效帧接收计数
        uint32_t invalid_frame_count;                        // 无效帧计数（ID 非法等）
        uint32_t checksum_error_count;                       // CRC 校验错误计数
        bool is_initialized;                                 // 是否已初始化
    } module_usb_comm_t;

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化 USB 通信模块
     * @param me 设备对象指针
     * @param config 配置参数
     * @return 执行状态
     */
    module_usb_comm_status_t module_usb_comm_init(
        module_usb_comm_t *me, const module_usb_comm_config_t *config);

    /**
     * @brief 发送一帧数据（自动补充帧头与 CRC8）
     * @param me 设备对象指针
     * @param data 协议数据指针（ID 须在 1~7 范围内）
     * @return 执行状态
     * @note 非阻塞发送：若 VCP 忙则返回 BUSY，调用者需稍后重试
     */
    module_usb_comm_status_t module_usb_comm_send(
        module_usb_comm_t *me, const module_usb_comm_data_t *data);

    /**
     * @brief 注入接收字节到流缓冲区并尝试解析帧
     * @param me 设备对象指针
     * @param received_bytes 接收到的原始字节
     * @param received_size 接收字节数
     * @return 执行状态
     * @note 使用状态机搜索帧头 0xA5/0x5A，并校验 ID 范围（1~7）和 CRC8
     */
    module_usb_comm_status_t module_usb_comm_feed_data(
        module_usb_comm_t *me, const uint8_t *received_bytes, size_t received_size);

    /**
     * @brief 获取最近一次成功接收的帧数据
     * @param me 设备对象指针
     * @param process_data 输出数据指针
     * @return 执行状态（NO_DATA 表示尚无有效数据）
     */
    module_usb_comm_status_t module_usb_comm_get_data(
        const module_usb_comm_t *me, module_usb_comm_process_data_t *process_data);

    /**
     * @brief 计算 CRC8 校验值
     * @param data 数据缓冲区
     * @param data_size 数据长度
     * @return CRC8 校验值（多项式 0x8C，初值 0xFF）
     */
    uint8_t module_usb_comm_crc8(const uint8_t *data, size_t data_size);

#ifdef __cplusplus
}
#endif
#endif /* MODULE_USB_COMM_H */
