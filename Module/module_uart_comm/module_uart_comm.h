/**
 * @file module_uart_comm.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 独立 UART 固定长度通信协议头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 基于 USART 的固定帧长点对点协议，帧头为 0xA5 + 0x5A，
 *       尾部为 CRC8 校验。数据长度通过 MODULE_UART_COMM_DATA_SIZE 宏配置。
 *       帧格式：帧头(2B) + 用户数据(N) + CRC8(1B)
 */
#ifndef MODULE_UART_COMM_H
#define MODULE_UART_COMM_H

#include "bsp_usart.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ======================== 协议常量 ======================== */

/** @brief 用户数据长度，可通过编译选项 -D 覆盖，默认为 8 字节 */
#ifndef MODULE_UART_COMM_DATA_SIZE
#define MODULE_UART_COMM_DATA_SIZE (8U)
#endif

#if (MODULE_UART_COMM_DATA_SIZE < 1U) || (MODULE_UART_COMM_DATA_SIZE > 252U)
#error "MODULE_UART_COMM_DATA_SIZE must be in range 1..252"
#endif

/** @brief 帧头首字节，固定为 0xA5 */
#define MODULE_UART_COMM_HEADER_FIRST (0xA5U)
/** @brief 帧头次字节，固定为 0x5A */
#define MODULE_UART_COMM_HEADER_SECOND (0x5AU)
/** @brief 完整帧大小 = 帧头(2) + 数据(N) + CRC8(1) */
#define MODULE_UART_COMM_FRAME_SIZE (MODULE_UART_COMM_DATA_SIZE + 3U)

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief UART 通信模块状态码
     */
    typedef enum
    {
        MODULE_UART_COMM_STATUS_OK = 0,           // 操作成功
        MODULE_UART_COMM_STATUS_INVALID_ARGUMENT, // 参数非法
        MODULE_UART_COMM_STATUS_NOT_INITIALIZED,  // 对象未初始化
        MODULE_UART_COMM_STATUS_TRANSPORT_ERROR,  // 串口传输错误
        MODULE_UART_COMM_STATUS_BUSY,             // 发送忙（上一帧未发完）
        MODULE_UART_COMM_STATUS_INVALID_FRAME,    // 无效帧（帧头不匹配）
        MODULE_UART_COMM_STATUS_CHECKSUM_ERROR,   // CRC8 校验失败
        MODULE_UART_COMM_STATUS_NO_DATA           // 尚无有效数据
    } module_uart_comm_status_t;

    /* ======================== 数据结构体 ======================== */

    /**
     * @brief 解析后的处理数据（含有效性标记和更新计数）
     */
    typedef struct
    {
        uint8_t data[MODULE_UART_COMM_DATA_SIZE]; // 用户数据负载
        uint32_t update_count;                    // 数据更新计数器
        bool is_valid;                            // 当前数据是否有效
    } module_uart_comm_process_data_t;

    /**
     * @brief UART 通信初始化配置
     */
    typedef struct
    {
        bsp_usart_t *usart;                // USART BSP 基类指针（必须已初始化）
        bsp_transfer_mode_t transmit_mode; // 发送传输模式（阻塞/中断/DMA）
        uint32_t transmit_timeout_ms;      // 发送超时时间（毫秒）
    } module_uart_comm_config_t;

    /**
     * @brief UART 通信设备对象
     */
    typedef struct
    {
        bsp_usart_t *usart;                                // USART BSP 基类指针
        bsp_transfer_mode_t transmit_mode;                 // 发送传输模式
        uint32_t transmit_timeout_ms;                      // 发送超时时间（毫秒）
        uint8_t transmit_frame[MODULE_UART_COMM_FRAME_SIZE]; // 发送帧缓冲区
        uint8_t stream_frame[MODULE_UART_COMM_FRAME_SIZE]; // 流式接收组装缓冲区
        size_t stream_size;                                 // 流缓冲区中已累积字节数
        module_uart_comm_process_data_t received_data;      // 最近一次成功接收的数据
        uint32_t valid_frame_count;                         // 有效帧接收计数
        uint32_t checksum_error_count;                      // CRC 校验错误计数
        bool is_initialized;                                // 是否已初始化
    } module_uart_comm_t;

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化 UART 通信模块
     * @param me 设备对象指针
     * @param config 配置参数
     * @return 执行状态
     */
    module_uart_comm_status_t module_uart_comm_init(
        module_uart_comm_t *me, const module_uart_comm_config_t *config);

    /**
     * @brief 发送一帧数据（自动补充帧头与 CRC8）
     * @param me 设备对象指针
     * @param data 用户数据指针（长度须等于 MODULE_UART_COMM_DATA_SIZE）
     * @param data_size 数据长度
     * @return 执行状态
     */
    module_uart_comm_status_t module_uart_comm_send(
        module_uart_comm_t *me, const uint8_t *data, size_t data_size);

    /**
     * @brief 注入接收字节到流缓冲区并尝试解析帧
     * @param me 设备对象指针
     * @param received_bytes 接收到的原始字节
     * @param received_size 接收字节数
     * @return 执行状态
     * @note 本函数采用状态机方式搜索帧头 0xA5/0x5A，帧头不匹配时自动丢弃并重新同步
     */
    module_uart_comm_status_t module_uart_comm_feed_data(
        module_uart_comm_t *me, const uint8_t *received_bytes, size_t received_size);

    /**
     * @brief 获取最近一次成功接收的帧数据
     * @param me 设备对象指针
     * @param process_data 输出数据指针
     * @return 执行状态（NO_DATA 表示尚无有效数据）
     */
    module_uart_comm_status_t module_uart_comm_get_data(
        const module_uart_comm_t *me, module_uart_comm_process_data_t *process_data);

    /**
     * @brief 计算 CRC8 校验值
     * @param data 数据缓冲区
     * @param data_size 数据长度
     * @return CRC8 校验值（多项式 0x8C，初值 0xFF）
     */
    uint8_t module_uart_comm_crc8(const uint8_t *data, size_t data_size);

#ifdef __cplusplus
}
#endif
#endif /* MODULE_UART_COMM_H */
