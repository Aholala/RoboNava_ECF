/**
 * @file module_nrf24l01_link.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 基于 nRF24L01 固定载荷的 ACE 点对点链路层协议头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供封包、序列号管理和 CRC16 校验的点对点帧协议。
 *       协议帧：HEADER(2B) + message_type(1B) + sequence(1B) + data_size(1B) + payload + CRC16(2B)
 */

#ifndef MODULE_NRF24L01_LINK_H
#define MODULE_NRF24L01_LINK_H

#include "module_nrf24l01.h"

/* ======================== 协议常量 ======================== */

/** @brief 链路地址长度（3 字节） */
#define MODULE_NRF24L01_LINK_ADDRESS_SIZE (3U)
/** @brief 帧头首字节（0xA5） */
#define MODULE_NRF24L01_LINK_HEADER_FIRST (0xA5U)
/** @brief 帧头次字节（0x5A） */
#define MODULE_NRF24L01_LINK_HEADER_SECOND (0x5AU)
/** @brief 协议开销：帧头(2) + 类型(1) + 序号(1) + 数据长(1) + CRC16(2) = 7 字节 */
#define MODULE_NRF24L01_LINK_OVERHEAD_SIZE (7U)
/** @brief 最大用户数据大小 = 射频载荷大小 - 协议开销 */
#define MODULE_NRF24L01_LINK_MAXIMUM_DATA_SIZE                                                 \
    (MODULE_NRF24L01_MAXIMUM_PAYLOAD_SIZE - MODULE_NRF24L01_LINK_OVERHEAD_SIZE)

#ifdef __cplusplus
extern "C"
{
#endif

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief 链路层协议状态码
     */
    typedef enum
    {
        MODULE_NRF24L01_LINK_STATUS_OK = 0,             // 操作成功
        MODULE_NRF24L01_LINK_STATUS_NO_DATA,            // 无数据可读
        MODULE_NRF24L01_LINK_STATUS_BUSY,               // 发送忙
        MODULE_NRF24L01_LINK_STATUS_MAXIMUM_RETRANSMIT, // 达到最大重发次数
        MODULE_NRF24L01_LINK_STATUS_INVALID_ARGUMENT,   // 参数非法
        MODULE_NRF24L01_LINK_STATUS_NOT_INITIALIZED,    // 对象未初始化
        MODULE_NRF24L01_LINK_STATUS_NOT_STARTED,        // 射频未启动
        MODULE_NRF24L01_LINK_STATUS_TRANSPORT_ERROR,    // 传输错误
        MODULE_NRF24L01_LINK_STATUS_INVALID_FRAME,      // 无效帧（帧头不匹配）
        MODULE_NRF24L01_LINK_STATUS_CHECKSUM_ERROR      // CRC16 校验失败
    } module_nrf24l01_link_status_t;

    /* ======================== 数据结构体 ======================== */

    /**
     * @brief 链路层解析后的数据包
     */
    typedef struct
    {
        uint8_t message_type;                                 // 消息类型标识
        uint8_t sequence;                                     // 接收到的序列号
        uint8_t data_size;                                    // 用户数据长度
        uint8_t data[MODULE_NRF24L01_LINK_MAXIMUM_DATA_SIZE]; // 用户数据
    } module_nrf24l01_link_packet_t;

    /**
     * @brief 链路层设备对象
     */
    typedef struct
    {
        module_nrf24l01_t *radio;       // 底层射频设备指针
        uint8_t next_transmit_sequence; // 下一个待发送序列号（自增）
        bool is_initialized;            // 是否已初始化
    } module_nrf24l01_link_t;

    /* ======================== 公共 API ======================== */

    /** @brief 默认链路地址，ASCII {'A', 'C', 'E'}。 */
    extern const uint8_t module_nrf24l01_link_address[MODULE_NRF24L01_LINK_ADDRESS_SIZE];

    /**
     * @brief 初始化链路层
     * @param me 链路层对象
     * @param radio 已初始化的 nRF24L01 设备
     * @return 执行状态
     */
    module_nrf24l01_link_status_t module_nrf24l01_link_init(module_nrf24l01_link_t *me,
                                                                    module_nrf24l01_t *radio);

    /**
     * @brief 发送数据包（封包、计算 CRC、发送）
     * @param me 链路层对象
     * @param message_type 消息类型
     * @param packet_data 用户数据
     * @param data_size 数据大小
     * @return 执行状态
     */
    module_nrf24l01_link_status_t module_nrf24l01_link_send(module_nrf24l01_link_t *me,
                                                                    uint8_t message_type,
                                                                    const uint8_t *packet_data,
                                                                    size_t data_size);

    /**
     * @brief 接收数据包（校验 CRC、解包）
     * @param me 链路层对象
     * @param packet 输出数据包指针
     * @param pipe_index 输出管道索引（可为 NULL）
     * @return 执行状态
     */
    module_nrf24l01_link_status_t
    module_nrf24l01_link_receive(module_nrf24l01_link_t *me,
                                     module_nrf24l01_link_packet_t *packet,
                                     uint8_t *pipe_index);

    /**
     * @brief 获取下一个发送序列号（查询，不递增）
     * @param me 链路层对象
     * @return 当前序列号
     */
    uint8_t module_nrf24l01_link_get_next_sequence(const module_nrf24l01_link_t *me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_NRF24L01_LINK_H */
