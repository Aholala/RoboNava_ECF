/**
 * @file module_nrf24l01_link.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief ACE 点对点链路层协议的封包、序列号和 CRC 校验实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 在 nRF24L01 固定载荷之上构建点对点帧协议，
 *       提供消息类型标识、递增序列号和 CRC16 完整性校验。
 */

#include "module_nrf24l01_link.h"

#include "alg_crc.h"

#include <stddef.h>
#include <string.h>

/** @brief 默认链路地址，ASCII 码 {'A', 'C', 'E'} */
const uint8_t module_nrf24l01_link_address[MODULE_NRF24L01_LINK_ADDRESS_SIZE] = {'A', 'C',
                                                                                         'E'};

/**
 * @brief 将 nRF24L01 驱动层状态码映射为链路层状态码
 * @param status 驱动层状态码
 * @return 链路层状态码
 */
static module_nrf24l01_link_status_t
module_nrf24l01_link_map_radio_status(module_nrf24l01_status_t status)
{
    switch (status)
    {
    case MODULE_NRF24L01_STATUS_OK:
        return MODULE_NRF24L01_LINK_STATUS_OK;
    case MODULE_NRF24L01_STATUS_NO_DATA:
        return MODULE_NRF24L01_LINK_STATUS_NO_DATA;
    case MODULE_NRF24L01_STATUS_BUSY:
        return MODULE_NRF24L01_LINK_STATUS_BUSY;
    case MODULE_NRF24L01_STATUS_MAXIMUM_RETRANSMIT:
        return MODULE_NRF24L01_LINK_STATUS_MAXIMUM_RETRANSMIT;
    case MODULE_NRF24L01_STATUS_INVALID_ARGUMENT:
        return MODULE_NRF24L01_LINK_STATUS_INVALID_ARGUMENT;
    case MODULE_NRF24L01_STATUS_NOT_INITIALIZED:
        return MODULE_NRF24L01_LINK_STATUS_NOT_INITIALIZED;
    case MODULE_NRF24L01_STATUS_NOT_STARTED:
        return MODULE_NRF24L01_LINK_STATUS_NOT_STARTED;
    case MODULE_NRF24L01_STATUS_TRANSPORT_ERROR:
    case MODULE_NRF24L01_STATUS_DEVICE_NOT_FOUND:
    default:
        return MODULE_NRF24L01_LINK_STATUS_TRANSPORT_ERROR;
    }
}

/**
 * @brief 计算 CCITT-FALSE CRC16 校验值
 * @param data 数据缓冲区
 * @param data_size 数据大小
 * @return CRC16 校验值
 */
static uint16_t module_nrf24l01_link_crc16(const uint8_t *data, size_t data_size)
{
    uint32_t result = 0U;

    return alg_crc_calculate(&alg_crc16_ccitt_false_config, data, data_size, &result)
               ? (uint16_t)result
               : 0U;
}

/**
 * @brief 初始化链路层
 * @param me 链路层对象
 * @param radio 已初始化的 nRF24L01 设备
 * @return 执行状态
 */
module_nrf24l01_link_status_t module_nrf24l01_link_init(module_nrf24l01_link_t *me,
                                                                module_nrf24l01_t *radio)
{
    if ((me == NULL) || (radio == NULL) || !radio->is_initialized ||
        (radio->payload_size < MODULE_NRF24L01_LINK_OVERHEAD_SIZE))
    {
        return MODULE_NRF24L01_LINK_STATUS_INVALID_ARGUMENT;
    }

    *me = (module_nrf24l01_link_t){
        .radio = radio,
        .next_transmit_sequence = 0U,
        .is_initialized = true,
    };
    return MODULE_NRF24L01_LINK_STATUS_OK;
}

/**
 * @brief 发送数据包（构建协议帧、计算 CRC、通过射频发送）
 * @param me 链路层对象
 * @param message_type 消息类型
 * @param packet_data 用户数据
 * @param data_size 数据大小
 * @return 执行状态
 */
module_nrf24l01_link_status_t module_nrf24l01_link_send(module_nrf24l01_link_t *me,
                                                                uint8_t message_type,
                                                                const uint8_t *packet_data,
                                                                size_t data_size)
{
    uint8_t radio_payload[MODULE_NRF24L01_MAXIMUM_PAYLOAD_SIZE] = {0U};
    uint16_t checksum;
    module_nrf24l01_status_t radio_status;

    if ((me == NULL) || ((packet_data == NULL) && (data_size != 0U)))
    {
        return MODULE_NRF24L01_LINK_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized || (me->radio == NULL))
    {
        return MODULE_NRF24L01_LINK_STATUS_NOT_INITIALIZED;
    }
    if ((me->radio->payload_size < MODULE_NRF24L01_LINK_OVERHEAD_SIZE) ||
        (data_size > (size_t)(me->radio->payload_size - MODULE_NRF24L01_LINK_OVERHEAD_SIZE)))
    {
        return MODULE_NRF24L01_LINK_STATUS_INVALID_ARGUMENT;
    }

    // 构建协议帧（共 7 + data_size 字节）：
    radio_payload[0] = MODULE_NRF24L01_LINK_HEADER_FIRST;   // 帧头首字节 0xA5
    radio_payload[1] = MODULE_NRF24L01_LINK_HEADER_SECOND;  // 帧头次字节 0x5A
    radio_payload[2] = message_type;                         // 消息类型（1 字节）
    radio_payload[3] = me->next_transmit_sequence;           // 序列号（1 字节）
    radio_payload[4] = (uint8_t)data_size;                   // 数据长度（1 字节）
    if (data_size != 0U)
    {
        memcpy(&radio_payload[5], packet_data, data_size);   // 用户数据（从第 6 字节开始）
    }
    // CRC16 对帧头到数据末尾的全部内容计算，结果以小端序追加
    checksum = module_nrf24l01_link_crc16(radio_payload, 5U + data_size);
    radio_payload[5U + data_size] = (uint8_t)checksum;          // CRC16 低字节
    radio_payload[6U + data_size] = (uint8_t)(checksum >> 8U);  // CRC16 高字节

    radio_status = module_nrf24l01_transmit(me->radio, radio_payload, me->radio->payload_size);
    if (radio_status == MODULE_NRF24L01_STATUS_OK)
    {
        ++me->next_transmit_sequence;
    }
    return module_nrf24l01_link_map_radio_status(radio_status);
}

/**
 * @brief 接收数据包（射频接收 + 校验 CRC + 解包）
 * @param me 链路层对象
 * @param packet 输出数据包指针
 * @param pipe_index 输出管道索引（可为 NULL）
 * @return 执行状态
 */
module_nrf24l01_link_status_t
module_nrf24l01_link_receive(module_nrf24l01_link_t *me,
                                 module_nrf24l01_link_packet_t *packet, uint8_t *pipe_index)
{
    uint8_t radio_payload[MODULE_NRF24L01_MAXIMUM_PAYLOAD_SIZE];
    size_t checksum_offset;
    uint16_t received_checksum;
    module_nrf24l01_status_t radio_status;

    if ((me == NULL) || (packet == NULL))
    {
        return MODULE_NRF24L01_LINK_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized || (me->radio == NULL))
    {
        return MODULE_NRF24L01_LINK_STATUS_NOT_INITIALIZED;
    }
    if (me->radio->payload_size < MODULE_NRF24L01_LINK_OVERHEAD_SIZE)
    {
        return MODULE_NRF24L01_LINK_STATUS_INVALID_ARGUMENT;
    }

    radio_status =
        module_nrf24l01_receive(me->radio, radio_payload, sizeof(radio_payload), pipe_index);
    if (radio_status != MODULE_NRF24L01_STATUS_OK)
    {
        return module_nrf24l01_link_map_radio_status(radio_status);
    }
    // 校验帧头 0xA5 + 0x5A 和声明数据长度是否合法
    if ((radio_payload[0] != MODULE_NRF24L01_LINK_HEADER_FIRST) ||
        (radio_payload[1] != MODULE_NRF24L01_LINK_HEADER_SECOND) ||
        (radio_payload[4] >
         (uint8_t)(me->radio->payload_size - MODULE_NRF24L01_LINK_OVERHEAD_SIZE)))
    {
        return MODULE_NRF24L01_LINK_STATUS_INVALID_FRAME;
    }

    // CRC16 计算范围 = 帧头(5B) + 数据(data_size)，CRC 值存储在 data 之后的小端 2 字节
    checksum_offset = 5U + radio_payload[4];
    received_checksum = (uint16_t)radio_payload[checksum_offset] |
                        (uint16_t)((uint16_t)radio_payload[checksum_offset + 1U] << 8U);
    if (received_checksum != module_nrf24l01_link_crc16(radio_payload, checksum_offset))
    {
        return MODULE_NRF24L01_LINK_STATUS_CHECKSUM_ERROR;
    }

    // 解包到输出结构体
    packet->message_type = radio_payload[2];
    packet->sequence = radio_payload[3];
    packet->data_size = radio_payload[4];
    memset(packet->data, 0, sizeof(packet->data));
    if (packet->data_size != 0U)
    {
        memcpy(packet->data, &radio_payload[5], packet->data_size);
    }
    return MODULE_NRF24L01_LINK_STATUS_OK;
}

/**
 * @brief 获取下一个发送序列号（查询，不递增）
 * @param me 链路层对象
 * @return 当前序列号
 */
uint8_t module_nrf24l01_link_get_next_sequence(const module_nrf24l01_link_t *me)
{
    return ((me != NULL) && me->is_initialized) ? me->next_transmit_sequence : 0U;
}
