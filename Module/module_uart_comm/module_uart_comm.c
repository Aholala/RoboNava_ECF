/**
 * @file module_uart_comm.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 独立 UART 固定长度通信协议实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 基于 USART 的固定帧长点对点协议实现。
 *       接收端通过状态机搜索帧头 0xA5/0x5A 实现字节流帧同步，
 *       发送端自动封装帧头和 CRC8 尾校验。
 */

#include "module_uart_comm.h"
#include "alg_crc.h"

#include <string.h>

/* ======================== 内部常量 ======================== */

/** @brief 用户数据在帧中的起始字节偏移（跳过 2 字节帧头） */
#define MODULE_UART_COMM_DATA_INDEX (2U)
/** @brief CRC8 校验在帧中的字节偏移（帧末最后一个字节） */
#define MODULE_UART_COMM_CRC_INDEX (MODULE_UART_COMM_FRAME_SIZE - 1U)

/* ======================== 内部函数 ======================== */

/**
 * @brief 饱和递增计数器（防溢出，到达 UINT32_MAX 后不再增加）
 * @param value 计数器指针
 */
static void module_uart_comm_increment_saturated(uint32_t *value)
{
    if (*value != UINT32_MAX)
    {
        ++(*value);
    }
}

/**
 * @brief 计算 CRC8 校验值
 * @param data 数据缓冲区
 * @param data_size 数据长度
 * @return CRC8 校验值
 * @note 使用 alg_crc 库的 CRC-8/0x8C/0xFF 配置
 */
uint8_t module_uart_comm_crc8(const uint8_t *data, size_t data_size)
{
    uint32_t result = 0U;
    return alg_crc_calculate(&alg_crc8_0x8c_ff_config, data, data_size, &result)
               ? (uint8_t)result
               : 0U;
}

/* ======================== 公共 API ======================== */

/**
 * @brief 初始化 UART 通信模块
 * @param me 设备对象指针
 * @param config 配置参数
 * @return 执行状态
 */
module_uart_comm_status_t module_uart_comm_init(
    module_uart_comm_t *me, const module_uart_comm_config_t *config)
{
    if ((me == NULL) || (config == NULL) || (config->usart == NULL) ||
        !config->usart->is_initialized ||
        !bsp_transfer_mode_is_valid(config->transmit_mode))
    {
        return MODULE_UART_COMM_STATUS_INVALID_ARGUMENT;
    }
    (void)memset(me, 0, sizeof(*me));
    me->usart = config->usart;
    me->transmit_mode = config->transmit_mode;
    me->transmit_timeout_ms = config->transmit_timeout_ms;
    me->is_initialized = true;
    return MODULE_UART_COMM_STATUS_OK;
}

/**
 * @brief 发送一帧数据
 * @param me 设备对象指针
 * @param data 用户数据指针
 * @param data_size 数据长度（须等于 MODULE_UART_COMM_DATA_SIZE）
 * @return 执行状态
 * @note 内部构建完整帧：帧头(0xA5, 0x5A) + 用户数据 + CRC8
 */
module_uart_comm_status_t module_uart_comm_send(
    module_uart_comm_t *me, const uint8_t *data, size_t data_size)
{
    bsp_status_t status;
    if ((me == NULL) || (data == NULL) || (data_size != MODULE_UART_COMM_DATA_SIZE))
    {
        return MODULE_UART_COMM_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_UART_COMM_STATUS_NOT_INITIALIZED;
    }
    // 构建帧：帧头 + 数据 + CRC8
    me->transmit_frame[0] = MODULE_UART_COMM_HEADER_FIRST;
    me->transmit_frame[1] = MODULE_UART_COMM_HEADER_SECOND;
    (void)memcpy(&me->transmit_frame[MODULE_UART_COMM_DATA_INDEX], data, data_size);
    me->transmit_frame[MODULE_UART_COMM_CRC_INDEX] =
        module_uart_comm_crc8(me->transmit_frame, MODULE_UART_COMM_CRC_INDEX);
    status = bsp_usart_transmit(me->usart, me->transmit_frame,
                                sizeof(me->transmit_frame), me->transmit_mode,
                                me->transmit_timeout_ms);
    if (status == BSP_STATUS_BUSY)
    {
        return MODULE_UART_COMM_STATUS_BUSY;
    }
    return (status == BSP_STATUS_OK) ? MODULE_UART_COMM_STATUS_OK
                                     : MODULE_UART_COMM_STATUS_TRANSPORT_ERROR;
}

/**
 * @brief 注入接收字节到流缓冲区并尝试解析帧
 * @param me 设备对象指针
 * @param received_bytes 接收到的原始字节
 * @param received_size 接收字节数
 * @return 执行状态
 * @note 使用状态机搜索帧头：先匹配 0xA5，再匹配 0x5A，然后累积到帧长并校验 CRC8
 */
module_uart_comm_status_t module_uart_comm_feed_data(
    module_uart_comm_t *me, const uint8_t *received_bytes, size_t received_size)
{
    bool received_valid = false;
    bool checksum_error = false;
    size_t index;
    if ((me == NULL) || ((received_bytes == NULL) && (received_size > 0U)))
    {
        return MODULE_UART_COMM_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_UART_COMM_STATUS_NOT_INITIALIZED;
    }
    for (index = 0U; index < received_size; ++index)
    {
        const uint8_t byte = received_bytes[index];
        // 状态 0：等待帧头首字节 0xA5
        if (me->stream_size == 0U)
        {
            if (byte == MODULE_UART_COMM_HEADER_FIRST)
            {
                me->stream_frame[0] = byte;
                me->stream_size = 1U;
            }
            continue;
        }
        // 状态 1：等待帧头次字节 0x5A
        if (me->stream_size == 1U)
        {
            if (byte == MODULE_UART_COMM_HEADER_SECOND)
            {
                me->stream_frame[1] = byte;
                me->stream_size = 2U;
            }
            // 若接收到非 0xA5 也非 0x5A 的字节，重置状态机从头搜索
            else if (byte != MODULE_UART_COMM_HEADER_FIRST)
            {
                me->stream_size = 0U;
            }
            continue;
        }
        // 状态 2：累积帧数据字节
        me->stream_frame[me->stream_size++] = byte;
        // 达到完整帧长度后，校验 CRC8
        if (me->stream_size == MODULE_UART_COMM_FRAME_SIZE)
        {
            // CRC8 校验：对帧中除 CRC 字节外的所有数据进行计算
            if (me->stream_frame[MODULE_UART_COMM_CRC_INDEX] ==
                module_uart_comm_crc8(me->stream_frame, MODULE_UART_COMM_CRC_INDEX))
            {
                (void)memcpy(me->received_data.data,
                             &me->stream_frame[MODULE_UART_COMM_DATA_INDEX],
                             MODULE_UART_COMM_DATA_SIZE);
                module_uart_comm_increment_saturated(&me->received_data.update_count);
                module_uart_comm_increment_saturated(&me->valid_frame_count);
                me->received_data.is_valid = true;
                received_valid = true;
            }
            else
            {
                module_uart_comm_increment_saturated(&me->checksum_error_count);
                checksum_error = true;
            }
            me->stream_size = 0U;
        }
    }
    if (received_valid)
    {
        return MODULE_UART_COMM_STATUS_OK;
    }
    return checksum_error ? MODULE_UART_COMM_STATUS_CHECKSUM_ERROR
                          : MODULE_UART_COMM_STATUS_OK;
}

/**
 * @brief 获取最近一次成功接收的帧数据
 * @param me 设备对象指针
 * @param process_data 输出数据指针
 * @return 执行状态（NO_DATA 表示尚无有效数据）
 */
module_uart_comm_status_t module_uart_comm_get_data(
    const module_uart_comm_t *me, module_uart_comm_process_data_t *process_data)
{
    if ((me == NULL) || (process_data == NULL))
    {
        return MODULE_UART_COMM_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_UART_COMM_STATUS_NOT_INITIALIZED;
    }
    if (!me->received_data.is_valid)
    {
        return MODULE_UART_COMM_STATUS_NO_DATA;
    }
    *process_data = me->received_data;
    return MODULE_UART_COMM_STATUS_OK;
}
