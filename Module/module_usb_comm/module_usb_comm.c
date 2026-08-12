/**
 * @file module_usb_comm.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief USB CDC 视觉双向固定帧协议实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 基于 USB VCP 的点对点协议实现，用于与上位机视觉系统通信。
 *       接收端通过状态机搜索帧头 0xA5/0x5A 实现字节流帧同步，
 *       校验 ID 范围（1~7）并通过 CRC8 验证完整性。
 */

#include "module_usb_comm.h"
#include "alg_crc.h"

#include <string.h>

/* ======================== 内部常量 ======================== */

/** @brief 模式字段在帧中的字节偏移（跳过 2 字节帧头） */
#define MODULE_USB_COMM_MODE_INDEX (2U)
/** @brief ID 字段在帧中的字节偏移 */
#define MODULE_USB_COMM_ID_INDEX (3U)
/** @brief 扩展数据在帧中的起始字节偏移 */
#define MODULE_USB_COMM_EXTRA_DATA_INDEX (4U)
/** @brief CRC8 校验在帧中的字节偏移（帧末最后一个字节） */
#define MODULE_USB_COMM_CRC_INDEX (MODULE_USB_COMM_FRAME_SIZE - 1U)

/* ======================== 内部函数 ======================== */

/**
 * @brief 饱和递增计数器（防溢出，到达 UINT32_MAX 后不再增加）
 * @param value 计数器指针
 */
static void module_usb_comm_increment_saturated(uint32_t *value)
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
uint8_t module_usb_comm_crc8(const uint8_t *data, size_t data_size)
{
    uint32_t result = 0U;
    return alg_crc_calculate(&alg_crc8_0x8c_ff_config, data, data_size, &result)
               ? (uint8_t)result
               : 0U;
}

/**
 * @brief 将协议数据结构编码为帧
 * @param frame 输出帧缓冲区
 * @param data 协议数据
 * @note 构建帧格式：HEADER + mode + id + extra_data + CRC8
 */
static void module_usb_comm_encode(uint8_t *frame, const module_usb_comm_data_t *data)
{
    frame[0] = MODULE_USB_COMM_HEADER_FIRST;
    frame[1] = MODULE_USB_COMM_HEADER_SECOND;
    frame[MODULE_USB_COMM_MODE_INDEX] = data->mode;
    frame[MODULE_USB_COMM_ID_INDEX] = data->id;
#if MODULE_USB_COMM_EXTRA_DATA_SIZE > 0U
    (void)memcpy(&frame[MODULE_USB_COMM_EXTRA_DATA_INDEX], data->extra_data,
                 MODULE_USB_COMM_EXTRA_DATA_SIZE);
#endif
    // 对帧中除 CRC 字节外的所有数据计算 CRC8
    frame[MODULE_USB_COMM_CRC_INDEX] = module_usb_comm_crc8(frame, MODULE_USB_COMM_CRC_INDEX);
}

/**
 * @brief 从帧中解码协议数据结构
 * @param frame 帧缓冲区
 * @param data 输出协议数据
 */
static void module_usb_comm_decode(const uint8_t *frame, module_usb_comm_data_t *data)
{
    data->mode = frame[MODULE_USB_COMM_MODE_INDEX];
    data->id = frame[MODULE_USB_COMM_ID_INDEX];
#if MODULE_USB_COMM_EXTRA_DATA_SIZE > 0U
    (void)memcpy(data->extra_data, &frame[MODULE_USB_COMM_EXTRA_DATA_INDEX],
                 MODULE_USB_COMM_EXTRA_DATA_SIZE);
#endif
}

/* ======================== 公共 API ======================== */

/**
 * @brief 初始化 USB 通信模块
 * @param me 设备对象指针
 * @param config 配置参数
 * @return 执行状态
 */
module_usb_comm_status_t module_usb_comm_init(
    module_usb_comm_t *me, const module_usb_comm_config_t *config)
{
    if ((me == NULL) || (config == NULL) || (config->usb_vcp == NULL) ||
        !config->usb_vcp->is_initialized)
    {
        return MODULE_USB_COMM_STATUS_INVALID_ARGUMENT;
    }
    (void)memset(me, 0, sizeof(*me));
    me->usb_vcp = config->usb_vcp;
    me->transmit_timeout_ms = config->transmit_timeout_ms;
    me->is_initialized = true;
    return MODULE_USB_COMM_STATUS_OK;
}

/**
 * @brief 发送一帧数据
 * @param me 设备对象指针
 * @param data 协议数据指针（ID 须在 1~7 范围内）
 * @return 执行状态
 * @note 非阻塞发送：先检查 VCP 是否忙，若忙则返回 BUSY
 */
module_usb_comm_status_t module_usb_comm_send(
    module_usb_comm_t *me, const module_usb_comm_data_t *data)
{
    bool is_busy;
    if ((me == NULL) || (data == NULL) ||
        (data->id < MODULE_USB_COMM_MINIMUM_ID) ||
        (data->id > MODULE_USB_COMM_MAXIMUM_ID))
    {
        return MODULE_USB_COMM_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_USB_COMM_STATUS_NOT_INITIALIZED;
    }
    // 检查 USB VCP 是否正在发送中
    if (bsp_usb_vcp_get_busy(me->usb_vcp, &is_busy) != BSP_STATUS_OK)
    {
        return MODULE_USB_COMM_STATUS_TRANSPORT_ERROR;
    }
    if (is_busy)
    {
        return MODULE_USB_COMM_STATUS_BUSY;
    }
    // 编码并发送帧
    module_usb_comm_encode(me->transmit_frame, data);
    return (bsp_usb_vcp_transmit(me->usb_vcp, me->transmit_frame,
                                 sizeof(me->transmit_frame),
                                 me->transmit_timeout_ms) == BSP_STATUS_OK)
               ? MODULE_USB_COMM_STATUS_OK
               : MODULE_USB_COMM_STATUS_TRANSPORT_ERROR;
}

/**
 * @brief 注入接收字节到流缓冲区并尝试解析帧
 * @param me 设备对象指针
 * @param received_bytes 接收到的原始字节
 * @param received_size 接收字节数
 * @return 执行状态
 * @note 使用三级状态机：搜索 0xA5 -> 匹配 0x5A -> 累积到帧长
 *       校验 ID 范围（1~7）和 CRC8，同时统计三类错误
 */
module_usb_comm_status_t module_usb_comm_feed_data(
    module_usb_comm_t *me, const uint8_t *received_bytes, size_t received_size)
{
    bool received_valid = false;
    bool checksum_error = false;
    bool invalid_frame = false;
    size_t index;
    if ((me == NULL) || ((received_bytes == NULL) && (received_size > 0U)))
    {
        return MODULE_USB_COMM_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_USB_COMM_STATUS_NOT_INITIALIZED;
    }
    for (index = 0U; index < received_size; ++index)
    {
        const uint8_t byte = received_bytes[index];
        // 状态 0：等待帧头首字节 0xA5
        if (me->stream_size == 0U)
        {
            if (byte == MODULE_USB_COMM_HEADER_FIRST)
            {
                me->stream_frame[0] = byte;
                me->stream_size = 1U;
            }
            continue;
        }
        // 状态 1：等待帧头次字节 0x5A（若收到非 0xA5 的字节，重置状态机）
        if (me->stream_size == 1U)
        {
            if (byte == MODULE_USB_COMM_HEADER_SECOND)
            {
                me->stream_frame[1] = byte;
                me->stream_size = 2U;
            }
            else if (byte != MODULE_USB_COMM_HEADER_FIRST)
            {
                me->stream_size = 0U;
            }
            continue;
        }
        // 状态 2：累积帧数据字节
        me->stream_frame[me->stream_size++] = byte;
        // 达到完整帧长度后，统一校验
        if (me->stream_size == MODULE_USB_COMM_FRAME_SIZE)
        {
            // 校验 ID 范围：必须在 1~7 之间
            const bool id_is_valid =
                (me->stream_frame[MODULE_USB_COMM_ID_INDEX] >= MODULE_USB_COMM_MINIMUM_ID) &&
                (me->stream_frame[MODULE_USB_COMM_ID_INDEX] <= MODULE_USB_COMM_MAXIMUM_ID);
            // CRC8 校验：对帧中除 CRC 字节外的所有数据进行计算
            const bool crc_is_valid =
                me->stream_frame[MODULE_USB_COMM_CRC_INDEX] ==
                module_usb_comm_crc8(me->stream_frame, MODULE_USB_COMM_CRC_INDEX);
            if (id_is_valid && crc_is_valid)
            {
                module_usb_comm_decode(me->stream_frame, &me->received_data.data);
                module_usb_comm_increment_saturated(&me->received_data.update_count);
                module_usb_comm_increment_saturated(&me->valid_frame_count);
                me->received_data.is_valid = true;
                received_valid = true;
            }
            else if (!crc_is_valid)
            {
                module_usb_comm_increment_saturated(&me->checksum_error_count);
                checksum_error = true;
            }
            else
            {
                module_usb_comm_increment_saturated(&me->invalid_frame_count);
                invalid_frame = true;
            }
            me->stream_size = 0U;
        }
    }
    if (received_valid)
    {
        return MODULE_USB_COMM_STATUS_OK;
    }
    if (checksum_error)
    {
        return MODULE_USB_COMM_STATUS_CHECKSUM_ERROR;
    }
    return invalid_frame ? MODULE_USB_COMM_STATUS_INVALID_FRAME
                         : MODULE_USB_COMM_STATUS_OK;
}

/**
 * @brief 获取最近一次成功接收的帧数据
 * @param me 设备对象指针
 * @param process_data 输出数据指针
 * @return 执行状态（NO_DATA 表示尚无有效数据）
 */
module_usb_comm_status_t module_usb_comm_get_data(
    const module_usb_comm_t *me, module_usb_comm_process_data_t *process_data)
{
    if ((me == NULL) || (process_data == NULL))
    {
        return MODULE_USB_COMM_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_USB_COMM_STATUS_NOT_INITIALIZED;
    }
    if (!me->received_data.is_valid)
    {
        return MODULE_USB_COMM_STATUS_NO_DATA;
    }
    *process_data = me->received_data;
    return MODULE_USB_COMM_STATUS_OK;
}
