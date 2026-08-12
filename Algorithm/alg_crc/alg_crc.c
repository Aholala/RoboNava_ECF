/**
 * @file alg_crc.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 通用 CRC 算法实现（逐位计算，8/16/32 位宽，MSB/LSB 双位序）
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 采用逐位计算实现，不依赖查表，代码尺寸小，适合嵌入式环境。
 *       支持 MSB-first（高 bit 先处理）和 LSB-first（低 bit 先处理）两种位序。
 *       预置三种 CRC 配置实例供直接使用。
 */

#include "alg_crc.h"

/* ======================== 预置 CRC 配置实例 ======================== */

/** @brief CRC-8/0x8C, init=0xFF, LSB-first */
const alg_crc_config_t alg_crc8_0x8c_ff_config = {
    .width = 8U,
    .polynomial = 0x8CU,
    .initial_value = 0xFFU,
    .output_xor = 0U,
    .bit_order = ALG_CRC_BIT_ORDER_LSB_FIRST,
};

/** @brief CRC-16/CCITT-FALSE, poly=0x1021, init=0xFFFF, MSB-first */
const alg_crc_config_t alg_crc16_ccitt_false_config = {
    .width = 16U,
    .polynomial = 0x1021U,
    .initial_value = 0xFFFFU,
    .output_xor = 0U,
    .bit_order = ALG_CRC_BIT_ORDER_MSB_FIRST,
};

/** @brief CRC-16/0x8408（反射 0x1021）, init=0xFFFF, LSB-first */
const alg_crc_config_t alg_crc16_0x8408_ff_config = {
    .width = 16U,
    .polynomial = 0x8408U,
    .initial_value = 0xFFFFU,
    .output_xor = 0U,
    .bit_order = ALG_CRC_BIT_ORDER_LSB_FIRST,
};

/* ======================== 公共 API ======================== */

/**
 * @brief 使用给定参数统一计算 CRC
 * @param config CRC 配置参数
 * @param data 待计算的数据缓冲区（可为 NULL 当 data_size=0）
 * @param data_size 数据长度（字节）
 * @param result 输出 CRC 计算结果
 * @return true=计算成功，false=参数无效
 * @note 算法流程：
 *       1. 校验参数合法性（指针、位宽、位序）
 *       2. 构造位掩码并初始化寄存器
 *       3. 逐字节处理：
 *          - LSB-first：字节异或到寄存器低位，逐位右移，若最低位为 1 则异或多项式
 *          - MSB-first：字节左移 (width-8) 位后异或到寄存器高位，逐位左移，若最高位为 1 则异或多项式
 *       4. 输出 XOR 掩码后返回结果
 */
bool alg_crc_calculate(const alg_crc_config_t *config,
                       const uint8_t *data,
                       size_t data_size,
                       uint32_t *result)
{
    uint32_t crc;
    uint32_t mask;
    size_t byte_index;

    // ---- 参数校验 ----
    if ((config == NULL) || (result == NULL) || ((data == NULL) && (data_size > 0U)) ||
        ((config->width != 8U) && (config->width != 16U) && (config->width != 32U)) ||
        (config->bit_order > ALG_CRC_BIT_ORDER_LSB_FIRST))
    {
        return false;
    }

    // 构造位掩码：32 位用全 1，其他位宽用 (1 << width) - 1
    mask = (config->width == 32U) ? UINT32_MAX
                                  : ((1UL << config->width) - 1UL);
    // 初始化 CRC 寄存器
    crc = config->initial_value & mask;

    for (byte_index = 0U; byte_index < data_size; ++byte_index)
    {
        uint8_t bit_index;

        if (config->bit_order == ALG_CRC_BIT_ORDER_LSB_FIRST)
        {
            // LSB-first：字节直接异或到寄存器低 8 位
            crc ^= data[byte_index];
            for (bit_index = 0U; bit_index < 8U; ++bit_index)
            {
                // 判断最低位：为 1 则右移后异或多项式，否则仅右移
                crc = ((crc & 1U) != 0U) ? ((crc >> 1U) ^ config->polynomial)
                                         : (crc >> 1U);
            }
        }
        else
        {
            // MSB-first：字节左移到寄存器高位
            const uint32_t top_bit = 1UL << (config->width - 1U);
            crc ^= (uint32_t)data[byte_index] << (config->width - 8U);
            for (bit_index = 0U; bit_index < 8U; ++bit_index)
            {
                // 判断最高位：为 1 则左移后异或多项式，否则仅左移
                crc = ((crc & top_bit) != 0U) ? ((crc << 1U) ^ config->polynomial)
                                              : (crc << 1U);
            }
        }
        // 每字节处理后截断掩码
        crc &= mask;
    }

    // 输出 XOR 后返回
    *result = (crc ^ config->output_xor) & mask;
    return true;
}
