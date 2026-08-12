/**
 * @file alg_crc.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 与硬件无关的通用 CRC 算法（8/16/32 位，支持 MSB/LSB 两种位序）
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供单一接口 alg_crc_calculate，通过 alg_crc_config_t 参数化所有 CRC 变体。
 *       预置三种常用配置：CRC-8/0x8C、CRC-16/CCITT-FALSE、CRC-16/0x8408。
 *       LSB-first 模式下 polynomial 应填写反射后的值（如 0x8408 对应 0x1021 的反射）。
 */
#ifndef ALG_CRC_H
#define ALG_CRC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ======================== 枚举定义 ======================== */

    /**
     * @brief CRC 位序枚举
     */
    typedef enum
    {
        ALG_CRC_BIT_ORDER_MSB_FIRST = 0, /**< 高位在前（先处理 MSB） */
        ALG_CRC_BIT_ORDER_LSB_FIRST      /**< 低位在前（先处理 LSB） */
    } alg_crc_bit_order_t;

/* ======================== 结构体定义 ======================== */

    /**
     * @brief CRC 参数配置
     * @note LSB-first 时 polynomial 应填写反射后的多项式，例如 0x1021 反射为 0x8408
     */
    typedef struct
    {
        uint8_t width;                    /**< CRC 位宽，仅支持 8、16 或 32 */
        uint32_t polynomial;              /**< CRC 生成多项式 */
        uint32_t initial_value;           /**< 初始寄存器值 */
        uint32_t output_xor;              /**< 输出异或掩码 */
        alg_crc_bit_order_t bit_order;    /**< 位处理顺序 */
    } alg_crc_config_t;

/* ======================== 预置 CRC 配置 ======================== */

    /** @brief CRC-8，多项式 0x8C，初始值 0xFF，LSB-first */
    extern const alg_crc_config_t alg_crc8_0x8c_ff_config;

    /** @brief CRC-16/CCITT-FALSE，多项式 0x1021，初始值 0xFFFF，MSB-first */
    extern const alg_crc_config_t alg_crc16_ccitt_false_config;

    /** @brief CRC-16，多项式 0x8408（反射），初始值 0xFFFF，LSB-first */
    extern const alg_crc_config_t alg_crc16_0x8408_ff_config;

/* ======================== 公共 API ======================== */

    /**
     * @brief 使用给定参数统一计算 CRC
     * @param config CRC 配置参数
     * @param data 待计算的数据缓冲区
     * @param data_size 数据长度（字节）
     * @param result 输出 CRC 计算结果
     * @return true=计算成功，false=参数无效
     * @note 支持 8/16/32 位宽的位序可配置的通用 CRC 计算。
     *       MSB-first 时将当前字节左移 (width-8) 位后加入寄存器；
     *       LSB-first 时逐字节异或后逐位右移处理。
     */
    bool alg_crc_calculate(const alg_crc_config_t *config,
                           const uint8_t *data, size_t data_size,
                           uint32_t *result);

#ifdef __cplusplus
}
#endif
#endif /* ALG_CRC_H */
