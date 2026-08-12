/**
 * @file bsp_common.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BSP 层公共工具实现
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供传输模式校验和错误记录/读取功能。
 */

#include "bsp_common.h"

/* ======================== 传输模式校验 ======================== */

/**
 * @brief 校验传输模式是否合法
 * @param mode 传输模式
 * @return true 合法
 */
bool bsp_transfer_mode_is_valid(bsp_transfer_mode_t mode)
{
    return mode == BSP_TRANSFER_MODE_BLOCKING ||
           mode == BSP_TRANSFER_MODE_INTERRUPT ||
           mode == BSP_TRANSFER_MODE_DMA;
}

/* ======================== 错误记录 ======================== */

/** @brief 全局最后错误记录（静态存储） */
static bsp_error_t last_error;

/**
 * @brief 记录一条错误信息
 * @param code 错误码
 * @param source 错误来源描述
 * @param detail 错误详情
 */
void bsp_error_record(bsp_status_t code, const char *source, int detail)
{
    last_error = (bsp_error_t){code, source, detail, true};
}

/**
 * @brief 读取最后记录的错误
 * @return 错误记录指针（只读）
 */
const bsp_error_t *bsp_error_read(void)
{
    return &last_error;
}
