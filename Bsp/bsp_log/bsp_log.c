/**
 * @file bsp_log.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BSP 日志模块实现，基于 SEGGER RTT
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 通过 SEGGER RTT（实时传输）提供调试日志输出。
 *       支持三种严重级别（信息、警告、错误），附带颜色编码
 *       输出和 ANSI 转义序列格式化。
 */

#include "bsp_log.h"

#include "SEGGER_RTT.h"

#include <stdarg.h>

/** @brief 日志输出所用的 RTT 缓冲区索引（通道 0） */
#define BSP_LOG_BUFFER_INDEX (0U)

/* ======================== 公共 API ======================== */

/**
 * @brief 初始化日志子系统
 * @note 必须在任何日志输出前调用一次。初始化 SEGGER RTT 控制块。
 */
void bsp_log_init(void)
{
    SEGGER_RTT_Init();
}

/**
 * @brief 打印格式化字符串到日志输出（无级别前缀）
 * @param format printf 风格的格式化字符串
 * @param ... 与格式化字符串匹配的可变参数
 * @return 写入字符数，format 为空返回 -1
 * @note 原始输出，不带严重级别前缀和颜色。如需带级别标签的输出，
 *       请使用 bsp_log_write()。
 */
int bsp_log_printf(const char *format, ...)
{
    int character_count;
    va_list arguments;

    if (format == NULL)
    {
        return -1;
    }

    va_start(arguments, format);
    character_count = SEGGER_RTT_vprintf(BSP_LOG_BUFFER_INDEX, format, &arguments);
    va_end(arguments);
    return character_count;
}

/**
 * @brief 写入带颜色编码和级别标签的日志消息
 * @param level 日志严重级别（信息/警告/错误）
 * @param format printf 风格的格式化字符串
 * @param ... 与格式化字符串匹配的可变参数
 * @return 写入字符数，参数无效返回 -1
 * @note 输出格式："[颜色][标签] message\\r\\n[重置]"。
 *       颜色：绿色（信息），黄色（警告），红色（错误）。
 */
int bsp_log_write(bsp_log_level_t level, const char *format, ...)
{
    static const char *const types[] = {"I:", "W:", "E:"};
    static const char *const colors[] = {RTT_CTRL_TEXT_BRIGHT_GREEN,
                                         RTT_CTRL_TEXT_BRIGHT_YELLOW,
                                         RTT_CTRL_TEXT_BRIGHT_RED};
    va_list arguments;
    int character_count;

    if ((level > BSP_LOG_LEVEL_ERROR) || (format == NULL))
    {
        return -1;
    }
    character_count = SEGGER_RTT_printf(BSP_LOG_BUFFER_INDEX, "  %s%s", colors[level],
                                         types[level]);
    va_start(arguments, format);
    character_count += SEGGER_RTT_vprintf(BSP_LOG_BUFFER_INDEX, format, &arguments);
    va_end(arguments);
    character_count += SEGGER_RTT_WriteString(BSP_LOG_BUFFER_INDEX, "\r\n" RTT_CTRL_RESET);
    return character_count;
}

/**
 * @brief 清除 RTT 终端输出缓冲区
 * @note 向终端发送 RTT 清屏控制序列。
 */
void bsp_log_clear(void)
{
    (void)SEGGER_RTT_WriteString(BSP_LOG_BUFFER_INDEX, "  " RTT_CTRL_CLEAR);
}
