/**
 * @file bsp_log.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BSP 日志模块公共接口
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供编译期可配置的调试日志宏和底层 log_write 函数。
 *       设置 BSP_LOG_DISABLED=1 可在编译期剔除所有日志调用。
 */

#ifndef BSP_LOG_H
#define BSP_LOG_H

#ifndef BSP_LOG_DISABLED
#define BSP_LOG_DISABLED (0)
#endif

/** @brief 日志严重级别 */
typedef enum {
    BSP_LOG_LEVEL_INFO = 0,     /**< 信息消息 */
    BSP_LOG_LEVEL_WARNING,      /**< 警告条件 */
    BSP_LOG_LEVEL_ERROR         /**< 错误条件 */
} bsp_log_level_t;

/* ======================== 公共 API ======================== */

void bsp_log_init(void);
int bsp_log_printf(const char *format, ...);
int bsp_log_write(bsp_log_level_t level, const char *format, ...);
void bsp_log_clear(void);

/* ======================== 便捷宏 ======================== */

#if BSP_LOG_DISABLED
#define BSP_LOG_INFO(format, ...) ((void)0)
#define BSP_LOG_WARNING(format, ...) ((void)0)
#define BSP_LOG_ERROR(format, ...) ((void)0)
#else
/** @brief 记录信息级别消息（绿色） */
#define BSP_LOG_INFO(format, ...) \
    ((void)bsp_log_write(BSP_LOG_LEVEL_INFO, format, ##__VA_ARGS__))
/** @brief 记录警告级别消息（黄色） */
#define BSP_LOG_WARNING(format, ...) \
    ((void)bsp_log_write(BSP_LOG_LEVEL_WARNING, format, ##__VA_ARGS__))
/** @brief 记录错误级别消息（红色） */
#define BSP_LOG_ERROR(format, ...) \
    ((void)bsp_log_write(BSP_LOG_LEVEL_ERROR, format, ##__VA_ARGS__))
#endif

#endif
