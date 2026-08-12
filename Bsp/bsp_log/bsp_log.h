#ifndef BSP_LOG_H
#define BSP_LOG_H

#ifndef BSP_LOG_DISABLED
#define BSP_LOG_DISABLED (0)
#endif

typedef enum {
    BSP_LOG_LEVEL_INFO = 0,
    BSP_LOG_LEVEL_WARNING,
    BSP_LOG_LEVEL_ERROR
} bsp_log_level_t;

void bsp_log_init(void);
int bsp_log_printf(const char *format, ...);
int bsp_log_write(bsp_log_level_t level, const char *format, ...);
void bsp_log_clear(void);

#if BSP_LOG_DISABLED
#define BSP_LOG_INFO(format, ...) ((void)0)
#define BSP_LOG_WARNING(format, ...) ((void)0)
#define BSP_LOG_ERROR(format, ...) ((void)0)
#else
#define BSP_LOG_INFO(format, ...) \
    ((void)bsp_log_write(BSP_LOG_LEVEL_INFO, format, ##__VA_ARGS__))
#define BSP_LOG_WARNING(format, ...) \
    ((void)bsp_log_write(BSP_LOG_LEVEL_WARNING, format, ##__VA_ARGS__))
#define BSP_LOG_ERROR(format, ...) \
    ((void)bsp_log_write(BSP_LOG_LEVEL_ERROR, format, ##__VA_ARGS__))
#endif

#endif
