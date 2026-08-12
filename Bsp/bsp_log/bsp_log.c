#include "bsp_log.h"

#include "SEGGER_RTT.h"

#include <stdarg.h>

#define BSP_LOG_BUFFER_INDEX (0U)

void bsp_log_init(void)
{
    SEGGER_RTT_Init();
}

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

void bsp_log_clear(void)
{
    (void)SEGGER_RTT_WriteString(BSP_LOG_BUFFER_INDEX, "  " RTT_CTRL_CLEAR);
}
