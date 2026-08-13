/**
 * @file app_safety.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 安全/心跳看门狗应用模块实现
 * @version 1.0
 * @date 2026-08-12
 * @copyright Copyright (c) 2026
 *
 * @note 基于 FreeRTOS 滴答计数器实现心跳超时检测，管理在线/离线状态转换与用户回调，周期性刷新硬件看门狗。
 */

#include "app_safety.h"

#include "bsp_log.h"
#include "module_motor.h"

/** @brief 安全管理器的内部数据结构。 */
typedef struct
{
    app_safety_monitor_t *monitors[APP_SAFETY_MAX_MONITOR_COUNT]; /**< 已注册监控器列表。 */
    size_t monitor_count;                                          /**< 当前已注册数量。 */
    bsp_watchdog_t *watchdog;                                      /**< 硬件看门狗句柄。 */
    bool output_enabled;
    bool output_allowed;
    bool is_initialized;                                           /**< 初始化守护标志。 */
} app_safety_manager_t;

static app_safety_manager_t app_safety_manager;

/* ======================== 内部辅助函数 ======================== */

/**
 * @brief  将监控器设置为离线状态并触发回调。
 * @param  monitor  目标监控器。
 *
 * 仅在当前状态不为离线时执行转换，记录日志并调用离线回调。
 */
static void app_safety_set_offline(app_safety_monitor_t *monitor)
{
    if (monitor->state == APP_SAFETY_STATE_OFFLINE)
    {
        return;
    }

    monitor->state = APP_SAFETY_STATE_OFFLINE;
    BSP_LOG_WARNING("%s offline", monitor->config.name);
    if (monitor->config.offline_callback != NULL)
    {
        monitor->config.offline_callback(monitor->config.user_context);
    }
}

/**
 * @brief  将监控器设置为在线状态并触发回调。
 * @param  monitor  目标监控器。
 *
 * 仅在当前状态不为在线时执行转换，清零离线时长，记录日志
 * 并调用在线回调。
 */
static void app_safety_set_online(app_safety_monitor_t *monitor)
{
    if (monitor->state == APP_SAFETY_STATE_ONLINE)
    {
        return;
    }

    monitor->state = APP_SAFETY_STATE_ONLINE;
    monitor->offline_time_ms = 0U;
    BSP_LOG_INFO("%s online", monitor->config.name);
    if (monitor->config.online_callback != NULL)
    {
        monitor->config.online_callback(monitor->config.user_context);
    }
}

/* ======================== 公共 API ======================== */

bsp_status_t app_safety_init(bsp_watchdog_t *watchdog)
{
    size_t index;

    for (index = 0U; index < APP_SAFETY_MAX_MONITOR_COUNT; ++index)
    {
        app_safety_manager.monitors[index] = NULL;
    }
    app_safety_manager.monitor_count = 0U;
    app_safety_manager.watchdog = watchdog;
    app_safety_manager.output_enabled = false;
    app_safety_manager.output_allowed = false;
    module_motor_set_output_allowed(false);
    app_safety_manager.is_initialized = true;
    return BSP_STATUS_OK;
}

bsp_status_t app_safety_set_watchdog(bsp_watchdog_t *watchdog)
{
    if (!app_safety_manager.is_initialized)
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    app_safety_manager.watchdog = watchdog;
    return BSP_STATUS_OK;
}

bsp_status_t app_safety_monitor_init(app_safety_monitor_t *me,
                                   const app_safety_monitor_config_t *config)
{
    if ((me == NULL) || (config == NULL) || (config->name == NULL) ||
        (config->timeout_ms == 0U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }

    me->config = *config;
    me->last_online_time_ms = 0U;
    me->heartbeat_received = false;
    me->offline_time_ms = 0U;
    me->state = APP_SAFETY_STATE_STARTING;
    me->is_registered = false;
    return BSP_STATUS_OK;
}

bsp_status_t app_safety_register(app_safety_monitor_t *monitor)
{
    if (!app_safety_manager.is_initialized)
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    if ((monitor == NULL) || (monitor->config.name == NULL) ||
        (monitor->config.timeout_ms == 0U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (monitor->is_registered)
    {
        return BSP_STATUS_BUSY;
    }
    if (app_safety_manager.monitor_count >= APP_SAFETY_MAX_MONITOR_COUNT)
    {
        return BSP_STATUS_NO_RESOURCE;
    }

    monitor->last_online_time_ms = 0U;
    app_safety_manager.monitors[app_safety_manager.monitor_count] = monitor;
    ++app_safety_manager.monitor_count;
    monitor->is_registered = true;
    return BSP_STATUS_OK;
}

void app_safety_notify_online(app_safety_monitor_t *monitor, uint32_t now_ms)
{
    if ((monitor == NULL) || !monitor->is_registered)
    {
        return;
    }

    monitor->last_online_time_ms = now_ms;
    monitor->heartbeat_received = true;
}

void app_safety_process(uint32_t now_ms)
{
    size_t index;

    if (!app_safety_manager.is_initialized)
    {
        return;
    }

    /* 遍历所有已注册的监控器，更新离线时长与状态。 */
    for (index = 0U; index < app_safety_manager.monitor_count; ++index)
    {
        app_safety_monitor_t *const monitor = app_safety_manager.monitors[index];
        const uint32_t elapsed_time_ms = now_ms - monitor->last_online_time_ms;

        monitor->offline_time_ms = elapsed_time_ms;
        if (monitor->heartbeat_received && (elapsed_time_ms <= monitor->config.timeout_ms))
        {
            app_safety_set_online(monitor);
        }
        else if (elapsed_time_ms > monitor->config.timeout_ms)
        {
            app_safety_set_offline(monitor);
        }
    }

    app_safety_manager.output_allowed =
        app_safety_manager.output_enabled && app_safety_all_required_online();
    module_motor_set_output_allowed(app_safety_manager.output_allowed);

    /* 每次处理周期刷新硬件看门狗。 */
    if (app_safety_manager.watchdog != NULL)
    {
        (void)bsp_watchdog_refresh(app_safety_manager.watchdog);
    }
}

void app_safety_set_output_enabled(bool enabled)
{
    app_safety_manager.output_enabled = enabled;
    if (!enabled)
    {
        app_safety_manager.output_allowed = false;
        module_motor_set_output_allowed(false);
    }
}

bool app_safety_output_allowed(void)
{
    return app_safety_manager.is_initialized && app_safety_manager.output_allowed;
}

app_safety_state_t app_safety_get_state(const app_safety_monitor_t *monitor)
{
    return (monitor != NULL) ? monitor->state : APP_SAFETY_STATE_OFFLINE;
}

uint32_t app_safety_get_offline_time_ms(const app_safety_monitor_t *monitor)
{
    return (monitor != NULL) ? monitor->offline_time_ms : UINT32_MAX;
}

bool app_safety_all_required_online(void)
{
    size_t index;
    bool has_required_monitor = false;

    if (!app_safety_manager.is_initialized)
    {
        return false;
    }
    for (index = 0U; index < app_safety_manager.monitor_count; ++index)
    {
        const app_safety_monitor_t *const monitor = app_safety_manager.monitors[index];
        has_required_monitor = has_required_monitor || monitor->config.required;
        if (monitor->config.required && (monitor->state != APP_SAFETY_STATE_ONLINE))
        {
            return false;
        }
    }
    return has_required_monitor;
}
