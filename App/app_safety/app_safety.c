#include "app_safety.h"
#include "bsp_log.h"
#include "module_motor.h"

static void set_offline(app_safety_t *me, app_safety_monitor_t *monitor)
{
    if (monitor->state == APP_SAFETY_STATE_OFFLINE) return;
    monitor->state = APP_SAFETY_STATE_OFFLINE;
    if (monitor->config.required) {
        me->output_enabled = false;
        me->output_allowed = false;
        module_motor_set_output_allowed(false);
    }
    BSP_LOG_WARNING("%s offline", monitor->config.name);
    if (monitor->config.offline_callback) monitor->config.offline_callback(monitor->config.user_context);
}

static void set_online(app_safety_monitor_t *monitor)
{
    if (monitor->state == APP_SAFETY_STATE_ONLINE) return;
    monitor->state = APP_SAFETY_STATE_ONLINE;
    monitor->offline_time_ms = 0U;
    BSP_LOG_INFO("%s online", monitor->config.name);
    if (monitor->config.online_callback) monitor->config.online_callback(monitor->config.user_context);
}

bsp_status_t app_safety_init(app_safety_t *me, bsp_watchdog_t *watchdog)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    *me = (app_safety_t){.watchdog = watchdog, .initialized = true};
    module_motor_set_output_allowed(false);
    return BSP_STATUS_OK;
}

bsp_status_t app_safety_set_watchdog(app_safety_t *me, bsp_watchdog_t *watchdog)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    if (!me->initialized) return BSP_STATUS_NOT_INITIALIZED;
    me->watchdog = watchdog;
    return BSP_STATUS_OK;
}

bsp_status_t app_safety_monitor_init(app_safety_monitor_t *me,
                                     const app_safety_monitor_config_t *config)
{
    if ((me == NULL) || (config == NULL) || (config->name == NULL) ||
        (config->timeout_ms == 0U)) return BSP_STATUS_INVALID_ARGUMENT;
    *me = (app_safety_monitor_t){.config = *config, .state = APP_SAFETY_STATE_STARTING};
    return BSP_STATUS_OK;
}

bsp_status_t app_safety_register(app_safety_t *me, app_safety_monitor_t *monitor)
{
    if ((me == NULL) || (monitor == NULL)) return BSP_STATUS_INVALID_ARGUMENT;
    if (!me->initialized) return BSP_STATUS_NOT_INITIALIZED;
    if (monitor->is_registered) return BSP_STATUS_BUSY;
    if (me->monitor_count >= APP_SAFETY_MAX_MONITOR_COUNT) return BSP_STATUS_NO_RESOURCE;
    me->monitors[me->monitor_count++] = monitor;
    monitor->is_registered = true;
    return BSP_STATUS_OK;
}

void app_safety_notify_online(app_safety_monitor_t *monitor, uint32_t now_ms)
{
    if ((monitor == NULL) || !monitor->is_registered) return;
    monitor->last_online_time_ms = now_ms;
    monitor->heartbeat_received = true;
}

bool app_safety_all_required_online(const app_safety_t *me)
{
    bool found = false;
    size_t i;
    if ((me == NULL) || !me->initialized) return false;
    for (i = 0U; i < me->monitor_count; ++i) {
        const app_safety_monitor_t *m = me->monitors[i];
        found |= m->config.required;
        if (m->config.required && (m->state != APP_SAFETY_STATE_ONLINE)) return false;
    }
    return found;
}

void app_safety_process(app_safety_t *me, uint32_t now_ms)
{
    size_t i;
    if ((me == NULL) || !me->initialized) return;
    for (i = 0U; i < me->monitor_count; ++i) {
        app_safety_monitor_t *m = me->monitors[i];
        const uint32_t elapsed = now_ms - m->last_online_time_ms;
        m->offline_time_ms = elapsed;
        if (m->heartbeat_received && (elapsed <= m->config.timeout_ms)) set_online(m);
        else if (elapsed > m->config.timeout_ms) set_offline(me, m);
    }
    me->output_allowed = me->output_enabled && app_safety_all_required_online(me);
    module_motor_set_output_allowed(me->output_allowed);
    if (me->watchdog) (void)bsp_watchdog_refresh(me->watchdog);
}

void app_safety_set_output_enabled(app_safety_t *me, bool enabled)
{
    if ((me == NULL) || !me->initialized) return;
    me->output_enabled = enabled;
    if (!enabled) {
        me->output_allowed = false;
        module_motor_set_output_allowed(false);
    }
}

bool app_safety_output_allowed(const app_safety_t *me)
{ return (me != NULL) && me->initialized && me->output_allowed; }
app_safety_state_t app_safety_get_state(const app_safety_monitor_t *monitor)
{ return monitor ? monitor->state : APP_SAFETY_STATE_OFFLINE; }
uint32_t app_safety_get_offline_time_ms(const app_safety_monitor_t *monitor)
{ return monitor ? monitor->offline_time_ms : UINT32_MAX; }
