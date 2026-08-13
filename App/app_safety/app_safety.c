/**
 * @file app_safety.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 安全应用模块实现
 * @version 1.0
 * @date 2026-08-13
 * @copyright Copyright (c) 2026
 *
 * @note 管理心跳监控、失联判定与整机电机输出门，可选刷新硬件看门狗。
 */

#include "app_safety.h"
#include "bsp_log.h"
#include "module_motor.h"

/** @brief 将监控器置为失联，必需监控器失联时关闭整机输出。 */
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

/** @brief 将监控器置为在线并复位失联计时。 */
static void set_online(app_safety_monitor_t *monitor)
{
    if (monitor->state == APP_SAFETY_STATE_ONLINE) return;
    monitor->state = APP_SAFETY_STATE_ONLINE;
    monitor->offline_time_ms = 0U;
    BSP_LOG_INFO("%s online", monitor->config.name);
    if (monitor->config.online_callback) monitor->config.online_callback(monitor->config.user_context);
}

/**
 * @brief  初始化安全模块，并关闭整机输出。
 * @param  me        指向调用方分配的实例。
 * @param  watchdog  可选硬件看门狗，可为 NULL。
 * @return 成功返回 BSP_STATUS_OK，参数无效返回 BSP_STATUS_INVALID_ARGUMENT。
 */
bsp_status_t app_safety_init(app_safety_t *me, bsp_watchdog_t *watchdog)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    *me = (app_safety_t){.watchdog = watchdog, .initialized = true};
    module_motor_set_output_allowed(false);
    return BSP_STATUS_OK;
}

/**
 * @brief  运行时更换硬件看门狗。
 * @param  me        已初始化的安全实例。
 * @param  watchdog  新看门狗，可为 NULL。
 * @return 成功返回 BSP_STATUS_OK，未初始化返回 BSP_STATUS_NOT_INITIALIZED。
 */
bsp_status_t app_safety_set_watchdog(app_safety_t *me, bsp_watchdog_t *watchdog)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    if (!me->initialized) return BSP_STATUS_NOT_INITIALIZED;
    me->watchdog = watchdog;
    return BSP_STATUS_OK;
}

/**
 * @brief  初始化单个心跳监控器。
 * @param  me      指向调用方分配的监控器实例。
 * @param  config  静态配置（内部拷贝）。
 * @return 成功返回 BSP_STATUS_OK，参数无效返回 BSP_STATUS_INVALID_ARGUMENT。
 */
bsp_status_t app_safety_monitor_init(app_safety_monitor_t *me,
                                     const app_safety_monitor_config_t *config)
{
    if ((me == NULL) || (config == NULL) || (config->name == NULL) ||
        (config->timeout_ms == 0U)) return BSP_STATUS_INVALID_ARGUMENT;
    *me = (app_safety_monitor_t){.config = *config, .state = APP_SAFETY_STATE_STARTING};
    return BSP_STATUS_OK;
}

/**
 * @brief  注册监控器到安全实例。
 * @param  me        已初始化的安全实例。
 * @param  monitor   已初始化的监控器。
 * @return 成功返回 BSP_STATUS_OK，已注册返回 BSP_STATUS_BUSY，超容量返回 BSP_STATUS_NO_RESOURCE。
 */
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

/**
 * @brief  上报监控器在线（收到有效帧时调用）。
 * @param  monitor  监控器实例。
 * @param  now_ms   当前时刻 [ms]。
 */
void app_safety_notify_online(app_safety_monitor_t *monitor, uint32_t now_ms)
{
    if ((monitor == NULL) || !monitor->is_registered) return;
    monitor->last_online_time_ms = now_ms;
    monitor->heartbeat_received = true;
}

/**
 * @brief  查询是否所有必需监控器均在线。
 * @param  me  已初始化的安全实例。
 * @return 存在必需监控器且全部在线时为 true。
 */
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

/**
 * @brief  执行一个安全周期：刷新监控状态并更新输出门。
 * @param  me      已初始化的安全实例。
 * @param  now_ms  当前时刻 [ms]。
 */
void app_safety_process(app_safety_t *me, uint32_t now_ms)
{
    size_t i;
    if ((me == NULL) || !me->initialized) return;
    for (i = 0U; i < me->monitor_count; ++i) {
        app_safety_monitor_t *m = me->monitors[i];
        const uint32_t elapsed = now_ms - m->last_online_time_ms;
        m->offline_time_ms = elapsed;
        /* 收到过心跳且在超时内判为在线，超时判为失联。 */
        if (m->heartbeat_received && (elapsed <= m->config.timeout_ms)) set_online(m);
        else if (elapsed > m->config.timeout_ms) set_offline(me, m);
    }
    me->output_allowed = me->output_enabled && app_safety_all_required_online(me);
    module_motor_set_output_allowed(me->output_allowed);
    if (me->watchdog) (void)bsp_watchdog_refresh(me->watchdog);
}

/**
 * @brief  设置操作者输出请求。
 * @param  me       已初始化的安全实例。
 * @param  enabled  是否请求输出。
 */
void app_safety_set_output_enabled(app_safety_t *me, bool enabled)
{
    if ((me == NULL) || !me->initialized) return;
    me->output_enabled = enabled;
    if (!enabled) {
        me->output_allowed = false;
        module_motor_set_output_allowed(false);
    }
}

/**
 * @brief  查询整机输出门状态。
 * @param  me  安全实例。
 * @return 输出允许时为 true。
 */
bool app_safety_output_allowed(const app_safety_t *me)
{ return (me != NULL) && me->initialized && me->output_allowed; }

/**
 * @brief  查询监控器在线状态。
 * @param  monitor  监控器实例。
 * @return 当前在线状态，实例为空时返回 APP_SAFETY_STATE_OFFLINE。
 */
app_safety_state_t app_safety_get_state(const app_safety_monitor_t *monitor)
{ return monitor ? monitor->state : APP_SAFETY_STATE_OFFLINE; }

/**
 * @brief  查询监控器已失联时长。
 * @param  monitor  监控器实例。
 * @return 失联时长 [ms]，实例为空时返回 UINT32_MAX。
 */
uint32_t app_safety_get_offline_time_ms(const app_safety_monitor_t *monitor)
{ return monitor ? monitor->offline_time_ms : UINT32_MAX; }
