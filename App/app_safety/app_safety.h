#ifndef APP_SAFETY_H
#define APP_SAFETY_H

#include "bsp_common.h"
#include "bsp_watchdog.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define APP_SAFETY_MAX_MONITOR_COUNT (16U)
#define APP_SAFETY_DEFAULT_TASK_PERIOD_MS (5U)

typedef enum { APP_SAFETY_STATE_STARTING = 0, APP_SAFETY_STATE_ONLINE,
               APP_SAFETY_STATE_OFFLINE } app_safety_state_t;
typedef void (*app_safety_callback_t)(void *user_context);

typedef struct {
    const char *name;
    uint32_t timeout_ms;
    bool required;
    app_safety_callback_t offline_callback;
    app_safety_callback_t online_callback;
    void *user_context;
} app_safety_monitor_config_t;

typedef struct {
    app_safety_monitor_config_t config;
    volatile uint32_t last_online_time_ms;
    volatile bool heartbeat_received;
    uint32_t offline_time_ms;
    app_safety_state_t state;
    bool is_registered;
} app_safety_monitor_t;

typedef struct {
    app_safety_monitor_t *monitors[APP_SAFETY_MAX_MONITOR_COUNT];
    size_t monitor_count;
    bsp_watchdog_t *watchdog;
    bool output_enabled;
    bool output_allowed;
    bool initialized;
} app_safety_t;

bsp_status_t app_safety_init(app_safety_t *me, bsp_watchdog_t *watchdog);
bsp_status_t app_safety_set_watchdog(app_safety_t *me, bsp_watchdog_t *watchdog);
bsp_status_t app_safety_monitor_init(app_safety_monitor_t *me,
                                     const app_safety_monitor_config_t *config);
bsp_status_t app_safety_register(app_safety_t *me, app_safety_monitor_t *monitor);
void app_safety_notify_online(app_safety_monitor_t *monitor, uint32_t now_ms);
void app_safety_process(app_safety_t *me, uint32_t now_ms);
void app_safety_set_output_enabled(app_safety_t *me, bool enabled);
bool app_safety_output_allowed(const app_safety_t *me);
app_safety_state_t app_safety_get_state(const app_safety_monitor_t *monitor);
uint32_t app_safety_get_offline_time_ms(const app_safety_monitor_t *monitor);
bool app_safety_all_required_online(const app_safety_t *me);

#endif
