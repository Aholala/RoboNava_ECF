#ifndef BSP_WATCHDOG_H
#define BSP_WATCHDOG_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bsp_status_t (*init)(void *device_handle);
    bsp_status_t (*deinit)(void *device_handle);
    bsp_status_t (*refresh)(void *device_handle);
    bsp_status_t (*get_timeout_ms)(const void *device_handle, uint32_t *timeout_ms);
    bsp_status_t (*get_reset_detected)(const void *device_handle, bool *reset_detected);
} bsp_watchdog_driver_ops_t;

typedef struct bsp_watchdog {
    void *device_handle;
    const bsp_watchdog_driver_ops_t *driver_ops;
    bool is_initialized;
} bsp_watchdog_t;

typedef struct {
    void *device_handle;
    const bsp_watchdog_driver_ops_t *driver_ops;
} bsp_watchdog_config_t;

bsp_status_t bsp_watchdog_init(bsp_watchdog_t *me, const bsp_watchdog_config_t *config);
bsp_status_t bsp_watchdog_deinit(bsp_watchdog_t *me);
bsp_status_t bsp_watchdog_refresh(bsp_watchdog_t *me);
bsp_status_t bsp_watchdog_get_timeout_ms(const bsp_watchdog_t *me, uint32_t *timeout_ms);
bsp_status_t bsp_watchdog_get_reset_detected(const bsp_watchdog_t *me, bool *reset_detected);

#ifdef __cplusplus
}
#endif
#endif
