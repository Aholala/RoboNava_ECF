#ifndef BSP_TIMER_H
#define BSP_TIMER_H
#include "bsp_common.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct bsp_timer bsp_timer_t;
typedef void (*bsp_timer_callback_t)(bsp_timer_t *, void *);
typedef struct {
    bsp_status_t (*init)(void *); bsp_status_t (*deinit)(void *);
    bsp_status_t (*start)(void *); bsp_status_t (*stop)(void *);
    bsp_status_t (*set_counter)(void *, uint32_t);
    bsp_status_t (*get_counter)(const void *, uint32_t *);
    bsp_status_t (*set_period)(void *, uint32_t);
    bsp_status_t (*get_period)(const void *, uint32_t *);
    bsp_status_t (*get_frequency)(const void *, uint32_t *);
} bsp_timer_driver_ops_t;
struct bsp_timer {
    void *device_handle; const bsp_timer_driver_ops_t *driver_ops;
    bsp_timer_callback_t callback; void *user_context; bool is_initialized;
};
typedef struct { void *device_handle; const bsp_timer_driver_ops_t *driver_ops;
                 bsp_timer_callback_t callback; void *user_context; } bsp_timer_config_t;
bsp_status_t bsp_timer_init(bsp_timer_t *, const bsp_timer_config_t *);
bsp_status_t bsp_timer_deinit(bsp_timer_t *);
bsp_status_t bsp_timer_set_callback(bsp_timer_t *, bsp_timer_callback_t, void *);
bsp_status_t bsp_timer_start(bsp_timer_t *); bsp_status_t bsp_timer_stop(bsp_timer_t *);
bsp_status_t bsp_timer_reset(bsp_timer_t *);
bsp_status_t bsp_timer_set_counter(bsp_timer_t *, uint32_t);
bsp_status_t bsp_timer_get_counter(const bsp_timer_t *, uint32_t *);
bsp_status_t bsp_timer_set_period(bsp_timer_t *, uint32_t);
bsp_status_t bsp_timer_get_period(const bsp_timer_t *, uint32_t *);
bsp_status_t bsp_timer_get_frequency(const bsp_timer_t *, uint32_t *);
void bsp_timer_notify_elapsed(bsp_timer_t *);
#ifdef __cplusplus
}
#endif
#endif
