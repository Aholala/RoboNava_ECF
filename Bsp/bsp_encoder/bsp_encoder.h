#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H
#include "bsp_common.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef enum { BSP_ENCODER_DIRECTION_STOPPED = 0, BSP_ENCODER_DIRECTION_FORWARD,
               BSP_ENCODER_DIRECTION_REVERSE } bsp_encoder_direction_t;
typedef struct {
    bsp_status_t (*init)(void *); bsp_status_t (*deinit)(void *);
    bsp_status_t (*start)(void *); bsp_status_t (*stop)(void *);
    bsp_status_t (*set_count)(void *, int32_t);
    bsp_status_t (*get_count)(const void *, int32_t *);
    bsp_status_t (*get_direction)(const void *, bsp_encoder_direction_t *);
} bsp_encoder_driver_ops_t;
typedef struct bsp_encoder {
    void *device_handle; const bsp_encoder_driver_ops_t *driver_ops;
    int32_t previous_count; uint32_t counter_modulus; bool is_initialized;
} bsp_encoder_t;
typedef struct { void *device_handle; const bsp_encoder_driver_ops_t *driver_ops;
                 uint32_t counter_modulus; } bsp_encoder_config_t;
bsp_status_t bsp_encoder_init(bsp_encoder_t *, const bsp_encoder_config_t *);
bsp_status_t bsp_encoder_deinit(bsp_encoder_t *);
bsp_status_t bsp_encoder_start(bsp_encoder_t *); bsp_status_t bsp_encoder_stop(bsp_encoder_t *);
bsp_status_t bsp_encoder_reset(bsp_encoder_t *);
bsp_status_t bsp_encoder_set_count(bsp_encoder_t *, int32_t);
bsp_status_t bsp_encoder_get_count(const bsp_encoder_t *, int32_t *);
bsp_status_t bsp_encoder_get_delta(bsp_encoder_t *, int32_t *);
bsp_status_t bsp_encoder_get_direction(const bsp_encoder_t *, bsp_encoder_direction_t *);
#ifdef __cplusplus
}
#endif
#endif
