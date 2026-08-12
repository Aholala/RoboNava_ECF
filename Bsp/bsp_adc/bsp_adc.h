#ifndef BSP_ADC_H
#define BSP_ADC_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bsp_status_t (*init)(void *, uint32_t);
    bsp_status_t (*deinit)(void *, uint32_t);
    bsp_status_t (*start)(void *, uint32_t);
    bsp_status_t (*stop)(void *, uint32_t);
    bsp_status_t (*calibrate)(void *);
    bsp_status_t (*read_raw)(void *, uint32_t, uint32_t *, uint32_t);
    bsp_status_t (*start_dma)(void *, uint32_t, uint32_t *, size_t);
    bsp_status_t (*stop_dma)(void *, uint32_t);
} bsp_adc_driver_ops_t;

typedef struct bsp_adc {
    void *device_handle;
    const bsp_adc_driver_ops_t *driver_ops;
    uint32_t channel;
    float reference_voltage_v;
    uint32_t maximum_raw_value;
    bsp_event_callback_t callback;
    void *user_context;
    bool is_initialized;
} bsp_adc_t;

typedef struct {
    void *device_handle;
    const bsp_adc_driver_ops_t *driver_ops;
    uint32_t channel;
    uint8_t resolution_bits;
    float reference_voltage_v;
    bsp_event_callback_t callback;
    void *user_context;
} bsp_adc_config_t;

bsp_status_t bsp_adc_init(bsp_adc_t *me, const bsp_adc_config_t *config);
bsp_status_t bsp_adc_deinit(bsp_adc_t *me);
bsp_status_t bsp_adc_set_callback(bsp_adc_t *me, bsp_event_callback_t callback, void *context);
bsp_status_t bsp_adc_start(bsp_adc_t *me);
bsp_status_t bsp_adc_stop(bsp_adc_t *me);
bsp_status_t bsp_adc_calibrate(bsp_adc_t *me);
bsp_status_t bsp_adc_read_raw(bsp_adc_t *me, uint32_t *raw_value, uint32_t timeout_ms);
bsp_status_t bsp_adc_read_normalized(bsp_adc_t *me, float *value, uint32_t timeout_ms);
bsp_status_t bsp_adc_read_voltage(bsp_adc_t *me, float *voltage_v, uint32_t timeout_ms);
bsp_status_t bsp_adc_start_dma(bsp_adc_t *me, uint32_t *buffer, size_t count);
bsp_status_t bsp_adc_stop_dma(bsp_adc_t *me);
void bsp_adc_notify(bsp_adc_t *me, bsp_event_t event, bsp_status_t status, size_t size);

#ifdef __cplusplus
}
#endif
#endif
