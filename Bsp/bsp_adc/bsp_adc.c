#include "bsp_adc.h"

#include <math.h>

static bsp_status_t bsp_adc_validate(const bsp_adc_t *me)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return me->is_initialized ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

bsp_status_t bsp_adc_init(bsp_adc_t *me, const bsp_adc_config_t *config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->resolution_bits == 0U) ||
        (config->resolution_bits > 31U) || !isfinite(config->reference_voltage_v) ||
        (config->reference_voltage_v <= 0.0F) || (config->driver_ops->start == NULL) ||
        (config->driver_ops->stop == NULL) || (config->driver_ops->calibrate == NULL) ||
        (config->driver_ops->read_raw == NULL)) return BSP_STATUS_INVALID_ARGUMENT;

    *me = (bsp_adc_t){0};
    if (config->driver_ops->init != NULL) {
        status = config->driver_ops->init(config->device_handle, config->channel);
        if (status != BSP_STATUS_OK) return status;
    }
    me->device_handle = config->device_handle;
    me->driver_ops = config->driver_ops;
    me->channel = config->channel;
    me->reference_voltage_v = config->reference_voltage_v;
    me->maximum_raw_value = (1UL << config->resolution_bits) - 1UL;
    me->callback = config->callback;
    me->user_context = config->user_context;
    me->is_initialized = true;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_adc_deinit(bsp_adc_t *me)
{
    bsp_status_t status = bsp_adc_validate(me);
    if (status != BSP_STATUS_OK) return status;
    status = (me->driver_ops->deinit != NULL)
                 ? me->driver_ops->deinit(me->device_handle, me->channel) : BSP_STATUS_OK;
    if (status == BSP_STATUS_OK) me->is_initialized = false;
    return status;
}

bsp_status_t bsp_adc_set_callback(bsp_adc_t *me, bsp_event_callback_t callback, void *context)
{
    const bsp_status_t status = bsp_adc_validate(me);
    if (status != BSP_STATUS_OK) return status;
    me->callback = callback;
    me->user_context = context;
    return BSP_STATUS_OK;
}

#define ADC_ACTION(name, member) \
    bsp_status_t name(bsp_adc_t *me) { \
        const bsp_status_t status = bsp_adc_validate(me); \
        return (status == BSP_STATUS_OK) \
            ? me->driver_ops->member(me->device_handle, me->channel) : status; \
    }
ADC_ACTION(bsp_adc_start, start)
ADC_ACTION(bsp_adc_stop, stop)

bsp_status_t bsp_adc_calibrate(bsp_adc_t *me)
{
    const bsp_status_t status = bsp_adc_validate(me);
    return (status == BSP_STATUS_OK) ? me->driver_ops->calibrate(me->device_handle) : status;
}

bsp_status_t bsp_adc_read_raw(bsp_adc_t *me, uint32_t *raw_value, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_adc_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if (raw_value == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    const bsp_status_t read_status = me->driver_ops->read_raw(
        me->device_handle, me->channel, raw_value, timeout_ms);
    if (read_status != BSP_STATUS_OK) return read_status;
    return (*raw_value <= me->maximum_raw_value) ? BSP_STATUS_OK : BSP_STATUS_IO_ERROR;
}

bsp_status_t bsp_adc_read_normalized(bsp_adc_t *me, float *value, uint32_t timeout_ms)
{
    uint32_t raw;
    if (value == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    const bsp_status_t status = bsp_adc_read_raw(me, &raw, timeout_ms);
    if (status == BSP_STATUS_OK) *value = (float)raw / (float)me->maximum_raw_value;
    return status;
}

bsp_status_t bsp_adc_read_voltage(bsp_adc_t *me, float *voltage_v, uint32_t timeout_ms)
{
    float normalized;
    if (voltage_v == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    const bsp_status_t status = bsp_adc_read_normalized(me, &normalized, timeout_ms);
    if (status == BSP_STATUS_OK) *voltage_v = normalized * me->reference_voltage_v;
    return status;
}

bsp_status_t bsp_adc_start_dma(bsp_adc_t *me, uint32_t *buffer, size_t count)
{
    const bsp_status_t status = bsp_adc_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if ((buffer == NULL) || (count == 0U)) return BSP_STATUS_INVALID_ARGUMENT;
    return (me->driver_ops->start_dma != NULL)
        ? me->driver_ops->start_dma(me->device_handle, me->channel, buffer, count)
        : BSP_STATUS_UNSUPPORTED;
}

bsp_status_t bsp_adc_stop_dma(bsp_adc_t *me)
{
    const bsp_status_t status = bsp_adc_validate(me);
    if (status != BSP_STATUS_OK) return status;
    return (me->driver_ops->stop_dma != NULL)
        ? me->driver_ops->stop_dma(me->device_handle, me->channel) : BSP_STATUS_UNSUPPORTED;
}

void bsp_adc_notify(bsp_adc_t *me, bsp_event_t event, bsp_status_t status, size_t size)
{
    if ((me != NULL) && me->is_initialized && (me->callback != NULL))
        me->callback(event, status, size, me->user_context);
}
