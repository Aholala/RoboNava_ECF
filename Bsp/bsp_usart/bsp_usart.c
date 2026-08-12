#include "bsp_usart.h"

static bsp_status_t bsp_usart_validate(const bsp_usart_t *me)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return me->is_initialized ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

bsp_status_t bsp_usart_init(bsp_usart_t *me, const bsp_usart_config_t *config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->transmit == NULL) ||
        (config->driver_ops->receive == NULL)) return BSP_STATUS_INVALID_ARGUMENT;
    *me = (bsp_usart_t){0};
    if (config->driver_ops->init != NULL) {
        status = config->driver_ops->init(config->device_handle);
        if (status != BSP_STATUS_OK) return status;
    }
    me->device_handle = config->device_handle;
    me->driver_ops = config->driver_ops;
    me->callback = config->callback;
    me->user_context = config->user_context;
    me->is_initialized = true;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_usart_deinit(bsp_usart_t *me)
{
    bsp_status_t status = bsp_usart_validate(me);
    if (status != BSP_STATUS_OK) return status;
    status = (me->driver_ops->deinit != NULL) ? me->driver_ops->deinit(me->device_handle)
                                              : BSP_STATUS_OK;
    if (status == BSP_STATUS_OK) me->is_initialized = false;
    return status;
}

bsp_status_t bsp_usart_set_callback(bsp_usart_t *me, bsp_event_callback_t callback, void *context)
{
    const bsp_status_t status = bsp_usart_validate(me);
    if (status != BSP_STATUS_OK) return status;
    me->callback = callback;
    me->user_context = context;
    return BSP_STATUS_OK;
}

static bool transfer_is_valid(const void *data, size_t size, bsp_transfer_mode_t mode)
{
    return (data != NULL) && (size > 0U) && bsp_transfer_mode_is_valid(mode);
}

bsp_status_t bsp_usart_transmit(bsp_usart_t *me, const uint8_t *data, size_t size,
                                bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_usart_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if (!transfer_is_valid(data, size, mode)) return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->transmit(me->device_handle, data, size, mode, timeout_ms);
}

bsp_status_t bsp_usart_receive(bsp_usart_t *me, uint8_t *data, size_t size,
                               bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_usart_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if (!transfer_is_valid(data, size, mode)) return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->receive(me->device_handle, data, size, mode, timeout_ms);
}

bsp_status_t bsp_usart_receive_to_idle(bsp_usart_t *me, uint8_t *data, size_t capacity,
                                       bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_usart_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if (!transfer_is_valid(data, capacity, mode)) return BSP_STATUS_INVALID_ARGUMENT;
    return (me->driver_ops->receive_to_idle != NULL)
               ? me->driver_ops->receive_to_idle(me->device_handle, data, capacity, mode, timeout_ms)
               : BSP_STATUS_UNSUPPORTED;
}

bsp_status_t bsp_usart_receive_to_idle_double_buffer(
    bsp_usart_t *me, uint8_t *first, uint8_t *second, size_t capacity,
    bsp_usart_double_buffer_callback_t callback, void *context)
{
    const bsp_status_t status = bsp_usart_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if ((first == NULL) || (second == NULL) || (first == second) || (capacity == 0U) ||
        (callback == NULL)) return BSP_STATUS_INVALID_ARGUMENT;
    if (me->driver_ops->receive_to_idle_double_buffer == NULL) return BSP_STATUS_UNSUPPORTED;
    me->double_buffer_callback = callback;
    me->double_buffer_user_context = context;
    return me->driver_ops->receive_to_idle_double_buffer(me->device_handle, first, second, capacity);
}

bsp_status_t bsp_usart_abort(bsp_usart_t *me)
{
    const bsp_status_t status = bsp_usart_validate(me);
    if (status != BSP_STATUS_OK) return status;
    return (me->driver_ops->abort != NULL) ? me->driver_ops->abort(me->device_handle)
                                           : BSP_STATUS_UNSUPPORTED;
}

bsp_status_t bsp_usart_get_busy(const bsp_usart_t *me, bool *is_busy)
{
    const bsp_status_t status = bsp_usart_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if (is_busy == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return (me->driver_ops->get_busy != NULL) ? me->driver_ops->get_busy(me->device_handle, is_busy)
                                              : BSP_STATUS_UNSUPPORTED;
}

void bsp_usart_notify(bsp_usart_t *me, bsp_event_t event, bsp_status_t status, size_t size)
{
    if ((me != NULL) && me->is_initialized && (me->callback != NULL))
        me->callback(event, status, size, me->user_context);
}

void bsp_usart_notify_double_buffer(bsp_usart_t *me, uint8_t index, size_t size)
{
    if ((me != NULL) && me->is_initialized && (me->double_buffer_callback != NULL))
        me->double_buffer_callback(index, size, me->double_buffer_user_context);
}
