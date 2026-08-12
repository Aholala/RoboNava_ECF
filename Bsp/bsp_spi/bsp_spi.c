#include "bsp_spi.h"

static bsp_status_t bsp_spi_validate(const bsp_spi_t *me)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return me->is_initialized ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

bsp_status_t bsp_spi_init(bsp_spi_t *me, const bsp_spi_config_t *config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->transmit == NULL) ||
        (config->driver_ops->receive == NULL)) return BSP_STATUS_INVALID_ARGUMENT;

    *me = (bsp_spi_t){0};
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

bsp_status_t bsp_spi_deinit(bsp_spi_t *me)
{
    bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK) return status;
    status = (me->driver_ops->deinit != NULL) ? me->driver_ops->deinit(me->device_handle)
                                              : BSP_STATUS_OK;
    if (status == BSP_STATUS_OK) me->is_initialized = false;
    return status;
}

bsp_status_t bsp_spi_set_callback(bsp_spi_t *me, bsp_event_callback_t callback, void *context)
{
    const bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK) return status;
    me->callback = callback;
    me->user_context = context;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_spi_transmit(bsp_spi_t *me, const uint8_t *data, size_t size,
                              bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if ((data == NULL) || (size == 0U) || !bsp_transfer_mode_is_valid(mode))
        return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->transmit(me->device_handle, data, size, mode, timeout_ms);
}

bsp_status_t bsp_spi_receive(bsp_spi_t *me, uint8_t *data, size_t size,
                             bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if ((data == NULL) || (size == 0U) || !bsp_transfer_mode_is_valid(mode))
        return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->receive(me->device_handle, data, size, mode, timeout_ms);
}

bsp_status_t bsp_spi_exchange(bsp_spi_t *me, const uint8_t *tx, uint8_t *rx, size_t size,
                              bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if ((tx == NULL) || (rx == NULL) || (size == 0U) || !bsp_transfer_mode_is_valid(mode))
        return BSP_STATUS_INVALID_ARGUMENT;
    return (me->driver_ops->exchange != NULL)
               ? me->driver_ops->exchange(me->device_handle, tx, rx, size, mode, timeout_ms)
               : BSP_STATUS_UNSUPPORTED;
}

bsp_status_t bsp_spi_abort(bsp_spi_t *me)
{
    const bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK) return status;
    return (me->driver_ops->abort != NULL) ? me->driver_ops->abort(me->device_handle)
                                           : BSP_STATUS_UNSUPPORTED;
}

bsp_status_t bsp_spi_get_busy(const bsp_spi_t *me, bool *is_busy)
{
    const bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if (is_busy == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return (me->driver_ops->get_busy != NULL) ? me->driver_ops->get_busy(me->device_handle, is_busy)
                                              : BSP_STATUS_UNSUPPORTED;
}

void bsp_spi_notify(bsp_spi_t *me, bsp_event_t event, bsp_status_t status, size_t size)
{
    if ((me != NULL) && me->is_initialized && (me->callback != NULL))
        me->callback(event, status, size, me->user_context);
}
