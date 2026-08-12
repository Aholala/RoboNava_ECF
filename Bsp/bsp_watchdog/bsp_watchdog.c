#include "bsp_watchdog.h"

static bsp_status_t bsp_watchdog_validate(const bsp_watchdog_t *me)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return me->is_initialized ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

bsp_status_t bsp_watchdog_init(bsp_watchdog_t *me, const bsp_watchdog_config_t *config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->refresh == NULL))
        return BSP_STATUS_INVALID_ARGUMENT;

    *me = (bsp_watchdog_t){0};
    if (config->driver_ops->init != NULL) {
        status = config->driver_ops->init(config->device_handle);
        if (status != BSP_STATUS_OK) return status;
    }
    me->device_handle = config->device_handle;
    me->driver_ops = config->driver_ops;
    me->is_initialized = true;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_watchdog_deinit(bsp_watchdog_t *me)
{
    bsp_status_t status = bsp_watchdog_validate(me);
    if (status != BSP_STATUS_OK) return status;
    status = (me->driver_ops->deinit != NULL) ? me->driver_ops->deinit(me->device_handle)
                                              : BSP_STATUS_OK;
    if (status == BSP_STATUS_OK) me->is_initialized = false;
    return status;
}

bsp_status_t bsp_watchdog_refresh(bsp_watchdog_t *me)
{
    const bsp_status_t status = bsp_watchdog_validate(me);
    return (status == BSP_STATUS_OK) ? me->driver_ops->refresh(me->device_handle) : status;
}

bsp_status_t bsp_watchdog_get_timeout_ms(const bsp_watchdog_t *me, uint32_t *timeout_ms)
{
    const bsp_status_t status = bsp_watchdog_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if (timeout_ms == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return (me->driver_ops->get_timeout_ms != NULL)
               ? me->driver_ops->get_timeout_ms(me->device_handle, timeout_ms)
               : BSP_STATUS_UNSUPPORTED;
}

bsp_status_t bsp_watchdog_get_reset_detected(const bsp_watchdog_t *me, bool *reset_detected)
{
    const bsp_status_t status = bsp_watchdog_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if (reset_detected == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return (me->driver_ops->get_reset_detected != NULL)
               ? me->driver_ops->get_reset_detected(me->device_handle, reset_detected)
               : BSP_STATUS_UNSUPPORTED;
}
