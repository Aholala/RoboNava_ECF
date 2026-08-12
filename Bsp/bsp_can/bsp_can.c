#include "bsp_can.h"

static bsp_status_t bsp_can_validate(const bsp_can_t *me)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return me->is_initialized ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

static bool bsp_can_is_id_valid(uint32_t id, bsp_can_id_type_t type)
{
    return (type == BSP_CAN_ID_STANDARD) ? (id <= 0x7FFU)
         : (type == BSP_CAN_ID_EXTENDED) ? (id <= 0x1FFFFFFFU)
                                         : false;
}

static bool bsp_can_is_frame_valid(const bsp_can_frame_t *frame)
{
    return (frame != NULL) && (frame->data_length <= 8U) &&
           ((frame->frame_type == BSP_CAN_FRAME_DATA) ||
            (frame->frame_type == BSP_CAN_FRAME_REMOTE)) &&
           bsp_can_is_id_valid(frame->identifier, frame->id_type);
}

bsp_status_t bsp_can_init(bsp_can_t *me, const bsp_can_config_t *config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->start == NULL) ||
        (config->driver_ops->stop == NULL) || (config->driver_ops->configure_filter == NULL) ||
        (config->driver_ops->transmit == NULL) || (config->driver_ops->receive == NULL))
        return BSP_STATUS_INVALID_ARGUMENT;
    *me = (bsp_can_t){0};
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

bsp_status_t bsp_can_deinit(bsp_can_t *me)
{
    bsp_status_t status = bsp_can_validate(me);
    if (status != BSP_STATUS_OK) return status;
    status = (me->driver_ops->deinit != NULL) ? me->driver_ops->deinit(me->device_handle)
                                              : BSP_STATUS_OK;
    if (status == BSP_STATUS_OK) me->is_initialized = false;
    return status;
}

bsp_status_t bsp_can_set_callback(bsp_can_t *me, bsp_event_callback_t callback, void *context)
{
    const bsp_status_t status = bsp_can_validate(me);
    if (status != BSP_STATUS_OK) return status;
    me->callback = callback;
    me->user_context = context;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_can_start(bsp_can_t *me)
{
    const bsp_status_t status = bsp_can_validate(me);
    return (status == BSP_STATUS_OK) ? me->driver_ops->start(me->device_handle) : status;
}

bsp_status_t bsp_can_stop(bsp_can_t *me)
{
    const bsp_status_t status = bsp_can_validate(me);
    return (status == BSP_STATUS_OK) ? me->driver_ops->stop(me->device_handle) : status;
}

bsp_status_t bsp_can_configure_filter(bsp_can_t *me, const bsp_can_filter_t *filter)
{
    const bsp_status_t status = bsp_can_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if ((filter == NULL) || !bsp_can_is_id_valid(filter->identifier, filter->id_type) ||
        !bsp_can_is_id_valid(filter->mask, filter->id_type) ||
        ((filter->receive_fifo != BSP_CAN_RX_FIFO_0) &&
         (filter->receive_fifo != BSP_CAN_RX_FIFO_1))) return BSP_STATUS_OUT_OF_RANGE;
    return me->driver_ops->configure_filter(me->device_handle, filter);
}

bsp_status_t bsp_can_transmit(bsp_can_t *me, const bsp_can_frame_t *frame, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_can_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if (!bsp_can_is_frame_valid(frame)) return BSP_STATUS_OUT_OF_RANGE;
    return me->driver_ops->transmit(me->device_handle, frame, timeout_ms);
}

bsp_status_t bsp_can_receive(bsp_can_t *me, bsp_can_receive_fifo_t fifo, bsp_can_frame_t *frame)
{
    bsp_status_t status = bsp_can_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if ((frame == NULL) || ((fifo != BSP_CAN_RX_FIFO_0) && (fifo != BSP_CAN_RX_FIFO_1)))
        return BSP_STATUS_INVALID_ARGUMENT;
    status = me->driver_ops->receive(me->device_handle, fifo, frame);
    if (status != BSP_STATUS_OK) return status;
    return bsp_can_is_frame_valid(frame) ? BSP_STATUS_OK : BSP_STATUS_IO_ERROR;
}

bsp_status_t bsp_can_get_transmit_free_level(const bsp_can_t *me, uint32_t *free_level)
{
    const bsp_status_t status = bsp_can_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if (free_level == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return (me->driver_ops->get_tx_free_level != NULL)
               ? me->driver_ops->get_tx_free_level(me->device_handle, free_level)
               : BSP_STATUS_UNSUPPORTED;
}

void bsp_can_notify(bsp_can_t *me, bsp_event_t event, bsp_status_t status, size_t size)
{
    if ((me != NULL) && me->is_initialized && (me->callback != NULL))
        me->callback(event, status, size, me->user_context);
}
