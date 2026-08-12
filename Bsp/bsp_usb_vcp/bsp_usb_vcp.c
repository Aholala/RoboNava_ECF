#include "bsp_usb_vcp.h"
static bsp_status_t validate(const bsp_usb_vcp_t *me){if(me==NULL)return BSP_STATUS_INVALID_ARGUMENT;
    return me->is_initialized?BSP_STATUS_OK:BSP_STATUS_NOT_INITIALIZED;}
bsp_status_t bsp_usb_vcp_init(bsp_usb_vcp_t *me,const bsp_usb_vcp_config_t *config){bsp_status_t status;
    if((me==NULL)||(config==NULL)||(config->device_handle==NULL)||(config->driver_ops==NULL)||
       (config->driver_ops->transmit==NULL)||(config->driver_ops->receive==NULL))return BSP_STATUS_INVALID_ARGUMENT;
    *me=(bsp_usb_vcp_t){0};if(config->driver_ops->init!=NULL){status=config->driver_ops->init(config->device_handle);
        if(status!=BSP_STATUS_OK)return status;}me->device_handle=config->device_handle;
    me->driver_ops=config->driver_ops;me->callback=config->callback;me->user_context=config->user_context;
    me->is_initialized=true;return BSP_STATUS_OK;}
bsp_status_t bsp_usb_vcp_deinit(bsp_usb_vcp_t *me){bsp_status_t s=validate(me);if(s!=BSP_STATUS_OK)return s;
    s=me->driver_ops->deinit?me->driver_ops->deinit(me->device_handle):BSP_STATUS_OK;
    if(s==BSP_STATUS_OK)me->is_initialized=false;return s;}
bsp_status_t bsp_usb_vcp_set_callback(bsp_usb_vcp_t *me,bsp_event_callback_t callback,void *context){
    const bsp_status_t s=validate(me);if(s!=BSP_STATUS_OK)return s;me->callback=callback;
    me->user_context=context;return BSP_STATUS_OK;}
bsp_status_t bsp_usb_vcp_transmit(bsp_usb_vcp_t *me,const uint8_t *data,size_t size,uint32_t timeout){
    const bsp_status_t s=validate(me);if(s!=BSP_STATUS_OK)return s;
    if((data==NULL)||(size==0U))return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->transmit(me->device_handle,data,size,timeout);}
bsp_status_t bsp_usb_vcp_receive(bsp_usb_vcp_t *me,uint8_t *data,size_t capacity){
    const bsp_status_t s=validate(me);if(s!=BSP_STATUS_OK)return s;
    if((data==NULL)||(capacity==0U))return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->receive(me->device_handle,data,capacity);}
bsp_status_t bsp_usb_vcp_abort(bsp_usb_vcp_t *me){const bsp_status_t s=validate(me);
    if(s!=BSP_STATUS_OK)return s;return me->driver_ops->abort?me->driver_ops->abort(me->device_handle)
        :BSP_STATUS_UNSUPPORTED;}
static bsp_status_t query(const bsp_usb_vcp_t *me,bool *value,
    bsp_status_t (*operation)(const void *,bool *)){const bsp_status_t s=validate(me);
    if(s!=BSP_STATUS_OK)return s;if(value==NULL)return BSP_STATUS_INVALID_ARGUMENT;
    return operation?operation(me->device_handle,value):BSP_STATUS_UNSUPPORTED;}
bsp_status_t bsp_usb_vcp_get_connected(const bsp_usb_vcp_t *me,bool *connected){
    return query(me,connected,(me!=NULL&&me->driver_ops!=NULL)?me->driver_ops->get_connected:NULL);}
bsp_status_t bsp_usb_vcp_get_busy(const bsp_usb_vcp_t *me,bool *busy){
    return query(me,busy,(me!=NULL&&me->driver_ops!=NULL)?me->driver_ops->get_busy:NULL);}
void bsp_usb_vcp_notify(bsp_usb_vcp_t *me,bsp_event_t event,bsp_status_t status,size_t size){
    if((me!=NULL)&&me->is_initialized&&(me->callback!=NULL))me->callback(event,status,size,me->user_context);}
