#include "bsp_timer.h"

static bsp_status_t validate(const bsp_timer_t *me) {
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return me->is_initialized ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}
bsp_status_t bsp_timer_init(bsp_timer_t *me,const bsp_timer_config_t *config){
    bsp_status_t status;
    if((me==NULL)||(config==NULL)||(config->device_handle==NULL)||(config->driver_ops==NULL)||
       (config->driver_ops->start==NULL)||(config->driver_ops->stop==NULL)||
       (config->driver_ops->set_counter==NULL)||(config->driver_ops->get_counter==NULL)||
       (config->driver_ops->set_period==NULL)||(config->driver_ops->get_period==NULL)||
       (config->driver_ops->get_frequency==NULL))return BSP_STATUS_INVALID_ARGUMENT;
    *me=(bsp_timer_t){0};
    if(config->driver_ops->init!=NULL){status=config->driver_ops->init(config->device_handle);
        if(status!=BSP_STATUS_OK)return status;}
    me->device_handle=config->device_handle; me->driver_ops=config->driver_ops;
    me->callback=config->callback; me->user_context=config->user_context; me->is_initialized=true;
    return BSP_STATUS_OK;
}
bsp_status_t bsp_timer_deinit(bsp_timer_t *me){
    bsp_status_t s=validate(me); if(s!=BSP_STATUS_OK)return s;
    s=me->driver_ops->deinit?me->driver_ops->deinit(me->device_handle):BSP_STATUS_OK;
    if(s==BSP_STATUS_OK)me->is_initialized=false; return s;
}
bsp_status_t bsp_timer_set_callback(bsp_timer_t *me,bsp_timer_callback_t callback,void *context){
    const bsp_status_t s=validate(me); if(s!=BSP_STATUS_OK)return s;
    me->callback=callback; me->user_context=context; return BSP_STATUS_OK;
}
#define ACTION(name,member) bsp_status_t name(bsp_timer_t *me){const bsp_status_t s=validate(me); \
    return s==BSP_STATUS_OK?me->driver_ops->member(me->device_handle):s;}
ACTION(bsp_timer_start,start)
ACTION(bsp_timer_stop,stop)
bsp_status_t bsp_timer_set_counter(bsp_timer_t *me,uint32_t value){const bsp_status_t s=validate(me);
    return s==BSP_STATUS_OK?me->driver_ops->set_counter(me->device_handle,value):s;}
bsp_status_t bsp_timer_reset(bsp_timer_t *me){return bsp_timer_set_counter(me,0U);}
bsp_status_t bsp_timer_get_counter(const bsp_timer_t *me,uint32_t *value){const bsp_status_t s=validate(me);
    if(s!=BSP_STATUS_OK)return s; return value?me->driver_ops->get_counter(me->device_handle,value):BSP_STATUS_INVALID_ARGUMENT;}
bsp_status_t bsp_timer_set_period(bsp_timer_t *me,uint32_t value){const bsp_status_t s=validate(me);
    if(s!=BSP_STATUS_OK)return s; return value?me->driver_ops->set_period(me->device_handle,value):BSP_STATUS_OUT_OF_RANGE;}
bsp_status_t bsp_timer_get_period(const bsp_timer_t *me,uint32_t *value){const bsp_status_t s=validate(me);
    if(s!=BSP_STATUS_OK)return s; return value?me->driver_ops->get_period(me->device_handle,value):BSP_STATUS_INVALID_ARGUMENT;}
bsp_status_t bsp_timer_get_frequency(const bsp_timer_t *me,uint32_t *value){const bsp_status_t s=validate(me);
    if(s!=BSP_STATUS_OK)return s; return value?me->driver_ops->get_frequency(me->device_handle,value):BSP_STATUS_INVALID_ARGUMENT;}
void bsp_timer_notify_elapsed(bsp_timer_t *me){if((me!=NULL)&&me->is_initialized&&(me->callback!=NULL))
    me->callback(me,me->user_context);}
