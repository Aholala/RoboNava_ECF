#include "bsp_encoder.h"
#include <limits.h>

static bsp_status_t validate(const bsp_encoder_t *me) {
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return me->is_initialized ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}
bsp_status_t bsp_encoder_init(bsp_encoder_t *me, const bsp_encoder_config_t *config) {
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || ((config->counter_modulus != 0U) &&
        (config->counter_modulus < 2U)) || (config->driver_ops->start == NULL) ||
        (config->driver_ops->stop == NULL) || (config->driver_ops->set_count == NULL) ||
        (config->driver_ops->get_count == NULL) || (config->driver_ops->get_direction == NULL))
        return BSP_STATUS_INVALID_ARGUMENT;
    *me = (bsp_encoder_t){0};
    if (config->driver_ops->init != NULL) {
        status = config->driver_ops->init(config->device_handle);
        if (status != BSP_STATUS_OK) return status;
    }
    me->device_handle = config->device_handle; me->driver_ops = config->driver_ops;
    me->counter_modulus = config->counter_modulus; me->is_initialized = true;
    return BSP_STATUS_OK;
}
bsp_status_t bsp_encoder_deinit(bsp_encoder_t *me) {
    bsp_status_t status = validate(me); if (status != BSP_STATUS_OK) return status;
    status = me->driver_ops->deinit ? me->driver_ops->deinit(me->device_handle) : BSP_STATUS_OK;
    if (status == BSP_STATUS_OK) me->is_initialized = false; return status;
}
#define ACTION(name, member) bsp_status_t name(bsp_encoder_t *me) { \
    const bsp_status_t s=validate(me); return s==BSP_STATUS_OK ? me->driver_ops->member(me->device_handle):s; }
ACTION(bsp_encoder_start, start)
ACTION(bsp_encoder_stop, stop)
bsp_status_t bsp_encoder_set_count(bsp_encoder_t *me, int32_t count) {
    const bsp_status_t s=validate(me); if(s!=BSP_STATUS_OK)return s;
    const bsp_status_t result=me->driver_ops->set_count(me->device_handle,count);
    if(result==BSP_STATUS_OK)me->previous_count=count; return result;
}
bsp_status_t bsp_encoder_reset(bsp_encoder_t *me) { return bsp_encoder_set_count(me,0); }
bsp_status_t bsp_encoder_get_count(const bsp_encoder_t *me, int32_t *count) {
    const bsp_status_t s=validate(me); if(s!=BSP_STATUS_OK)return s;
    return count ? me->driver_ops->get_count(me->device_handle,count):BSP_STATUS_INVALID_ARGUMENT;
}
bsp_status_t bsp_encoder_get_delta(bsp_encoder_t *me, int32_t *delta) {
    int32_t current; int64_t difference; if(delta==NULL)return BSP_STATUS_INVALID_ARGUMENT;
    const bsp_status_t s=bsp_encoder_get_count(me,&current); if(s!=BSP_STATUS_OK)return s;
    difference=(int64_t)current-me->previous_count;
    if(me->counter_modulus>0U){const int64_t m=me->counter_modulus,h=m/2;
        if(difference>h)difference-=m; else if(difference < -h)difference+=m;}
    if((difference>INT32_MAX)||(difference<INT32_MIN))return BSP_STATUS_OUT_OF_RANGE;
    *delta=(int32_t)difference; me->previous_count=current; return BSP_STATUS_OK;
}
bsp_status_t bsp_encoder_get_direction(const bsp_encoder_t *me,bsp_encoder_direction_t *direction){
    const bsp_status_t s=validate(me); if(s!=BSP_STATUS_OK)return s;
    return direction ? me->driver_ops->get_direction(me->device_handle,direction):BSP_STATUS_INVALID_ARGUMENT;
}
