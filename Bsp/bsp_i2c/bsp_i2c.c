#include "bsp_i2c.h"
static bsp_status_t validate(const bsp_i2c_t *me){if(me==NULL)return BSP_STATUS_INVALID_ARGUMENT;
    return me->is_initialized?BSP_STATUS_OK:BSP_STATUS_NOT_INITIALIZED;}
static bool address_valid(uint16_t address){return address<=0x7FU;}
static bool transfer_valid(uint16_t address,const void *data,size_t size,bsp_transfer_mode_t mode){
    return address_valid(address)&&(data!=NULL)&&(size>0U)&&bsp_transfer_mode_is_valid(mode);}
bsp_status_t bsp_i2c_init(bsp_i2c_t *me,const bsp_i2c_config_t *config){bsp_status_t status;
    if((me==NULL)||(config==NULL)||(config->device_handle==NULL)||(config->driver_ops==NULL)||
       (config->driver_ops->transmit==NULL)||(config->driver_ops->receive==NULL))return BSP_STATUS_INVALID_ARGUMENT;
    *me=(bsp_i2c_t){0}; if(config->driver_ops->init!=NULL){status=config->driver_ops->init(config->device_handle);
        if(status!=BSP_STATUS_OK)return status;} me->device_handle=config->device_handle;
    me->driver_ops=config->driver_ops; me->callback=config->callback; me->user_context=config->user_context;
    me->is_initialized=true; return BSP_STATUS_OK;}
bsp_status_t bsp_i2c_deinit(bsp_i2c_t *me){bsp_status_t s=validate(me);if(s!=BSP_STATUS_OK)return s;
    s=me->driver_ops->deinit?me->driver_ops->deinit(me->device_handle):BSP_STATUS_OK;
    if(s==BSP_STATUS_OK)me->is_initialized=false;return s;}
bsp_status_t bsp_i2c_set_callback(bsp_i2c_t *me,bsp_event_callback_t callback,void *context){
    const bsp_status_t s=validate(me);if(s!=BSP_STATUS_OK)return s;me->callback=callback;
    me->user_context=context;return BSP_STATUS_OK;}
bsp_status_t bsp_i2c_transmit(bsp_i2c_t *me,uint16_t address,const uint8_t *data,size_t size,
                              bsp_transfer_mode_t mode,uint32_t timeout){const bsp_status_t s=validate(me);
    if(s!=BSP_STATUS_OK)return s;if(!transfer_valid(address,data,size,mode))return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->transmit(me->device_handle,address,data,size,mode,timeout);}
bsp_status_t bsp_i2c_receive(bsp_i2c_t *me,uint16_t address,uint8_t *data,size_t size,
                             bsp_transfer_mode_t mode,uint32_t timeout){const bsp_status_t s=validate(me);
    if(s!=BSP_STATUS_OK)return s;if(!transfer_valid(address,data,size,mode))return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->receive(me->device_handle,address,data,size,mode,timeout);}
static bool memory_valid(uint16_t address,bsp_i2c_memory_address_size_t address_size,const void *data,
                         size_t size,bsp_transfer_mode_t mode){return transfer_valid(address,data,size,mode)&&
    ((address_size==BSP_I2C_MEMORY_ADDRESS_8_BIT)||(address_size==BSP_I2C_MEMORY_ADDRESS_16_BIT));}
bsp_status_t bsp_i2c_memory_write(bsp_i2c_t *me,uint16_t address,uint16_t memory,
    bsp_i2c_memory_address_size_t address_size,const uint8_t *data,size_t size,bsp_transfer_mode_t mode,
    uint32_t timeout){const bsp_status_t s=validate(me);if(s!=BSP_STATUS_OK)return s;
    if(!memory_valid(address,address_size,data,size,mode))return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->memory_write?me->driver_ops->memory_write(me->device_handle,address,memory,
        address_size,data,size,mode,timeout):BSP_STATUS_UNSUPPORTED;}
bsp_status_t bsp_i2c_memory_read(bsp_i2c_t *me,uint16_t address,uint16_t memory,
    bsp_i2c_memory_address_size_t address_size,uint8_t *data,size_t size,bsp_transfer_mode_t mode,
    uint32_t timeout){const bsp_status_t s=validate(me);if(s!=BSP_STATUS_OK)return s;
    if(!memory_valid(address,address_size,data,size,mode))return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->memory_read?me->driver_ops->memory_read(me->device_handle,address,memory,
        address_size,data,size,mode,timeout):BSP_STATUS_UNSUPPORTED;}
bsp_status_t bsp_i2c_is_device_ready(bsp_i2c_t *me,uint16_t address,uint32_t trials,uint32_t timeout){
    const bsp_status_t s=validate(me);if(s!=BSP_STATUS_OK)return s;
    if(!address_valid(address)||(trials==0U))return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->is_device_ready?me->driver_ops->is_device_ready(me->device_handle,address,trials,timeout)
        :BSP_STATUS_UNSUPPORTED;}
bsp_status_t bsp_i2c_abort(bsp_i2c_t *me,uint16_t address){const bsp_status_t s=validate(me);
    if(s!=BSP_STATUS_OK)return s;if(!address_valid(address))return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->abort?me->driver_ops->abort(me->device_handle,address):BSP_STATUS_UNSUPPORTED;}
bsp_status_t bsp_i2c_get_busy(const bsp_i2c_t *me,bool *busy){const bsp_status_t s=validate(me);
    if(s!=BSP_STATUS_OK)return s;if(busy==NULL)return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->get_busy?me->driver_ops->get_busy(me->device_handle,busy):BSP_STATUS_UNSUPPORTED;}
void bsp_i2c_notify(bsp_i2c_t *me,bsp_event_t event,bsp_status_t status,size_t size){
    if((me!=NULL)&&me->is_initialized&&(me->callback!=NULL))me->callback(event,status,size,me->user_context);}
