#ifndef BSP_I2C_H
#define BSP_I2C_H
#include "bsp_common.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef enum { BSP_I2C_MEMORY_ADDRESS_8_BIT=1, BSP_I2C_MEMORY_ADDRESS_16_BIT=2 }
    bsp_i2c_memory_address_size_t;
typedef struct {
    bsp_status_t (*init)(void *); bsp_status_t (*deinit)(void *);
    bsp_status_t (*transmit)(void *,uint16_t,const uint8_t *,size_t,bsp_transfer_mode_t,uint32_t);
    bsp_status_t (*receive)(void *,uint16_t,uint8_t *,size_t,bsp_transfer_mode_t,uint32_t);
    bsp_status_t (*memory_write)(void *,uint16_t,uint16_t,bsp_i2c_memory_address_size_t,
                                 const uint8_t *,size_t,bsp_transfer_mode_t,uint32_t);
    bsp_status_t (*memory_read)(void *,uint16_t,uint16_t,bsp_i2c_memory_address_size_t,
                                uint8_t *,size_t,bsp_transfer_mode_t,uint32_t);
    bsp_status_t (*is_device_ready)(void *,uint16_t,uint32_t,uint32_t);
    bsp_status_t (*abort)(void *,uint16_t); bsp_status_t (*get_busy)(const void *,bool *);
} bsp_i2c_driver_ops_t;
typedef struct bsp_i2c { void *device_handle; const bsp_i2c_driver_ops_t *driver_ops;
    bsp_event_callback_t callback; void *user_context; bool is_initialized; } bsp_i2c_t;
typedef struct { void *device_handle; const bsp_i2c_driver_ops_t *driver_ops;
    bsp_event_callback_t callback; void *user_context; } bsp_i2c_config_t;
bsp_status_t bsp_i2c_init(bsp_i2c_t *,const bsp_i2c_config_t *); bsp_status_t bsp_i2c_deinit(bsp_i2c_t *);
bsp_status_t bsp_i2c_set_callback(bsp_i2c_t *,bsp_event_callback_t,void *);
bsp_status_t bsp_i2c_transmit(bsp_i2c_t *,uint16_t,const uint8_t *,size_t,bsp_transfer_mode_t,uint32_t);
bsp_status_t bsp_i2c_receive(bsp_i2c_t *,uint16_t,uint8_t *,size_t,bsp_transfer_mode_t,uint32_t);
bsp_status_t bsp_i2c_memory_write(bsp_i2c_t *,uint16_t,uint16_t,bsp_i2c_memory_address_size_t,
                                  const uint8_t *,size_t,bsp_transfer_mode_t,uint32_t);
bsp_status_t bsp_i2c_memory_read(bsp_i2c_t *,uint16_t,uint16_t,bsp_i2c_memory_address_size_t,
                                 uint8_t *,size_t,bsp_transfer_mode_t,uint32_t);
bsp_status_t bsp_i2c_is_device_ready(bsp_i2c_t *,uint16_t,uint32_t,uint32_t);
bsp_status_t bsp_i2c_abort(bsp_i2c_t *,uint16_t); bsp_status_t bsp_i2c_get_busy(const bsp_i2c_t *,bool *);
void bsp_i2c_notify(bsp_i2c_t *,bsp_event_t,bsp_status_t,size_t);
#ifdef __cplusplus
}
#endif
#endif
