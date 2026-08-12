/**
 * @file bsp_i2c.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BSP I2C 外设抽象层公共接口
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 定义 I2C 驱动操作接口、实例结构体、配置结构体以及
 *       平台无关 I2C 主设备通信的公共 API。
 */

#ifndef BSP_I2C_H
#define BSP_I2C_H
#include "bsp_common.h"
#ifdef __cplusplus
extern "C" {
#endif

/** @brief I2C 寄存器地址宽度选择 */
typedef enum { BSP_I2C_MEMORY_ADDRESS_8_BIT=1, BSP_I2C_MEMORY_ADDRESS_16_BIT=2 }
    bsp_i2c_memory_address_size_t;

/** @brief 平台特定 I2C 驱动操作虚表 */
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

/** @brief I2C 外设实例（运行时状态） */
typedef struct bsp_i2c { void *device_handle; const bsp_i2c_driver_ops_t *driver_ops;
    bsp_event_callback_t callback; void *user_context; bool is_initialized; } bsp_i2c_t;

/** @brief I2C 外设配置（初始化参数） */
typedef struct { void *device_handle; const bsp_i2c_driver_ops_t *driver_ops;
    bsp_event_callback_t callback; void *user_context; } bsp_i2c_config_t;

/* ======================== 生命周期 ======================== */
bsp_status_t bsp_i2c_init(bsp_i2c_t *,const bsp_i2c_config_t *); bsp_status_t bsp_i2c_deinit(bsp_i2c_t *);
bsp_status_t bsp_i2c_set_callback(bsp_i2c_t *,bsp_event_callback_t,void *);

/* ======================== 基础传输 ======================== */
bsp_status_t bsp_i2c_transmit(bsp_i2c_t *,uint16_t,const uint8_t *,size_t,bsp_transfer_mode_t,uint32_t);
bsp_status_t bsp_i2c_receive(bsp_i2c_t *,uint16_t,uint8_t *,size_t,bsp_transfer_mode_t,uint32_t);

/* ======================== 存储器访问 ======================== */
bsp_status_t bsp_i2c_memory_write(bsp_i2c_t *,uint16_t,uint16_t,bsp_i2c_memory_address_size_t,
                                  const uint8_t *,size_t,bsp_transfer_mode_t,uint32_t);
bsp_status_t bsp_i2c_memory_read(bsp_i2c_t *,uint16_t,uint16_t,bsp_i2c_memory_address_size_t,
                                 uint8_t *,size_t,bsp_transfer_mode_t,uint32_t);

/* ======================== 设备状态 ======================== */
bsp_status_t bsp_i2c_is_device_ready(bsp_i2c_t *,uint16_t,uint32_t,uint32_t);
bsp_status_t bsp_i2c_abort(bsp_i2c_t *,uint16_t); bsp_status_t bsp_i2c_get_busy(const bsp_i2c_t *,bool *);

/* ======================== 内部函数 ======================== */
void bsp_i2c_notify(bsp_i2c_t *,bsp_event_t,bsp_status_t,size_t);

#ifdef __cplusplus
}
#endif
#endif
