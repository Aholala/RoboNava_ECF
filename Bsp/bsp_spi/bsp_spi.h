/**
 * @file bsp_spi.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BSP SPI 外设抽象层公共接口
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 定义 SPI 驱动操作接口、实例结构体、配置结构体以及
 *       平台无关 SPI 主设备通信的公共 API，包括发送、接收和
 *       全双工交换。
 */

#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 平台特定 SPI 驱动操作虚表 */
typedef struct {
    bsp_status_t (*init)(void *);
    bsp_status_t (*deinit)(void *);
    bsp_status_t (*transmit)(void *, const uint8_t *, size_t, bsp_transfer_mode_t, uint32_t);
    bsp_status_t (*receive)(void *, uint8_t *, size_t, bsp_transfer_mode_t, uint32_t);
    bsp_status_t (*exchange)(void *, const uint8_t *, uint8_t *, size_t, bsp_transfer_mode_t,
                             uint32_t);
    bsp_status_t (*abort)(void *);
    bsp_status_t (*get_busy)(const void *, bool *);
} bsp_spi_driver_ops_t;

/** @brief SPI 外设实例（运行时状态） */
typedef struct bsp_spi {
    void *device_handle;                    /**< 平台 SPI 外设句柄 */
    const bsp_spi_driver_ops_t *driver_ops; /**< 平台驱动操作 */
    bsp_event_callback_t callback;          /**< 异步事件回调 */
    void *user_context;                     /**< 回调用户上下文 */
    bool is_initialized;                    /**< 初始化标志 */
} bsp_spi_t;

/** @brief SPI 外设配置（初始化参数） */
typedef struct {
    void *device_handle;                    /**< 平台 SPI 外设句柄 */
    const bsp_spi_driver_ops_t *driver_ops; /**< 平台驱动操作 */
    bsp_event_callback_t callback;          /**< 异步事件回调 */
    void *user_context;                     /**< 回调用户上下文 */
} bsp_spi_config_t;

/* ======================== 生命周期 ======================== */
bsp_status_t bsp_spi_init(bsp_spi_t *me, const bsp_spi_config_t *config);
bsp_status_t bsp_spi_deinit(bsp_spi_t *me);
bsp_status_t bsp_spi_set_callback(bsp_spi_t *me, bsp_event_callback_t callback,
                                  void *user_context);

/* ======================== 数据传输 ======================== */
bsp_status_t bsp_spi_transmit(bsp_spi_t *me, const uint8_t *data, size_t size,
                              bsp_transfer_mode_t mode, uint32_t timeout_ms);
bsp_status_t bsp_spi_receive(bsp_spi_t *me, uint8_t *data, size_t size,
                             bsp_transfer_mode_t mode, uint32_t timeout_ms);
bsp_status_t bsp_spi_exchange(bsp_spi_t *me, const uint8_t *tx, uint8_t *rx, size_t size,
                              bsp_transfer_mode_t mode, uint32_t timeout_ms);

/* ======================== 设备状态 ======================== */
bsp_status_t bsp_spi_abort(bsp_spi_t *me);
bsp_status_t bsp_spi_get_busy(const bsp_spi_t *me, bool *is_busy);

/* ======================== 内部函数 ======================== */
void bsp_spi_notify(bsp_spi_t *me, bsp_event_t event, bsp_status_t status, size_t transferred_size);

#ifdef __cplusplus
}
#endif
#endif
