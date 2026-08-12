#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

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

typedef struct bsp_spi {
    void *device_handle;
    const bsp_spi_driver_ops_t *driver_ops;
    bsp_event_callback_t callback;
    void *user_context;
    bool is_initialized;
} bsp_spi_t;

typedef struct {
    void *device_handle;
    const bsp_spi_driver_ops_t *driver_ops;
    bsp_event_callback_t callback;
    void *user_context;
} bsp_spi_config_t;

bsp_status_t bsp_spi_init(bsp_spi_t *me, const bsp_spi_config_t *config);
bsp_status_t bsp_spi_deinit(bsp_spi_t *me);
bsp_status_t bsp_spi_set_callback(bsp_spi_t *me, bsp_event_callback_t callback,
                                  void *user_context);
bsp_status_t bsp_spi_transmit(bsp_spi_t *me, const uint8_t *data, size_t size,
                              bsp_transfer_mode_t mode, uint32_t timeout_ms);
bsp_status_t bsp_spi_receive(bsp_spi_t *me, uint8_t *data, size_t size,
                             bsp_transfer_mode_t mode, uint32_t timeout_ms);
bsp_status_t bsp_spi_exchange(bsp_spi_t *me, const uint8_t *tx, uint8_t *rx, size_t size,
                              bsp_transfer_mode_t mode, uint32_t timeout_ms);
bsp_status_t bsp_spi_abort(bsp_spi_t *me);
bsp_status_t bsp_spi_get_busy(const bsp_spi_t *me, bool *is_busy);
void bsp_spi_notify(bsp_spi_t *me, bsp_event_t event, bsp_status_t status, size_t transferred_size);

#ifdef __cplusplus
}
#endif
#endif
