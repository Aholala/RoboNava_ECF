#ifndef BSP_USART_H
#define BSP_USART_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*bsp_usart_double_buffer_callback_t)(uint8_t, size_t, void *);

typedef struct {
    bsp_status_t (*init)(void *);
    bsp_status_t (*deinit)(void *);
    bsp_status_t (*transmit)(void *, const uint8_t *, size_t, bsp_transfer_mode_t, uint32_t);
    bsp_status_t (*receive)(void *, uint8_t *, size_t, bsp_transfer_mode_t, uint32_t);
    bsp_status_t (*receive_to_idle)(void *, uint8_t *, size_t, bsp_transfer_mode_t, uint32_t);
    bsp_status_t (*receive_to_idle_double_buffer)(void *, uint8_t *, uint8_t *, size_t);
    bsp_status_t (*abort)(void *);
    bsp_status_t (*get_busy)(const void *, bool *);
} bsp_usart_driver_ops_t;

typedef struct bsp_usart {
    void *device_handle;
    const bsp_usart_driver_ops_t *driver_ops;
    bsp_event_callback_t callback;
    void *user_context;
    bsp_usart_double_buffer_callback_t double_buffer_callback;
    void *double_buffer_user_context;
    bool is_initialized;
} bsp_usart_t;

typedef struct {
    void *device_handle;
    const bsp_usart_driver_ops_t *driver_ops;
    bsp_event_callback_t callback;
    void *user_context;
} bsp_usart_config_t;

bsp_status_t bsp_usart_init(bsp_usart_t *me, const bsp_usart_config_t *config);
bsp_status_t bsp_usart_deinit(bsp_usart_t *me);
bsp_status_t bsp_usart_set_callback(bsp_usart_t *me, bsp_event_callback_t callback, void *context);
bsp_status_t bsp_usart_transmit(bsp_usart_t *me, const uint8_t *data, size_t size,
                                bsp_transfer_mode_t mode, uint32_t timeout_ms);
bsp_status_t bsp_usart_receive(bsp_usart_t *me, uint8_t *data, size_t size,
                               bsp_transfer_mode_t mode, uint32_t timeout_ms);
bsp_status_t bsp_usart_receive_to_idle(bsp_usart_t *me, uint8_t *data, size_t capacity,
                                       bsp_transfer_mode_t mode, uint32_t timeout_ms);
bsp_status_t bsp_usart_receive_to_idle_double_buffer(
    bsp_usart_t *me, uint8_t *first, uint8_t *second, size_t capacity,
    bsp_usart_double_buffer_callback_t callback, void *context);
bsp_status_t bsp_usart_abort(bsp_usart_t *me);
bsp_status_t bsp_usart_get_busy(const bsp_usart_t *me, bool *is_busy);
void bsp_usart_notify(bsp_usart_t *me, bsp_event_t event, bsp_status_t status, size_t size);
void bsp_usart_notify_double_buffer(bsp_usart_t *me, uint8_t buffer_index, size_t received_size);

#ifdef __cplusplus
}
#endif
#endif
