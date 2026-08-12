#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H
#include "bsp_can.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef enum {BSP_FDCAN_FORMAT_CLASSIC=0,BSP_FDCAN_FORMAT_FD_NO_BRS,BSP_FDCAN_FORMAT_FD_BRS} bsp_fdcan_format_t;
typedef struct {uint32_t identifier;bsp_can_id_type_t id_type;bsp_can_frame_type_t frame_type;
    bsp_fdcan_format_t format;uint8_t data_length;uint8_t data[64];} bsp_fdcan_frame_t;
typedef struct {bool is_bus_off;bool is_error_passive;bool has_warning;uint8_t transmit_error_count;
    uint8_t receive_error_count;uint32_t last_error_code;} bsp_fdcan_protocol_status_t;
typedef struct {bsp_status_t(*init)(void*);bsp_status_t(*deinit)(void*);bsp_status_t(*start)(void*);
    bsp_status_t(*stop)(void*);bsp_status_t(*configure_filter)(void*,const bsp_can_filter_t*);
    bsp_status_t(*transmit)(void*,const bsp_fdcan_frame_t*,uint32_t);
    bsp_status_t(*receive)(void*,bsp_can_receive_fifo_t,bsp_fdcan_frame_t*);
    bsp_status_t(*get_protocol_status)(const void*,bsp_fdcan_protocol_status_t*);
    bsp_status_t(*get_transmit_free_level)(const void*,uint32_t*);} bsp_fdcan_driver_ops_t;
typedef struct bsp_fdcan {void *device_handle;const bsp_fdcan_driver_ops_t *driver_ops;
    bsp_event_callback_t callback;void *user_context;bool is_initialized;} bsp_fdcan_t;
typedef struct {void *device_handle;const bsp_fdcan_driver_ops_t *driver_ops;
    bsp_event_callback_t callback;void *user_context;} bsp_fdcan_config_t;
bsp_status_t bsp_fdcan_init(bsp_fdcan_t*,const bsp_fdcan_config_t*);bsp_status_t bsp_fdcan_deinit(bsp_fdcan_t*);
bsp_status_t bsp_fdcan_set_callback(bsp_fdcan_t*,bsp_event_callback_t,void*);
bsp_status_t bsp_fdcan_start(bsp_fdcan_t*);bsp_status_t bsp_fdcan_stop(bsp_fdcan_t*);
bsp_status_t bsp_fdcan_configure_filter(bsp_fdcan_t*,const bsp_can_filter_t*);
bsp_status_t bsp_fdcan_transmit(bsp_fdcan_t*,const bsp_fdcan_frame_t*,uint32_t);
bsp_status_t bsp_fdcan_receive(bsp_fdcan_t*,bsp_can_receive_fifo_t,bsp_fdcan_frame_t*);
bsp_status_t bsp_fdcan_get_protocol_status(const bsp_fdcan_t*,bsp_fdcan_protocol_status_t*);
bsp_status_t bsp_fdcan_get_transmit_free_level(const bsp_fdcan_t*,uint32_t*);
void bsp_fdcan_notify(bsp_fdcan_t*,bsp_event_t,bsp_status_t,size_t);
#ifdef __cplusplus
}
#endif
#endif
