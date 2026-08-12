#ifndef BSP_USB_VCP_H
#define BSP_USB_VCP_H
#include "bsp_common.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct { bsp_status_t (*init)(void *); bsp_status_t (*deinit)(void *);
    bsp_status_t (*transmit)(void *,const uint8_t *,size_t,uint32_t);
    bsp_status_t (*receive)(void *,uint8_t *,size_t); bsp_status_t (*abort)(void *);
    bsp_status_t (*get_connected)(const void *,bool *); bsp_status_t (*get_busy)(const void *,bool *);
} bsp_usb_vcp_driver_ops_t;
typedef struct bsp_usb_vcp { void *device_handle; const bsp_usb_vcp_driver_ops_t *driver_ops;
    bsp_event_callback_t callback; void *user_context; bool is_initialized; } bsp_usb_vcp_t;
typedef struct { void *device_handle; const bsp_usb_vcp_driver_ops_t *driver_ops;
    bsp_event_callback_t callback; void *user_context; } bsp_usb_vcp_config_t;
bsp_status_t bsp_usb_vcp_init(bsp_usb_vcp_t *,const bsp_usb_vcp_config_t *);
bsp_status_t bsp_usb_vcp_deinit(bsp_usb_vcp_t *);
bsp_status_t bsp_usb_vcp_set_callback(bsp_usb_vcp_t *,bsp_event_callback_t,void *);
bsp_status_t bsp_usb_vcp_transmit(bsp_usb_vcp_t *,const uint8_t *,size_t,uint32_t);
bsp_status_t bsp_usb_vcp_receive(bsp_usb_vcp_t *,uint8_t *,size_t);
bsp_status_t bsp_usb_vcp_abort(bsp_usb_vcp_t *);
bsp_status_t bsp_usb_vcp_get_connected(const bsp_usb_vcp_t *,bool *);
bsp_status_t bsp_usb_vcp_get_busy(const bsp_usb_vcp_t *,bool *);
void bsp_usb_vcp_notify(bsp_usb_vcp_t *,bsp_event_t,bsp_status_t,size_t);
#ifdef __cplusplus
}
#endif
#endif
