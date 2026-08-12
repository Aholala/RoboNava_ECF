#ifndef BSP_COMMON_H
#define BSP_COMMON_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef enum {BSP_STATUS_OK=0,BSP_STATUS_INVALID_ARGUMENT,BSP_STATUS_OUT_OF_RANGE,
    BSP_STATUS_NOT_INITIALIZED,BSP_STATUS_BUSY,BSP_STATUS_TIMEOUT,BSP_STATUS_IO_ERROR,
    BSP_STATUS_NO_RESOURCE,BSP_STATUS_UNSUPPORTED} bsp_status_t;
#define BSP_STATIC_ASSERT_SUPER_FIRST(type) _Static_assert(offsetof(type,super)==0U,#type " must place super first")
typedef enum {BSP_TRANSFER_MODE_BLOCKING=0,BSP_TRANSFER_MODE_INTERRUPT,BSP_TRANSFER_MODE_DMA} bsp_transfer_mode_t;
typedef enum {BSP_EVENT_TRANSMIT_COMPLETE=0,BSP_EVENT_RECEIVE_COMPLETE,BSP_EVENT_TRANSFER_COMPLETE,
    BSP_EVENT_RECEIVE_PENDING,BSP_EVENT_ABORT_COMPLETE,BSP_EVENT_ERROR} bsp_event_t;
typedef void(*bsp_event_callback_t)(bsp_event_t,bsp_status_t,size_t,void*);
bool bsp_transfer_mode_is_valid(bsp_transfer_mode_t mode);
typedef struct {bsp_status_t code;const char *source;int detail;bool is_valid;} bsp_error_t;
void bsp_error_record(bsp_status_t code,const char *source,int detail);
const bsp_error_t *bsp_error_read(void);
#ifdef __cplusplus
}
#endif
#endif
