#include "bsp_common.h"
bool bsp_transfer_mode_is_valid(bsp_transfer_mode_t mode){return mode==BSP_TRANSFER_MODE_BLOCKING||
    mode==BSP_TRANSFER_MODE_INTERRUPT||mode==BSP_TRANSFER_MODE_DMA;}
static bsp_error_t last_error;
void bsp_error_record(bsp_status_t code,const char *source,int detail){last_error=(bsp_error_t){code,source,detail,true};}
const bsp_error_t *bsp_error_read(void){return &last_error;}
