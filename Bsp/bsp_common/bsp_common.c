/**
 * @file bsp_common.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BSP 通用基础设施实现
 * @note 提供 BSP 设备生命周期、传输模式校验和错误寄存器。
 * @version 2.0
 * @date 2026-08-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "bsp_common.h" // 包含公共头文件

#include <stddef.h> // 提供 NULL 和 size_t

bsp_status_t bsp_device_init(bsp_device_t *const me, const bsp_device_ops_t *const vptr,
                             void *const device_handle)
{
    if ((me == NULL) || (vptr == NULL) || (device_handle == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    me->vptr = vptr;
    me->device_handle = device_handle;
    me->object_magic = BSP_DEVICE_OBJECT_MAGIC;
    me->is_initialized = true;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_device_deinit(bsp_device_t *const me)
{
    bsp_status_t status;

    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!bsp_device_is_initialized(me))
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    status = (me->vptr->deinit != NULL) ? me->vptr->deinit(me) : BSP_STATUS_OK;
    if (status == BSP_STATUS_OK)
    {
        me->vptr = NULL;
        me->device_handle = NULL;
        me->object_magic = 0U;
        me->is_initialized = false;
    }
    return status;
}

bool bsp_device_is_initialized(const bsp_device_t *const me)
{
    return (me != NULL) && (me->object_magic == BSP_DEVICE_OBJECT_MAGIC) && me->is_initialized &&
           (me->vptr != NULL) && (me->device_handle != NULL);
}

/**
 * @brief 获取设备句柄（只读）
 */
void *bsp_device_get_handle(const bsp_device_t *const me)
{
    return bsp_device_is_initialized(me) ? me->device_handle : NULL;
}

/* ========================================================================
 * 传输模式校验
 * ======================================================================== */

/**
 * @brief 校验传输模式是否合法
 */
bool bsp_transfer_mode_is_valid(bsp_transfer_mode_t transfer_mode)
{
    return (transfer_mode == BSP_TRANSFER_MODE_BLOCKING) ||
           (transfer_mode == BSP_TRANSFER_MODE_INTERRUPT) ||
           (transfer_mode == BSP_TRANSFER_MODE_DMA);
}

/* ========================================================================
 * 全局错误寄存器
 * ======================================================================== */

static bsp_error_t bsp_error_last;

void bsp_error_record(bsp_status_t code, const char *source, int detail)
{
    bsp_error_last.code = code;
    bsp_error_last.source = source;
    bsp_error_last.detail = detail;
    bsp_error_last.is_valid = true;
}

const bsp_error_t *bsp_error_read(void)
{
    return &bsp_error_last;
}
