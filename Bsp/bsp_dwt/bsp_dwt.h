/**
 * @file bsp_dwt.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief DWT 周期计数的硬件无关换算与时间差实现。
 * @version 1.0
 * @date 2026-06-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BSP_DWT_H
#define BSP_DWT_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief DWT 时间点（由周期计数表示） */
    typedef struct
    {
        uint32_t cycle_count; /**< 周期计数值 */
    } bsp_dwt_time_point_t;

    /** @brief DWT 底层驱动操作集 */
    typedef struct
    {
        bsp_status_t (*init)(void *device_handle);                                  /**< 初始化 DWT */
        bsp_status_t (*reset)(void *device_handle);                                /**< 复位周期计数器 */
        bsp_status_t (*get_cycle_count)(const void *device_handle, uint32_t *cycle_count); /**< 读取当前周期计数 */
        bsp_status_t (*get_frequency_hz)(const void *device_handle, uint32_t *frequency_hz); /**< 获取计数器频率 */
    } bsp_dwt_driver_ops_t;

    /** @brief DWT 初始化配置 */
    typedef struct
    {
        void *device_handle;                /**< 硬件句柄 */
        const bsp_dwt_driver_ops_t *driver_ops; /**< 驱动操作集 */
    } bsp_dwt_config_t;

    /** @brief DWT 实例对象 */
    typedef struct
    {
        void *device_handle;                /**< 硬件句柄 */
        const bsp_dwt_driver_ops_t *driver_ops; /**< 驱动操作集 */
        bool is_initialized;                /**< 初始化标志 */
    } bsp_dwt_t;

    /* ======================== 公共 API ======================== */

    bsp_status_t bsp_dwt_init(bsp_dwt_t *me, const bsp_dwt_config_t *config);
    bool bsp_dwt_is_initialized(const bsp_dwt_t *me);
    bsp_status_t bsp_dwt_reset(bsp_dwt_t *me);
    bsp_status_t bsp_dwt_get_cycle_count(const bsp_dwt_t *me, uint32_t *cycle_count);
    bsp_status_t bsp_dwt_get_frequency_hz(const bsp_dwt_t *me, uint32_t *frequency_hz);
    bsp_status_t bsp_dwt_now(const bsp_dwt_t *me, bsp_dwt_time_point_t *time_point);
    bsp_status_t bsp_dwt_elapsed_cycles(const bsp_dwt_t *me, bsp_dwt_time_point_t start_time,
                                        uint32_t *elapsed_cycles);
    bsp_status_t bsp_dwt_cycles_to_us(const bsp_dwt_t *me, uint32_t cycle_count, uint32_t *time_us);
    bsp_status_t bsp_dwt_us_to_cycles(const bsp_dwt_t *me, uint32_t time_us, uint32_t *cycle_count);
    bsp_status_t bsp_dwt_delay_us(const bsp_dwt_t *me, uint32_t delay_us);
    bsp_status_t bsp_dwt_has_elapsed_us(const bsp_dwt_t *me, bsp_dwt_time_point_t start_time,
                                        uint32_t duration_us, bool *has_elapsed);

#ifdef __cplusplus
}
#endif

#endif /* BSP_DWT_H */
