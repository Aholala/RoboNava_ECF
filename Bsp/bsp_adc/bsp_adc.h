/**
 * @file bsp_adc.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief ADC 外设抽象层头文件
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供 ADC 的硬件无关接口，包括原始值、归一化值和电压读取，
 *       以及 DMA 连续采样和回调通知。
 */

#ifndef BSP_ADC_H
#define BSP_ADC_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief ADC 底层驱动操作集 */
typedef struct {
    bsp_status_t (*init)(void *, uint32_t);                         /**< 初始化 ADC 通道 */
    bsp_status_t (*deinit)(void *, uint32_t);                       /**< 反初始化 ADC 通道 */
    bsp_status_t (*start)(void *, uint32_t);                        /**< 启动 ADC 转换 */
    bsp_status_t (*stop)(void *, uint32_t);                         /**< 停止 ADC 转换 */
    bsp_status_t (*calibrate)(void *);                              /**< 校准 ADC */
    bsp_status_t (*read_raw)(void *, uint32_t, uint32_t *, uint32_t); /**< 读取原始 ADC 值 */
    bsp_status_t (*start_dma)(void *, uint32_t, uint32_t *, size_t);  /**< 启动 DMA 连续采样 */
    bsp_status_t (*stop_dma)(void *, uint32_t);                       /**< 停止 DMA 连续采样 */
} bsp_adc_driver_ops_t;

/** @brief ADC 实例对象 */
typedef struct bsp_adc {
    void *device_handle;                    /**< 硬件句柄 */
    const bsp_adc_driver_ops_t *driver_ops; /**< 驱动操作集 */
    uint32_t channel;                       /**< ADC 通道号 */
    float reference_voltage_v;              /**< 参考电压（V） */
    uint32_t maximum_raw_value;             /**< 最大原始值（由分辨率计算） */
    bsp_event_callback_t callback;          /**< 事件回调 */
    void *user_context;                     /**< 回调用户上下文 */
    bool is_initialized;                    /**< 初始化标志 */
} bsp_adc_t;

/** @brief ADC 初始化配置 */
typedef struct {
    void *device_handle;                    /**< 硬件句柄 */
    const bsp_adc_driver_ops_t *driver_ops; /**< 驱动操作集 */
    uint32_t channel;                       /**< ADC 通道号 */
    uint8_t resolution_bits;                /**< ADC 分辨率位数（1~31） */
    float reference_voltage_v;              /**< 参考电压（V） */
    bsp_event_callback_t callback;          /**< 事件回调 */
    void *user_context;                     /**< 回调用户上下文 */
} bsp_adc_config_t;

/* ======================== 公共 API ======================== */

bsp_status_t bsp_adc_init(bsp_adc_t *me, const bsp_adc_config_t *config);
bsp_status_t bsp_adc_deinit(bsp_adc_t *me);
bsp_status_t bsp_adc_set_callback(bsp_adc_t *me, bsp_event_callback_t callback, void *context);
bsp_status_t bsp_adc_start(bsp_adc_t *me);
bsp_status_t bsp_adc_stop(bsp_adc_t *me);
bsp_status_t bsp_adc_calibrate(bsp_adc_t *me);
bsp_status_t bsp_adc_read_raw(bsp_adc_t *me, uint32_t *raw_value, uint32_t timeout_ms);
bsp_status_t bsp_adc_read_normalized(bsp_adc_t *me, float *value, uint32_t timeout_ms);
bsp_status_t bsp_adc_read_voltage(bsp_adc_t *me, float *voltage_v, uint32_t timeout_ms);
bsp_status_t bsp_adc_start_dma(bsp_adc_t *me, uint32_t *buffer, size_t count);
bsp_status_t bsp_adc_stop_dma(bsp_adc_t *me);
void bsp_adc_notify(bsp_adc_t *me, bsp_event_t event, bsp_status_t status, size_t size);

#ifdef __cplusplus
}
#endif
#endif
