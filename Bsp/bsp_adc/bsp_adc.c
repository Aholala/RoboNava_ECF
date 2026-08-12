/**
 * @file bsp_adc.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief ADC 外设抽象层实现
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供 ADC 初始化、反初始化、原始值读取、归一化值和电压换算，
 *       以及 DMA 连续采样和事件回调通知。
 */

#include "bsp_adc.h"

#include <math.h>

/* ======================== 内部函数 ======================== */

/**
 * @brief 验证 ADC 对象是否有效
 * @param me ADC 对象指针
 * @return BSP_STATUS_OK 有效，否则返回相应错误码
 */
static bsp_status_t bsp_adc_validate(const bsp_adc_t *me)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return me->is_initialized ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

/* ======================== 公共 API ======================== */

/**
 * @brief 初始化 ADC 实例
 * @param me ADC 对象指针
 * @param config 配置参数
 * @return 执行状态
 * @note 根据分辨率位数计算最大原始值（maximum_raw_value = 2^bits - 1）
 */
bsp_status_t bsp_adc_init(bsp_adc_t *me, const bsp_adc_config_t *config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->resolution_bits == 0U) ||
        (config->resolution_bits > 31U) || !isfinite(config->reference_voltage_v) ||
        (config->reference_voltage_v <= 0.0F) || (config->driver_ops->start == NULL) ||
        (config->driver_ops->stop == NULL) || (config->driver_ops->calibrate == NULL) ||
        (config->driver_ops->read_raw == NULL)) return BSP_STATUS_INVALID_ARGUMENT;

    *me = (bsp_adc_t){0};
    if (config->driver_ops->init != NULL) {
        status = config->driver_ops->init(config->device_handle, config->channel);
        if (status != BSP_STATUS_OK) return status;
    }
    me->device_handle = config->device_handle;
    me->driver_ops = config->driver_ops;
    me->channel = config->channel;
    me->reference_voltage_v = config->reference_voltage_v;
    me->maximum_raw_value = (1UL << config->resolution_bits) - 1UL;
    me->callback = config->callback;
    me->user_context = config->user_context;
    me->is_initialized = true;
    return BSP_STATUS_OK;
}

/**
 * @brief 反初始化 ADC 实例
 * @param me ADC 对象指针
 * @return 执行状态
 */
bsp_status_t bsp_adc_deinit(bsp_adc_t *me)
{
    bsp_status_t status = bsp_adc_validate(me);
    if (status != BSP_STATUS_OK) return status;
    status = (me->driver_ops->deinit != NULL)
                 ? me->driver_ops->deinit(me->device_handle, me->channel) : BSP_STATUS_OK;
    if (status == BSP_STATUS_OK) me->is_initialized = false;
    return status;
}

/**
 * @brief 设置 ADC 事件回调
 * @param me ADC 对象指针
 * @param callback 回调函数
 * @param context 回调用户上下文
 * @return 执行状态
 */
bsp_status_t bsp_adc_set_callback(bsp_adc_t *me, bsp_event_callback_t callback, void *context)
{
    const bsp_status_t status = bsp_adc_validate(me);
    if (status != BSP_STATUS_OK) return status;
    me->callback = callback;
    me->user_context = context;
    return BSP_STATUS_OK;
}

/* ======================== ADC 操作宏 ======================== */

/** @brief 生成 ADC 启停操作的宏函数 */
#define ADC_ACTION(name, member) \
    bsp_status_t name(bsp_adc_t *me) { \
        const bsp_status_t status = bsp_adc_validate(me); \
        return (status == BSP_STATUS_OK) \
            ? me->driver_ops->member(me->device_handle, me->channel) : status; \
    }
ADC_ACTION(bsp_adc_start, start)
ADC_ACTION(bsp_adc_stop, stop)

/**
 * @brief 校准 ADC
 * @param me ADC 对象指针
 * @return 执行状态
 */
bsp_status_t bsp_adc_calibrate(bsp_adc_t *me)
{
    const bsp_status_t status = bsp_adc_validate(me);
    return (status == BSP_STATUS_OK) ? me->driver_ops->calibrate(me->device_handle) : status;
}

/**
 * @brief 读取 ADC 原始值
 * @param me ADC 对象指针
 * @param raw_value 输出原始值
 * @param timeout_ms 超时时间（ms）
 * @return 执行状态
 * @note 读取后会校验 raw_value 是否超出 maximum_raw_value 范围
 */
bsp_status_t bsp_adc_read_raw(bsp_adc_t *me, uint32_t *raw_value, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_adc_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if (raw_value == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    const bsp_status_t read_status = me->driver_ops->read_raw(
        me->device_handle, me->channel, raw_value, timeout_ms);
    if (read_status != BSP_STATUS_OK) return read_status;
    return (*raw_value <= me->maximum_raw_value) ? BSP_STATUS_OK : BSP_STATUS_IO_ERROR;
}

/**
 * @brief 读取 ADC 归一化值 [0.0, 1.0]
 * @param me ADC 对象指针
 * @param value 输出归一化值
 * @param timeout_ms 超时时间（ms）
 * @return 执行状态
 * @note 归一化值 = raw / maximum_raw_value
 */
bsp_status_t bsp_adc_read_normalized(bsp_adc_t *me, float *value, uint32_t timeout_ms)
{
    uint32_t raw;
    if (value == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    const bsp_status_t status = bsp_adc_read_raw(me, &raw, timeout_ms);
    if (status == BSP_STATUS_OK) *value = (float)raw / (float)me->maximum_raw_value;
    return status;
}

/**
 * @brief 读取 ADC 电压值
 * @param me ADC 对象指针
 * @param voltage_v 输出电压（V）
 * @param timeout_ms 超时时间（ms）
 * @return 执行状态
 * @note 电压值 = 归一化值 * 参考电压
 */
bsp_status_t bsp_adc_read_voltage(bsp_adc_t *me, float *voltage_v, uint32_t timeout_ms)
{
    float normalized;
    if (voltage_v == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    const bsp_status_t status = bsp_adc_read_normalized(me, &normalized, timeout_ms);
    if (status == BSP_STATUS_OK) *voltage_v = normalized * me->reference_voltage_v;
    return status;
}

/**
 * @brief 启动 DMA 连续采样
 * @param me ADC 对象指针
 * @param buffer 数据缓冲区
 * @param count 采样数量
 * @return 执行状态
 */
bsp_status_t bsp_adc_start_dma(bsp_adc_t *me, uint32_t *buffer, size_t count)
{
    const bsp_status_t status = bsp_adc_validate(me);
    if (status != BSP_STATUS_OK) return status;
    if ((buffer == NULL) || (count == 0U)) return BSP_STATUS_INVALID_ARGUMENT;
    return (me->driver_ops->start_dma != NULL)
        ? me->driver_ops->start_dma(me->device_handle, me->channel, buffer, count)
        : BSP_STATUS_UNSUPPORTED;
}

/**
 * @brief 停止 DMA 连续采样
 * @param me ADC 对象指针
 * @return 执行状态
 */
bsp_status_t bsp_adc_stop_dma(bsp_adc_t *me)
{
    const bsp_status_t status = bsp_adc_validate(me);
    if (status != BSP_STATUS_OK) return status;
    return (me->driver_ops->stop_dma != NULL)
        ? me->driver_ops->stop_dma(me->device_handle, me->channel) : BSP_STATUS_UNSUPPORTED;
}

/**
 * @brief ADC 事件通知（由底层驱动在中断中调用）
 * @param me ADC 对象指针
 * @param event 事件类型
 * @param status 操作状态
 * @param size 传输数据大小
 * @note 若注册了回调则转发事件给用户回调
 */
void bsp_adc_notify(bsp_adc_t *me, bsp_event_t event, bsp_status_t status, size_t size)
{
    if ((me != NULL) && me->is_initialized && (me->callback != NULL))
        me->callback(event, status, size, me->user_context);
}
