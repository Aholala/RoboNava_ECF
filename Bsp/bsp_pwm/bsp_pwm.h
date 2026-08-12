/**
 * @file bsp_pwm.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BSP PWM 外设抽象层公共接口
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 定义 PWM 驱动操作接口、实例结构体、配置结构体以及
 *       平台无关 PWM 输出控制的公共 API，包括频率、脉宽和占空比。
 */

#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 平台特定 PWM 驱动操作虚表 */
typedef struct
{
    bsp_status_t (*init)(void *handle, uint32_t channel);
    bsp_status_t (*deinit)(void *handle, uint32_t channel);
    bsp_status_t (*start)(void *handle, uint32_t channel);
    bsp_status_t (*stop)(void *handle, uint32_t channel);
    bsp_status_t (*set_frequency)(void *handle, uint32_t channel, uint32_t frequency_hz);
    bsp_status_t (*get_frequency)(const void *handle, uint32_t channel, uint32_t *frequency_hz);
    bsp_status_t (*set_pulse)(void *handle, uint32_t channel, uint32_t pulse_ticks);
    bsp_status_t (*get_pulse)(const void *handle, uint32_t channel, uint32_t *pulse_ticks);
    bsp_status_t (*get_period)(const void *handle, uint32_t channel, uint32_t *period_ticks);
} bsp_pwm_driver_ops_t;

/** @brief PWM 通道实例（运行时状态） */
typedef struct bsp_pwm
{
    void *device_handle;        /**< 平台定时器外设句柄 */
    uint32_t channel;           /**< PWM 通道索引 */
    bool is_initialized;        /**< 初始化标志 */
} bsp_pwm_t;

/** @brief PWM 通道配置（初始化参数） */
typedef struct
{
    void *device_handle;                    /**< 平台定时器外设句柄 */
    const bsp_pwm_driver_ops_t *driver_ops; /**< 平台驱动操作 */
    uint32_t channel;                       /**< PWM 通道索引 */
} bsp_pwm_config_t;

/* ======================== 平台绑定 ======================== */
bsp_status_t bsp_pwm_bind_platform(const bsp_pwm_driver_ops_t *driver_ops);

/* ======================== 生命周期 ======================== */
bsp_status_t bsp_pwm_init(bsp_pwm_t *me, const bsp_pwm_config_t *config);
bsp_status_t bsp_pwm_deinit(bsp_pwm_t *me);
bool bsp_pwm_is_initialized(const bsp_pwm_t *me);

/* ======================== 输出控制 ======================== */
bsp_status_t bsp_pwm_start(bsp_pwm_t *me);
bsp_status_t bsp_pwm_stop(bsp_pwm_t *me);

/* ======================== 频率 ======================== */
bsp_status_t bsp_pwm_set_frequency(bsp_pwm_t *me, uint32_t frequency_hz);
bsp_status_t bsp_pwm_get_frequency(const bsp_pwm_t *me, uint32_t *frequency_hz);

/* ======================== 脉宽 ======================== */
bsp_status_t bsp_pwm_set_pulse(bsp_pwm_t *me, uint32_t pulse_ticks);
bsp_status_t bsp_pwm_get_pulse(const bsp_pwm_t *me, uint32_t *pulse_ticks);

/* ======================== 占空比 ======================== */
bsp_status_t bsp_pwm_set_duty_cycle(bsp_pwm_t *me, float duty_cycle);
bsp_status_t bsp_pwm_get_duty_cycle(const bsp_pwm_t *me, float *duty_cycle);

#ifdef __cplusplus
}
#endif
#endif
