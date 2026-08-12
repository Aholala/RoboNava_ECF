/**
 * @file bsp_timer.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BSP 硬件定时器抽象层
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供驱动无关的硬件定时器接口。
 *       支持周期配置、计数器读写和溢出回调通知。
 *       遵循面向对象的 BSP 模式，具备初始化/反初始化生命周期。
 */
#ifndef BSP_TIMER_H
#define BSP_TIMER_H
#include "bsp_common.h"
#ifdef __cplusplus
extern "C" {
#endif
/** @brief 定时器对象的不透明前向声明 */
typedef struct bsp_timer bsp_timer_t;

/** @brief 定时器溢出回调函数签名
 *  @param timer 触发溢出的定时器指针
 *  @param context 用户提供的上下文指针
 */
typedef void (*bsp_timer_callback_t)(bsp_timer_t *, void *);

/* ======================== 驱动操作表 ======================== */
/** @brief 硬件定时器后端的虚拟驱动操作表 */
typedef struct {
    bsp_status_t (*init)(void *);              /**< @brief 初始化硬件外设 */
    bsp_status_t (*deinit)(void *);            /**< @brief 反初始化硬件外设 */
    bsp_status_t (*start)(void *);             /**< @brief 启动定时器计数 */
    bsp_status_t (*stop)(void *);              /**< @brief 停止定时器计数 */
    bsp_status_t (*set_counter)(void *, uint32_t);        /**< @brief 设置当前计数值 */
    bsp_status_t (*get_counter)(const void *, uint32_t *);/**< @brief 读取当前计数值 */
    bsp_status_t (*set_period)(void *, uint32_t);         /**< @brief 设置自动重载周期 */
    bsp_status_t (*get_period)(const void *, uint32_t *); /**< @brief 获取自动重载周期 */
    bsp_status_t (*get_frequency)(const void *, uint32_t *);/**< @brief 获取定时器时钟频率（Hz） */
} bsp_timer_driver_ops_t;

/* ======================== 定时器对象 ======================== */
/** @brief 定时器实例（不透明成员，仅通过公共 API 访问） */
struct bsp_timer {
    void *device_handle;                      /**< @brief 指向硬件特定句柄的指针 */
    const bsp_timer_driver_ops_t *driver_ops; /**< @brief 虚拟驱动操作表 */
    bsp_timer_callback_t callback;            /**< @brief 定时器溢出时调用的用户回调 */
    void *user_context;                       /**< @brief 传递给回调的不透明用户上下文 */
    bool is_initialized;                      /**< @brief 成功初始化后为真 */
};

/* ======================== 配置 ======================== */
/** @brief 定时器实例的初始化配置 */
typedef struct {
    void *device_handle;                      /**< @brief 指向硬件特定句柄的指针 */
    const bsp_timer_driver_ops_t *driver_ops; /**< @brief 虚拟驱动操作表 */
    bsp_timer_callback_t callback;            /**< @brief 可选回调（可为 NULL） */
    void *user_context;                       /**< @brief 可选的用户上下文 */
} bsp_timer_config_t;

/* ======================== 公共 API ======================== */
bsp_status_t bsp_timer_init(bsp_timer_t *, const bsp_timer_config_t *);
bsp_status_t bsp_timer_deinit(bsp_timer_t *);
bsp_status_t bsp_timer_set_callback(bsp_timer_t *, bsp_timer_callback_t, void *);
bsp_status_t bsp_timer_start(bsp_timer_t *);
bsp_status_t bsp_timer_stop(bsp_timer_t *);
bsp_status_t bsp_timer_reset(bsp_timer_t *);
bsp_status_t bsp_timer_set_counter(bsp_timer_t *, uint32_t);
bsp_status_t bsp_timer_get_counter(const bsp_timer_t *, uint32_t *);
bsp_status_t bsp_timer_set_period(bsp_timer_t *, uint32_t);
bsp_status_t bsp_timer_get_period(const bsp_timer_t *, uint32_t *);
bsp_status_t bsp_timer_get_frequency(const bsp_timer_t *, uint32_t *);
void bsp_timer_notify_elapsed(bsp_timer_t *);
#ifdef __cplusplus
}
#endif
#endif
