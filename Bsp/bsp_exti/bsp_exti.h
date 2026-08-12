/**
 * @file bsp_exti.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 外部中断抽象层头文件 — 平台无关的 EXTI 封装
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 通过 bsp_exti_bind_platform() 绑定平台驱动后，
 *       所有 EXTI 实例共享同一套驱动操作表（单例模式）。
 *       ISR 中调用 bsp_exti_notify() 将中断事件转发给用户回调。
 */

#ifndef BSP_EXTI_H
#define BSP_EXTI_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 类型定义 ======================== */

/** @brief 外部中断实例（前向声明） */
typedef struct bsp_exti bsp_exti_t;

/** @brief 外部中断回调函数签名
 *  @param me 触发中断的 EXTI 对象
 *  @param user_context 用户上下文指针 */
typedef void (*bsp_exti_callback_t)(bsp_exti_t *me, void *user_context);

/** @brief 外部中断驱动操作表 — 由平台层实现 */
typedef struct
{
    bsp_status_t (*init)(void *handle);     /**< 硬件初始化（可选） */
    bsp_status_t (*deinit)(void *handle);   /**< 硬件反初始化（可选） */
    bsp_status_t (*enable)(void *handle);   /**< 使能中断 */
    bsp_status_t (*disable)(void *handle);  /**< 禁用中断 */
} bsp_exti_driver_ops_t;

/** @brief 外部中断实例对象 */
struct bsp_exti
{
    void *device_handle;            /**< 硬件句柄 */
    bsp_exti_callback_t callback;   /**< 用户回调 */
    void *user_context;             /**< 用户上下文 */
    bool is_initialized;            /**< 初始化标志 */
};

/** @brief 外部中断初始化配置 */
typedef struct
{
    void *device_handle;                      /**< 硬件句柄 */
    const bsp_exti_driver_ops_t *driver_ops;  /**< 驱动操作表 */
    bsp_exti_callback_t callback;             /**< 初始回调 */
    void *user_context;                       /**< 用户上下文 */
} bsp_exti_config_t;

/* ======================== 公共 API ======================== */

bsp_status_t bsp_exti_bind_platform(const bsp_exti_driver_ops_t *driver_ops);
bsp_status_t bsp_exti_init(bsp_exti_t *me, const bsp_exti_config_t *config);
bsp_status_t bsp_exti_deinit(bsp_exti_t *me);
bool bsp_exti_is_initialized(const bsp_exti_t *me);
bsp_status_t bsp_exti_set_callback(bsp_exti_t *me, bsp_exti_callback_t callback,
                                   void *user_context);
bsp_status_t bsp_exti_enable(bsp_exti_t *me);
bsp_status_t bsp_exti_disable(bsp_exti_t *me);

/**
 * @brief 中断通知入口 — 由 ISR 调用，触发用户回调
 * @param me EXTI 对象指针
 * @note 内部检查初始化状态和回调非空后才调用
 */
void bsp_exti_notify(bsp_exti_t *me);

#ifdef __cplusplus
}
#endif
#endif
