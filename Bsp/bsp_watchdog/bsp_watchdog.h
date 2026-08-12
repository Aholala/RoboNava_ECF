/**
 * @file bsp_watchdog.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BSP 独立看门狗抽象层
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供驱动无关的看门狗定时器外设接口。
 *       支持喂狗、超时查询和复位原因检测。
 *       遵循面向对象的 BSP 模式，具备初始化/反初始化生命周期。
 */
#ifndef BSP_WATCHDOG_H
#define BSP_WATCHDOG_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 驱动操作表 ======================== */
/** @brief 看门狗硬件后端的虚拟驱动操作表 */
typedef struct {
    bsp_status_t (*init)(void *device_handle);                                  /**< @brief 初始化并使能看门狗 */
    bsp_status_t (*deinit)(void *device_handle);                                /**< @brief 反初始化看门狗（如果支持） */
    bsp_status_t (*refresh)(void *device_handle);                               /**< @brief 刷新（喂狗）看门狗计数器 */
    bsp_status_t (*get_timeout_ms)(const void *device_handle, uint32_t *timeout_ms);       /**< @brief 获取配置的超时时间（毫秒） */
    bsp_status_t (*get_reset_detected)(const void *device_handle, bool *reset_detected);   /**< @brief 检查上次复位是否由看门狗引起 */
} bsp_watchdog_driver_ops_t;

/* ======================== 看门狗对象 ======================== */
/** @brief 看门狗实例（不透明成员，仅通过公共 API 访问） */
typedef struct bsp_watchdog {
    void *device_handle;                              /**< @brief 指向硬件特定句柄的指针 */
    const bsp_watchdog_driver_ops_t *driver_ops;      /**< @brief 虚拟驱动操作表 */
    bool is_initialized;                              /**< @brief 成功初始化后为真 */
} bsp_watchdog_t;

/* ======================== 配置 ======================== */
/** @brief 看门狗实例的初始化配置 */
typedef struct {
    void *device_handle;                              /**< @brief 指向硬件特定句柄的指针 */
    const bsp_watchdog_driver_ops_t *driver_ops;      /**< @brief 虚拟驱动操作表 */
} bsp_watchdog_config_t;

/* ======================== 公共 API ======================== */
bsp_status_t bsp_watchdog_init(bsp_watchdog_t *me, const bsp_watchdog_config_t *config);
bsp_status_t bsp_watchdog_deinit(bsp_watchdog_t *me);
bsp_status_t bsp_watchdog_refresh(bsp_watchdog_t *me);
bsp_status_t bsp_watchdog_get_timeout_ms(const bsp_watchdog_t *me, uint32_t *timeout_ms);
bsp_status_t bsp_watchdog_get_reset_detected(const bsp_watchdog_t *me, bool *reset_detected);

#ifdef __cplusplus
}
#endif
#endif
