/**
 * @file bsp_gpio.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief GPIO 抽象层头文件 — 平台无关的通用 IO 封装
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 通过 bsp_gpio_bind_platform() 绑定平台驱动后，
 *       所有 GPIO 实例共享同一套驱动操作表（单例模式）。
 *       支持读/写/翻转三种基本操作。
 */

#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 类型定义 ======================== */

/** @brief GPIO 驱动操作表 — 由平台层实现 */
typedef struct
{
    bsp_status_t (*init)(void *handle);             /**< 硬件初始化（可选） */
    bsp_status_t (*deinit)(void *handle);           /**< 硬件反初始化（可选） */
    bsp_status_t (*read)(const void *handle, bool *level);  /**< 读取引脚电平 */
    bsp_status_t (*write)(void *handle, bool level);        /**< 设置引脚电平 */
    bsp_status_t (*toggle)(void *handle);                   /**< 翻转引脚电平 */
} bsp_gpio_driver_ops_t;

/** @brief GPIO 实例对象 */
typedef struct bsp_gpio
{
    void *device_handle;    /**< 硬件句柄 */
    bool is_initialized;    /**< 初始化标志 */
} bsp_gpio_t;

/** @brief GPIO 初始化配置 */
typedef struct
{
    void *device_handle;                      /**< 硬件句柄 */
    const bsp_gpio_driver_ops_t *driver_ops;  /**< 驱动操作表 */
} bsp_gpio_config_t;

/* ======================== 公共 API ======================== */

bsp_status_t bsp_gpio_bind_platform(const bsp_gpio_driver_ops_t *driver_ops);
bsp_status_t bsp_gpio_init(bsp_gpio_t *me, const bsp_gpio_config_t *config);
bsp_status_t bsp_gpio_deinit(bsp_gpio_t *me);
bool bsp_gpio_is_initialized(const bsp_gpio_t *me);

/**
 * @brief 读取 GPIO 引脚电平
 * @param me GPIO 对象指针
 * @param level 输出参数，true = 高电平，false = 低电平
 * @return 执行状态
 */
bsp_status_t bsp_gpio_read(const bsp_gpio_t *me, bool *level);

/**
 * @brief 设置 GPIO 引脚电平
 * @param me GPIO 对象指针
 * @param level true = 高电平，false = 低电平
 * @return 执行状态
 */
bsp_status_t bsp_gpio_write(bsp_gpio_t *me, bool level);

/**
 * @brief 翻转 GPIO 引脚电平
 * @param me GPIO 对象指针
 * @return 执行状态
 */
bsp_status_t bsp_gpio_toggle(bsp_gpio_t *me);

#ifdef __cplusplus
}
#endif
#endif
