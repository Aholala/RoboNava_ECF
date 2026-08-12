/**
 * @file bsp_encoder.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 编码器抽象层头文件 — 增量/绝对编码器统一接口
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 通过 bsp_encoder_driver_ops_t 解耦硬件实现，支持任意编码器类型。
 *       counter_modulus > 0 时启用环形计数器模式，自动处理溢出回绕。
 */

#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H
#include "bsp_common.h"
#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 类型定义 ======================== */

/** @brief 编码器旋转方向 */
typedef enum { BSP_ENCODER_DIRECTION_STOPPED = 0, BSP_ENCODER_DIRECTION_FORWARD,
               BSP_ENCODER_DIRECTION_REVERSE } bsp_encoder_direction_t;

/** @brief 编码器底层驱动操作表 — 由平台层实现 */
typedef struct {
    bsp_status_t (*init)(void *);              /**< 硬件初始化（可选） */
    bsp_status_t (*deinit)(void *);            /**< 硬件反初始化（可选） */
    bsp_status_t (*start)(void *);             /**< 启动计数 */
    bsp_status_t (*stop)(void *);              /**< 停止计数 */
    bsp_status_t (*set_count)(void *, int32_t);            /**< 设置计数值 */
    bsp_status_t (*get_count)(const void *, int32_t *);    /**< 读取计数值 */
    bsp_status_t (*get_direction)(const void *, bsp_encoder_direction_t *); /**< 读取方向 */
} bsp_encoder_driver_ops_t;

/** @brief 编码器实例对象 */
typedef struct bsp_encoder {
    void *device_handle;                        /**< 硬件句柄 */
    const bsp_encoder_driver_ops_t *driver_ops; /**< 驱动操作表 */
    int32_t previous_count;                     /**< 上一次读取的计数值（用于增量计算） */
    uint32_t counter_modulus;                   /**< 计数器模数（0 = 线性模式） */
    bool is_initialized;                        /**< 初始化标志 */
} bsp_encoder_t;

/** @brief 编码器初始化配置 */
typedef struct { void *device_handle; const bsp_encoder_driver_ops_t *driver_ops;
                 uint32_t counter_modulus; } bsp_encoder_config_t;

/* ======================== 公共 API ======================== */

bsp_status_t bsp_encoder_init(bsp_encoder_t *, const bsp_encoder_config_t *);
bsp_status_t bsp_encoder_deinit(bsp_encoder_t *);
bsp_status_t bsp_encoder_start(bsp_encoder_t *); bsp_status_t bsp_encoder_stop(bsp_encoder_t *);
bsp_status_t bsp_encoder_reset(bsp_encoder_t *);
bsp_status_t bsp_encoder_set_count(bsp_encoder_t *, int32_t);
bsp_status_t bsp_encoder_get_count(const bsp_encoder_t *, int32_t *);
bsp_status_t bsp_encoder_get_delta(bsp_encoder_t *, int32_t *);
bsp_status_t bsp_encoder_get_direction(const bsp_encoder_t *, bsp_encoder_direction_t *);

#ifdef __cplusplus
}
#endif
#endif
