/**
 * @file app_vision.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 视觉应用模块接口 -- USB VCP 视觉目标通信
 * @version 1.0
 * @date 2026-08-13
 * @copyright Copyright (c) 2026
 *
 * @note 定义视觉配置与实例结构体，通过 USB VCP 收发固定 12 字节视觉帧。
 */

#ifndef APP_VISION_H
#define APP_VISION_H

#include "app_types.h"
#include "bsp_common.h"
#include "bsp_usb_vcp.h"

#include <stdbool.h>
#include <stdint.h>

#define APP_VISION_FRAME_SIZE (12U)  /**< 视觉帧固定长度 [字节]。 */

/** @brief 视觉工作模式。 */
typedef enum
{
    APP_VISION_MODE_MANUAL = 0,    /**< 手动模式。 */
    APP_VISION_MODE_AUTOMATIC = 1  /**< 自动/跟踪模式。 */
} app_vision_mode_t;

/** @brief 视觉模块的静态配置。 */
typedef struct
{
    bsp_usb_vcp_t *usb_vcp;      /**< USB VCP 实例。 */
    uint32_t target_timeout_ms;  /**< 目标超时时间 [ms]，超时后目标置为无效。 */
    uint32_t transmit_period_ms; /**< 姿态发送周期 [ms]。 */
} app_vision_config_t;

/** @brief 视觉模块运行时实例。 */
typedef struct
{
    app_vision_config_t config;                             /**< 静态配置的副本。 */
    app_vision_target_t target;                             /**< 最近一次视觉目标。 */
    uint8_t receive_frame[APP_VISION_FRAME_SIZE];           /**< 接收帧缓冲区。 */
    uint8_t transmit_frame[APP_VISION_FRAME_SIZE];          /**< 发送帧缓冲区。 */
    uint32_t target_elapsed_ms;                             /**< 距上次有效目标的经过时间 [ms]。 */
    uint32_t transmit_elapsed_ms;                           /**< 距上次发送姿态的经过时间 [ms]。 */
    app_vision_mode_t mode;                                 /**< 当前工作模式。 */
    bool initialized;                                       /**< 初始化阶段已成功完成。 */
} app_vision_t;

/**
 * @brief  初始化视觉模块。
 * @param  me      指向调用方分配的实例。
 * @param  config  静态配置（内部拷贝）。
 * @return 成功返回 BSP_STATUS_OK，参数无效返回 BSP_STATUS_INVALID_ARGUMENT。
 */
bsp_status_t app_vision_init(app_vision_t *me, const app_vision_config_t *config);

/**
 * @brief  设置视觉工作模式。
 * @param  me    已初始化的视觉实例。
 * @param  mode  目标工作模式。
 * @return 成功返回 BSP_STATUS_OK，参数无效/未初始化返回对应错误码。
 */
bsp_status_t app_vision_set_mode(app_vision_t *me, app_vision_mode_t mode);

/**
 * @brief  执行一个视觉通信周期。
 * @param  me              已初始化的视觉实例。
 * @param  imu             IMU 姿态快照，可为 NULL（跳过姿态发送）。
 * @param  elapsed_time_ms 距上次调用的经过时间 [ms]。
 * @return 成功返回 BSP_STATUS_OK，发送失败返回 BSP_STATUS_IO_ERROR。
 */
bsp_status_t app_vision_update(app_vision_t *me,
                               const app_imu_snapshot_t *imu,
                               uint32_t elapsed_time_ms);

/**
 * @brief  读取最近一次视觉目标。
 * @param  me  已初始化的视觉实例。
 * @return 只读目标指针，实例无效时返回 NULL。
 */
const app_vision_target_t *app_vision_get_target(const app_vision_t *me);

#endif
