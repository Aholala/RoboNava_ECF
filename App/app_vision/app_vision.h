/**
 * @file app_vision.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 视觉应用模块接口 -- 通过 USB VCP 与外部视觉系统通信
 * @version 1.0
 * @date 2026-08-12
 * @copyright Copyright (c) 2026
 *
 * @note 定义视觉模块配置结构体及工作模式枚举，通过12字节帧协议收发视觉目标与 IMU 姿态数据。
 */

#ifndef APP_VISION_H
#define APP_VISION_H

#include "bsp_common.h"
#include "bsp_usb_vcp.h"

#include <stdbool.h>
#include <stdint.h>

/** @brief 视觉 USB 协议的固定帧长度 [字节]。 */
#define APP_VISION_FRAME_SIZE (12U)

/** @brief 视觉系统工作模式。 */
typedef enum
{
    APP_VISION_MODE_MANUAL = 0,     /**< 视觉系统未跟踪。 */
    APP_VISION_MODE_AUTOMATIC = 1   /**< 视觉系统正在跟踪。 */
} app_vision_mode_t;

/** @brief 视觉模块的静态配置。 */
typedef struct
{
    bsp_usb_vcp_t *usb_vcp;         /**< USB VCP 实例。 */
    uint32_t target_timeout_ms;     /**< 目标数据有效超时 [ms]。 */
    uint32_t transmit_period_ms;    /**< IMU 数据回传周期 [ms]。 */
} app_vision_config_t;

/**
 * @brief  初始化视觉模块（单例）。
 * @param  config  静态配置（内部拷贝）。
 * @return 成功返回 BSP_STATUS_OK。
 */
bsp_status_t app_vision_init(const app_vision_config_t *config);

/**
 * @brief  设置回传给视觉系统的工作模式。
 * @param  mode  期望模式。
 */
void app_vision_set_mode(app_vision_mode_t mode);

/**
 * @brief  执行一个视觉处理周期。
 * @param  elapsed_time_ms  距上次调用的经过时间 [ms]。
 */
void app_vision_update(uint32_t elapsed_time_ms);

#endif
