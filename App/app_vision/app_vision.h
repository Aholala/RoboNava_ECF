/**
 * @file app_vision.h
 * @brief USB VCP vision protocol application.
 */
#ifndef APP_VISION_H
#define APP_VISION_H

#include "app_types.h"
#include "bsp_common.h"
#include "bsp_usb_vcp.h"

#include <stdbool.h>
#include <stdint.h>

#define APP_VISION_FRAME_SIZE (12U)

typedef enum
{
    APP_VISION_MODE_MANUAL = 0,
    APP_VISION_MODE_AUTOMATIC = 1
} app_vision_mode_t;

typedef struct
{
    bsp_usb_vcp_t *usb_vcp;
    uint32_t target_timeout_ms;
    uint32_t transmit_period_ms;
} app_vision_config_t;

typedef struct
{
    app_vision_config_t config;
    app_vision_target_t target;
    uint8_t receive_frame[APP_VISION_FRAME_SIZE];
    uint8_t transmit_frame[APP_VISION_FRAME_SIZE];
    uint32_t target_elapsed_ms;
    uint32_t transmit_elapsed_ms;
    app_vision_mode_t mode;
    bool initialized;
} app_vision_t;

bsp_status_t app_vision_init(app_vision_t *me, const app_vision_config_t *config);
bsp_status_t app_vision_set_mode(app_vision_t *me, app_vision_mode_t mode);
bsp_status_t app_vision_update(app_vision_t *me,
                               const app_imu_snapshot_t *imu,
                               uint32_t elapsed_time_ms);
const app_vision_target_t *app_vision_get_target(const app_vision_t *me);

#endif
