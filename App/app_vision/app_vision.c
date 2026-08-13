/**
 * @file app_vision.c
 * @brief USB VCP vision protocol application.
 */
#include "app_vision.h"

#include <string.h>

#define APP_VISION_HEADER_FIRST (0xA5U)
#define APP_VISION_HEADER_SECOND (0x5AU)
#define APP_VISION_MODE_INDEX (2U)
#define APP_VISION_PITCH_INDEX (3U)
#define APP_VISION_YAW_INDEX (7U)
#define APP_VISION_CHECKSUM_INDEX (11U)

static uint8_t app_vision_crc8(const uint8_t *data, size_t data_size)
{
    uint8_t crc = 0xFFU;
    size_t index;
    uint8_t bit;
    for (index = 0U; index < data_size; ++index)
    {
        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit)
        {
            crc = ((crc & 0x80U) != 0U) ? (uint8_t)((crc << 1U) ^ 0x31U)
                                         : (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

static bool app_vision_frame_is_valid(const uint8_t *frame)
{
    return (frame[0] == APP_VISION_HEADER_FIRST) && (frame[1] == APP_VISION_HEADER_SECOND) &&
           (frame[APP_VISION_MODE_INDEX] <= (uint8_t)APP_VISION_MODE_AUTOMATIC) &&
           (frame[APP_VISION_CHECKSUM_INDEX] ==
            app_vision_crc8(frame, APP_VISION_CHECKSUM_INDEX));
}

static void app_vision_add_elapsed(uint32_t *timer_ms, uint32_t elapsed_time_ms)
{
    *timer_ms = (*timer_ms <= UINT32_MAX - elapsed_time_ms)
                    ? *timer_ms + elapsed_time_ms
                    : UINT32_MAX;
}

bsp_status_t app_vision_init(app_vision_t *me, const app_vision_config_t *config)
{
    if ((me == NULL) || (config == NULL) || (config->usb_vcp == NULL) ||
        (config->target_timeout_ms == 0U) || (config->transmit_period_ms == 0U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *me = (app_vision_t){
        .config = *config,
        .target_elapsed_ms = config->target_timeout_ms,
        .mode = APP_VISION_MODE_MANUAL,
        .initialized = true,
    };
    return BSP_STATUS_OK;
}

bsp_status_t app_vision_set_mode(app_vision_t *me, app_vision_mode_t mode)
{
    if ((me == NULL) || (mode > APP_VISION_MODE_AUTOMATIC))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!me->initialized)
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    me->mode = mode;
    return BSP_STATUS_OK;
}

bsp_status_t app_vision_update(app_vision_t *me,
                               const app_imu_snapshot_t *imu,
                               uint32_t elapsed_time_ms)
{
    bool usb_busy = false;
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!me->initialized)
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }

    app_vision_add_elapsed(&me->target_elapsed_ms, elapsed_time_ms);
    app_vision_add_elapsed(&me->transmit_elapsed_ms, elapsed_time_ms);
    memset(me->receive_frame, 0, sizeof(me->receive_frame));
    if ((bsp_usb_vcp_receive(me->config.usb_vcp, me->receive_frame,
                             sizeof(me->receive_frame)) == BSP_STATUS_OK) &&
        app_vision_frame_is_valid(me->receive_frame))
    {
        memcpy(&me->target.target_pitch_rad, &me->receive_frame[APP_VISION_PITCH_INDEX],
               sizeof(float));
        memcpy(&me->target.target_yaw_rad, &me->receive_frame[APP_VISION_YAW_INDEX],
               sizeof(float));
        me->target.target_valid =
            me->receive_frame[APP_VISION_MODE_INDEX] == APP_VISION_MODE_AUTOMATIC;
        me->target.tracking_ready = me->target.target_valid;
        ++me->target.update_count;
        me->target_elapsed_ms = 0U;
    }
    if (me->target_elapsed_ms > me->config.target_timeout_ms)
    {
        me->target.target_valid = false;
        me->target.tracking_ready = false;
    }

    if ((me->transmit_elapsed_ms < me->config.transmit_period_ms) || (imu == NULL) ||
        (bsp_usb_vcp_get_busy(me->config.usb_vcp, &usb_busy) != BSP_STATUS_OK) || usb_busy)
    {
        return BSP_STATUS_OK;
    }
    memset(me->transmit_frame, 0, sizeof(me->transmit_frame));
    me->transmit_frame[0] = APP_VISION_HEADER_FIRST;
    me->transmit_frame[1] = APP_VISION_HEADER_SECOND;
    me->transmit_frame[APP_VISION_MODE_INDEX] = (uint8_t)me->mode;
    memcpy(&me->transmit_frame[APP_VISION_PITCH_INDEX], &imu->pitch_rad, sizeof(float));
    memcpy(&me->transmit_frame[APP_VISION_YAW_INDEX], &imu->yaw_rad, sizeof(float));
    me->transmit_frame[APP_VISION_CHECKSUM_INDEX] =
        app_vision_crc8(me->transmit_frame, APP_VISION_CHECKSUM_INDEX);
    if (bsp_usb_vcp_transmit(me->config.usb_vcp, me->transmit_frame,
                             sizeof(me->transmit_frame), 2U) != BSP_STATUS_OK)
    {
        return BSP_STATUS_IO_ERROR;
    }
    me->transmit_elapsed_ms = 0U;
    return BSP_STATUS_OK;
}

const app_vision_target_t *app_vision_get_target(const app_vision_t *me)
{
    return ((me != NULL) && me->initialized) ? &me->target : NULL;
}
