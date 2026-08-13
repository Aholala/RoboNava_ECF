/**
 * @file app_command.h
 * @brief Convert project-independent remote input into robot commands.
 */
#ifndef APP_COMMAND_H
#define APP_COMMAND_H

#include "app_types.h"
#include "bsp_common.h"

#include <stdbool.h>

typedef struct
{
    int16_t channel_maximum_offset;
    float maximum_yaw_rate_rad_per_s;
    float maximum_pitch_rate_rad_per_s;
    float minimum_pitch_rad;
    float maximum_pitch_rad;
    float maximum_chassis_velocity_m_per_s;
    float maximum_chassis_spin_rad_per_s;
    float friction_velocity_rad_per_s;
} app_command_config_t;

typedef struct
{
    app_chassis_command_t chassis;
    app_gimbal_command_t gimbal;
    app_shooter_command_t shooter;
    bool automatic_vision_requested;
} app_command_output_t;

typedef struct
{
    app_command_config_t config;
    app_command_output_t output;
    float yaw_target_rad;
    float pitch_target_rad;
    uint32_t sequence;
    bool initialized;
} app_command_t;

bsp_status_t app_command_init(app_command_t *me, const app_command_config_t *config);
bsp_status_t app_command_update(app_command_t *me,
                                const app_remote_input_t *remote,
                                const app_gimbal_feedback_t *gimbal_feedback,
                                const app_vision_target_t *vision_target,
                                float delta_time_s);
const app_command_output_t *app_command_get_output(const app_command_t *me);

#endif
