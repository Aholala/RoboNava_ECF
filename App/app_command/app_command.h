#ifndef APP_COMMAND_H
#define APP_COMMAND_H

#include "bsp_common.h"
#include "module_board_comm.h"
#include "module_dr16.h"

#include <stdbool.h>

typedef struct
{
    module_dr16_t *dr16;
    module_board_comm_t *board_comm;
    bool dr16_is_local;
    float maximum_yaw_rate_rad_per_s;
    float maximum_pitch_rate_rad_per_s;
    float minimum_pitch_rad;
    float maximum_pitch_rad;
    float maximum_chassis_velocity_m_per_s;
    float maximum_chassis_spin_rad_per_s;
} app_command_config_t;

bsp_status_t app_command_init(const app_command_config_t *config);
void app_command_update(float delta_time_s);

#endif
