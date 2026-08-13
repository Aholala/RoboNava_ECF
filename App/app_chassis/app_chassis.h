/**
 * @file app_chassis.h
 * @brief Unified Mecanum, omni-wheel and swerve chassis application.
 */
#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include "alg_mecanum.h"
#include "alg_omni.h"
#include "alg_swerve.h"
#include "app_types.h"
#include "bsp_common.h"
#include "module_motor.h"
#include "module_swerve.h"

#include <stdbool.h>

#define APP_CHASSIS_MAX_WHEEL_COUNT (4U)

typedef enum
{
    APP_CHASSIS_TYPE_MECANUM = 0,
    APP_CHASSIS_TYPE_OMNI,
    APP_CHASSIS_TYPE_SWERVE
} app_chassis_type_t;

typedef struct
{
    app_chassis_type_t type;
    float follow_gain;
    float stop_deadband;
    union
    {
        struct
        {
            alg_mecanum_t *kinematics;
            module_motor_t *motors[ALG_MECANUM_WHEEL_COUNT];
        } mecanum;
        struct
        {
            alg_omni_t *kinematics;
            module_motor_t *motors[APP_CHASSIS_MAX_WHEEL_COUNT];
            size_t wheel_count;
        } omni;
        struct
        {
            alg_swerve_t *kinematics;
            module_swerve_t *modules[ALG_SWERVE_RECTANGULAR_MODULE_COUNT];
        } swerve;
    } drive;
} app_chassis_config_t;

typedef struct
{
    app_chassis_config_t config;
    app_chassis_feedback_t feedback;
    bool initialized;
} app_chassis_t;

bsp_status_t app_chassis_init(app_chassis_t *me, const app_chassis_config_t *config);
bsp_status_t app_chassis_update(app_chassis_t *me,
                                const app_chassis_command_t *command,
                                float delta_time_s);
const app_chassis_feedback_t *app_chassis_get_feedback(const app_chassis_t *me);

#endif
