#ifndef APP_IMU_H
#define APP_IMU_H

#include "bsp_common.h"
#include "alg_imu_ekf.h"
#include "app_types.h"
#include "module_bmi088.h"

#include <stdbool.h>

typedef struct
{
    module_bmi088_t *sensor;
    const alg_imu_ekf_config_t *ekf_config;
} app_imu_config_t;

typedef struct
{
    module_bmi088_t *sensor;
    alg_imu_ekf_t ekf;
    app_imu_snapshot_t snapshot;
    bool attitude_initialized;
    bool initialized;
} app_imu_t;

bsp_status_t app_imu_init(app_imu_t *me, const app_imu_config_t *config);
void app_imu_update(app_imu_t *me, float delta_time_s);

#endif
