#ifndef APP_IMU_H
#define APP_IMU_H

#include "alg_imu_ekf.h"
#include "app_types.h"
#include "bsp_common.h"
#include "module_bmi088.h"
#include <stdbool.h>

typedef void (*app_imu_set_heater_t)(void *context, float duty_ratio);
typedef enum { APP_IMU_STATE_UNCALIBRATED = 0, APP_IMU_STATE_WARMING,
               APP_IMU_STATE_READY, APP_IMU_STATE_FAULT } app_imu_state_t;

typedef struct {
    module_bmi088_t *sensor;
    const alg_imu_ekf_config_t *ekf_config;
    uint32_t calibration_sample_count;
    uint32_t calibration_sample_interval_ms;
    float calibration_max_gyro_deviation_rad_per_s;
    float calibration_max_acceleration_deviation_m_per_s2;
    float target_temperature_c;
    float temperature_tolerance_c;
    float temperature_stable_time_s;
    float heater_kp;
    float heater_ki;
    app_imu_set_heater_t set_heater;
    void *heater_context;
} app_imu_config_t;

typedef struct {
    app_imu_config_t config;
    alg_imu_ekf_t ekf;
    app_imu_snapshot_t snapshot;
    float heater_integral;
    float temperature_stable_elapsed_s;
    app_imu_state_t state;
    bool attitude_initialized;
    bool initialized;
} app_imu_t;

bsp_status_t app_imu_init(app_imu_t *me, const app_imu_config_t *config);
bsp_status_t app_imu_calibrate(app_imu_t *me);
bsp_status_t app_imu_update(app_imu_t *me, float delta_time_s);
const app_imu_snapshot_t *app_imu_get_snapshot(const app_imu_t *me);
app_imu_state_t app_imu_get_state(const app_imu_t *me);

#endif
