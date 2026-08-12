#include "app_imu.h"

#include "app_exchange.h"
#include "app_types.h"

bsp_status_t app_imu_init(app_imu_t *me, const app_imu_config_t *config)
{
    alg_imu_ekf_config_t default_ekf_config;
    const alg_imu_ekf_config_t *ekf_config;

    if ((me == NULL) || (config == NULL) || (config->sensor == NULL) ||
        !config->sensor->is_initialized)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (config->ekf_config == NULL)
    {
        if (alg_imu_ekf_config_init(&default_ekf_config) != ALG_IMU_EKF_STATUS_OK)
        {
            return BSP_STATUS_INVALID_ARGUMENT;
        }
        ekf_config = &default_ekf_config;
    }
    else
    {
        ekf_config = config->ekf_config;
    }

    *me = (app_imu_t){.sensor = config->sensor};
    if (alg_imu_ekf_init(&me->ekf, ekf_config) != ALG_IMU_EKF_STATUS_OK)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    me->initialized = true;
    return BSP_STATUS_OK;
}

void app_imu_update(app_imu_t *me, float delta_time_s)
{
    const module_bmi088_process_data_t *data;
    alg_imu_ekf_euler_t euler;
    float corrected_gyroscope[3];
    alg_imu_ekf_status_t status;

    if ((me == NULL) || !me->initialized || (delta_time_s <= 0.0F))
    {
        return;
    }
    if (module_bmi088_read(me->sensor) != MODULE_BMI088_STATUS_OK)
    {
        me->snapshot.valid = false;
        app_exchange_publish_imu(&me->snapshot);
        return;
    }
    data = module_bmi088_get_data(me->sensor);
    if ((data == NULL) || !data->is_valid)
    {
        me->snapshot.valid = false;
        app_exchange_publish_imu(&me->snapshot);
        return;
    }

    if (!me->attitude_initialized)
    {
        status = alg_imu_ekf_reset_from_accelerometer(&me->ekf, data->acceleration_m_per_s2);
        me->attitude_initialized = (status == ALG_IMU_EKF_STATUS_OK);
    }
    else
    {
        status = alg_imu_ekf_update(&me->ekf, data->angular_velocity_rad_per_s,
                                    data->acceleration_m_per_s2, delta_time_s, NULL);
    }
    if ((status != ALG_IMU_EKF_STATUS_OK) ||
        (alg_imu_ekf_get_euler(&me->ekf, &euler) != ALG_IMU_EKF_STATUS_OK) ||
        (alg_imu_ekf_get_corrected_gyroscope(&me->ekf, data->angular_velocity_rad_per_s,
                                             corrected_gyroscope) != ALG_IMU_EKF_STATUS_OK))
    {
        me->snapshot.valid = false;
        app_exchange_publish_imu(&me->snapshot);
        return;
    }

    me->snapshot.roll_rad = euler.roll_rad;
    me->snapshot.pitch_rad = euler.pitch_rad;
    me->snapshot.yaw_rad = euler.yaw_rad;
    me->snapshot.angular_velocity_rad_per_s[0] = corrected_gyroscope[0];
    me->snapshot.angular_velocity_rad_per_s[1] = corrected_gyroscope[1];
    me->snapshot.angular_velocity_rad_per_s[2] = corrected_gyroscope[2];
    me->snapshot.sample_count = data->sample_count;
    me->snapshot.valid = true;
    app_exchange_publish_imu(&me->snapshot);
}
