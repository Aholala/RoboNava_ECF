#include "app_command.h"
#include "app_gimbal.h"
#include "app_chassis.h"
#include "app_safety.h"
#include "app_vision.h"
#include "alg_imu_ekf.h"
#include "module_dji_motor.h"
#include "module_dm_motor_bus.h"
#include "module_motor.h"
#include "module_referee_crc.h"

#include <assert.h>
#include <string.h>

static unsigned enables;
static unsigned disables;
static unsigned updates;
static module_motor_status_t offline_enable_status;
static bsp_can_frame_t last_can_frame;


static module_motor_status_t fake_enable(module_motor_t *motor)
{
    ++enables;
    motor->state = MODULE_MOTOR_STATE_ENABLED;
    return MODULE_MOTOR_STATUS_OK;
}

static module_motor_status_t fake_disable(module_motor_t *motor)
{
    ++disables;
    motor->state = MODULE_MOTOR_STATE_DISABLED;
    return MODULE_MOTOR_STATUS_OK;
}

static module_motor_status_t fake_clear(const module_motor_t *motor)
{
    (void)motor;
    return MODULE_MOTOR_STATUS_OK;
}

static module_motor_status_t fake_target(module_motor_t *motor, float target)
{
    (void)motor;
    (void)target;
    return MODULE_MOTOR_STATUS_OK;
}

static module_motor_status_t fake_update(module_motor_t *motor, float dt)
{
    (void)motor;
    (void)dt;
    ++updates;
    return MODULE_MOTOR_STATUS_OK;
}

static void try_enable_while_offline(void *context)
{
    offline_enable_status = module_motor_enable((module_motor_t *)context);
}

static bsp_status_t capture_can_frame(void *handle, const bsp_can_frame_t *frame,
                                      uint32_t timeout_ms)
{
    (void)handle;
    (void)timeout_ms;
    last_can_frame = *frame;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_watchdog_refresh(bsp_watchdog_t *watchdog)
{
    (void)watchdog;
    return BSP_STATUS_OK;
}

int main(void)
{
    const bsp_can_driver_ops_t can_ops = {.transmit = capture_can_frame};
    const module_motor_ops_t ops = {
        fake_enable, fake_disable, fake_clear, fake_target, fake_update};
    app_chassis_command_t received = {0};
    app_remote_input_t remote = {.channel = {0, 0, 0, 660},
                                 .left_switch = APP_SWITCH_MIDDLE,
                                 .right_switch = APP_SWITCH_MIDDLE,
                                 .dial = 501,
                                 .online = true};
    app_gimbal_command_t gimbal_command;
    app_command_t command_app = {0};
    const app_command_output_t *command_output;
    app_gimbal_t uninitialized_gimbal = {0};
    app_vision_t uninitialized_vision = {0};
    app_imu_snapshot_t imu_snapshot = {0};
    app_shooter_command_t shooter_command;
    const app_command_config_t command_config = {
        .channel_maximum_offset = 660,
        .maximum_yaw_rate_rad_per_s = 2.0F,
        .maximum_pitch_rate_rad_per_s = 2.0F,
        .minimum_pitch_rad = -1.0F,
        .maximum_pitch_rad = 1.0F,
        .maximum_chassis_velocity_m_per_s = 4.0F,
        .maximum_chassis_spin_rad_per_s = 6.0F,
        .friction_velocity_rad_per_s = 500.0F,
    };
    app_safety_monitor_t monitor = {0};
    app_safety_t safety = {0};
    module_motor_t motor = {0};
    bsp_can_t can = {.driver_ops = &can_ops, .is_initialized = true};
    module_dji_motor_t dji_motor = {0};
    module_dji_motor_bus_t dji_bus = {0};
    module_dji_motor_config_t dji_config = {
        .name = "chassis_1",
        .motor_bus = &dji_bus,
        .motor_model = MODULE_DJI_MOTOR_M3508,
        .control_mode = MODULE_DJI_CONTROL_DIRECT,
        .motor_identifier = 5U,
        .direction_sign = 1.0F,
        .maximum_temperature_c = 80.0F,
        .position_reference = MODULE_DJI_POSITION_BOOT_RELATIVE,
    };
    module_dm_motor_t dm_motor = {0};
    module_dm_motor_t *dm_storage[1];
    module_dm_motor_bus_t dm_bus = {0};
    module_dm_motor_config_t dm_config = {
        .name = "gimbal_yaw",
        .motor_bus = &dm_bus,
        .control_mode = MODULE_DM_MODE_VELOCITY,
        .master_identifier = 1U,
        .feedback_identifier = 0x11U,
        .transmit_timeout_ms = 1U,
        .limits = {-12.5F, 12.5F, -30.0F, 30.0F, -10.0F, 10.0F,
                   0.0F, 500.0F, 0.0F, 5.0F},
    };
    const app_safety_monitor_config_t monitor_config = {
        "remote", 10U, true, try_enable_while_offline, NULL, &motor};
    alg_imu_ekf_config_t ekf_config;
    alg_mecanum_t mecanum = {0};
    app_chassis_t chassis = {0};
    const alg_mecanum_config_t mecanum_config = {
        .wheel_radius_m = 0.076F,
        .half_wheelbase_m = 0.2F,
        .half_track_width_m = 0.2F,
        .direction_sign = {1.0F, 1.0F, 1.0F, 1.0F},
        .odometry_weight = {1.0F, 1.0F, 1.0F, 1.0F},
        .maximum_wheel_angular_velocity_rad_per_s = 100.0F,
        .roller_arrangement = ALG_MECANUM_ROLLER_X,
    };
    alg_imu_ekf_t ekf = {0};
    alg_imu_ekf_diagnostics_t diagnostics;
    uint8_t crc8_frame[5] = {0xA5U, 0x01U, 0x00U, 0x00U, 0U};
    uint8_t crc16_frame[8] = {0xA5U, 0x01U, 0x00U, 0x00U, 0U, 0x12U, 0U, 0U};
    const float gyro[3] = {0.0F, 0.0F, 0.0F};
    const float acceleration[3] = {0.0F, 0.0F, 9.80665F};

    assert(app_command_init(&command_app, &command_config) == BSP_STATUS_OK);
    assert(app_command_update(&command_app, &remote, NULL, NULL, 0.001F) == BSP_STATUS_OK);
    command_output = app_command_get_output(&command_app);
    assert(command_output != NULL);
    received = command_output->chassis;
    gimbal_command = command_output->gimbal;
    shooter_command = command_output->shooter;
    assert(received.velocity_x_m_per_s == 4.0F);
    assert(gimbal_command.enabled);
    assert(shooter_command.friction_enabled && shooter_command.fire_requested);
    remote.online = false;
    assert(app_command_update(&command_app, &remote, NULL, NULL, 0.001F) == BSP_STATUS_OK);
    received = app_command_get_output(&command_app)->chassis;
    assert(!received.enabled && received.mode == APP_CHASSIS_MODE_NO_FORCE);
    assert(app_gimbal_update(&uninitialized_gimbal, &gimbal_command, &imu_snapshot, 0.001F) ==
           BSP_STATUS_NOT_INITIALIZED);
    assert(app_vision_update(&uninitialized_vision, &imu_snapshot, 1U) ==
           BSP_STATUS_NOT_INITIALIZED);

    assert(module_motor_init_base(&motor, &ops, "gimbal_yaw") == MODULE_MOTOR_STATUS_OK);
    assert(strcmp(module_motor_get_name(&motor), "gimbal_yaw") == 0);
    motor.is_registered = true;
    assert(module_motor_notify_feedback(&motor) == MODULE_MOTOR_STATUS_OK);

    assert(app_safety_init(&safety, NULL) == BSP_STATUS_OK);
    assert(!app_safety_output_allowed(&safety));
    assert(app_safety_monitor_init(&monitor, &monitor_config) == BSP_STATUS_OK);
    assert(app_safety_register(&safety, &monitor) == BSP_STATUS_OK);
    app_safety_set_output_enabled(&safety, true);
    app_safety_notify_online(&monitor, UINT32_MAX - 3U);
    app_safety_process(&safety, 3U);
    assert(app_safety_output_allowed(&safety));
    assert(module_motor_enable(&motor) == MODULE_MOTOR_STATUS_OK);
    assert(enables == 1U);
    assert(module_motor_update(&motor, 0.002F) == MODULE_MOTOR_STATUS_OK);
    assert(module_motor_get_last_delta_time_s(&motor) == 0.002F);
    assert(module_motor_get_enabled_runtime_us(&motor) == 2000U);

    app_safety_process(&safety, 8U);
    assert(!app_safety_output_allowed(&safety));
    assert(offline_enable_status == MODULE_MOTOR_STATUS_OUTPUT_INHIBITED);
    assert(module_motor_update(&motor, 0.001F) == MODULE_MOTOR_STATUS_OK);
    assert(disables == 1U);
    assert(updates == 1U);
    assert(module_motor_get_last_delta_time_s(&motor) == 0.002F);
    assert(module_motor_get_enabled_runtime_us(&motor) == 2000U);

    assert(module_dji_motor_bus_init(&dji_bus, &can, 1U) == MODULE_MOTOR_STATUS_OK);
    assert(module_dji_motor_init(&dji_motor, &dji_config) == MODULE_MOTOR_STATUS_OK);
    assert(module_dji_motor_register(&dji_motor) == MODULE_MOTOR_STATUS_OK);
    dji_motor.super.state = MODULE_MOTOR_STATE_ENABLED;
    dji_motor.command_value = 321;
    assert(module_dji_motor_bus_update(&dji_bus, 0.001F) == MODULE_MOTOR_STATUS_OK);
    assert(dji_motor.super.state == MODULE_MOTOR_STATE_DISABLED);
    assert(dji_motor.command_value == 0);
    assert(last_can_frame.data[0] == 0U && last_can_frame.data[1] == 0U);
    app_safety_notify_online(&monitor, 9U);
    app_safety_process(&safety, 9U);
    assert(!app_safety_output_allowed(&safety));
    app_safety_set_output_enabled(&safety, true);
    app_safety_process(&safety, 9U);
    assert(app_safety_output_allowed(&safety));
    assert(alg_mecanum_init(&mecanum, &mecanum_config) == ALG_CHASSIS_STATUS_OK);
    {
        const app_chassis_config_t chassis_config = {
            .type = APP_CHASSIS_TYPE_MECANUM,
            .follow_gain = 5.0F,
            .stop_deadband = 0.02F,
            .drive.mecanum = {&mecanum, {&motor, &motor, &motor, &motor}},
        };
        const app_chassis_command_t chassis_command = {
            .velocity_x_m_per_s = 1.0F,
            .mode = APP_CHASSIS_MODE_NORMAL,
            .enabled = true,
        };
        unsigned updates_before = updates;
        assert(app_chassis_init(&chassis, &chassis_config) == BSP_STATUS_OK);
        assert(app_chassis_update(&chassis, &chassis_command, 0.001F) == BSP_STATUS_OK);
        assert(updates == updates_before);
    }
    assert(module_dji_motor_bus_update(&dji_bus, 0.001F) == MODULE_MOTOR_STATUS_OK);
    assert(last_can_frame.data[0] == 0U && last_can_frame.data[1] == 0U);

    assert(module_dm_motor_bus_init(&dm_bus, &can, dm_storage, 1U, 1U) ==
           MODULE_MOTOR_STATUS_OK);
    assert(module_dm_motor_init(&dm_motor, &dm_config) == MODULE_MOTOR_STATUS_OK);
    assert(module_dm_motor_register(&dm_motor) == MODULE_MOTOR_STATUS_OK);
    assert(module_motor_notify_feedback(&dm_motor.super) == MODULE_MOTOR_STATUS_OK);
    assert(module_motor_enable(&dm_motor.super) == MODULE_MOTOR_STATUS_OK);
    assert(module_dm_motor_set_velocity_target(&dm_motor, 3.0F) == MODULE_MOTOR_STATUS_OK);
    assert(module_dm_motor_bus_update(&dm_bus, 0.002F) == MODULE_MOTOR_STATUS_OK);
    assert(last_can_frame.identifier == 0x201U && last_can_frame.data_length == 4U);
    assert(module_motor_get_last_delta_time_s(&dm_motor.super) == 0.002F);
    assert(module_motor_get_enabled_runtime_us(&dm_motor.super) == 2000U);

    assert(module_referee_crc8_append(crc8_frame, sizeof(crc8_frame)));
    assert(module_referee_crc8_verify(crc8_frame, sizeof(crc8_frame)));
    crc8_frame[1] ^= 1U;
    assert(!module_referee_crc8_verify(crc8_frame, sizeof(crc8_frame)));
    assert(module_referee_crc16_append(crc16_frame, sizeof(crc16_frame)));
    assert(module_referee_crc16_verify(crc16_frame, sizeof(crc16_frame)));
    crc16_frame[5] ^= 1U;
    assert(!module_referee_crc16_verify(crc16_frame, sizeof(crc16_frame)));

    assert(alg_imu_ekf_config_init(&ekf_config) == ALG_IMU_EKF_STATUS_OK);
    assert(alg_imu_ekf_init(&ekf, &ekf_config) == ALG_IMU_EKF_STATUS_OK);
    assert(alg_imu_ekf_reset_from_accelerometer(&ekf, acceleration) == ALG_IMU_EKF_STATUS_OK);
    assert(alg_imu_ekf_update(&ekf, gyro, acceleration, 0.001F, NULL) ==
           ALG_IMU_EKF_STATUS_OK);
    assert(alg_imu_ekf_get_diagnostics(&ekf, &diagnostics) == ALG_IMU_EKF_STATUS_OK);
    assert(diagnostics.is_stable);
    return 0;
}
