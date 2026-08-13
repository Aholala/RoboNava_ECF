#include "app_exchange.h"
#include "app_safety.h"
#include "alg_imu_ekf.h"
#include "module_dji_motor.h"
#include "module_motor.h"
#include "module_referee_crc.h"

#include <assert.h>
#include <string.h>

static unsigned lock_enters;
static unsigned lock_exits;
static unsigned enables;
static unsigned disables;
static unsigned updates;
static module_motor_status_t offline_enable_status;
static bsp_can_frame_t last_can_frame;

static void enter_lock(void *context) { (void)context; ++lock_enters; }
static void exit_lock(void *context) { (void)context; ++lock_exits; }

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
    const app_exchange_lock_t lock = {enter_lock, exit_lock, NULL};
    const app_exchange_lock_t incomplete_lock = {enter_lock, NULL, NULL};
    const bsp_can_driver_ops_t can_ops = {.transmit = capture_can_frame};
    const module_motor_ops_t ops = {
        fake_enable, fake_disable, fake_clear, fake_target, fake_update};
    app_chassis_command_t sent = {0};
    app_chassis_command_t received = {0};
    app_safety_monitor_t monitor = {0};
    module_motor_t motor = {0};
    bsp_can_t can = {.driver_ops = &can_ops, .is_initialized = true};
    module_dji_motor_t dji_motor = {0};
    module_dji_motor_bus_t dji_bus = {.can = &can, .is_initialized = true};
    const app_safety_monitor_config_t monitor_config = {
        "remote", 10U, true, try_enable_while_offline, NULL, &motor};
    alg_imu_ekf_config_t ekf_config;
    alg_imu_ekf_t ekf = {0};
    alg_imu_ekf_diagnostics_t diagnostics;
    uint8_t crc8_frame[5] = {0xA5U, 0x01U, 0x00U, 0x00U, 0U};
    uint8_t crc16_frame[8] = {0xA5U, 0x01U, 0x00U, 0x00U, 0U, 0x12U, 0U, 0U};
    const float gyro[3] = {0.0F, 0.0F, 0.0F};
    const float acceleration[3] = {0.0F, 0.0F, 9.80665F};

    app_exchange_init(&lock);
    sent.velocity_x_m_per_s = 1.25F;
    app_exchange_publish_chassis_command(&sent);
    app_exchange_read_chassis_command(&received);
    assert(memcmp(&sent, &received, sizeof(sent)) == 0);
    assert(lock_enters == lock_exits);
    assert(lock_enters == 3U);
    app_exchange_init(&incomplete_lock);
    assert(lock_enters == 3U);

    assert(module_motor_init_base(&motor, &ops, "gimbal_yaw") == MODULE_MOTOR_STATUS_OK);
    assert(strcmp(module_motor_get_name(&motor), "gimbal_yaw") == 0);
    motor.is_registered = true;
    assert(module_motor_notify_feedback(&motor) == MODULE_MOTOR_STATUS_OK);

    assert(app_safety_init(NULL) == BSP_STATUS_OK);
    assert(!app_safety_output_allowed());
    assert(app_safety_monitor_init(&monitor, &monitor_config) == BSP_STATUS_OK);
    assert(app_safety_register(&monitor) == BSP_STATUS_OK);
    app_safety_set_output_enabled(true);
    app_safety_notify_online(&monitor, UINT32_MAX - 3U);
    app_safety_process(3U);
    assert(app_safety_output_allowed());
    assert(module_motor_enable(&motor) == MODULE_MOTOR_STATUS_OK);
    assert(enables == 1U);
    assert(module_motor_update(&motor, 0.002F) == MODULE_MOTOR_STATUS_OK);
    assert(module_motor_get_last_delta_time_s(&motor) == 0.002F);
    assert(module_motor_get_enabled_runtime_us(&motor) == 2000U);

    app_safety_process(8U);
    assert(!app_safety_output_allowed());
    assert(offline_enable_status == MODULE_MOTOR_STATUS_OUTPUT_INHIBITED);
    assert(module_motor_update(&motor, 0.001F) == MODULE_MOTOR_STATUS_OK);
    assert(disables == 1U);
    assert(updates == 1U);
    assert(module_motor_get_last_delta_time_s(&motor) == 0.002F);
    assert(module_motor_get_enabled_runtime_us(&motor) == 2000U);

    dji_motor.super.state = MODULE_MOTOR_STATE_ENABLED;
    dji_motor.command_value = 321;
    dji_bus.motor_slots[0][0] = &dji_motor;
    dji_bus.group_is_used[0] = true;
    assert(module_dji_motor_bus_flush(&dji_bus) == MODULE_MOTOR_STATUS_OK);
    assert(dji_motor.super.state == MODULE_MOTOR_STATE_DISABLED);
    assert(dji_motor.command_value == 0);
    assert(last_can_frame.data[0] == 0U && last_can_frame.data[1] == 0U);
    app_safety_notify_online(&monitor, 9U);
    app_safety_process(9U);
    assert(!app_safety_output_allowed());
    app_safety_set_output_enabled(true);
    app_safety_process(9U);
    assert(app_safety_output_allowed());
    assert(module_dji_motor_bus_flush(&dji_bus) == MODULE_MOTOR_STATUS_OK);
    assert(last_can_frame.data[0] == 0U && last_can_frame.data[1] == 0U);

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
