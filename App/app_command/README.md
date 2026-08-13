# app_command

`app_command` 把与设备无关的遥控输入转换成底盘、云台和发射命令。它不依赖 DR16、板间通信、视觉模块或 `app_exchange`。

DR16 适配层直接复制减去 1024 后的 `channel[]` 和 `dial` 整数，典型通道范围为 `[-660, 660]`。

## 初始化

```c
app_command_t command_app;
app_command_config_t config = {
    .channel_maximum_offset = 660,
    .maximum_yaw_rate_rad_per_s = 3.14f,
    .maximum_pitch_rate_rad_per_s = 2.09f,
    .minimum_pitch_rad = -0.52f,
    .maximum_pitch_rad = 0.35f,
    .maximum_chassis_velocity_m_per_s = 3.0f,
    .maximum_chassis_spin_rad_per_s = 6.28f,
    .friction_velocity_rad_per_s = 500.0f,
};
app_command_init(&command_app, &config);
```

## Task 调用

```c
app_command_update(&command_app, &remote_input,
                   app_gimbal_get_feedback(&gimbal),
                   vision_target_or_null, dt_s);

const app_command_output_t *command = app_command_get_output(&command_app);
app_chassis_update(&chassis, &command->chassis, dt_s);
app_gimbal_update(&gimbal, &command->gimbal, &imu_snapshot, dt_s);
app_shooter_update(&shooter, &command->shooter,
                   app_gimbal_get_feedback(&gimbal), dt_s);
```

`gimbal_feedback` 和 `vision_target` 都允许传 `NULL`。`automatic_vision_requested` 只是项目层请求，项目自行决定是否切换视觉模式。

多任务工程可以把 `app_command_output_t` 复制到自己的 RTOS 队列或项目快照；单任务工程直接使用返回的只读快照即可。
