# App — 可复用控制流程

App 层只组织机器人控制逻辑，不绑定 HAL、FreeRTOS、CAN ID 或具体任务。

| 模块 | 职责 |
|---|---|
| `app_command` | 通用遥控输入转换为底盘、云台和发射命令 |
| `app_chassis` | 麦轮、全向轮和舵轮底盘控制 |
| `app_gimbal` | 双轴云台控制，支持编码器或 IMU 反馈 |
| `app_shooter` | 摩擦轮、拨弹和自动开火逻辑 |
| `app_imu` | BMI088 采样和姿态解算 |
| `app_vision` | USB 视觉目标通信 |
| `app_safety` | 心跳、失联和整机输出门控 |
| `app_types.h` | App 之间显式传递的命令和反馈类型 |

所有有状态 App 都由调用者持有实例。输入通过参数传入，输出通过 getter 读取：

```c
app_vision_update(&vision, app_imu_get_snapshot(&imu), elapsed_ms);
app_command_update(&command, &remote,
                   app_gimbal_get_feedback(&gimbal),
                   app_vision_get_target(&vision), dt_s);

const app_command_output_t *output = app_command_get_output(&command);
app_chassis_update(&chassis, &output->chassis, dt_s);
app_gimbal_update(&gimbal, &output->gimbal,
                  app_imu_get_snapshot(&imu), dt_s);
app_shooter_update(&shooter, &output->shooter,
                   app_gimbal_get_feedback(&gimbal), dt_s);

module_dji_motor_bus_update(&dji_bus, dt_s);
module_dm_motor_bus_update(&dm_bus, dt_s);
```

多任务同步由机器人项目负责：使用目标 RTOS 的队列、任务通知或项目局部快照。ECF 不提供全局交换层。
