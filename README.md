# RoboNava ECF

面向 RoboMaster 的可移植 C11 控制框架。框架提供算法、外设抽象、设备模块和可复用控制流程；具体机器人的 HAL 句柄、CAN ID、任务、优先级和整机参数留在项目中。

## 分层

```text
Robot Project   任务、板级句柄、CAN ID、整机装配
      ↓
App             command/chassis/gimbal/shooter/imu/vision/safety
      ↓
Module          DJI/DM 电机、BMI088、DR16、裁判系统等
      ↓
BSP             CAN/SPI/UART/USB/PWM 等驱动接口

Algorithm       PID、EKF、底盘运动学等纯算法
```

ECF 不依赖 FreeRTOS，不包含全局数据交换层。单任务直接传递快照；多任务由项目使用 RTOS 队列、任务通知或项目局部锁。

## 主要能力

- DJI M2006/M3508/GM6020 和 DM 电机，总线周期统一更新与发送。
- 麦克纳姆轮、三/四轮全向轮和四舵轮统一 `app_chassis` 接口。
- BMI088 静止标定、温度控制、温稳判定和姿态 EKF。
- DR16 原始去中值整数输入。
- 云台、摩擦轮、拨弹状态机、视觉 USB 协议。
- 裁判系统解析、UI、功率/热量数据仓库。
- 实例化 Safety 心跳监控和整机电机输出门控。
- 静态内存、SI 单位、显式 `dt`。

## 控制周期

```c
app_imu_update(&robot.imu, dt_s);
app_vision_update(&robot.vision, app_imu_get_snapshot(&robot.imu), elapsed_ms);

app_command_update(&robot.command, &remote_input,
                   app_gimbal_get_feedback(&robot.gimbal),
                   app_vision_get_target(&robot.vision), dt_s);

const app_command_output_t *cmd = app_command_get_output(&robot.command);
app_chassis_update(&robot.chassis, &cmd->chassis, dt_s);
app_gimbal_update(&robot.gimbal, &cmd->gimbal,
                  app_imu_get_snapshot(&robot.imu), dt_s);
app_shooter_update(&robot.shooter, &cmd->shooter,
                   app_gimbal_get_feedback(&robot.gimbal), dt_s);

module_dji_motor_bus_update(&robot.dji_bus, dt_s);
module_dm_motor_bus_update(&robot.dm_bus, dt_s);
app_safety_process(&robot.safety, now_ms);
```

App 和组合 Module 只设置电机目标。每条电机总线每周期更新一次，避免 PID、`dt` 和运行时间重复累计。

## IMU 启动

```c
app_imu_init(&robot.imu, &imu_config);
if (app_imu_calibrate(&robot.imu) != BSP_STATUS_OK) {
    /* 保持电机输出禁止，提示设备必须静止后重试 */
}
```

静止标定成功后进入 `WARMING`；温度在目标容差内连续稳定达到配置时间后进入 `READY`，此时快照才标记有效。加热输出通过项目提供的占空比回调连接 PWM，不绑定特定定时器。

## 电机命名

- 云台：`gimbal_yaw`、`gimbal_pitch`
- 拨弹：`shooter_motor`
- 普通底盘：按布局使用 `chassis_front_left` 等明确名称
- 舵轮：`swerve_drive_1..4`、`swerve_steer_1..4`

名称指针必须在电机生命周期内有效。可通过 `module_motor_get_name()`、`module_motor_get_last_delta_time_s()` 和 `module_motor_get_enabled_runtime_us()` 调试。

## 文档

- [移植手册](移植手册.md)
- [App 使用说明](App/README.md)
- [底盘配置](App/app_chassis/README.md)
- [命令映射](App/app_command/README.md)
- [视觉协议](App/app_vision/README.md)
- [电机说明](Module/module_motor/README.md)

## 验证

```powershell
./Tests/run_tests.ps1
```

测试覆盖控制链、安全门、电机总线、CRC、IMU EKF，以及三种底盘的黄金结果。硬件移植后仍需实测 CAN 零输出、DM 失能、BMI088 轴向/温控和真实任务周期。
