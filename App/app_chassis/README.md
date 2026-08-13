# app_chassis -- 麦轮/全向轮/舵轮统一底盘控制

## 功能概述

`app_chassis` 用同一套接口支持麦克纳姆轮、全向轮和舵轮三种底盘。底盘类型只在初始化时选择，周期任务始终调用 `app_chassis_update()`。指令使用 m/s、rad/s，`dt_s` 使用秒。

**数据流向：** `chassis_command` --> `app_chassis` --> `module_motor / module_swerve` --> `app_chassis_get_feedback()`

## 核心结构体

### 配置结构体 `app_chassis_config_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `type` | `app_chassis_type_t` | 底盘类型，决定 `drive` 联合体中哪个成员有效 |
| `follow_gain` | `float` | 跟随云台模式下的偏航角比例增益 |
| `stop_deadband` | `float` | 判定已停车的速度死区 [m/s 或 rad/s] |
| `drive.mecanum.kinematics` | `alg_mecanum_t *` | 麦轮运动学实例 |
| `drive.mecanum.motors` | `module_motor_t *[4]` | 四轮电机，顺序：左前、右前、左后、右后 |
| `drive.omni.kinematics` | `alg_omni_t *` | 全向轮运动学实例 |
| `drive.omni.motors` | `module_motor_t *[4]` | 轮电机数组，顺序与运动学配置一致 |
| `drive.omni.wheel_count` | `size_t` | 实际轮数（3 或 4） |
| `drive.swerve.kinematics` | `alg_swerve_t *` | 舵轮运动学实例 |
| `drive.swerve.modules` | `module_swerve_t *[4]` | 四模块，顺序：左前、右前、左后、右后 |

### 运行时实例 `app_chassis_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `config` | `app_chassis_config_t` | 静态配置的副本 |
| `feedback` | `app_chassis_feedback_t` | 最近一次底盘反馈 |
| `initialized` | `bool` | 初始化阶段已成功完成 |

### 底盘类型枚举 `app_chassis_type_t`

| 枚举值 | 说明 |
|--------|------|
| `APP_CHASSIS_TYPE_MECANUM` | 麦克纳姆轮底盘 |
| `APP_CHASSIS_TYPE_OMNI` | 全向轮底盘 |
| `APP_CHASSIS_TYPE_SWERVE` | 舵轮底盘 |

### 交换数据类型（定义在 `app_types.h`）

**`app_chassis_command_t`** -- 命令层发布的底盘运动指令：

| 字段 | 类型 | 说明 |
|------|------|------|
| `velocity_x_m_per_s` | `float` | 期望前向速度 [m/s] |
| `velocity_y_m_per_s` | `float` | 期望横向速度 [m/s] |
| `angular_velocity_rad_per_s` | `float` | 期望偏航角速率 [rad/s] |
| `gimbal_yaw_rad` | `float` | 当前云台偏航角 [rad] |
| `mode` | `app_chassis_mode_t` | 底盘驱动模式 |
| `self_lock_when_stopped` | `bool` | 速度为零时启用自锁 |
| `enabled` | `bool` | 指令有效标志，`false` 时电机断电 |
| `sequence` | `uint32_t` | 单调递增的帧序号 |

**`app_chassis_feedback_t`** -- 每周期发布的底盘反馈：

| 字段 | 类型 | 说明 |
|------|------|------|
| `velocity_x_m_per_s` | `float` | 指令前向速度 [m/s] |
| `velocity_y_m_per_s` | `float` | 指令横向速度 [m/s] |
| `angular_velocity_rad_per_s` | `float` | 指令偏航角速率 [rad/s] |
| `mode` | `app_chassis_mode_t` | 当前驱动模式 |
| `self_lock_active` | `bool` | 自锁配置已激活 |
| `motors_online` | `bool` | 全部底盘模块在线 |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `app_chassis_init(me, config)` | 初始化底盘实例，校验配置并拷贝 | `BSP_STATUS_OK` / `BSP_STATUS_INVALID_ARGUMENT` |
| `app_chassis_update(me, command, dt_s)` | 计算并设置电机/舵轮目标 | `bsp_status_t` |
| `app_chassis_get_feedback(me)` | 读取最近底盘反馈 | 只读指针或 `NULL` |

## 底盘模式

```
command.mode
    |
    +-- APP_CHASSIS_MODE_NO_FORCE ------> 禁用全部执行器
    |
    +-- APP_CHASSIS_MODE_NORMAL --------> 直接下发三轴速度
    |
    +-- APP_CHASSIS_MODE_SPIN ----------> 按最大角速率持续旋转
    |
    +-- APP_CHASSIS_MODE_FOLLOW_GIMBAL -> follow_gain * wrap(gimbal_yaw_rad)
```

停车自锁（`self_lock_when_stopped` 且三轴速度均低于 `stop_deadband`）：舵轮使用 X 锁，麦轮/全向轮无转向机构，使用零轮速。

## 使用示例

### 麦轮配置

```c
alg_mecanum_t mecanum;
alg_mecanum_config_t geometry = {
    .wheel_radius_m = 0.076f,
    .half_wheelbase_m = 0.20f,
    .half_track_width_m = 0.18f,
    .direction_sign = {1, 1, 1, 1},
    .odometry_weight = {1, 1, 1, 1},
    .maximum_wheel_angular_velocity_rad_per_s = 100.0f,
    .roller_arrangement = ALG_MECANUM_ROLLER_X,
};
alg_mecanum_init(&mecanum, &geometry);

app_chassis_config_t config = {
    .type = APP_CHASSIS_TYPE_MECANUM,
    .follow_gain = 5.0f,
    .stop_deadband = 0.02f,
    .drive.mecanum = {
        .kinematics = &mecanum,
        .motors = {&motor_fl.super, &motor_fr.super,
                   &motor_rl.super, &motor_rr.super},
    },
};
app_chassis_init(&chassis, &config);
```

### 全向轮配置

```c
app_chassis_config_t config = {
    .type = APP_CHASSIS_TYPE_OMNI,
    .follow_gain = 5.0f,
    .stop_deadband = 0.02f,
    .drive.omni = {
        .kinematics = &omni,
        .motors = {&omni_1.super, &omni_2.super, &omni_3.super},
        .wheel_count = 3,
    },
};
app_chassis_init(&chassis, &config);
```

### 舵轮配置

```c
app_chassis_config_t config = {
    .type = APP_CHASSIS_TYPE_SWERVE,
    .follow_gain = 5.0f,
    .stop_deadband = 0.02f,
    .drive.swerve = {
        .kinematics = &swerve,
        .modules = {&module_fl, &module_fr, &module_rl, &module_rr},
    },
};
app_chassis_init(&chassis, &config);
```

### Task 调用

```c
void chassis_task(void)
{
    const float dt_s = project_get_chassis_dt_s();
    app_chassis_command_t command = project_get_chassis_command();

    if (app_chassis_update(&chassis, &command, dt_s) != BSP_STATUS_OK) {
        project_report_chassis_fault();
    }
    module_dji_motor_bus_update(&dji_bus, dt_s);
    module_dm_motor_bus_update(&dm_bus, dt_s);
}
```

## 注意事项

1. **电机总线统一刷新**：`app_chassis_update()` 只计算并设置电机目标。Task 在所有 App 更新完成后，必须对每条 DJI/DM 总线各调用一次 `module_*_motor_bus_update(bus, dt_s)`，不要再单独调用 `module_motor_update()`。
2. **轮序固定**：麦轮与舵轮顺序固定为左前、右前、左后、右后；全向轮数组顺序必须与 `alg_omni_t` 的轮组配置顺序一致。
3. **跟随云台需要参考角**：`APP_CHASSIS_MODE_FOLLOW_GIMBAL` 使用 `command.gimbal_yaw_rad` 计算偏航角速率，调用方需传入有效云台偏航角。
4. **无力模式禁用全部电机**：`mode == APP_CHASSIS_MODE_NO_FORCE` 或 `enabled == false` 时，模块禁用全部执行器并发布无力反馈。
5. **停车自锁按底盘类型区分**：舵轮自锁为 X 锁（交叉转向锁死），麦轮/全向轮没有转向机构，自锁等价于零轮速。
