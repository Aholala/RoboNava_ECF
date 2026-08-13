# app_chassis

`app_chassis` 用同一套接口支持麦克纳姆轮、全向轮和舵轮底盘。底盘类型只在初始化时选择，周期任务始终调用：

```c
app_chassis_update(&chassis, &command, dt_s);
```

## 公共接口

```c
bsp_status_t app_chassis_init(app_chassis_t *me,
                              const app_chassis_config_t *config);
bsp_status_t app_chassis_update(app_chassis_t *me,
                                const app_chassis_command_t *command,
                                float dt_s);
const app_chassis_feedback_t *app_chassis_get_feedback(const app_chassis_t *me);
```

Task 不需要判断底盘类型。`command` 使用 m/s、rad/s，`dt_s` 使用秒。

## 麦轮配置

轮序固定为左前、右前、左后、右后。

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

## 全向轮配置

支持三轮或四轮。电机数组顺序必须与 `alg_omni_t` 的轮组配置顺序相同。

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

## 舵轮配置

四个模块顺序固定为左前、右前、左后、右后。

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

## Task 调用

```c
void chassis_task(void)
{
    const float dt_s = project_get_chassis_dt_s();
    app_chassis_command_t command = project_get_chassis_command();

    if (app_chassis_update(&chassis, &command, dt_s) != BSP_STATUS_OK) {
        project_report_chassis_fault();
    }
}
```

无力模式会禁用全部电机。停车自锁对舵轮使用 X 锁；麦轮和全向轮没有转向机构，因此使用零轮速。
