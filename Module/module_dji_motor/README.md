# module_dji_motor — DJI 电机

一个 `module_dji_motor_t` 支持 M2006、M3508 和 GM6020。型号由
`module_dji_motor_config_t.motor_model` 指定；减速比、命令范围和 CAN 分组由模块内部选择。

## 接入顺序

1. `module_dji_motor_bus_init()` 初始化一条 CAN 总线。
2. 填写 `module_dji_motor_config_t`，选择 `motor_model` 和 `control_mode`。
3. `module_dji_motor_init()` 后调用 `module_dji_motor_register()`。
4. 通过 `module_motor_enable/set_target/update()` 控制 `motor.super`。
5. CAN 接收调用 `module_dji_motor_bus_handle_feedback()`，周期末调用
   `module_dji_motor_bus_flush()`。

```c
module_dji_motor_t motor;
module_dji_motor_config_t config = {
    .motor_name = "pitch",
    .motor_bus = &bus,
    .motor_model = MODULE_DJI_MOTOR_M3508,
    .control_mode = MODULE_DJI_CONTROL_ANGLE,
    .motor_identifier = 2,
    .direction_sign = 1.0f,
    // 其余保护、位置参考和 PID 参数按项目填写
};

module_dji_motor_init(&motor, &config);
module_dji_motor_register(&motor);
module_motor_enable(&motor.super);
module_motor_set_target(&motor.super, 0.5f);
module_motor_update(&motor.super, 0.001f);
module_dji_motor_bus_flush(&bus);
```

`MODULE_DJI_CONTROL_DIRECT/CURRENT/VELOCITY/ANGLE` 分别把 target 解释为原始协议命令、
电流 A、速度 rad/s 和角度 rad。反馈统一通过 `module_motor_get_feedback(&motor.super)` 读取。
