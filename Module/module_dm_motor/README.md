# module_dm_motor

达妙电机模块使用一个 `module_dm_motor_t`，通过 `module_dm_control_mode_t` 选择 MIT、速度、位置速度或力位混合模式。模式在初始化时确定，不使用模式继承层。

```c
static module_dm_motor_t motor;
static module_dm_motor_t *bus_storage[4];
static module_dm_motor_bus_t bus;

void dm_init(void)
{
    const module_dm_motor_config_t config = {
        .name = "shooter_motor",
        .can = &can1,
        .control_mode = MODULE_DM_MODE_MIT,
        .master_identifier = 0x01U,
        .feedback_identifier = 0x01U,
        .transmit_timeout_ms = 1U,
        .limits = protocol_limits, /* 必须与电机调试工具中的参数一致 */
    };

    (void)module_dm_motor_bus_init(&bus, &can1, bus_storage, 4U, 4U);
    (void)module_dm_motor_init(&motor, &config);
    (void)module_dm_motor_register(&motor, &bus);
}
```

控制目标通过对应模式的 `module_dm_motor_set_*_target()` 设置，再由 `module_motor_update()` 或 `module_dm_motor_bus_update()` 发送。全局安全门控关闭时拒绝使能和控制输出，但始终允许失能命令。

DM4310 的便捷配置和协议范围说明见 [dm4310.md](dm4310.md)，完整工程组织见 [使用与移植指南](../../Docs/USAGE_AND_PORTING.md)。
