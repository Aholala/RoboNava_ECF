# module_dm_motor

达妙电机模块只使用一个 `module_dm_motor_t`，DM4310 也直接使用该类型。通过 `module_dm_control_mode_t` 选择 MIT、速度、位置速度或力位混合模式；模式在初始化时确定，不使用模式继承层。

```c
static module_dm_motor_t motor;
static module_dm_motor_t *bus_storage[4];
static module_dm_motor_bus_t bus;

void dm_init(void)
{
    const module_dm_motor_config_t config = {
        .name = "shooter_motor",
        .motor_bus = &bus,
        .control_mode = MODULE_DM_MODE_MIT,
        .master_identifier = 0x01U,
        .feedback_identifier = 0x01U,
        .transmit_timeout_ms = 1U,
        .limits = protocol_limits, /* 必须与电机调试工具中的参数一致 */
    };

    (void)module_dm_motor_bus_init(&bus, &can1, bus_storage, 4U, 4U);
    (void)module_dm_motor_init(&motor, &config);
    (void)module_dm_motor_register(&motor);
}
```

控制目标通过对应模式的 `module_dm_motor_set_*_target()` 设置，只由 `module_dm_motor_bus_update()` 用真实 `dt` 更新并发送。不要再单独调用 `module_motor_update()`。全局安全门控关闭时拒绝使能和控制输出，但始终允许失能命令。

## 控制模式

| 模式 | 目标接口 | 载荷 | 发送 ID |
| --- | --- | ---: | ---: |
| MIT | `module_dm_motor_set_mit_target()` | 8 字节 | `master_identifier` |
| 速度 | `module_dm_motor_set_velocity_target()` | 4 字节 | `master_identifier + 0x200` |
| 位置速度 | `module_dm_motor_set_position_velocity_target()` | 8 字节 | `master_identifier + 0x100` |
| 力位混合 | `module_dm_motor_set_force_position_target()` | 8 字节 | `master_identifier + 0x300` |

ACE_ECF_26 将前三种模式拆成三个类；本框架只复用其协议编码思路，内部由独立编码函数和 `control_mode` 选择，避免重复 CAN 和状态逻辑。

DM4310 的 PMAX、VMAX、TMAX、Kp/Kd 范围以及 CAN ID 可由调试工具修改，必须把电机当前实际值填入 `limits`，不提供可能与固件不一致的型号默认值。

状态命令直接使用 `module_dm_motor_send_state_command()`；通信超时通过 `module_dm_motor_set_communication_timeout()` 设置，需要持久化时再调用 `module_dm_motor_save_parameters()`。完整工程组织见 [移植手册](../../移植手册.md)。
