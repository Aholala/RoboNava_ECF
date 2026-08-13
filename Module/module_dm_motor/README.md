# module_dm_motor -- 达妙电机 CAN 协议驱动

## 功能概述

达妙电机模块只使用一个 `module_dm_motor_t`，DM4310 也直接复用该类型。通过 `module_dm_control_mode_t` 选择 MIT、速度、位置速度或力位混合模式，模式在初始化时确定。

控制目标通过对应模式的 `module_dm_motor_set_*_target()` 设置，只由 `module_dm_motor_bus_update()` 用真实 `dt` 更新并发送。全局安全门控关闭时拒绝使能和控制输出，但始终允许失能命令。

**数据流向：** CAN 反馈 --> `bus_handle_feedback`（按反馈 ID 路由）--> 反馈 --> `set_*_target` --> `bus_update`（轮询编码发送）

## 核心结构体

### 配置结构体 `module_dm_motor_config_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `name` | `const char *` | 调试显示名称 |
| `motor_bus` | `module_dm_motor_bus_t *` | 所属 DM 总线 |
| `control_mode` | `module_dm_control_mode_t` | 控制模式 |
| `master_identifier` | `uint32_t` | 主机标识符（CAN ID 基址） |
| `feedback_identifier` | `uint32_t` | 反馈标识符（CAN ID） |
| `transmit_timeout_ms` | `uint32_t` | CAN 发送超时 [ms] |
| `limits` | `module_dm_limits_t` | 限制参数（必须与调试工具一致） |

### 限制参数 `module_dm_limits_t`

用于浮点量量化到协议字段，必须与具体固件协议一致：

| 字段 | 说明 |
|------|------|
| `position_min_rad` / `position_max_rad` | 位置范围 [rad] |
| `velocity_min_rad_per_s` / `velocity_max_rad_per_s` | 速度范围 [rad/s] |
| `torque_min_nm` / `torque_max_nm` | 扭矩范围 [Nm] |
| `proportional_gain_min` / `proportional_gain_max` | Kp 范围 |
| `derivative_gain_min` / `derivative_gain_max` | Kd 范围 |

### 总线对象 `module_dm_motor_bus_t`

| 字段 | 说明 |
|------|------|
| `can` | CAN BSP 基类 |
| `motor_storage` | 电机数组（由调用者分配） |
| `motor_capacity` / `motor_count` | 数组容量 / 当前电机数 |
| `next_transmit_index` | 下一个要发送的电机索引（轮询） |
| `maximum_transmits_per_cycle` | 每周期最大发送帧数 |

### 控制模式枚举 `module_dm_control_mode_t`

| 枚举值 | 说明 |
|--------|------|
| `MODULE_DM_MODE_MIT` | MIT 模式（位置+速度+Kp+Kd+扭矩） |
| `MODULE_DM_MODE_VELOCITY` | 速度模式 |
| `MODULE_DM_MODE_POSITION_VELOCITY` | 位置+速度模式 |
| `MODULE_DM_MODE_FORCE_POSITION` | 力位混合模式 |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `module_dm_motor_bus_init(me, can, storage, cap, max_tx)` | 初始化总线 | `module_motor_status_t` |
| `module_dm_motor_init(me, config)` | 初始化电机实例 | `module_motor_status_t` |
| `module_dm_motor_register(me)` / `unregister(me)` | 注册 / 注销电机 | `module_motor_status_t` |
| `module_dm_motor_as_base(me)` | 向上转型为 `module_motor_t *` | 基类指针或 `NULL` |
| `module_dm_motor_bus_handle_feedback(me, frame)` | 路由反馈到对应电机 | `module_motor_status_t` |
| `module_dm_motor_bus_update(me, dt_s)` | 轮询编码并发送 | `module_motor_status_t` |
| `module_dm_motor_set_mit_target(me, cmd)` | 设置 MIT 目标 | `module_motor_status_t` |
| `module_dm_motor_set_velocity_target(me, v)` | 设置速度目标 | `module_motor_status_t` |
| `module_dm_motor_set_position_velocity_target(me, p, v)` | 设置位置+速度目标 | `module_motor_status_t` |
| `module_dm_motor_set_force_position_target(me, cmd)` | 设置力位混合目标 | `module_motor_status_t` |
| `module_dm_motor_send_state_command(me, cmd)` | 发送状态命令 | `module_motor_status_t` |
| `module_dm_motor_read_parameter(me, addr)` | 读取参数寄存器 | `module_motor_status_t` |
| `module_dm_motor_write_parameter_u32/float(me, addr, v)` | 写参数（不保存） | `module_motor_status_t` |
| `module_dm_motor_save_parameters(me)` | 保存参数到 Flash | `module_motor_status_t` |
| `module_dm_motor_set_communication_timeout(me, counts)` | 设置通信丢失超时 | `module_motor_status_t` |
| `module_dm_motor_get_fault(me)` | 读取当前故障码 | `module_dm_fault_t` |
| `module_dm_motor_get_mos_temperature_c(me)` | 读取 MOS 温度 [°C] | `float` |

## 控制模式与发送

| 模式 | 目标接口 | 载荷 | 发送 ID |
|------|----------|------|---------|
| MIT | `module_dm_motor_set_mit_target()` | 8 字节 | `master_identifier` |
| 速度 | `module_dm_motor_set_velocity_target()` | 4 字节 | `master_identifier + 0x200` |
| 位置速度 | `module_dm_motor_set_position_velocity_target()` | 8 字节 | `master_identifier + 0x100` |
| 力位混合 | `module_dm_motor_set_force_position_target()` | 8 字节 | `master_identifier + 0x300` |

## 状态命令 `module_dm_state_command_t`

| 命令 | 说明 |
|------|------|
| `MODULE_DM_COMMAND_DISABLE` | 失能 |
| `MODULE_DM_COMMAND_ENABLE` | 使能 |
| `MODULE_DM_COMMAND_SET_ZERO` | 将当前位置设为输出轴零位 |
| `MODULE_DM_COMMAND_CLEAR_FAULT` | 清除故障 |

## 故障码 `module_dm_fault_t`

| 值 | 含义 |
|----|------|
| 8 | 过压 |
| 9 | 欠压 |
| 10 | 过流 |
| 11 | MOS 管过温 |
| 12 | 电机过温 |
| 13 | 通信丢失 |
| 14 | 过载 |

## 使用示例

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

## 注意事项

1. **总线统一更新**：控制目标由 `module_dm_motor_bus_update()` 用真实 `dt` 更新并发送，不要再单独调用 `module_motor_update()`。
2. **`limits` 必须与固件一致**：DM4310 的 PMAX、VMAX、TMAX、Kp/Kd 范围及 CAN ID 可由调试工具修改，必须把电机当前实际值填入 `limits`，本模块不提供可能与固件不一致的型号默认值。
3. **安全门控**：全局安全门控关闭时拒绝使能和控制输出，但始终允许失能命令。
4. **参数保存有寿命**：`module_dm_motor_save_parameters()` 写入 Flash 约 10000 次寿命，且仅在 `DISABLED` 状态有效，禁止放进周期任务或每次启动调用。
5. **`SAVE_ZERO` 会持久化**：`MODULE_DM_COMMAND_SET_ZERO` 修改驱动器持久状态，需在机械位置确认、输出关闭后调用。
