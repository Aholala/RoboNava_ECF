# module_dji_motor -- DJI M2006/M3508/GM6020 CAN 协议驱动

## 功能概述

一个 `module_dji_motor_t` 支持 M2006、M3508 和 GM6020 三种电机，型号由 `module_dji_motor_config_t.motor_model` 指定。减速比、命令量程和 CAN 分组由模块内部按型号选择，反馈统一通过 `module_motor_get_feedback(&motor.super)` 读取。

**数据流向：** CAN 反馈 --> `bus_handle_feedback` --> 解码/多圈累计 --> 反馈 --> `module_motor_update`（级联 PID）--> `bus_update` 打包发送

## 核心结构体

### 配置结构体 `module_dji_motor_config_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `name` | `const char *` | 调试显示名称 |
| `motor_bus` | `module_dji_motor_bus_t *` | 所属总线 |
| `motor_model` | `module_dji_motor_model_t` | 电机型号 |
| `control_mode` | `module_dji_control_mode_t` | 控制模式 |
| `motor_identifier` | `uint8_t` | 电机标识符（1~8） |
| `direction_sign` | `float` | 方向符号（`+1` 或 `-1`） |
| `maximum_temperature_c` | `float` | 最大允许温度 [°C] |
| `current_scale_a_per_count` | `float` | 电流换算因子 [A/原始值] |
| `position_reference` | `module_dji_position_reference_t` | 位置参考方式 |
| `encoder_zero_count` | `uint16_t` | 机械零位编码器值（0~8191） |
| `position_offset_rad` | `float` | 位置附加偏移 [rad] |
| `current_pid_config` | `module_motor_pid_config_t` | 电流环 PID 配置 |
| `velocity_pid_config` | `module_motor_pid_config_t` | 速度环 PID 配置 |
| `angle_pid_config` | `module_motor_pid_config_t` | 角度环 PID 配置 |

### 电机对象 `module_dji_motor_t`

| 字段 | 说明 |
|------|------|
| `super` | 电机基类（`module_motor_t`），通过 `.super` 使用通用接口 |
| `motor_bus` / `motor_model` / `control_mode` | 所属总线、型号、控制模式 |
| `current_pid` / `velocity_pid` / `angle_pid` | 三级 PID 控制器 |
| `target_current_a` / `target_velocity_rad_per_s` / `target_angle_rad` | 各环目标 |
| `accumulated_encoder_count` | 累积编码器值（多圈） |
| `receive_identifier` / `group_index` / `group_slot` | 协议映射结果 |

### 总线对象 `module_dji_motor_bus_t`

| 字段 | 说明 |
|------|------|
| `can` | CAN BSP 基类 |
| `motor_slots[5][4]` | 5 个发送组 × 4 个槽位 |
| `group_is_used[5]` | 各组是否被使用 |
| `transmit_timeout_ms` | CAN 发送超时 [ms] |

### 型号枚举 `module_dji_motor_model_t`

| 枚举值 | 说明 | 减速比 | 命令量程 |
|--------|------|--------|----------|
| `MODULE_DJI_MOTOR_M2006` | M2006 直流无刷 | 36:1 | 10000 |
| `MODULE_DJI_MOTOR_M3508` | M3508 直流无刷 | 3591/187 | 16384 |
| `MODULE_DJI_MOTOR_GM6020` | GM6020 云台电机 | 1:1 | 25000（电压）/ 16384（电流） |

### 位置参考枚举 `module_dji_position_reference_t`

| 枚举值 | 说明 |
|--------|------|
| `MODULE_DJI_POSITION_BOOT_RELATIVE` | 首次反馈位置作为 0 rad |
| `MODULE_DJI_POSITION_ENCODER_ABSOLUTE` | 使用编码器机械零位作为单圈绝对参考 |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `module_dji_motor_bus_init(me, can, timeout)` | 初始化一条 CAN 总线 | `module_motor_status_t` |
| `module_dji_motor_init(me, config)` | 初始化电机实例 | `module_motor_status_t` |
| `module_dji_motor_register(me)` | 注册电机到总线槽位 | `module_motor_status_t` |
| `module_dji_motor_unregister(me)` | 从总线槽位注销电机 | `module_motor_status_t` |
| `module_dji_motor_as_base(me)` | 向上转型为 `module_motor_t *` | 基类指针或 `NULL` |
| `module_dji_motor_bus_handle_feedback(me, frame)` | 解码 CAN 反馈帧 | `module_motor_status_t` |
| `module_dji_motor_bus_update(me, dt_s)` | 统一计算并打包发送 | `module_motor_status_t` |
| `module_dji_motor_get_command(me)` | 读取当前命令值（调试） | `int16_t` |
| `module_dji_motor_reset_position(me, rad)` | 重定义当前位置 | `module_motor_status_t` |

## 控制模式

`MODULE_DJI_CONTROL_DIRECT/CURRENT/VELOCITY/ANGLE` 分别把 `set_target()` 的 target 解释为原始协议命令、电流 A、速度 rad/s 和角度 rad。

```
ANGLE    目标角   -> angle_pid    -> velocity_pid -> current_pid -> command
VELOCITY 目标速度 ->               velocity_pid -> current_pid -> command
CURRENT  目标电流 ->                               current_pid -> command
DIRECT   协议命令 ->                                               command
```

## CAN 协议映射

| 型号 | 接收 ID | 发送组 |
|------|---------|--------|
| M2006 / M3508（ID 1~4） | 0x201~0x204 | 0x200 |
| M2006 / M3508（ID 5~8） | 0x205~0x208 | 0x1FF |
| GM6020（ID 1~4，电压） | 0x205~0x208 | 0x1FF |
| GM6020（ID 5~7，电压） | 0x209~0x20B | 0x2FF |
| GM6020（ID 1~4，电流） | 0x205~0x208 | 0x1FE |
| GM6020（ID 5~7，电流） | 0x209~0x20B | 0x2FE |

反馈帧为标准 8 字节数据帧：编码器 13 位（0~8191，高字节在前）、速度 RPM、电流原始值、温度。

## 使用示例

接入顺序：

1. `module_dji_motor_bus_init()` 初始化一条 CAN 总线。
2. 填写 `module_dji_motor_config_t`，选择 `motor_model` 和 `control_mode`。
3. `module_dji_motor_init()` 后调用 `module_dji_motor_register()`。
4. 通过 `module_motor_enable/set_target()` 控制 `motor.super`。
5. CAN 接收调用 `module_dji_motor_bus_handle_feedback()`，每个控制周期调用 `module_dji_motor_bus_update()`。

```c
module_dji_motor_t motor;
module_dji_motor_config_t config = {
    .name = "gimbal_yaw",
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
module_dji_motor_bus_update(&bus, 0.001f);
```

## 注意事项

1. **总线统一刷新**：由 `module_dji_motor_bus_update()` 统一计算并打包发送，不要再单独调用 `module_motor_update()`。
2. **电机 ID 唯一**：同一总线上每个电机的 `motor_identifier` 必须唯一，注册时会检查槽位占用与重复接收 ID。
3. **`current_scale_a_per_count` 与型号匹配**：非 `DIRECT` 模式必须大于 0，否则电流反馈无效且 PID 无法运行。
4. **编码器多圈累计**：模块内部处理 13 位编码器回绕并累积多圈位置；`position_reference` 决定上电零位是相对还是绝对。
5. **过温保护**：反馈温度超过 `maximum_temperature_c` 时命令清零并进入 `MODULE_MOTOR_STATE_FAULT`。
