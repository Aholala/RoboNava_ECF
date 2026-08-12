# app_command -- 遥控器解码与高层指令生成

## 功能概述

命令模块是整个机器人控制链的起点。它读取遥控器（DR16 接收机）的摇杆、拨杆、拨轮和鼠标/键盘输入，将其解码为底盘、云台、射击器三类高层指令，并通过交换层发布给对应的执行模块。支持 DR16 本地直连和板间通信转发两种数据来源，并处理视觉辅助超控（按住鼠标右键时云台锁定视觉目标）。

**数据流向：** DR16（硬件） / 板间通信 --> `app_command`（解码生成三类指令） --> `app_exchange`（分别 publish_chassis/gimbal/shooter_command） --> 各执行模块

## 核心结构体

### 配置结构体 `app_command_config_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `dr16` | `module_dr16_t *` | 本地 DR16 接收机实例（远端模式时为 NULL） |
| `board_comm` | `module_board_comm_t *` | 可选的板间通信链路，用于从其他单板获取遥控数据或转发本板数据 |
| `dr16_is_local` | `bool` | `true` 表示 DR16 连接在本板，`false` 时从 `board_comm` 获取 |
| `maximum_yaw_rate_rad_per_s` | `float` | 云台偏航最大角速率 [rad/s]，摇杆满幅对应此速率 |
| `maximum_pitch_rate_rad_per_s` | `float` | 云台俯仰最大角速率 [rad/s] |
| `minimum_pitch_rad` | `float` | 俯仰角机械下限 [rad] |
| `maximum_pitch_rad` | `float` | 俯仰角机械上限 [rad] |
| `maximum_chassis_velocity_m_per_s` | `float` | 底盘最大平移速度 [m/s] |
| `maximum_chassis_spin_rad_per_s` | `float` | 自旋模式下底盘最大偏航角速率 [rad/s] |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `app_command_init(config)` | 初始化命令模块单例，校验并保存配置 | `BSP_STATUS_OK` / `BSP_STATUS_INVALID_ARGUMENT` |
| `app_command_update(delta_time_s)` | 执行一个解码周期：读取遥控器 -> 生成底盘/云台/射击器指令 -> 发布到交换层 | `void` |

## 遥控器映射规则

### 摇杆通道分配

| 通道 | 映射功能 | 输出范围 |
|------|---------|---------|
| `channel[0]` | 云台偏航速率 | 满幅 = `maximum_yaw_rate_rad_per_s` |
| `channel[1]` | 云台俯仰速率 | 满幅 = `maximum_pitch_rate_rad_per_s`，钳位在 `[minimum_pitch_rad, maximum_pitch_rad]` |
| `channel[2]` | 底盘横向速度 (Y) | 满幅 = `maximum_chassis_velocity_m_per_s` |
| `channel[3]` | 底盘前向速度 (X) | 满幅 = `maximum_chassis_velocity_m_per_s` |

### 底盘模式切换（拨杆控制）

| 左拨杆 | 右拨杆 | 底盘模式 | 说明 |
|--------|--------|---------|------|
| DOWN | 任意 | `NO_FORCE` | 禁用动力输出 |
| UP | 任意 | `SPIN` | 以 `maximum_chassis_spin_rad_per_s` 自旋 |
| MIDDLE | DOWN | `FOLLOW_GIMBAL` | 偏航跟随云台朝向 |
| MIDDLE | UP/MIDDLE | `NORMAL` | 标准速度控制 |

### 云台反馈模式切换（右拨杆）

| 右拨杆 | 反馈模式 | 说明 |
|--------|---------|------|
| MIDDLE | `APP_GIMBAL_FEEDBACK_IMU` | 使用 IMU 姿态作为位置反馈 |
| UP/DOWN | `APP_GIMBAL_FEEDBACK_ENCODER` | 使用电机编码器作为位置反馈 |

### 射击器触发

| 操作 | 行为 |
|------|------|
| 拨轮 > 100 | 摩擦轮使能 |
| 拨轮 > 500 | 触发单发射击（每帧上升沿发射1发） |
| 鼠标右键按住 + 视觉跟踪就绪 | 启动自动连发 |

### 视觉辅助超控

当按住鼠标右键且视觉系统报告有效目标时，云台的偏航和俯仰目标角直接替换为视觉系统给出的目标角，反馈模式强制切换为 `APP_GIMBAL_FEEDBACK_IMU`。此超控优先级高于遥控器摇杆输入。

## 使用示例

```c
#include "app_command.h"
#include "module_dr16.h"
#include "module_board_comm.h"

/* --- 初始化阶段 --- */

module_dr16_t dr16;
// ... 初始化 DR16 (SPI/DMA 等) ...

app_command_config_t cmd_config = {
    .dr16                             = &dr16,
    .board_comm                       = NULL,   // 单板不转发
    .dr16_is_local                    = true,
    .maximum_yaw_rate_rad_per_s       = 3.14f,  // 180 deg/s
    .maximum_pitch_rate_rad_per_s     = 2.09f,  // 120 deg/s
    .minimum_pitch_rad               = -0.52f,  // -30 deg
    .maximum_pitch_rad               = 0.35f,   // +20 deg
    .maximum_chassis_velocity_m_per_s = 3.0f,   // 3 m/s
    .maximum_chassis_spin_rad_per_s   = 6.28f,  // 360 deg/s (1 rps)
};

bsp_status_t rc = app_command_init(&cmd_config);
if (rc != BSP_STATUS_OK) {
    // 错误处理
}

/* --- 周期性任务中（如 1kHz） --- */

void command_task(float dt_s)
{
    app_command_update(dt_s);
    // 指令已自动发布到 app_exchange，无需其他操作
}
```

## 双板（本地+远端）部署示例

当系统有两块 MCU 板（如底盘板和云台板），其中一块连接 DR16 并转发给另一块：

```c
// === 板 A（连接 DR16 的主板） ===
app_command_config_t config_a = {
    .dr16          = &dr16,
    .board_comm    = &board_comm,
    .dr16_is_local = true,
    // ... 其他参数 ...
};
app_command_init(&config_a);

// === 板 B（通过 CAN 接收遥控数据） ===
app_command_config_t config_b = {
    .dr16          = NULL,
    .board_comm    = &board_comm,
    .dr16_is_local = false,
    // ... 其他参数 ...
};
app_command_init(&config_b);
```

板 A 的 `app_command_update` 会同时将遥控数据通过 `module_board_comm_send_remote` 转发，板 B 的 `app_command_update` 通过 `module_board_comm_get_remote` 获取。

## 注意事项

1. **云台目标角是累积量**：偏航和俯仰目标角以速率限制方式逐帧累积，断电重启后从 0 rad 开始。首次启动时云台会从当前位置移动到 0 rad —— 建议上电时云台初始位置与 0 对齐，或在初始化后立即校准。
2. **单例模式**：本模块为全局单例（静态变量存储配置和状态），只需调用一次 `init`，不可多实例。
3. **遥控离线时所有指令 disable**：当 `app_command_get_remote` 返回 `false`（DR16 掉线且板间通信也无数据），所有三类指令的 `enabled` 均为 `false`，执行模块将断电。
4. **拨轮阈值和摩擦轮转速当前为硬编码**：`dial > 100`、`dial > 500`、`friction_velocity_rad_per_s = 500.0f` 在 `update` 中写死，如需调整需修改源码。
5. **视觉辅助超控优先级最高**：按住鼠标右键时云台直接锁定视觉目标，原有摇杆输入被覆盖。
