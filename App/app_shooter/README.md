# app_shooter -- 发射机构控制（摩擦轮 + 拨弹）

## 功能概述

射击器模块管理摩擦轮启停与转速控制，以及拨弹机构的单发和自动连发逻辑。通过 `fire_requested` 上升沿检测实现每次按下只发射 1 发，通过 `automatic_fire_enabled` 配合视觉跟踪和云台锁定状态实现持续连发。支持本地和远端分体部署（摩擦轮和拨弹可能在不同单板上）。

**数据流向：** `app_exchange`（shooter_command + gimbal_feedback） --> `app_shooter` --> `module_shooter`（硬件驱动）--> 反馈发布在模块内部

## 核心结构体

### 配置结构体 `app_shooter_config_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `shooter` | `module_shooter_t *` | 射击器硬件抽象实例（管理摩擦轮电机和拨弹电机） |
| `board_comm` | `module_board_comm_t *` | 可选的板间通信链路，用于转发射击器反馈（可为 NULL） |

### 运行时实例 `app_shooter_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `config` | `app_shooter_config_t` | 静态配置的副本 |
| `previous_fire_request` | `bool` | 上一帧的开火标志（用于上升沿检测单发） |
| `initialized` | `bool` | 初始化阶段已成功完成 |

### 交换数据类型（定义在 `app_types.h`）

**`app_shooter_command_t`** -- 命令层发布的射击指令：

| 字段 | 类型 | 说明 |
|------|------|------|
| `friction_enabled` | `bool` | 摩擦轮使能旋转 |
| `fire_requested` | `bool` | 单发触发（每帧上升沿发射 1 发） |
| `automatic_fire_enabled` | `bool` | 跟踪时允许自动连续发射 |
| `friction_velocity_rad_per_s` | `float` | 摩擦轮目标转速 [rad/s] |
| `sequence` | `uint32_t` | 单调递增的帧序号 |

**`app_shooter_feedback_t`** -- 射击器反馈：

| 字段 | 类型 | 说明 |
|------|------|------|
| `state` | `uint8_t` | 射击器状态机当前状态（来自 `module_shooter_get_state`） |
| `jam_retry_count` | `uint8_t` | 连续卡弹重试次数 |
| `friction_ready` | `bool` | 摩擦轮已达到目标转速 |
| `fire_permission` | `bool` | 裁判系统/安全开火许可 |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `app_shooter_init(me, config)` | 初始化射击器实例，校验参数并拷贝配置 | `BSP_STATUS_OK` / `BSP_STATUS_INVALID_ARGUMENT` |
| `app_shooter_update(me, delta_time_s)` | 执行一个射击器控制周期：读指令 -> 控制摩擦轮 -> 处理单发/连发 -> 更新硬件 -> 转发反馈 | `void` |

## 发射逻辑

### 单发射击（上升沿检测）

```
previous_fire_request    fire_requested    行为
       false                  false        无操作
       false                  true         发射 1 发 (module_shooter_request_shots(1))
       true                   true         不发射（已触发过，等待下降沿重置）
       true                   false        previous_fire_request = false（复位，等待下次上升沿）
```

### 自动连发

```
automatic_fire_enabled == true
  且 gimbal.target_locked == true（云台已锁定目标）
  且 referee_allows_fire == true（裁判系统允许）
  --> module_shooter_update_fire_control() 持续发射
```

### 分体部署（远端摩擦轮）

当本板没有本地摩擦轮（`shooter->has_local_friction == false`）但远端板的摩擦轮在线时，通过 `module_board_comm_get_shooter` 获取远端摩擦轮的就绪状态，调用 `module_shooter_set_external_friction_ready` 同步给本地 `module_shooter`。

## 使用示例

```c
#include "app_shooter.h"
#include "app_exchange.h"
#include "module_shooter.h"

/* --- 初始化阶段 --- */

module_shooter_t shooter;
// 假设 shooter 已初始化（绑定摩擦轮电机、拨弹电机等）...

app_shooter_t app_shooter;
app_shooter_config_t config = {
    .shooter    = &shooter,
    .board_comm = NULL,     // 无裁判系统
};

bsp_status_t rc = app_shooter_init(&app_shooter, &config);
if (rc != BSP_STATUS_OK) {
    // 错误处理
}

/* --- 周期性任务中（如 1kHz） --- */

void shooter_task(float dt_s)
{
    app_shooter_update(&app_shooter, dt_s);
}
```

## 射击器状态查询示例

射击器反馈（`app_shooter_feedback_t`）中的 `state` 字段来自 `module_shooter` 层。典型的状态包括：

| state 值 | 含义 |
|---------|------|
| `MODULE_SHOOTER_STATE_DISABLED` | 摩擦轮未启动 |
| `MODULE_SHOOTER_STATE_SPINNING_UP` | 摩擦轮加速中 |
| `MODULE_SHOOTER_STATE_READY` | 摩擦轮转速就绪，可发射 |
| `MODULE_SHOOTER_STATE_FIRING` | 正在发射 |
| ... | （其他由 `module_shooter` 定义的状态） |

## 注意事项

1. **单发依赖上升沿检测**：`fire_requested` 必须从 `false` 变为 `true` 才会触发射击。如果指令生产者每帧都写 `true`，则只有第一帧会触发单发 —— 这是设计行为。释放后再按下才会触发下一发。
2. **自动连发需要云台锁定**：`automatic_fire_enabled` 为 `true` 时，还需满足 `gimbal.target_locked == true` 才会实际发射。这确保只有对准目标时才发射。
3. **摩擦轮启停有延迟**：`friction_enabled` 切换摩擦轮启停时，需要等待 `module_shooter` 层的状态机完成加速/减速。通过 `friction_ready` 反馈标志可判断是否就绪。
4. **分体部署**：如果拨弹电机在本板而摩擦轮在远端板，需正确配置 `board_comm` 和 `shooter->has_local_friction`。远端摩擦轮的就绪状态通过板间通信同步。
5. **裁判系统权限**：当前代码中 `referee_allows_fire` 硬编码为 `true`，实际集成裁判系统时需替换为从裁判系统读取的真实值。
