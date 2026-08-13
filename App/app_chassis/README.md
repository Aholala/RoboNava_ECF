# app_chassis -- 全向轮系（Swerve）底盘运动控制

## 功能概述

底盘控制模块负责将显式传入的运动指令逆解为四个舵轮模块的转角与转速目标，并保存最新反馈。

**数据流向：** `app_command`（发布指令） --> `app_exchange`（中转） --> `app_chassis`（消费指令，执行控制） --> `app_chassis_get_feedback()`

## 核心结构体

### 配置结构体 `app_chassis_config_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `kinematics` | `alg_swerve_t *` | 轮系运动学模型实例（含底盘几何参数） |
| `modules` | `module_swerve_t *[4]` | 四个舵轮模块实例数组 |
| `follow_gain` | `float` | 跟随云台模式的增益系数，将 yaw 误差映射为角速率 |
| `stop_deadband` | `float` | 判定停车的速度死区 [m/s 或 rad/s]，三轴均低于此值视为停止 |

### 运行时实例 `app_chassis_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `config` | `app_chassis_config_t` | 静态配置的副本 |
| `feedback` | `app_chassis_feedback_t` | 最近一次控制反馈 |
| `initialized` | `bool` | 初始化阶段已成功完成 |

### 底盘驱动模式 `app_chassis_mode_t`（定义在 `app_types.h`）

| 枚举值 | 说明 |
|--------|------|
| `APP_CHASSIS_MODE_NO_FORCE` | 无动力输出，模块被禁用或拨杆打到下位 |
| `APP_CHASSIS_MODE_NORMAL` | 标准速度控制，摇杆直接映射平移+旋转 |
| `APP_CHASSIS_MODE_SPIN` | 以配置的最大角速率持续自旋 |
| `APP_CHASSIS_MODE_FOLLOW_GIMBAL` | 偏航角自动收敛至云台朝向，`follow_gain` 控制收敛速率 |

### 交换数据类型（定义在 `app_types.h`）

**`app_chassis_command_t`** -- 命令层发布的底盘运动指令：

| 字段 | 类型 | 说明 |
|------|------|------|
| `velocity_x_m_per_s` | `float` | 期望前向速度 [m/s] |
| `velocity_y_m_per_s` | `float` | 期望横向速度 [m/s] |
| `angular_velocity_rad_per_s` | `float` | 期望偏航角速率 [rad/s] |
| `gimbal_yaw_rad` | `float` | 当前云台偏航角 [rad]（用于跟随模式和参考系转换） |
| `mode` | `app_chassis_mode_t` | 底盘驱动模式 |
| `self_lock_when_stopped` | `bool` | 速度为零时启用自锁 |
| `enabled` | `bool` | 指令有效标志，`false` 时所有模块断电 |
| `sequence` | `uint32_t` | 单调递增的帧序号 |

**`app_chassis_feedback_t`** -- 底盘反馈：

| 字段 | 类型 | 说明 |
|------|------|------|
| `velocity_x_m_per_s` | `float` | 指令前向速度 [m/s] |
| `velocity_y_m_per_s` | `float` | 指令横向速度 [m/s] |
| `angular_velocity_rad_per_s` | `float` | 指令偏航角速率 [rad/s] |
| `mode` | `app_chassis_mode_t` | 当前驱动模式 |
| `self_lock_active` | `bool` | 自锁配置已激活 |
| `motors_online` | `bool` | 全部四个舵轮模块均正常在线 |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `app_chassis_init(me, config)` | 初始化底盘实例，校验参数并拷贝配置 | `BSP_STATUS_OK` / `BSP_STATUS_INVALID_ARGUMENT` |
| `app_chassis_update(me, command, delta_time_s)` | 执行一个底盘控制周期：显式指令 -> 运动学逆解 -> 驱动舵轮模块 -> 保存反馈 | `bsp_status_t` |
| `app_chassis_get_feedback(me)` | 读取最近底盘反馈 | 只读指针或 `NULL` |

## 状态机 / 工作模式

```
                  +------------------+
                  |   NO_FORCE       |  <-- enabled=false 或 mode=NO_FORCE
                  | (所有模块断电)    |
                  +-------+----------+
                          |
                          | enabled=true 且模式切换
                          v
          +---------------+---------------+
          |               |               |
          v               v               v
  +-------+----+  +-------+----+  +-------+----------+
  | NORMAL     |  | SPIN       |  | FOLLOW_GIMBAL    |
  | 摇杆映射    |  | 持续自旋    |  | 偏航角锁定云台    |
  +-----+------+  +-----+------+  +-----+------------+
        |               |               |
        +-------+-------+-------+-------+
                |
                | 摇杆归中 (三轴均在 deadband 内)
                v
        +-------+--------+
        | 自锁 (SELF-LOCK)|
        | 模块锁定当前位置 |
        +----------------+
```

模式优先级（由 `app_command` 决定）：NO_FORCE > SPIN > FOLLOW_GIMBAL > NORMAL。

## 使用示例

```c
#include "app_chassis.h"
#include "alg_swerve.h"
#include "module_swerve.h"

/* --- 初始化阶段 --- */

// 1. 准备运动学模型（以矩形布局为例）
alg_swerve_t kinematics;
alg_swerve_rectangular_config_t geo = {
    .wheel_base_m  = 0.40f,   // 轴距
    .track_width_m = 0.35f,   // 轮距
};
alg_swerve_init_rectangular(&kinematics, &geo);

// 2. 获取四个舵轮模块（假设 board_config 已初始化）
module_swerve_t *sw[4];
for (int i = 0; i < 4; i++) {
    sw[i] = board_config_get_swerve_module(i);
}

// 3. 配置并初始化底盘
app_chassis_t chassis;
app_chassis_config_t config = {
    .kinematics    = &kinematics,
    .modules       = { sw[0], sw[1], sw[2], sw[3] },
    .follow_gain   = 5.0f,       // 跟随云台的角速率增益
    .stop_deadband = 0.02f,      // 2cm/s 或 0.02rad/s 以下视为停止
};

bsp_status_t rc = app_chassis_init(&chassis, &config);
if (rc != BSP_STATUS_OK) {
    // 错误处理
}

/* --- 周期性任务中（如 1kHz） --- */

void chassis_task(float dt_s)
{
    app_chassis_update(&chassis, &command, dt_s);
}
```

## 注意事项

1. **运动学模型与模块数量匹配**：`kinematics` 和 `modules[]` 必须配置为 `ALG_SWERVE_RECTANGULAR_MODULE_COUNT`（4 个），否则逆解会失败。
2. **`stop_deadband` 要合理**：设置过大会在轻微操作时误触发自锁，过小则无法正常进入自锁。建议从 `0.02f` 开始调校。
3. **`follow_gain` 影响跟随手感**：该增益直接覆盖角速率，过大会导致底盘剧烈旋转追赶云台，过小则跟随迟钝。
4. **通信在项目层处理**：通过 `app_chassis_get_feedback()` 读取通用反馈，再由具体项目决定是否发送。
5. **`delta_time_s` 必须 > 0**：`update` 中会将其传递给 `module_swerve_apply_target`，零或负值会跳过控制。
6. **自锁依赖逆解结果**：`alg_swerve_calculate_self_lock` 失败会直接 `disable_all`，避免模块跑飞。
