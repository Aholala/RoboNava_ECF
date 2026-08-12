# alg_trajectory -- 一维轨迹规划与多轴同步

## 功能概述

纯 C11 轨迹生成模块，提供两个层次的功能：

1. **`alg_trajectory_t`**：一维轨迹生成器
   - 梯形速度剖面（Trapezoidal）：恒定加速/减速
   - S 曲线剖面（S-Curve）：加加速度（Jerk）限制
   - 位置目标模式：到达指定位置
   - 速度目标模式：达到并保持指定速度
   - 运行中平滑切换目标和剖面类型

2. **`alg_trajectory_group_t`**：多轴同步轨迹组
   - 任意数量轴的同步起停
   - 五次多项式时间缩放
   - 根据各轴限制自动计算共同持续时间
   - 零端速度/零端加速度结束

**不依赖**：HAL、CMSIS、RTOS、动态内存。每个轨迹对象独立，支持多实例。

**典型应用**：底盘运动规划、云台 pitch/yaw 联动、多舵轮转向同步、机械臂关节联动。

## 核心结构体

### `alg_trajectory_status_t` -- 状态码

| 枚举值 | 含义 |
|--------|------|
| `ALG_TRAJECTORY_STATUS_OK` | 正常更新（目标未完成） |
| `ALG_TRAJECTORY_STATUS_FINISHED` | 目标已完成 |
| `ALG_TRAJECTORY_STATUS_INVALID_ARGUMENT` | 参数非法 |
| `ALG_TRAJECTORY_STATUS_NOT_INITIALIZED` | 对象未初始化 |
| `ALG_TRAJECTORY_STATUS_NUMERICAL_ERROR` | 数值错误 |

### `alg_trajectory_profile_t` -- 剖面类型

| 枚举值 | 含义 |
|--------|------|
| `ALG_TRAJECTORY_PROFILE_TRAPEZOIDAL` | 梯形速度（恒加/减速，Jerk 无限） |
| `ALG_TRAJECTORY_PROFILE_S_CURVE` | S 曲线（Jerk 受限，加/减速平滑过渡） |

### `alg_trajectory_target_type_t` -- 目标类型（内部）

| 枚举值 | 含义 |
|--------|------|
| `ALG_TRAJECTORY_TARGET_POSITION` | 位置目标（到达指定位置） |
| `ALG_TRAJECTORY_TARGET_VELOCITY` | 速度目标（达到指定速度并保持） |

### `alg_trajectory_config_t` -- 轨迹配置

| 字段 | 类型 | 含义 | 约束 |
|------|------|------|------|
| `maximum_velocity_per_s` | `float` | 最大速度 | > 0 |
| `maximum_acceleration_per_s2` | `float` | 最大加速度 | > 0 |
| `maximum_deceleration_per_s2` | `float` | 最大减速度 | > 0 |
| `maximum_jerk_per_s3` | `float` | 最大加加速度（S 曲线） | > 0 |
| `position_tolerance` | `float` | 位置到达容差 | >= 0 |
| `velocity_tolerance_per_s` | `float` | 速度到达容差（速度模式） | >= 0 |

### `alg_trajectory_state_t` -- 轨迹状态

| 字段 | 类型 | 含义 |
|------|------|------|
| `position` | `float` | 当前位置 |
| `velocity_per_s` | `float` | 当前速度 |
| `acceleration_per_s2` | `float` | 当前加速度 |

### `alg_trajectory_t` -- 一维轨迹生成器实例

| 字段 | 类型 | 含义 |
|------|------|------|
| `config` | `alg_trajectory_config_t` | 配置参数 |
| `state` | `alg_trajectory_state_t` | 当前状态 |
| `target_position` | `float` | 目标位置（位置模式） |
| `target_velocity_per_s` | `float` | 目标速度（速度模式/位置模式的终端速度） |
| `profile` | `alg_trajectory_profile_t` | 当前剖面类型 |
| `target_type` | `alg_trajectory_target_type_t` | 当前目标类型 |
| `is_finished` | `bool` | 是否已完成 |
| `is_initialized` | `bool` | 是否已初始化 |

### `alg_trajectory_group_t` -- 多轴同步轨迹组

| 字段 | 类型 | 含义 |
|------|------|------|
| `axis_configs` | `alg_trajectory_config_t *` | 每轴配置数组（外部持有） |
| `axis_states` | `alg_trajectory_state_t *` | 每轴当前状态数组 |
| `start_positions` | `float *` | 每轴起始位置数组 |
| `target_positions` | `float *` | 每轴目标位置数组 |
| `axis_count` | `size_t` | 轴数量 |
| `elapsed_time_s` | `float` | 已运行时间 |
| `duration_s` | `float` | 总持续时间 |
| `is_finished` | `bool` | 是否已完成 |
| `is_initialized` | `bool` | 是否已初始化 |

## 数学原理

### 梯形速度剖面

运动分为三个阶段：

1. **加速段**：以 `max_acceleration` 加速到 `max_velocity`
2. **匀速段**：以 `max_velocity` 匀速
3. **减速段**：以 `max_deceleration` 减速到目标位置

```
加速时间 = (max_velocity - v0) / max_acceleration
加速距离 = v0 * t_acc + 0.5 * max_acceleration * t_acc^2

减速时间 = (max_velocity - v_terminal) / max_deceleration
减速距离 = max_velocity * t_dec + 0.5 * max_deceleration * t_dec^2

匀速距离 = total_distance - 加速距离 - 减速距离
匀速时间 = 匀速距离 / max_velocity
```

若位移不足以达到 `max_velocity`（三角形剖面），跳过匀速段，直接以较小的峰值速度过渡到减速段。

### S 曲线剖面（加加速度限制）

在梯形速度的基础上，对加速度的变化率（Jerk）施加限制。每个速度变化段（加速/减速）进一步分为三个阶段：

1. **加加速度段**（Jerk = +J_max）：加速度从 0 线性增加到 A_max
2. **恒加速度段**（Jerk = 0）：加速度保持 A_max
3. **减加速度段**（Jerk = -J_max）：加速度从 A_max 线性减小到 0

```
t_j = A_max / J_max       // 加加速度持续时间
v_j = 0.5 * J_max * t_j^2 // 加加速度段速度增量
d_j = (1/6) * J_max * t_j^3 // 加加速度段位移
```

S 曲线运动更平滑但总时间更长。对于短行程轨迹可能退化为仅加/减加速度段。

### 位置目标模式

```
while not finished:
    计算当前阶段期望的加速度 a（基于剖面类型）
    v_new = clamp(v_old + a * dt, [-v_max, v_max])
    p_new = p_old + v_old * dt + 0.5 * a * dt^2

    if |p_new - target| <= tolerance AND |v_new - terminal_vel| <= tolerance:
        p = target, v = terminal_vel, a = 0
        finished = true
```

### 速度目标模式

```
while not finished:
    a = calculate_acceleration_to_reach(v_target)
    v_new = v_old + a * dt

    if |v_new - v_target| <= tolerance:
        v = v_target, a = 0
        finished = true
```

### 多轴同步 -- 五次多项式时间缩放

所有轴同时起停，使用归一化时间 s = t / T（T 为共同持续时间）：

```
position(s) = start + (target - start) * (6*s^5 - 15*s^4 + 10*s^3)
velocity(s) = (target - start) / T * (30*s^4 - 60*s^3 + 30*s^2)
acceleration(s) = (target - start) / T^2 * (120*s^3 - 180*s^2 + 60*s)
```

多项式满足边界条件：

```
p(0)=start, p(1)=target, v(0)=0, v(1)=0, a(0)=0, a(1)=0
```

共同持续时间 T 通过取所有轴所需时间中的最大值确定：

```
T = max(T_i), 其中 T_i 根据各轴的最大速度和加速度限制计算
```

## API 速查

### 一维轨迹生成器

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_trajectory_init(me, &config, profile, &initial_state)` | 初始化轨迹生成器 | `alg_trajectory_status_t` |
| `alg_trajectory_reset(me, &state)` | 软复位（状态变为指定值，标记为已完成） | `alg_trajectory_status_t` |
| `alg_trajectory_set_position_target(me, target_pos, terminal_vel)` | 设置位置目标（含终端速度） | `alg_trajectory_status_t` |
| `alg_trajectory_set_velocity_target(me, target_vel)` | 设置速度目标 | `alg_trajectory_status_t` |
| `alg_trajectory_set_profile(me, profile)` | 运行时切换剖面类型 | `alg_trajectory_status_t` |
| `alg_trajectory_update(me, dt, &output_state)` | 单步积分（输出状态） | `alg_trajectory_status_t`（FINISHED=目标完成） |
| `alg_trajectory_get_state(me, &state)` | 获取当前状态（只读） | `alg_trajectory_status_t` |
| `alg_trajectory_is_finished(me)` | 查询是否已完成 | `bool` |
| `alg_trajectory_calculate_stopping_distance(vel, decel)` | 计算制动距离 | `float`（无效输入返回 NaN） |

### 多轴同步轨迹组

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_trajectory_group_init(me, config_stor, state_stor, start_stor, target_stor, count, init_states, configs)` | 初始化轨迹组 | `alg_trajectory_status_t` |
| `alg_trajectory_group_set_target(me, &target_positions)` | 设置同步目标（所有轴同时起停） | `alg_trajectory_status_t` |
| `alg_trajectory_group_update(me, dt)` | 更新所有轴（五次多项式插值） | `alg_trajectory_status_t` |
| `alg_trajectory_group_get_state(me, axis_index)` | 获取指定轴当前状态 | `const alg_trajectory_state_t *`（无效返回 NULL） |
| `alg_trajectory_group_is_finished(me)` | 查询是否已完成 | `bool` |

## 使用示例

### 梯形速度位置控制

```c
#include "alg_trajectory.h"

alg_trajectory_t traj;

// 1. 配置梯形速度轨迹
alg_trajectory_config_t cfg = {
    .maximum_velocity_per_s = 2.0f,          // 2 m/s 最大速度
    .maximum_acceleration_per_s2 = 5.0f,     // 5 m/s^2 加速度
    .maximum_deceleration_per_s2 = 5.0f,     // 5 m/s^2 减速度
    .maximum_jerk_per_s3 = 0.0f,             // 梯形时不使用
    .position_tolerance = 0.01f,             // 1cm 容差
    .velocity_tolerance_per_s = 0.02f,
};

// 2. 初始化（静止在原点）
alg_trajectory_state_t init = {0.0f, 0.0f, 0.0f};
alg_trajectory_init(&traj, &cfg, ALG_TRAJECTORY_PROFILE_TRAPEZOIDAL, &init);

// 3. 设置目标
alg_trajectory_set_position_target(&traj, 1.0f, 0.0f);  // 移动到 1m，终端速度 0

// 4. 周期更新（1kHz）
alg_trajectory_state_t state;
while (!alg_trajectory_is_finished(&traj)) {
    alg_trajectory_status_t status = alg_trajectory_update(&traj, 0.001f, &state);

    // state.position       -> 当前位置（给位置环 PID）
    // state.velocity_per_s -> 规划速度（给速度前馈）
    // state.acceleration_per_s2 -> 规划加速度（给加速度前馈）

    if (status == ALG_TRAJECTORY_STATUS_FINISHED) {
        // 目标到达，状态已对齐到目标值
        break;
    }
}
```

### S 曲线平滑运动

```c
// 使用 S 曲线时设置 Jerk
cfg.maximum_jerk_per_s3 = 50.0f;  // 50 m/s^3
alg_trajectory_init(&traj, &cfg, ALG_TRAJECTORY_PROFILE_S_CURVE, &init);

// 运行时也可以切换
alg_trajectory_set_profile(&traj, ALG_TRAJECTORY_PROFILE_TRAPEZOIDAL);
```

### 速度控制模式

```c
// 设置速度目标（不限制位置）
alg_trajectory_set_velocity_target(&traj, 3.0f);  // 加速到 3 m/s 并保持

while (1) {
    alg_trajectory_update(&traj, 0.001f, &state);
    // 使用 state.velocity_per_s 作为当前速度指令
}
```

### 制动距离预判

```c
float stopping_dist = alg_trajectory_calculate_stopping_distance(2.0f, 5.0f);
// 在 2 m/s 速度下以 5 m/s^2 减速度制动需要 0.4m

if (current_pos + stopping_dist > limit_position) {
    // 触发紧急制动
}
```

### 多轴同步 -- 云台双轴联动

```c
alg_trajectory_group_t sync;

// 为 yaw 和 pitch 准备存储
#define AXIS_COUNT 2
alg_trajectory_config_t configs[AXIS_COUNT];
alg_trajectory_state_t states[AXIS_COUNT];
float starts[AXIS_COUNT];
float targets[AXIS_COUNT];

// Yaw 配置
configs[0] = (alg_trajectory_config_t){
    .maximum_velocity_per_s = 3.0f,            // 3 rad/s
    .maximum_acceleration_per_s2 = 10.0f,      // 10 rad/s^2
};
states[0] = (alg_trajectory_state_t){0.0f, 0.0f, 0.0f};
starts[0] = 0.0f;

// Pitch 配置
configs[1] = (alg_trajectory_config_t){
    .maximum_velocity_per_s = 1.5f,            // 1.5 rad/s（俯仰较慢）
    .maximum_acceleration_per_s2 = 5.0f,
};
states[1] = (alg_trajectory_state_t){0.0f, 0.0f, 0.0f};
starts[1] = 0.0f;

alg_trajectory_group_init(&sync, configs, states, starts, targets,
                           AXIS_COUNT, states, configs);

// 设置同步目标
float group_targets[2] = {1.57f, -0.5f};  // yaw=90deg, pitch=-28deg
alg_trajectory_group_set_target(&sync, group_targets);
// 自动计算共同持续时间：取 yaw 和 pitch 所需时间中的较大值

// 周期更新
while (!alg_trajectory_group_is_finished(&sync)) {
    alg_trajectory_group_update(&sync, 0.001f);

    // 读取各轴状态
    const alg_trajectory_state_t *yaw_state = alg_trajectory_group_get_state(&sync, 0);
    const alg_trajectory_state_t *pitch_state = alg_trajectory_group_get_state(&sync, 1);
    // 发送到云台 PID 控制器
}
```

## 注意事项

1. **单位一致性**：所有物理量需使用一致的单位。例如使用米和秒（m, m/s, m/s^2, m/s^3）或弧度和秒（rad, rad/s, rad/s^2, rad/s^3），模块不做量纲转换。
2. **运行中切换目标**：`set_position_target` 和 `set_velocity_target` 可在轨迹进行中调用，模块从当前速度和位置重新规划，实现平滑切换。
3. **终端速度**：位置模式的 `terminal_velocity_per_s` 允许到达目标时保持非零速度，用于连续轨迹拼接（如 waypoint 导航）。
4. **S 曲线复杂度**：S 曲线比梯形速度剖面计算量大，且需要更多状态变量。在 MCU 上多个实例同时运行时注意 CPU 负载。
5. **同步组静止起始**：`alg_trajectory_group_init` 要求所有轴的初始速度和加速度为零（静止起始）。
6. **位移为零**：同步组中位移为零的轴保持静止，不影响共同持续时间的计算。
7. **制动距离估计**：`alg_trajectory_calculate_stopping_distance` 不考虑 Jerk 过渡段，仅基于恒定减速度估计，实际制动距离略大于此值。
