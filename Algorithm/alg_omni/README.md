# alg_omni -- 全向轮底盘运动学

## 功能概述

纯算法模块，实现任意数量、任意位置、任意驱动方向的全向轮底盘运动学。与 `alg_mecanum`（固定 45 度辊子）不同，全向轮的方向角是配置参数，可描述三轮/四轮/更多轮的对称和非对称布局。

提供：
- 逆运动学（速度 -> 各轮转速）
- 正运动学（轮速 -> 速度，加权最小二乘）
- 任意旋转中心支持
- 轮速饱和保护
- 缺轮降级（可用轮数不足时锁定已知分量）
- 均匀圆周切向布局自动生成

**依赖**：`alg_chassis_motion` 模块提供的公共类型和最小二乘求解器。

## 核心结构体

### `alg_omni_wheel_config_t` -- 单轮配置

| 字段 | 类型 | 含义 | 约束 |
|------|------|------|------|
| `position_x_m` | `float` | 轮心相对车体原点的 X 坐标 | 米 |
| `position_y_m` | `float` | 轮心相对车体原点的 Y 坐标 | 米 |
| `drive_direction_rad` | `float` | 驱动方向角（相对车体 X 轴） | 弧度 |
| `wheel_radius_m` | `float` | 轮子半径 | > 0 |
| `direction_sign` | `float` | 电机安装方向符号 | +1 或 -1 |
| `odometry_weight` | `float` | 正解时该轮权重 | > 0 |

驱动方向角定义了轮子产生牵引力的方向。例如 0 表示沿 X 轴方向驱动，pi/2 表示沿 Y 轴方向驱动。

### `alg_omni_t` -- 全向底盘运动学实例

| 字段 | 类型 | 含义 |
|------|------|------|
| `wheel_configs` | `const alg_omni_wheel_config_t *` | 轮组配置数组（外部持有，对象仅引用） |
| `wheel_count` | `size_t` | 轮子数量 |
| `maximum_wheel_angular_velocity_rad_per_s` | `float` | 轮速上限（rad/s） |
| `is_initialized` | `bool` | 是否已初始化 |

## 数学原理

### 逆运动学

对于每个全向轮 i（位置 `(px_i, py_i)`，驱动方向角 `theta_i`，半径 `r_i`）：

该轮在驱动方向上的线速度为：

```
v_wheel_i = (vx + wy * py_i) * cos(theta_i) + (vy - wy * px_i) * sin(theta_i)
```

角速度：

```
omega_wheel_i = v_wheel_i / (r_i * direction_sign_i)
```

### 正运动学

每个可用轮子 i 提供一个线性约束：

```
omega_i * r_i * sign_i = vx * cos(theta_i) + vy * sin(theta_i) + wz * (py_i*cos(theta_i) - px_i*sin(theta_i))
```

使用加权最小二乘法求解，权重为各轮的 `odometry_weight`。

### 均匀圆周切向布局

对于对称布局（三轮 120 度、四轮 90 度等），使用 `alg_omni_configure_tangential_layout` 自动生成配置数组：

```
position_angle_i = first_angle + i * 2*pi / N
px_i = R * cos(position_angle_i)
py_i = R * sin(position_angle_i)
drive_direction_i = position_angle_i + pi/2 + tangential_sign * pi/2
```

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_omni_init(me, wheel_configs, count, max_speed)` | 初始化模型 | `alg_chassis_status_t` |
| `alg_omni_configure_tangential_layout(configs, count, dist, radius, first_angle, sign, dir_signs, weight)` | 自动生成均匀圆周切向布局 | `alg_chassis_status_t` |
| `alg_omni_inverse(me, &vel, wheel_ok, wheels, capacity, &scale)` | 逆解（绕原点） | `alg_chassis_status_t` |
| `alg_omni_inverse_with_center_of_rotation(me, &center_vel, cx, cy, wheel_ok, wheels, capacity, &scale)` | 逆解（绕任意旋转中心） | `alg_chassis_status_t` |
| `alg_omni_forward(me, wheels, wheel_ok, known_mask, known_vel, ws, ws_cap, &solution)` | 正解（加权最小二乘） | `alg_chassis_status_t` |

## 使用示例

### 三轮全向底盘（120 度对称布局）

```c
#include "alg_omni.h"

alg_omni_t omni;
alg_omni_wheel_config_t wheels[3];

// 自动生成 120 度对称布局
alg_omni_configure_tangential_layout(
    wheels,
    3,                    // 三轮
    0.15f,                // 轮心到原点距离 15cm
    0.05f,                // 轮半径 5cm
    0.0f,                 // 第一个轮子位置角（0 度 = 正前方）
    1.0f,                 // 切向方向（+1 = 逆时针切向）
    NULL,                 // 各轮方向符号（NULL = 全部 +1）
    1.0f                  // 里程计权重
);

alg_omni_init(&omni, wheels, 3, 60.0f);

// 逆解：车体速度 -> 各轮角速度
alg_chassis_velocity_t cmd = {1.0f, 0.0f, 0.0f};  // 前进 1m/s
float wheel_speeds[3];
float scale;
alg_omni_inverse(&omni, &cmd, NULL, wheel_speeds, 3, &scale);

// 正解：实测轮速 -> 车体速度
alg_chassis_constraint_t ws[3];
alg_chassis_solution_t sol;
alg_omni_forward(&omni, actual_speeds, NULL, 0, NULL, ws, 3, &sol);
```

### 四轮非对称全向底盘（自定义布局）

```c
alg_omni_wheel_config_t custom_wheels[4] = {
    { .position_x_m = 0.2f,  .position_y_m = 0.15f, .drive_direction_rad = 0.785f,  // 45 度
      .wheel_radius_m = 0.05f, .direction_sign = 1.0f, .odometry_weight = 1.0f },
    { .position_x_m = 0.2f,  .position_y_m = -0.15f, .drive_direction_rad = -0.785f, // -45 度
      .wheel_radius_m = 0.05f, .direction_sign = 1.0f, .odometry_weight = 1.0f },
    { .position_x_m = -0.2f, .position_y_m = 0.15f, .drive_direction_rad = -0.785f,
      .wheel_radius_m = 0.05f, .direction_sign = 1.0f, .odometry_weight = 1.0f },
    { .position_x_m = -0.2f, .position_y_m = -0.15f, .drive_direction_rad = 0.785f,
      .wheel_radius_m = 0.05f, .direction_sign = 1.0f, .odometry_weight = 1.0f },
};

alg_omni_t omni;
alg_omni_init(&omni, custom_wheels, 4, 80.0f);
```

### 缺轮降级

```c
// 假设第 2 个轮子故障
bool wheel_ok[3] = {true, false, true};

// 缺轮时（仅有 2 个约束），需提供已知分量避免欠定
alg_chassis_solution_t sol;
alg_chassis_constraint_t ws[3];

// 锁定横向速度为零（差速模式降级）
alg_chassis_velocity_t known = {0, 0, 0};
alg_omni_forward(&omni, speeds, wheel_ok,
    ALG_CHASSIS_COMPONENT_VELOCITY_Y,  // vy 已知
    &known, ws, 3, &sol);
```

## 与 `alg_mecanum` 的对比

| 特性 | `alg_omni` | `alg_mecanum` |
|------|-----------|---------------|
| 轮数 | 任意 | 固定 4 轮 |
| 驱动方向 | 配置参数（任意角度） | 固定 45 度（辊子） |
| 布局 | 任意位置 | 矩形布局 |
| 复杂度 | 通用但配置稍多 | 专用但简单 |
| 均匀布局生成 | `configure_tangential_layout` | 无需（固定布局） |
| 缺轮 | 支持，需提供已知分量 | 支持 |

## 注意事项

1. **配置数组生命周期**：`alg_omni_init` 保存的是 `wheel_configs` 的指针（不复制），调用者必须保证配置数组在对象生命周期内有效且不被修改。
2. **最少约束数**：正解至少需要 3 个可用轮子（3 个未知量：vx, vy, wz）。不足 3 个时必须通过 `known_component_mask` 锁定部分分量。
3. **驱动方向角**：方向角为牵引力方向与车体 X 轴的夹角（弧度）。0 表示纯 X 方向，pi/2 表示纯 Y 方向。
4. **轮速上限**：逆解中对所有可用轮执行统一缩放。`applied_scale` 返回实际缩放系数（< 1.0 表示目标超过限速）。
5. **正解工作区**：`alg_omni_forward` 需要调用者提供至少 `wheel_count` 个 `alg_chassis_constraint_t` 作为工作区。
6. **均匀布局生成**：`alg_omni_configure_tangential_layout` 的 `tangential_direction_sign` 决定每个轮子的驱动方向是顺时针切向（-1）还是逆时针切向（+1）。
