# alg_mecanum -- 四轮麦克纳姆轮底盘运动学

## 功能概述

纯算法模块，实现四轮麦克纳姆底盘的运动学逆解（车体速度 -> 轮速）、正解（轮速 -> 车体速度）、任意旋转中心支持、X/O 辊子布局切换、缺轮降级和轮速饱和保护。

依赖 `alg_chassis_motion` 模块提供的公共类型（`alg_chassis_velocity_t`、`alg_chassis_solution_t` 等）和最小二乘求解器。

**不依赖**：HAL、CMSIS、RTOS。所有数据由调用者静态管理，不使用动态内存。

## 核心结构体

### `alg_mecanum_roller_arrangement_t` -- 辊子排列类型

| 枚举值 | 含义 |
|--------|------|
| `ALG_MECANUM_ROLLER_X` | X 型排列（常见）：左右轮辊子方向相同 |
| `ALG_MECANUM_ROLLER_O` | O 型排列：左右轮辊子方向相反（横向系数反向） |

### `alg_mecanum_wheel_index_t` -- 轮子索引

| 枚举值 | 索引 | 位置 |
|--------|------|------|
| `ALG_MECANUM_WHEEL_FRONT_LEFT` | 0 | 左前轮（FL） |
| `ALG_MECANUM_WHEEL_FRONT_RIGHT` | 1 | 右前轮（FR） |
| `ALG_MECANUM_WHEEL_REAR_LEFT` | 2 | 左后轮（RL） |
| `ALG_MECANUM_WHEEL_REAR_RIGHT` | 3 | 右后轮（RR） |

### `alg_mecanum_config_t` -- 底盘配置

| 字段 | 类型 | 含义 | 约束 |
|------|------|------|------|
| `wheel_radius_m` | `float` | 轮子半径（米） | > 0 |
| `half_wheelbase_m` | `float` | 半轴距（纵向距离的一半） | > 0 |
| `half_track_width_m` | `float` | 半轮距（横向距离的一半） | > 0 |
| `direction_sign[4]` | `float[4]` | 各轮电机安装方向 | +1 或 -1 |
| `odometry_weight[4]` | `float[4]` | 正解时各轮权重 | > 0 |
| `maximum_wheel_angular_velocity_rad_per_s` | `float` | 轮角速度上限（rad/s） | > 0 |
| `roller_arrangement` | `alg_mecanum_roller_arrangement_t` | 辊子排列类型 | X 或 O |

### `alg_mecanum_t` -- 运动学实例

| 字段 | 类型 | 含义 |
|------|------|------|
| `config` | `alg_mecanum_config_t` | 用户配置副本 |
| `lateral_coefficient[4]` | `float[4]` | 横向速度系数（内部预计算，+/-1） |
| `angular_coefficient_m[4]` | `float[4]` | 角速度系数（内部预计算，单位米） |
| `is_initialized` | `bool` | 是否已初始化 |

## 数学原理

### 逆运动学（车体速度 -> 各轮角速度）

麦克纳姆轮的特殊之处在于每个轮子除了纵向驱动力外，还有辊子产生的侧向速度分量。

对于每个轮子 i（位置 x_i, y_i）：

```
v_wheel_i = (vx - sign_i * vy + wz * angular_coeff_i) / radius
```

其中：
- `vx, vy`：车体在车体坐标系中的平移速度
- `wz`：车体绕 Z 轴的角速度
- `sign_i`：由辊子布局（X/O）决定的横向速度符号
- `angular_coeff_i = |x_i| + |y_i|`：角速度到线速度的转换系数

**X 型布局**（FL=+1 FR=-1 RL=-1 RR=+1）：

```
omega_FL = (vx + vy + wz*(a+b)) / r
omega_FR = (vx - vy - wz*(a+b)) / r
omega_RL = (vx - vy + wz*(a+b)) / r
omega_RR = (vx + vy - wz*(a+b)) / r
```

其中 a = half_wheelbase, b = half_track_width。

### 正运动学（各轮转速 -> 车体速度）

使用加权最小二乘法求解超定方程组。每个可用轮子提供一个线性约束：

```
omega_i * r = vx + sign_i * vy + angular_coeff_i * wz
```

构成约束矩阵 A（n_wheels x 3），求解：

```
A' * W * A * x = A' * W * b
x = [vx, vy, wz]^T
```

其中 W 为对角权重矩阵（对角线元素为 `odometry_weight[i]`）。

### 任意旋转中心

若期望绕点 (cx, cy) 而非原点旋转，先将旋转中心速度转换为车体原点速度：

```
v_origin.x = v_center.x + wz * cy
v_origin.y = v_center.y - wz * cx
```

### 轮速饱和保护

```
max_omega = max(|omega_i|) for available wheels
if max_omega > omega_limit:
    scale = omega_limit / max_omega
    omega_i *= scale  (对所有可用轮)
```

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_mecanum_init(me, config)` | 初始化模型，预计算系数 | `alg_chassis_status_t` |
| `alg_mecanum_inverse(me, chassis_velocity, wheel_is_available, wheel_velocities, &applied_scale)` | 逆解（绕原点） | `alg_chassis_status_t` |
| `alg_mecanum_inverse_with_center_of_rotation(me, center_velocity, cx, cy, wheel_is_available, wheel_velocities, &applied_scale)` | 逆解（绕任意旋转中心） | `alg_chassis_status_t` |
| `alg_mecanum_forward(me, wheel_velocities, wheel_is_available, known_component_mask, known_velocity, &solution)` | 正解（加权最小二乘） | `alg_chassis_status_t` |

## 使用示例

```c
#include "alg_mecanum.h"

// 1. 配置底盘几何
alg_mecanum_t mecanum;
alg_mecanum_config_t cfg = {
    .wheel_radius_m = 0.076f,            // 76mm 轮径
    .half_wheelbase_m = 0.15f,           // 前后半轴距
    .half_track_width_m = 0.15f,         // 左右半轮距
    .direction_sign = {1.0f, 1.0f, 1.0f, 1.0f},
    .odometry_weight = {1.0f, 1.0f, 1.0f, 1.0f},
    .maximum_wheel_angular_velocity_rad_per_s = 50.0f,
    .roller_arrangement = ALG_MECANUM_ROLLER_X,
};
alg_mecanum_init(&mecanum, &cfg);

// 2. 逆解：车体目标速度 -> 各轮角速度
alg_chassis_velocity_t cmd = {
    .velocity_x_m_per_s = 1.0f,          // 前进 1 m/s
    .velocity_y_m_per_s = 0.0f,          // 无横向运动
    .angular_velocity_rad_per_s = 0.5f,  // 旋转 0.5 rad/s
};

float wheel_speeds[4];                   // 单位 rad/s
bool wheel_ok[4] = {true, true, true, true};
float scale;

alg_mecanum_inverse(&mecanum, &cmd, wheel_ok, wheel_speeds, &scale);
// scale = 1.0 表示未缩放，scale < 1.0 表示已按比例缩速

// 3. 正解：实测轮速 -> 估计车速
alg_chassis_solution_t solution;
alg_mecanum_forward(&mecanum, actual_speeds, wheel_ok, 0, NULL, &solution);
// solution.velocity    -> 估计的车体速度
// solution.residual_root_mean_square_m_per_s -> 拟合残差
// solution.is_degraded -> 是否有轮子不可用

// 4. 里程计（使用 alg_chassis_motion 公共接口）
alg_chassis_pose_t pose = {0};
alg_chassis_integrate_odometry(&pose, &solution.velocity, 0.001f,
                               ALG_CHASSIS_INTEGRATION_MIDPOINT);
```

### 绕任意点旋转（如绕云台轴）

```c
alg_chassis_velocity_t center_cmd = {
    .velocity_x_m_per_s = 0.0f,
    .velocity_y_m_per_s = 0.0f,
    .angular_velocity_rad_per_s = 3.0f,  // 绕云台轴旋转
};

alg_mecanum_inverse_with_center_of_rotation(
    &mecanum, &center_cmd,
    0.1f, 0.0f,   // 旋转中心在底盘前方 10cm 处
    wheel_ok, wheel_speeds, &scale
);
```

## 坐标系约定

```
        +x (前)
         ^
    ┌────┴────┐
    │  FL  FR  │   FL = 左前, FR = 右前
    │          │
    │  RL  RR  │   RL = 左后, RR = 右后
    └────┬────┘
         |
        +y (左)
    +z 向上，逆时针角速度为正
```

## 注意事项

1. **辊子类型**：X 型和 O 型的横向速度系数符号相反，切换 `roller_arrangement` 即可适配。初始化时内部预计算 `lateral_coefficient` 和 `angular_coefficient`。
2. **缺轮处理**：通过 `wheel_is_available` 数组标记故障轮子。不可用轮子输出轮速为 0，正解时该轮约束被排除。可用轮数少于 3 时需提供 `known_component_mask` 和 `known_velocity` 避免奇异。
3. **轮速限幅**：在逆解中自动执行，所有可用轮统一缩放保持运动方向。`applied_scale` 返回实际缩放系数（1.0 表示目标完全可达）。
4. **正解权重**：`odometry_weight` 决定各轮对正解的影响，可对精度较高的轮子赋予更大权重。
5. **里程计漂移**：纯轮速积分漂移随时间累积，长时间定位需融合 IMU、视觉或其他绝对观测。
