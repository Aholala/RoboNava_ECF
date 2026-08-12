# alg_swerve -- 舵轮底盘运动学

## 功能概述

纯算法模块，实现任意数量、任意位置布局的舵轮（Swerve Drive）模块运动学。每个模块具有独立的舵角（steering angle）和轮速（wheel velocity）两个自由度。

提供：

- **逆运动学**：车体运动命令 -> 各模块舵角 + 轮速
- **正运动学**：各模块实测舵角 + 轮速 -> 车体速度（加权最小二乘）
- **舵角最短路径优化**：通过反转轮速减少转向行程（控制在 pi/2 以内）
- **静止自锁**：各模块舵角指向车体中心形成机械自锁
- **任意旋转中心**：支持绕车体任意点（如云台轴）旋转
- **模块失效降级**：标记不可用模块，自动降级解算
- **坐标变换**：支持参考坐标系（云台系）和车体坐标系之间的命令转换

**依赖**：`alg_chassis_motion` 模块提供的公共类型和最小二乘求解器。

## 核心结构体

### `alg_swerve_status_t` -- 状态码

| 枚举值 | 含义 |
|--------|------|
| `ALG_SWERVE_STATUS_OK` | 操作成功（所有模块均可用） |
| `ALG_SWERVE_STATUS_DEGRADED` | 部分模块不可用，已降级运行 |
| `ALG_SWERVE_STATUS_INVALID_ARGUMENT` | 参数非法 |
| `ALG_SWERVE_STATUS_NOT_INITIALIZED` | 对象未初始化 |

### `alg_swerve_module_geometry_t` -- 模块几何位置

| 字段 | 类型 | 含义 |
|------|------|------|
| `position_x_m` | `float` | 模块相对车体原点的 X 坐标（米） |
| `position_y_m` | `float` | 模块相对车体原点的 Y 坐标（米） |

### `alg_swerve_command_t` -- 运动命令

| 字段 | 类型 | 含义 |
|------|------|------|
| `velocity_x_m_per_s` | `float` | 平移速度 X 分量（m/s） |
| `velocity_y_m_per_s` | `float` | 平移速度 Y 分量（m/s） |
| `angular_velocity_rad_per_s` | `float` | 角速度（rad/s，逆时针为正） |
| `reference_heading_rad` | `float` | 参考航向角（弧度，用于坐标变换） |
| `center_of_rotation_x_m` | `float` | 旋转中心 X 坐标（相对车体原点） |
| `center_of_rotation_y_m` | `float` | 旋转中心 Y 坐标（相对车体原点） |
| `command_is_reference_relative` | `bool` | true=平移速度相对参考航向坐标系；false=相对车体系 |

### `alg_swerve_module_target_t` -- 模块目标（逆解输出）

| 字段 | 类型 | 含义 |
|------|------|------|
| `wheel_velocity_m_per_s` | `float` | 轮子线速度（m/s，正值表示前进） |
| `steering_angle_rad` | `float` | 舵角（弧度，相对车体 X 轴） |

### `alg_swerve_rectangular_module_index_t` -- 矩形布局模块索引

| 枚举值 | 索引 | 位置 |
|--------|------|------|
| `ALG_SWERVE_MODULE_FRONT_LEFT` | 0 | 左前 |
| `ALG_SWERVE_MODULE_FRONT_RIGHT` | 1 | 右前 |
| `ALG_SWERVE_MODULE_REAR_LEFT` | 2 | 左后 |
| `ALG_SWERVE_MODULE_REAR_RIGHT` | 3 | 右后 |
| `ALG_SWERVE_RECTANGULAR_MODULE_COUNT` | -- | 固定为 4 |

### `alg_swerve_t` -- 舵轮底盘运动学实例

| 字段 | 类型 | 含义 |
|------|------|------|
| `module_geometry` | `const alg_swerve_module_geometry_t *` | 模块几何数组（外部持有） |
| `module_count` | `size_t` | 模块数量 |
| `maximum_wheel_velocity_m_per_s` | `float` | 最大轮线速度（m/s） |
| `is_initialized` | `bool` | 是否已初始化 |

## 数学原理

### 逆运动学

对于每个舵轮模块 i（位置 `(px_i, py_i)`），从期望的车体运动 `(vx, vy, wz)` 计算该模块的目标线速度矢量：

```
vx_module_i = vx - wz * py_i
vy_module_i = vy + wz * px_i
```

模块目标线速度大小和方向：

```
wheel_velocity_i = sqrt(vx_module_i^2 + vy_module_i^2)
steering_angle_i = atan2(vy_module_i, vx_module_i)
```

### 任意旋转中心

若命令指定旋转中心 `(cx, cy)`，先转换到车体原点：

```
vx_origin = vx_center + wz * cy
vy_origin = vy_center - wz * cx
```

### 舵角最短路径优化

为避免舵角大角度转向（机械舵机行程有限、响应慢），对每个模块的目标执行优化：

```
delta_angle = |target_angle - current_angle|
if delta_angle > pi/2:
    target_angle = target_angle + pi       // 翻转 180 度
    wheel_velocity = -wheel_velocity       // 反转轮速方向
    target_angle = wrap(target_angle)      // 回绕到 [-pi, pi)
```

优化后舵角变化量 <= pi/2，同时轮速取反保持等效运动。

### 正运动学

每个模块 (i) 提供两个约束方程（X 和 Y 方向的轮速分量）：

```
c_i:  v_wheel_i * cos(steer_i) = vx - wz * py_i
       v_wheel_i * sin(steer_i) = vy + wz * px_i
```

使用加权最小二乘求解 2*N 个约束中的 vx, vy, wz。

### 静止自锁

```
steering_angle_i = atan2(-py_i, -px_i)    // 指向车体中心
wheel_velocity_i = 0.0                     // 轮速为零
```

形成各轮指向中心的"X"构型，通过轮子侧向摩擦力抵抗外力推动。

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_swerve_init(me, geometry, count, max_velocity)` | 初始化运动学模型 | `alg_swerve_status_t` |
| `alg_swerve_configure_rectangular_layout(geometry[4], half_wb, half_tw)` | 生成标准矩形四轮布局 | `alg_swerve_status_t` |
| `alg_swerve_calculate(me, &command, targets, capacity)` | 逆解（所有模块可用，绕原点） | `alg_swerve_status_t` |
| `alg_swerve_calculate_with_availability(me, &command, available, targets, capacity)` | 逆解（支持模块可用性和任意旋转中心） | `alg_swerve_status_t` |
| `alg_swerve_forward(me, measured, available, weights, known_mask, known_vel, ws, ws_cap, &sol)` | 正解（加权最小二乘） | `alg_chassis_status_t` |
| `alg_swerve_optimize_target(current_steer_angle, &target)` | 舵角最短路径优化 | `alg_swerve_status_t` |
| `alg_swerve_calculate_self_lock(me, targets, capacity)` | 计算静止自锁目标 | `alg_swerve_status_t` |
| `alg_swerve_wrap_angle_rad(angle)` | 角度回绕到 [-pi, pi) | `float` |

## 使用示例

### 矩形四轮舵轮底盘

```c
#include "alg_swerve.h"

alg_swerve_t swerve;
alg_swerve_module_geometry_t modules[4];

// 1. 生成标准矩形布局
alg_swerve_configure_rectangular_layout(modules, 0.15f, 0.15f);
// 半轴距 15cm，半轮距 15cm

// 2. 初始化
alg_swerve_init(&swerve, modules, 4, 5.0f);  // 最大轮速 5 m/s

// 3. 逆解：云台坐标系命令 -> 各模块目标
alg_swerve_command_t cmd = {
    .velocity_x_m_per_s = 1.0f,
    .velocity_y_m_per_s = 0.0f,
    .angular_velocity_rad_per_s = 0.5f,
    .reference_heading_rad = 0.785f,     // 云台朝 45 度
    .center_of_rotation_x_m = 0.0f,
    .center_of_rotation_y_m = 0.0f,
    .command_is_reference_relative = true,  // 速度相对云台坐标系
};

alg_swerve_module_target_t targets[4];
alg_swerve_calculate(&swerve, &cmd, targets, 4);

// 4. 舵角最短路径优化（逆解后、发送执行器前）
for (int i = 0; i < 4; i++) {
    alg_swerve_optimize_target(current_steer_angles[i], &targets[i]);
}

// 5. 发送到执行器
for (int i = 0; i < 4; i++) {
    steering_motor_set_angle(i, targets[i].steering_angle_rad);
    drive_motor_set_velocity(i, targets[i].wheel_velocity_m_per_s);
}
```

### 绕云台轴旋转

```c
// 绕车体前方 10cm 处的云台轴旋转
cmd.center_of_rotation_x_m = 0.1f;
cmd.center_of_rotation_y_m = 0.0f;
cmd.angular_velocity_rad_per_s = 2.0f;
cmd.velocity_x_m_per_s = 0.0f;
cmd.velocity_y_m_per_s = 0.0f;

alg_swerve_calculate(&swerve, &cmd, targets, 4);
```

### 正运动学 -- 里程计

```c
alg_swerve_module_target_t measured[4];
// ... 从编码器和舵角传感器读取实测值 ...

alg_chassis_constraint_t ws[8];  // 每个模块 2 个约束
alg_chassis_solution_t sol;
float odom_weights[4] = {1.0f, 1.0f, 1.0f, 1.0f};

alg_swerve_forward(&swerve, measured, NULL, odom_weights,
                   0, NULL, ws, 8, &sol);

// sol.velocity 为估计的车体速度
// 用于里程计积分
```

### 静止自锁

```c
alg_swerve_module_target_t lock_targets[4];
alg_swerve_calculate_self_lock(&swerve, lock_targets, 4);

// 各模块舵角指向车体中心，轮速为零
// 适用于待机状态防止被推动
for (int i = 0; i < 4; i++) {
    steering_motor_set_angle(i, lock_targets[i].steering_angle_rad);
    drive_motor_set_velocity(i, 0.0f);
}
```

### 模块失效处理

```c
// 右前模块故障
bool module_ok[4] = {true, false, true, true};

alg_swerve_calculate_with_availability(&swerve, &cmd, module_ok, targets, 4);
// 故障模块 target 中 wheel_velocity = 0, steering_angle = 0
// 其余模块自动调整以弥补缺失的自由度
```

## 坐标系约定

```
        +x (前)
         ^
    ┌────┴────┐
    │  FL  FR  │   坐标系：+x 前，+y 左，+z 上
    │          │   逆时针角速度为正
    │  RL  RR  │   舵角 0 = 指向 +x（前方）
    └────┬────┘   舵角 pi/2 = 指向 +y（左方）
         |
        +y (左)
```

## 注意事项

1. **舵角闭环**：本模块只计算舵角目标值，不包含舵机闭环控制。舵角的实际执行由独立的舵机 PID 控制器完成。
2. **舵角最短路径**：`alg_swerve_optimize_target` 必须在逆解后、每个模块独立调用。优化会导致轮速符号反转和舵角翻转 180 度，等效运动不变。
3. **正解最少约束**：正解至少需要 3 个可用模块（每个模块提供 2 个约束 = 6 个约束解 3 个未知量）。不足 3 个时必须提供 `known_component_mask`。
4. **坐标变换**：`command_is_reference_relative = true` 时，模块内部自动执行参考系到车体系的旋转，无需调用者额外处理。
5. **轮速限幅**：逆解中自动检查所有模块轮速，超限时统一缩放。`maximum_wheel_velocity_m_per_s` 需根据电机和轮径设定。
6. **几何数组生命周期**：`module_geometry` 数组由调用者持有，模块仅保存指针。调用者必须保证数组在模块生命周期内有效且不被修改。
7. **自锁限制**：静止自锁的保持力取决于舵机闭环刚度、驱动器使能状态和轮地摩擦力，不能替代制动器。
