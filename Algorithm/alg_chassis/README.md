# alg_chassis -- 底盘运动学公共接口

## 功能概述

底盘运动学的通用数学内核，不实现具体轮系运动学（麦克纳姆、全向轮、舵轮等由各自的子模块实现），只提供所有底盘类型共享的：

1. **加权最小二乘速度求解**：从各轮提供的速度约束中解算车体速度
2. **约束残差计算**：评估各轮拟合质量，用于故障检测
3. **坐标变换**：参考坐标系速度 -> 车体坐标系速度
4. **旋转中心转换**：任意旋转中心速度 -> 车体原点速度
5. **轮速统一缩放**：保持速度向量比例的方向优先限幅
6. **里程计积分**：Euler / Midpoint / Exact 三种积分方法
7. **车轮状态监测**：基于残差的故障/恢复判定（滞回 + 防抖）

**子模块包含两个头文件**：
- `alg_chassis_motion.h`：公共类型定义 + 运动学求解 API
- `alg_chassis_wheel_monitor.h`：车轮状态监测器 API

## 核心结构体

### `alg_chassis_status_t` -- 状态码枚举

| 枚举值 | 含义 |
|--------|------|
| `ALG_CHASSIS_STATUS_OK` | 操作成功 |
| `ALG_CHASSIS_STATUS_DEGRADED` | 降级运行（可用约束数少于名义值） |
| `ALG_CHASSIS_STATUS_INVALID_ARGUMENT` | 参数非法 |
| `ALG_CHASSIS_STATUS_NOT_INITIALIZED` | 对象未初始化 |
| `ALG_CHASSIS_STATUS_UNDERDETERMINED` | 欠定（约束数少于未知分量数） |
| `ALG_CHASSIS_STATUS_SINGULAR` | 奇异（无法求解） |
| `ALG_CHASSIS_STATUS_NUMERICAL_ERROR` | 数值错误 |

### `alg_chassis_velocity_t` -- 底盘速度

| 字段 | 类型 | 含义 | 单位 |
|------|------|------|------|
| `velocity_x_m_per_s` | `float` | X 方向速度（前进方向） | m/s |
| `velocity_y_m_per_s` | `float` | Y 方向速度（侧向） | m/s |
| `angular_velocity_rad_per_s` | `float` | 角速度（绕 Z 轴） | rad/s |

### `alg_chassis_pose_t` -- 底盘位姿

| 字段 | 类型 | 含义 | 单位 |
|------|------|------|------|
| `position_x_m` | `float` | X 位置 | 米 |
| `position_y_m` | `float` | Y 位置 | 米 |
| `heading_rad` | `float` | 航向角 | 弧度 |

### `alg_chassis_integration_method_t` -- 里程计积分方法

| 枚举值 | 含义 |
|--------|------|
| `ALG_CHASSIS_INTEGRATION_EULER` | 欧拉积分（计算量最低，精度最差） |
| `ALG_CHASSIS_INTEGRATION_MIDPOINT` | 中点积分（常用折中，适合大多数场景） |
| `ALG_CHASSIS_INTEGRATION_EXACT` | 精确积分（恒定速度模型，计算量最大） |

### `alg_chassis_constraint_t` -- 速度约束

| 字段 | 类型 | 含义 |
|------|------|------|
| `velocity_x_coefficient` | `float` | vx 的系数（方向投影） |
| `velocity_y_coefficient` | `float` | vy 的系数（方向投影） |
| `angular_velocity_coefficient_m` | `float` | wz 的系数（距离 -> 线速度，单位米） |
| `measured_velocity_m_per_s` | `float` | 测量速度（m/s） |
| `weight` | `float` | 权重（>= 0，0 表示禁用） |
| `is_available` | `bool` | 是否可用 |

约束方程：

```
c_vx * vx + c_vy * vy + c_wz * wz = measured_velocity
```

### `alg_chassis_solution_t` -- 求解结果

| 字段 | 类型 | 含义 |
|------|------|------|
| `velocity` | `alg_chassis_velocity_t` | 求解出的车体速度 |
| `residual_root_mean_square_m_per_s` | `float` | 残差均方根（衡量拟合质量） |
| `used_constraint_count` | `size_t` | 实际使用的约束数 |
| `unknown_component_count` | `size_t` | 未知分量数 |
| `is_degraded` | `bool` | 是否降级（约束数 < 名义值） |

### 速度分量位掩码

| 宏 | 值 | 含义 |
|----|-----|------|
| `ALG_CHASSIS_COMPONENT_VELOCITY_X` | `1 << 0` | X 方向速度已知 |
| `ALG_CHASSIS_COMPONENT_VELOCITY_Y` | `1 << 1` | Y 方向速度已知 |
| `ALG_CHASSIS_COMPONENT_ANGULAR_VELOCITY` | `1 << 2` | 角速度已知 |
| `ALG_CHASSIS_COMPONENT_ALL` | `0x07` | 所有分量已知 |

### `alg_chassis_wheel_monitor_wheel_state_t` -- 单轮监测状态

| 字段 | 类型 | 含义 |
|------|------|------|
| `residual_m_per_s` | `float` | 当前残差绝对值（m/s） |
| `fault_confirmation_count` | `uint32_t` | 故障确认计数（连续超过故障阈值次数） |
| `recovery_confirmation_count` | `uint32_t` | 恢复确认计数（连续低于恢复阈值次数） |
| `is_faulted` | `bool` | 是否已标记为故障 |

### `alg_chassis_wheel_monitor_config_t` -- 车轮监测器配置

| 字段 | 类型 | 含义 | 约束 |
|------|------|------|------|
| `wheel_count` | `size_t` | 轮子数量 | -- |
| `fault_residual_threshold_m_per_s` | `float` | 故障残差阈值（m/s） | > recovery_threshold |
| `recovery_residual_threshold_m_per_s` | `float` | 恢复残差阈值（m/s） | < fault_threshold |
| `fault_confirmation_samples` | `uint32_t` | 故障确认所需连续样本数 | -- |
| `recovery_confirmation_samples` | `uint32_t` | 恢复确认所需连续样本数 | -- |
| `wheel_state_storage` | `wheel_state_t *` | 状态存储数组（调用者提供） | -- |

### `alg_chassis_wheel_monitor_t` -- 车轮监测器对象

| 字段 | 类型 | 含义 |
|------|------|------|
| `wheel_count` | `size_t` | 轮子数量 |
| `fault_residual_threshold_m_per_s` | `float` | 故障阈值 |
| `recovery_residual_threshold_m_per_s` | `float` | 恢复阈值 |
| `fault_confirmation_samples` | `uint32_t` | 故障确认样本数 |
| `recovery_confirmation_samples` | `uint32_t` | 恢复确认样本数 |
| `wheel_states` | `wheel_state_t *` | 状态数组指针 |
| `is_initialized` | `bool` | 是否已初始化 |

## 数学原理

### 加权最小二乘求解

给定 n 个约束方程（每个轮子提供一条）：

```
A * x = b，其中 x = [vx, vy, wz]^T
```

加权最小二乘解：

```
(A^T * W * A) * x = A^T * W * b
x = (A^T * W * A)^(-1) * A^T * W * b
```

使用 QR 分解求解，可处理超定和欠定情况。

### 坐标变换（参考系 -> 车体系）

```
vx_body =  vx_ref * cos(heading) + vy_ref * sin(heading)
vy_body = -vx_ref * sin(heading) + vy_ref * cos(heading)
wz_body =  wz_ref  （角速度不变）
```

### 旋转中心转换

```
vx_origin = vx_center + wz * cy_center
vy_origin = vy_center - wz * cx_center
```

### 里程计积分

**Euler 积分**：

```
px += vx * cos(h) * dt - vy * sin(h) * dt
py += vx * sin(h) * dt + vy * cos(h) * dt
h  += wz * dt
```

**Midpoint 积分**：先计算中点位姿，再以中点航向积分位置。

**Exact 积分**：恒定速度模型下精确解析积分，适用于转弯等非线性轨迹。

## API 速查

### alg_chassis_motion 模块

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_chassis_solve_velocity(constraints, count, known_mask, known_vel, nominal_count, &solution)` | 加权最小二乘求解速度 | `alg_chassis_status_t` |
| `alg_chassis_calculate_constraint_residuals(constraints, count, &velocity, residuals, capacity)` | 计算约束残差 | `alg_chassis_status_t` |
| `alg_chassis_transform_reference_to_body(&ref_vel, ref_heading, &body_vel)` | 参考系速度 -> 车体系速度 | `alg_chassis_status_t` |
| `alg_chassis_convert_center_velocity_to_origin(&center_vel, cx, cy, &origin_vel)` | 旋转中心速度 -> 原点速度 | `alg_chassis_status_t` |
| `alg_chassis_scale_wheel_velocities(wheels, available, count, max_speed, &scale)` | 统一缩放轮速 | `alg_chassis_status_t` |
| `alg_chassis_integrate_odometry(&pose, &body_vel, dt, method)` | 里程计积分（原地修改位姿） | `alg_chassis_status_t` |

### alg_chassis_wheel_monitor 模块

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_chassis_wheel_monitor_init(me, config)` | 初始化车轮监测器 | `alg_chassis_status_t` |
| `alg_chassis_wheel_monitor_update(me, residuals, sensor_available, wheel_available, capacity)` | 更新监测状态（滞回判定） | `alg_chassis_status_t` |
| `alg_chassis_wheel_monitor_reset_wheel(me, index, assume_available)` | 重置指定轮子状态 | `alg_chassis_status_t` |
| `alg_chassis_wheel_monitor_get_wheel_state(me, index)` | 获取指定轮子状态（只读） | 状态指针（NULL=无效） |

## 使用示例

### 运动学求解

```c
#include "alg_chassis_motion.h"

// 构建约束数组（由具体轮系模块填充）
alg_chassis_constraint_t constraints[4];
// ... 填充每个轮子的约束系数 ...

alg_chassis_solution_t solution;
alg_chassis_status_t status = alg_chassis_solve_velocity(
    constraints, 4,
    0,       // known_component_mask：无已知分量
    NULL,    // known_velocity
    4,       // nominal_constraint_count
    &solution
);

// 使用求解结果
float vx = solution.velocity.velocity_x_m_per_s;
float vy = solution.velocity.velocity_y_m_per_s;
float wz = solution.velocity.angular_velocity_rad_per_s;
float residual = solution.residual_root_mean_square_m_per_s;
```

### 坐标变换（云台控制 -> 车体速度）

```c
// 云台坐标系下的运动命令
alg_chassis_velocity_t gimbal_cmd = {
    .velocity_x_m_per_s = 1.0f,
    .velocity_y_m_per_s = 0.0f,
    .angular_velocity_rad_per_s = 0.0f,
};

// 车体航向 30 度
float heading = 0.5236f;

alg_chassis_velocity_t body_cmd;
alg_chassis_transform_reference_to_body(&gimbal_cmd, heading, &body_cmd);
// body_cmd 即车体系下的等效命令
```

### 里程计积分

```c
alg_chassis_pose_t odom = {0};
alg_chassis_velocity_t vel = {1.0f, 0.0f, 0.2f};  // 前进 + 慢转

// 每 1ms 调用
alg_chassis_integrate_odometry(&odom, &vel, 0.001f,
                               ALG_CHASSIS_INTEGRATION_MIDPOINT);

// odom.position_x_m, odom.position_y_m, odom.heading_rad 持续更新
```

### 车轮状态监测

```c
#include "alg_chassis_wheel_monitor.h"

alg_chassis_wheel_monitor_wheel_state_t wheel_states[4];
alg_chassis_wheel_monitor_t monitor;

alg_chassis_wheel_monitor_config_t mon_cfg = {
    .wheel_count = 4,
    .fault_residual_threshold_m_per_s = 0.3f,     // 残差 > 0.3m/s 怀疑故障
    .recovery_residual_threshold_m_per_s = 0.1f,  // 残差 < 0.1m/s 认为恢复
    .fault_confirmation_samples = 10,              // 连续 10 次超阈值确认故障
    .recovery_confirmation_samples = 20,           // 连续 20 次低于阈值确认恢复
    .wheel_state_storage = wheel_states,
};

alg_chassis_wheel_monitor_init(&monitor, &mon_cfg);

// 每周期调用
float residuals[4];  // 由正解残差计算得到
bool sensor_ok[4] = {true, true, true, true};
bool wheel_ok[4];

alg_chassis_wheel_monitor_update(&monitor, residuals, sensor_ok, wheel_ok, 4);
// wheel_ok[i] 可用于逆解和正解的 wheel_is_available 参数
```

## 坐标系约定

```
        +x (前)
         ^
         |
   +y <--+  (左)
         |
        +z (上，逆时针角速度为正)
```

## 注意事项

1. **间接使用**：`alg_chassis_motion` 通常不直接被应用层调用，而是被 `alg_mecanum`、`alg_omni`、`alg_swerve` 等子模块内部使用。
2. **约束构建**：`alg_chassis_constraint_t` 中的系数是线性约束系数，需要由调用者（运动学子模块）根据轮子几何提前计算。
3. **已知分量**：`known_component_mask` 用于锁定部分速度分量（如差速底盘强制 vy = 0），减少未知数个数。
4. **残差诊断**：`residual_root_mean_square_m_per_s` 过大意味着模型与实际测量不吻合，可能存在轮子打滑、编码器故障或几何参数误差。
5. **车轮监测滞回**：故障阈值 > 恢复阈值，形成滞回避免状态反复跳变。确认计数提供防抖，滤除瞬时噪声。
6. **积分漂移**：纯轮速积分无绝对参考，漂移随时间累积。长时间定位需融合 IMU、视觉或 UWB 等外部观测。
