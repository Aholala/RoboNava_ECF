# alg_pid -- 车载 PID 控制器

## 功能概述

面向 RoboMaster 机器人控制的精简 PID 库。只保留车辆控制中最常用的三种形态：

1. **单环 PID**（`alg_pid_t`）：经典 PID，支持前馈（速度/加速度/附加）、微分低通滤波、条件积分抗饱和、分量分解调试
2. **位置-速度串级 PID**（`alg_pid_cascade_t`）：位置环（外环）+ 速度环（内环），支持位置环降频运行
3. **角度串级 PID**（`alg_pid_angle_t`）：为弧度量纲命名的薄封装，内部委托给 `alg_pid_cascade_t`

所有对象由调用者静态分配，时间步长通过 `delta_time_s` 显式传入，不依赖 HAL 或 RTOS。

**不再提供**：增量式 PID、模糊 PID、在线增益调度、复杂二自由度参数。这些属于项目应用层。

## 核心结构体

### `alg_pid_status_t` -- 状态码

| 枚举值 | 含义 |
|--------|------|
| `ALG_PID_STATUS_OK` | 执行成功 |
| `ALG_PID_STATUS_INVALID_ARGUMENT` | 无效参数（NULL 指针等） |
| `ALG_PID_STATUS_OUT_OF_RANGE` | 参数超出有效范围 |
| `ALG_PID_STATUS_NOT_INITIALIZED` | 对象未初始化 |
| `ALG_PID_STATUS_NUMERICAL_ERROR` | 数值计算错误（NaN/Inf） |

### `alg_pid_config_t` -- 单环 PID 配置

| 字段 | 类型 | 含义 | 默认值/约束 |
|------|------|------|-------------|
| `proportional_gain` | `float` | 比例增益 Kp | 0.0 |
| `integral_gain` | `float` | 积分增益 Ki | 0.0 |
| `derivative_gain` | `float` | 微分增益 Kd | 0.0 |
| `derivative_filter_cutoff_hz` | `float` | 微分项低通截止频率（Hz） | 0（无滤波） |
| `integral_min` | `float` | 积分项下限 | -FLT_MAX |
| `integral_max` | `float` | 积分项上限 | +FLT_MAX |
| `output_min` | `float` | 输出下限 | -FLT_MAX |
| `output_max` | `float` | 输出上限 | +FLT_MAX |
| `derivative_on_measurement` | `bool` | true=微分作用于测量值（推荐），false=微分作用于误差 | true |

### `alg_pid_input_t` -- 高级输入（含前馈）

| 字段 | 类型 | 含义 |
|------|------|------|
| `setpoint` | `float` | 目标值 |
| `measurement` | `float` | 当前测量值 |
| `velocity_feedforward` | `float` | 速度前馈分量 |
| `acceleration_feedforward` | `float` | 加速度前馈分量 |
| `additional_feedforward` | `float` | 附加前馈（摩擦力补偿等） |
| `delta_time_s` | `float` | 时间步长（秒） |

所有前馈字段由上层应用换算为与控制输出相同的单位，模块直接求和，不再隐藏额外增益。

### `alg_pid_terms_t` -- 各分量分解（调试用）

| 字段 | 类型 | 含义 |
|------|------|------|
| `proportional` | `float` | 比例分量 |
| `integral` | `float` | 积分分量 |
| `derivative` | `float` | 微分分量 |
| `feedforward` | `float` | 前馈分量总和 |
| `unsaturated_output` | `float` | 饱和前的总输出 |
| `output` | `float` | 限幅后的实际输出 |

### `alg_pid_t` -- 单环 PID 实例

| 字段 | 类型 | 含义 |
|------|------|------|
| `config` | `alg_pid_config_t` | 控制器配置 |
| `terms` | `alg_pid_terms_t` | 当前各分量值 |
| `previous_error` | `float` | 上一周期误差（微分用） |
| `previous_measurement` | `float` | 上一周期测量值（微分用） |
| `filtered_derivative` | `float` | 经低通滤波的微分信号 |
| `has_previous_sample` | `bool` | 是否有历史采样数据 |
| `is_initialized` | `bool` | 是否已完成初始化 |

### `alg_pid_cascade_config_t` -- 串级 PID 配置

| 字段 | 类型 | 含义 |
|------|------|------|
| `position_config` | `alg_pid_config_t` | 位置环（外环）PID 配置 |
| `velocity_config` | `alg_pid_config_t` | 速度环（内环）PID 配置 |
| `position_loop_divider` | `uint32_t` | 位置环降频因子（1=每周期运行） |
| `velocity_setpoint_min` | `float` | 速度设定值下限 |
| `velocity_setpoint_max` | `float` | 速度设定值上限 |

### `alg_pid_cascade_input_t` -- 串级 PID 输入

| 字段 | 类型 | 含义 |
|------|------|------|
| `position_setpoint` | `float` | 位置目标值 |
| `position_measurement` | `float` | 位置测量值 |
| `velocity_measurement` | `float` | 速度测量值 |
| `velocity_feedforward` | `float` | 速度前馈（直接作用于速度环） |
| `actuator_feedforward` | `float` | 执行器前馈（作用于速度环输出） |
| `delta_time_s` | `float` | 时间步长（秒） |

### `alg_pid_cascade_t` -- 串级 PID 实例

| 字段 | 类型 | 含义 |
|------|------|------|
| `position_controller` | `alg_pid_t` | 位置环 PID |
| `velocity_controller` | `alg_pid_t` | 速度环 PID |
| `position_loop_divider` | `uint32_t` | 位置环降频因子 |
| `position_loop_counter` | `uint32_t` | 位置环降频计数器 |
| `position_elapsed_time_s` | `float` | 位置环累计时间 |
| `velocity_setpoint_min` | `float` | 速度设定值下限 |
| `velocity_setpoint_max` | `float` | 速度设定值上限 |
| `velocity_setpoint` | `float` | 当前速度设定值（位置环输出） |
| `is_initialized` | `bool` | 是否已完成初始化 |

### `alg_pid_angle_config_t` -- 角度串级配置

| 字段 | 类型 | 含义 |
|------|------|------|
| `cascade_config` | `alg_pid_cascade_config_t` | 内部串级 PID 配置 |

### `alg_pid_angle_input_t` -- 角度串级输入

| 字段 | 类型 | 含义 |
|------|------|------|
| `target_position_rad` | `float` | 目标角度（弧度） |
| `target_velocity_rad_per_s` | `float` | 目标角速度（rad/s），用于前馈 |
| `measured_position_rad` | `float` | 测量角度（弧度） |
| `measured_velocity_rad_per_s` | `float` | 测量角速度（rad/s） |
| `actuator_feedforward` | `float` | 执行器前馈 |
| `delta_time_s` | `float` | 时间步长（秒） |

### `alg_pid_angle_t` -- 角度串级实例

| 字段 | 类型 | 含义 |
|------|------|------|
| `cascade` | `alg_pid_cascade_t` | 内部串级 PID |

## 数学原理

### 单环 PID 控制律

```
e = setpoint - measurement

P = Kp * e

I_new = I_old + Ki * e * dt
I = clamp(I_new, I_min, I_max)

if derivative_on_measurement:
    D_raw = -Kd * (measurement - prev_measurement) / dt
else:
    D_raw = Kd * (e - prev_error) / dt

D = lowpass(D_raw, cutoff_hz, dt)  // 可选

output_raw = P + I + D + feedforward_sum
output = clamp(output_raw, output_min, output_max)
```

### 条件积分抗饱和（Conditional Anti-Windup）

```
if output_raw > output_max AND P + I > output_max:
    I = I_old  // 冻结积分
elif output_raw < output_min AND P + I < output_min:
    I = I_old  // 冻结积分
else:
    I = I_new  // 允许积分更新
```

### 微分项低通滤波

一阶低通滤波微分信号：

```
tau = 1 / (2 * pi * fc)
alpha = dt / (tau + dt)
D_filtered = D_filtered + alpha * (D_raw - D_filtered)
```

当 `fc = 0` 时无滤波，直接使用原始微分。

### 串级 PID 结构

```
            位置环              速度环
目标位置 ---->[PID_pos]---->速度目标---->[PID_vel]---->输出
              ^   |            ^   |
              |   |            |   |
         位置测量  速度前馈    速度测量  执行器前馈
```

位置环按 `position_loop_divider` 降频运行，速度环每周期执行：

```
counter++

if counter >= divider:
    velocity_setpoint = PID_pos(position_setpoint, position_measurement)
    velocity_setpoint = clamp(velocity_setpoint, v_min, v_max)
    counter = 0

output = PID_vel(velocity_setpoint, velocity_measurement) + actuator_feedforward
```

### 无扰切换（Bumpless Transfer）

`alg_pid_reset` 将积分项设为 `initial_output`（限幅后），确保控制器从当前输出值无跳变启动：

```
I = clamp(initial_output, I_min, I_max)
prev_error = 0
prev_measurement = measurement  // 避免第一次微分尖峰
```

## API 速查

### 单环 PID

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_pid_config_init(&config)` | 配置初始化为默认值 | `alg_pid_status_t` |
| `alg_pid_init(me, &config)` | 初始化控制器 | `alg_pid_status_t` |
| `alg_pid_reset(me, measurement, initial_output)` | 重置（无扰切换） | `alg_pid_status_t` |
| `alg_pid_update(me, setpoint, measurement, dt, &output)` | 简化更新（无前馈） | `alg_pid_status_t` |
| `alg_pid_update_advanced(me, &input, &output)` | 高级更新（含前馈） | `alg_pid_status_t` |
| `alg_pid_get_terms(me)` | 获取分量分解（调试用） | `const alg_pid_terms_t *` |

### 串级 PID

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_pid_cascade_init(me, &config)` | 初始化串级 PID | `alg_pid_status_t` |
| `alg_pid_cascade_reset(me, pos_m, vel_m, initial_output)` | 重置 | `alg_pid_status_t` |
| `alg_pid_cascade_update(me, &input, &output)` | 更新 | `alg_pid_status_t` |
| `alg_pid_cascade_get_velocity_setpoint(me)` | 获取当前速度设定值 | `float` |

### 角度串级 PID

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_pid_angle_init(me, &config)` | 初始化 | `alg_pid_status_t` |
| `alg_pid_angle_reset(me, pos_rad, vel_rad_s, initial_output)` | 重置 | `alg_pid_status_t` |
| `alg_pid_angle_update(me, &input, &output)` | 更新 | `alg_pid_status_t` |
| `alg_pid_angle_get_velocity_setpoint(me)` | 获取角速度设定值 | `float` |

## 使用示例

### 单环 PID -- 电机速度控制

```c
#include "alg_pid.h"

alg_pid_t speed_pid;

void init_speed_pid(void)
{
    alg_pid_config_t cfg;
    alg_pid_config_init(&cfg);

    cfg.proportional_gain = 10.0f;
    cfg.integral_gain = 2.0f;
    cfg.derivative_gain = 0.1f;
    cfg.integral_min = -5.0f;
    cfg.integral_max = 5.0f;
    cfg.output_min = -16384.0f;    // CAN 电流指令范围
    cfg.output_max = 16384.0f;
    cfg.derivative_on_measurement = true;  // 推荐：避免设定值跳变微分尖峰
    cfg.derivative_filter_cutoff_hz = 30.0f;  // 30Hz 低通滤波微分

    alg_pid_init(&speed_pid, &cfg);
}

// 每 1ms 控制周期
float measure_speed = encoder_get_speed_rad_per_s();
float target_speed = 10.0f;  // rad/s

float current_output;
alg_pid_update(&speed_pid, target_speed, measure_speed, 0.001f, &current_output);
motor_set_current(current_output);
```

### 单环 PID -- 含前馈的高级用法

```c
alg_pid_input_t input = {
    .setpoint = target_speed,
    .measurement = measured_speed,
    .velocity_feedforward = 1.5f,      // 速度比例前馈
    .acceleration_feedforward = 0.3f,  // 加速度前馈
    .additional_feedforward = 0.5f,    // 摩擦力补偿
    .delta_time_s = 0.001f,
};

float output;
alg_pid_update_advanced(&speed_pid, &input, &output);

// 调试：查看各分量
const alg_pid_terms_t *terms = alg_pid_get_terms(&speed_pid);
// terms->proportional, integral, derivative, feedforward, output
```

### 串级 PID -- 电机位置控制

```c
alg_pid_cascade_t pos_pid;

void init_position_pid(void)
{
    alg_pid_cascade_config_t cfg = {0};

    // 位置环（外环）：低频、小增益
    cfg.position_config.proportional_gain = 5.0f;
    cfg.position_config.integral_gain = 0.5f;
    cfg.position_config.integral_min = -2.0f;
    cfg.position_config.integral_max = 2.0f;
    cfg.position_config.derivative_on_measurement = true;

    // 速度环（内环）：高频、大增益
    cfg.velocity_config.proportional_gain = 20.0f;
    cfg.velocity_config.integral_gain = 3.0f;
    cfg.velocity_config.integral_min = -10.0f;
    cfg.velocity_config.integral_max = 10.0f;
    cfg.velocity_config.output_min = -16384.0f;
    cfg.velocity_config.output_max = 16384.0f;
    cfg.velocity_config.derivative_on_measurement = true;

    // 位置环每 5ms 运行一次（速度环每 1ms 运行）
    cfg.position_loop_divider = 5;
    cfg.velocity_setpoint_min = -50.0f;
    cfg.velocity_setpoint_max = 50.0f;

    alg_pid_cascade_init(&pos_pid, &cfg);
}

// 每 1ms 调用
alg_pid_cascade_input_t input = {
    .position_setpoint = 3.14f,       // 目标角度 pi rad
    .position_measurement = encoder_get_angle_rad(),
    .velocity_measurement = encoder_get_speed_rad_per_s(),
    .velocity_feedforward = 0.0f,
    .actuator_feedforward = 0.0f,
    .delta_time_s = 0.001f,
};

float output;
alg_pid_cascade_update(&pos_pid, &input, &output);
motor_set_current(output);
```

### 角度串级 PID -- 云台角度控制

```c
alg_pid_angle_t gimbal_pid;

void init_gimbal_pid(void)
{
    alg_pid_angle_config_t cfg;
    cfg.cascade_config.position_config.proportional_gain = 8.0f;
    cfg.cascade_config.position_config.integral_gain = 0.2f;
    cfg.cascade_config.velocity_config.proportional_gain = 15.0f;
    cfg.cascade_config.velocity_config.integral_gain = 2.0f;
    cfg.cascade_config.velocity_config.output_min = -15000.0f;
    cfg.cascade_config.velocity_config.output_max = 15000.0f;
    cfg.cascade_config.position_loop_divider = 1;  // 每周期运行
    cfg.cascade_config.velocity_setpoint_min = -20.0f;
    cfg.cascade_config.velocity_setpoint_max = 20.0f;

    alg_pid_angle_init(&gimbal_pid, &cfg);
}

// 每周期
alg_pid_angle_input_t input = {
    .target_position_rad = 0.5f,
    .target_velocity_rad_per_s = 0.0f,
    .measured_position_rad = imu_get_yaw_rad(),
    .measured_velocity_rad_per_s = gyro_z_rad_per_s,
    .actuator_feedforward = 0.0f,
    .delta_time_s = 0.001f,
};

float current_cmd;
alg_pid_angle_update(&gimbal_pid, &input, &current_cmd);
```

## 注意事项

1. **微分作用对象**：`derivative_on_measurement = true` 是推荐设置，避免设定值阶跃产生微分尖峰。设为 false 时微分作用于误差，响应更快但可能产生冲击。
2. **前馈单位一致**：所有前馈分量应由上层换算为与控制输出相同的单位。例如控制输出为 CAN 电流指令（-16384~16384），前馈也应是相同量纲的值。模块不做额外增益缩放。
3. **条件积分防饱和**：输出饱和时冻结积分，防止 windup。恢复时积分从上次冻结值继续累积。
4. **无扰切换**：模式切换（如手动->自动）时调用 `alg_pid_reset` 并传入当前控制输出，避免跳变。
5. **降频因子**：`position_loop_divider` 允许位置环以较低的频率运行（如 200Hz 速度环 + 50Hz 位置环），降低位置传感器延迟和计算负载。`delta_time_s` 在位置环更新时使用累加时间。
6. **检查返回状态**：每次 `init`、`reset` 和 `update` 调用都应检查返回状态码，尤其是在量产代码中。
