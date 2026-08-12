# alg_attitude -- 六轴 IMU 姿态估计（Mahony / Madgwick）

## 功能概述

纯算法模块，从三轴陀螺仪 + 三轴加速度计实时估计刚体姿态。支持两种互补滤波算法：

- **Mahony 互补滤波**：通过 PI 反馈将加速度计观测的姿态误差修正陀螺仪角速度，具备积分项补偿陀螺零偏。
- **Madgwick 梯度下降滤波**：通过梯度下降最小化四元数误差函数，等效于可变增益的互补滤波。

加速度模长超出配置窗口时自动退化为纯陀螺积分，避免运动加速度引入的姿态误差。六轴 IMU 无法长期观测偏航角（Yaw），需通过 `alg_attitude_correct_yaw()` 定期注入外部航向（磁力计、视觉、机构约束）。

## 核心结构体

### `alg_attitude_status_t` -- 状态码枚举

| 枚举值 | 含义 |
|--------|------|
| `ALG_ATTITUDE_STATUS_OK` | 操作成功（使用了加速度修正） |
| `ALG_ATTITUDE_STATUS_GYRO_ONLY` | 仅陀螺仪积分（加速度超出有效范围） |
| `ALG_ATTITUDE_STATUS_INVALID_ARGUMENT` | 参数非法 |
| `ALG_ATTITUDE_STATUS_NOT_INITIALIZED` | 对象未初始化 |
| `ALG_ATTITUDE_STATUS_NUMERICAL_ERROR` | 数值错误（四元数归一化失败等） |

### `alg_attitude_method_t` -- 算法类型

| 枚举值 | 含义 |
|--------|------|
| `ALG_ATTITUDE_METHOD_MAHONY` | Mahony 互补滤波（PI 反馈修正陀螺仪） |
| `ALG_ATTITUDE_METHOD_MADGWICK` | Madgwick 梯度下降滤波 |

### `alg_attitude_config_t` -- 配置参数

| 字段 | 类型 | 含义 | 典型值 |
|------|------|------|--------|
| `method` | `alg_attitude_method_t` | 算法选择 | -- |
| `proportional_gain` | `float` | Mahony 比例增益 Kp，控制修正速度 | 0.5 ~ 2.0 |
| `integral_gain` | `float` | Mahony 积分增益 Ki，补偿陀螺零偏 | 0.0 ~ 0.1（通常设 0） |
| `madgwick_beta` | `float` | Madgwick 梯度下降步长 beta | 0.01 ~ 0.5 |
| `acceleration_min_m_per_s2` | `float` | 加速度有效下限（m/s^2） | 8.0（1g 附近） |
| `acceleration_max_m_per_s2` | `float` | 加速度有效上限（m/s^2） | 12.0（1g 附近） |

### `alg_attitude_quaternion_t` -- 四元数

| 字段 | 含义 |
|------|------|
| `q0` | 标量分量（w），恒正 |
| `q1` | X 轴矢量分量 |
| `q2` | Y 轴矢量分量 |
| `q3` | Z 轴矢量分量 |

始终保证归一化（|q| = 1.0）。q = q0 + q1*i + q2*j + q3*k。

### `alg_attitude_rotation_matrix_t` -- 旋转矩阵

| 字段 | 含义 |
|------|------|
| `element[3][3]` | 3x3 从机体系到世界系的旋转矩阵 |

### `alg_attitude_t` -- 姿态估计器对象

| 字段 | 类型 | 含义 |
|------|------|------|
| `config` | `alg_attitude_config_t` | 配置参数 |
| `quaternion` | `alg_attitude_quaternion_t` | 当前姿态四元数 |
| `integral_error_x/y/z` | `float` | Mahony 三轴积分误差（仅 Mahony 使用） |
| `is_initialized` | `bool` | 是否已初始化 |

## 数学原理

### Mahony 互补滤波

**步骤 1**：用当前四元数将归一化加速度计旋转到世界系，计算与参考重力向量 [0, 0, 1] 的叉积作为姿态误差：

```
e = v_estimated x v_reference
```

**步骤 2**：PI 反馈修正陀螺仪角速度：

```
omega_corrected = omega_raw + Kp * e + Ki * integral(e) * dt
```

**步骤 3**：用修正后的角速度积分四元数：

```
q = q + 0.5 * dt * Omega(omega_corrected) * q
q = q / |q|
```

### Madgwick 梯度下降

目标是最小化误差函数 f(q, a_ref, a_meas)，用梯度下降更新：

```
q_grad = -beta * grad(f) / |grad(f)|
q = q + 0.5 * dt * Omega(omega) * q + q_grad * dt
q = q / |q|
```

### 加速度模长判决

```
effective_g = |a| / g
valid = (effective_g >= min) AND (effective_g <= max)
```

不满足时陀螺仪不加修正量，仅积分。

### 外部航向修正

```
yaw_error = wrap(measured_yaw - estimated_yaw)
delta_yaw = correction_gain * yaw_error
q_corrected = q * q_delta_yaw
```

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_attitude_init(me, config, initial_quaternion)` | 初始化姿态估计器 | `alg_attitude_status_t` |
| `alg_attitude_reset(me, quaternion)` | 重置到指定四元数 | `alg_attitude_status_t` |
| `alg_attitude_update(me, gx, gy, gz, ax, ay, az, dt)` | 周期更新姿态 | `alg_attitude_status_t`（OK=有加速度修正，GYRO_ONLY=仅陀螺积分） |
| `alg_attitude_correct_yaw(me, measured_yaw_rad, correction_gain)` | 注入外部航向修正 | `alg_attitude_status_t` |
| `alg_attitude_get_euler(me, &roll, &pitch, &yaw)` | 获取欧拉角（弧度） | `alg_attitude_status_t` |
| `alg_attitude_get_rotation_matrix(me, &R)` | 获取旋转矩阵 | `alg_attitude_status_t` |

## 使用示例

```c
#include "alg_attitude.h"

// 1. 创建并配置姿态估计器（Mahony 模式）
alg_attitude_t attitude;
alg_attitude_config_t cfg = {
    .method = ALG_ATTITUDE_METHOD_MAHONY,
    .proportional_gain = 1.0f,           // Kp
    .integral_gain = 0.0f,               // Ki（一般设 0）
    .madgwick_beta = 0.1f,               // Madgwick 参数（Mahony 模式无效）
    .acceleration_min_m_per_s2 = 8.0f,   // 小于 0.82g 视为异常
    .acceleration_max_m_per_s2 = 12.0f,  // 大于 1.22g 视为异常
};

// 2. 初始化为水平状态
alg_attitude_quaternion_t init_q = {1.0f, 0.0f, 0.0f, 0.0f};
alg_attitude_init(&attitude, &cfg, &init_q);

// 3. 周期更新（IMU 中断回调中调用，如 1kHz）
void imu_callback(float gx, float gy, float gz, float ax, float ay, float az)
{
    alg_attitude_status_t status = alg_attitude_update(
        &attitude,
        gx, gy, gz,   // 陀螺仪 rad/s
        ax, ay, az,   // 加速度计 m/s^2
        0.001f        // dt = 1ms
    );

    if (status == ALG_ATTITUDE_STATUS_GYRO_ONLY) {
        // 加速度异常，仅陀螺积分——ATT 指示灯闪烁提示
    }
}

// 4. 定期注入磁航向（如 50Hz）
void compass_callback(float magnetic_yaw_rad)
{
    alg_attitude_correct_yaw(&attitude, magnetic_yaw_rad, 0.02f);
}

// 5. 读取姿态
float roll, pitch, yaw;
alg_attitude_get_euler(&attitude, &roll, &pitch, &yaw);
// roll/pitch/yaw 单位均为弧度，范围：roll [-pi, pi], pitch [-pi/2, pi/2], yaw [-pi, pi]
```

## 注意事项

1. **Yaw 不可观测**：六轴 IMU 只能长期校正 Roll/Pitch，Yaw 会随时间漂移，必须通过磁力计、视觉或机构约束定期修正。
2. **加速度有效范围**：`acceleration_min` 和 `acceleration_max` 需根据实际应用调整。静止时 |a| 约等于 g，剧烈机动时超出范围自动退化为纯陀螺积分。
3. **初始化方向**：`alg_attitude_init` 需要初始四元数。通常可在静止时用加速度计计算 Roll/Pitch，Yaw 设为零，然后转换为四元数传入。
4. **Mahony 积分增益**：Ki > 0 时会累计陀螺零偏，但 Ki 过大可能导致姿态漂移。一般先设 Ki = 0，如有长期零偏再适当增大。
5. **计时精度**：`delta_time_s` 必须准确，dt 误差直接导致积分偏差。
