# alg_imu_ekf -- IMU 四元数扩展卡尔曼滤波器

## 功能概述

基于 6 轴 IMU（三轴陀螺仪 + 三轴加速度计）的四元数扩展卡尔曼滤波器（EKF），输出实时姿态四元数、欧拉角、陀螺零偏估计和诊断信息。核心特点：

- **6 维状态向量**：`[qw, qx, qy, qz, bias_x, bias_y]`，包含姿态四元数和 X/Y 轴陀螺零偏（Z 轴零偏六轴系统不可观测）
- **陀螺消零偏**：在线估计并补偿 X/Y 轴陀螺零偏，提供校正后的角速度输出
- **自适应测量噪声**：基于归一化创新平方（NIS）的卡方自适应方案，动态抑制异常加速度干扰
- **硬拒绝 + 软适应**：加速度模长偏离 1g 过大时硬拒绝整个观测，中等偏差时自动增大测量噪声协方差
- **加速度预滤波**：内置三通道一阶低通滤波，预处理加速度计输入
- **四元数归一化与协方差投影**：每步保持四元数单位长度，并将协方差投影到单位四元数约束流形上
- **连续 Yaw 角**：检测 +/-pi 跳变并累积圈数，实现无跳变连续偏航角输出

**依赖**：`alg_kalman`（通用 EKF 框架）、`alg_filter`（加速度计低通滤波）。

---

## 完整数学原理

### 1. 状态空间模型

**状态向量** x（6 维）：

```
x = [qw, qx, qy, qz, bias_x, bias_y]^T
```

其中前 4 维为姿态四元数（从机体系到世界系，世界系 +Z 向上），后 2 维为 X/Y 轴陀螺仪零偏。

### 2. 预测方程（状态转移）

**四元数微分方程**（Hamilton 约定）：

```
dq/dt = 0.5 * q ⊗ ω
```

展开为分量形式：

```
dw/dt = -0.5 * (ω_x*q_x + ω_y*q_y + ω_z*q_z)
dx/dt =  0.5 * (ω_x*q_w + ω_y*q_z - ω_z*q_y)
dy/dt =  0.5 * (ω_y*q_w - ω_x*q_z + ω_z*q_x)
dz/dt =  0.5 * (ω_z*q_w + ω_x*q_y - ω_y*q_x)
```

使用欧拉前向积分（一阶）：

```
q_{k+1} = q_k + dt * dq/dt
q_{k+1} = q_{k+1} / ||q_{k+1}||   （归一化）
```

其中 ω = ω_measured - bias（X/Y 轴减零偏，Z 轴不减）。

等效的矩阵形式（使用四元数乘法矩阵 Ω）：

```
Omega(ω) = [  0    -ω_x  -ω_y  -ω_z ]
           [ ω_x    0     ω_z  -ω_y ]
           [ ω_y  -ω_z    0     ω_x ]
           [ ω_z   ω_y  -ω_x    0   ]

q_pred = q + 0.5 * dt * Omega(ω_corrected) * q
```

零偏状态的预测：

```
bias_x_{k+1} = bias_x_k   （不变）
bias_y_{k+1} = bias_y_k   （不变）
```

零偏变化通过过程噪声 Q（随机游走模型）间接建模。

### 3. 状态雅可比矩阵 F（6x6）

F = df/dx，分块结构：

```
F = [ ∂q'/∂q (4x4)      ∂q'/∂b (4x2) ]
    [ 0 (2x4)             I (2x2)       ]
```

**左上 4x4 块 ∂q'/∂q**：对 q' = q + 0.5*dt*Ω(ω)*q 求偏导。

```
∂q'/∂q = I + 0.5*dt*Ω(ω)

具体：
F(0,0)=1,  F(0,1)=-0.5*dt*ω_x,  F(0,2)=-0.5*dt*ω_y,  F(0,3)=-0.5*dt*ω_z
F(1,0)=0.5*dt*ω_x,  F(1,1)=1,   F(1,2)=0.5*dt*ω_z,   F(1,3)=-0.5*dt*ω_y
F(2,0)=0.5*dt*ω_y,  F(2,1)=-0.5*dt*ω_z,  F(2,2)=1,  F(2,3)=0.5*dt*ω_x
F(3,0)=0.5*dt*ω_z,  F(3,1)=0.5*dt*ω_y,   F(3,2)=-0.5*dt*ω_x,  F(3,3)=1
```

**右上 4x2 块 ∂q'/∂b**：对 zero bias 求偏导，使用链式法则 ∂q'/∂b = ∂q'/∂ω * ∂ω/∂b。

```
F(0,4)=0.5*dt*q_x, F(0,5)=0.5*dt*q_y,
F(1,4)=-0.5*dt*q_w, F(1,5)=0.5*dt*q_z,
...
（ω 对 bias_x 偏导为 [-1,0,0]，乘以 0.5*dt 后得各分量）
```

**右下 2x2 块**：I（单位矩阵，零偏不变）。

### 4. 观测方程（测量模型）

观测为归一化重力方向向量。世界系重力（归一化）g_w = [0, 0, 1]，旋转到机体系：

```
h(x) = R(q)^T * [0, 0, 1]^T

展开：
h_x = 2*(q_x*q_z - q_w*q_y)
h_y = 2*(q_w*q_x + q_y*q_z)
h_z = q_w² - q_x² - q_y² + q_z²
```

### 5. 测量雅可比矩阵 H（3x6）

H = dh/dx，对四元数分量求偏导（后 2 列全 0）：

```
第一行 ∂h_x/∂q：
H(0,0) = -2*q_y     H(0,1) = 2*q_z      H(0,2) = -2*q_w    H(0,3) = 2*q_x

第二行 ∂h_y/∂q：
H(1,0) = 2*q_x      H(1,1) = 2*q_w      H(1,2) = 2*q_z     H(1,3) = 2*q_y

第三行 ∂h_z/∂q：
H(2,0) = 2*q_w      H(2,1) = -2*q_x     H(2,2) = -2*q_y    H(2,3) = 2*q_z
```

### 6. 过程噪声矩阵 Q（6x6）

```
Q = G * Q_gyro * G^T * dt + Q_bias * dt

其中 Q_gyro = σ_gyro² * I_3x3（对角矩阵）
Q_bias: 只有(4,4)和(5,5)为非零 σ_bias_rw²

G 矩阵（4x3）为四元数对陀螺仪噪声的映射：
G = 0.5 * [
    [-q_x, -q_y, -q_z],
    [ q_w, -q_z,  q_y],
    [ q_z,  q_w, -q_x],
    [-q_y,  q_x,  q_w]
]

由于 Q_gyro 各向同性，实际计算简化为：
Q_quat(i,j) = σ_gyro² * dt * Σ_k G(i,k)*G(j,k)   (k = x,y,z)
```

### 7. 四元数归一化与协方差投影

每次预测和校正后执行。

**归一化雅可比** J（6x6）：J = d(q/||q||)/d([q, bias])。

```
四元数部分（4x4）：J_ij = (δ_ij - q_i*q_j) / ||q||
零偏部分：单位矩阵

其中 δ_ij 为 Kronecker delta，q_i 为归一化后的四元数分量。
```

该矩阵的几何含义：将沿四元数径向（模长方向）的方差分量投影为零，只保留切向（旋转方向）的方差分量。

**协方差投影** P' = J * P * J^T。

**对称化** P = (P + P^T) / 2 消除舍入误差导致的非对称性。

### 8. 创新统计与自适应机制

**创新残差** y = z - h(x)（z 为归一化加速度方向）

**创新协方差** S = H * P * H^T + R（3x3）

**NIS（归一化创新平方）** = y^T * S^-1 * y

NIS 服从自由度为 3 的卡方分布（χ²(3)）：

| 置信度 | NIS 阈值 |
|-------|---------|
| 68%   | 3.5     |
| 90%   | 6.3     |
| 95%   | 7.8     |
| 99%   | 11.3    |

**自适应噪声倍率**（NIS 在 adaptation 和 rejection 阈值之间）：

```
ratio = (NIS - adapt_thr) / (reject_thr - adapt_thr)
noise_scale = 1 + (max_scale - 1) * ratio^2    （二次递增）
```

二次递增策略：初期温和增大噪声（信任观测），接近拒绝阈值时快速放大（抑制异常影响）。

**收敛判断**：NIS 连续低于 reject_threshold / 2 时标记 `has_converged = true`。

### 9. 零偏协方差渐消（Fading Memory）

每个预测步对协方差矩阵中零偏相关的元素乘 sqrt(fading_factor) 的对应次幂。

```
P(i,j) *= scale_factor
scale_factor = 1.0
if i >= 4: scale_factor *= sqrt(fading_factor)
if j >= 4: scale_factor *= sqrt(fading_factor)
```

作用：人为增加零偏状态的不确定度，防止协方差过早收缩导致零偏估计不再响应新观测。等价于增大了过程噪声 Q 的零偏部分。

### 10. 连续 Yaw 回绕处理

atan2 输出的 yaw 范围是 [-pi, pi]。当机器人旋转经过 +/-pi 边界时，yaw 会跳变 +/-2pi。

```
yaw_delta = yaw_curr - yaw_prev
if yaw_delta > +pi: revolution_count -= 1    （-pi 方向回绕）
if yaw_delta < -pi: revolution_count += 1    （+pi 方向回绕）

continuous_yaw = yaw_curr + revolution_count * 2*pi
```

---

## 核心结构体

### `alg_imu_ekf_status_t` -- 状态码

| 枚举值 | 含义 |
|--------|------|
| `ALG_IMU_EKF_STATUS_OK` | 操作成功 |
| `ALG_IMU_EKF_STATUS_INVALID_ARGUMENT` | 参数非法（空指针等） |
| `ALG_IMU_EKF_STATUS_OUT_OF_RANGE` | 参数超出范围（非有限数等） |
| `ALG_IMU_EKF_STATUS_NOT_INITIALIZED` | 对象未初始化 |
| `ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED` | 加速度观测被拒绝（模长异常或 NIS 超阈值） |
| `ALG_IMU_EKF_STATUS_NUMERICAL_ERROR` | 数值错误（非有限数、除零等） |
| `ALG_IMU_EKF_STATUS_KALMAN_ERROR` | 底层卡尔曼错误 |

### `alg_imu_ekf_quaternion_t` -- 四元数

| 字段 | 含义 |
|------|------|
| `w` | 标量分量（实部） |
| `x` | X 轴分量（i） |
| `y` | Y 轴分量（j） |
| `z` | Z 轴分量（k） |

表示从机体坐标系到世界坐标系（+Z 向上）的旋转。约束条件：w² + x² + y² + z² = 1。

### `alg_imu_ekf_euler_t` -- 欧拉角（ZYX 顺序）

| 字段 | 含义 | 范围 |
|------|------|------|
| `roll_rad` | 滚转角（绕 X 轴旋转） | [-pi, pi] |
| `pitch_rad` | 俯仰角（绕 Y 轴旋转） | [-pi/2, pi/2] |
| `yaw_rad` | 偏航角（绕 Z 轴旋转）| [-pi, pi] |

ZYX 旋转顺序：先绕 Z（Yaw），再绕新 Y（Pitch），最后绕新 X（Roll）。

### `alg_imu_ekf_config_t` -- 配置参数

| 字段 | 含义 | 推荐值 | 说明 |
|------|------|--------|------|
| `gravity_m_s2` | 标准重力加速度（m/s²） | 9.80665 | 中国地区约 9.79~9.80 |
| `gyro_noise_std_rad_s` | 陀螺仪白噪声标准差（rad/s） | 0.001~0.1 | 消费级 0.01~0.1，工业级 0.001~0.01 |
| `gyro_bias_random_walk_std_rad_s2` | X/Y 零偏随机游走标准差（rad/s²） | 1e-5~1e-2 | 描述零偏随时间的变化速率 |
| `accelerometer_direction_noise_std` | 加速度方向观测噪声标准差 | 0.01~0.1 | 对应约 1~10 度噪声 |
| `accelerometer_lpf_cutoff_hz` | 加速度计低通滤波截止频率（Hz） | 20~50 | 0 = 禁用低通 |
| `accelerometer_rejection_threshold_g` | 模长偏离 1g 的硬拒绝阈值（G） | 0.15~0.30 | 例如 0.20 表示模长超 0.8g~1.2g 时拒绝 |
| `chi_square_adaptation_threshold` | 开始自适应增大噪声的 NIS 阈值 | 5e-9 | 设为 0 禁用自适应 |
| `chi_square_rejection_threshold` | 完全拒绝观测的 NIS 上限阈值 | 1e-8 | 需大于 adaptation_threshold |
| `maximum_measurement_noise_scale` | 自适应测量噪声最大倍率 | 10~30 | 过大则异常观测影响过大 |
| `gyro_bias_fading_factor` | 零偏协方差渐消因子（>=1.0） | 1.0~1.01 | 1.0 表示不渐消 |
| `initial_attitude_variance` | 初始四元数分量方差 | 0.1 | 约 10 度初始不确定度 |
| `initial_gyro_bias_variance` | 初始 X/Y 零偏方差（rad²/s²） | 0.01 | 约 0.1 rad/s 初始不确定度 |

### `alg_imu_ekf_diagnostics_t` -- 诊断信息

| 字段 | 含义 |
|------|------|
| `filtered_accelerometer_m_s2[3]` | 低通滤波后的三轴加速度（m/s²） |
| `innovation[3]` | 三维创新残差向量 y = z - h(x) |
| `accelerometer_norm_m_s2` | 原始加速度模长（m/s²），判断运动状态 |
| `accelerometer_deviation_g` | 加速度模长相对 1g 的偏差 = \|norm - g\| / g |
| `normalized_innovation_squared` | 归一化创新平方（NIS），服从 χ²(3) 分布 |
| `measurement_noise_scale` | 当前测量噪声放大倍率（1.0=基准） |
| `was_accelerometer_used` | 本次更新是否使用了加速度观测 |
| `is_stable` | 陀螺仪模长 < 0.3 rad/s 且加速度偏离 < 0.5 m/s² |
| `has_converged` | 滤波器是否已收敛（NIS 持续低值） |
| `rejection_count` | 连续拒绝观测的累计次数 |

### `alg_imu_ekf_t` -- EKF 对象（约 2.5 KB）

| 字段 | 含义 |
|------|------|
| `config` | 配置参数（值拷贝） |
| `kalman` | 通用 EKF 实例 |
| `state[6]` | 状态向量 [qw, qx, qy, qz, bias_x, bias_y] |
| `covariance[36]` | 6x6 协方差矩阵 |
| `process_noise[36]` | 过程噪声矩阵 Q |
| `measurement_noise[9]` | 测量噪声矩阵 R（3x3） |
| `kalman_workspace[...]` | 通用 EKF 工作区（ALG_KALMAN_WORKSPACE_SIZE(6,3)=366 float）|
| `normalization_workspace[...]` | 四元数归一化工作区（72 float）|
| `innovation_workspace[60]` | 创新计算工作区 |
| `accelerometer_filter[3]` | 三轴加速度一阶低通滤波器 |
| `update_count` | 总更新次数 |
| `yaw_revolution_count` | Yaw 回绕圈数（int32_t） |
| `continuous_yaw_rad` | 连续偏航角 |
| `is_initialized` | 是否已初始化 |

**注意**：初始化后不能按值复制或移动（内部含指针），始终使用指针传递。

---

## 完整 API（14 个函数）

| 函数 | 功能 |
|------|------|
| `alg_imu_ekf_config_init(&config)` | 配置参数初始化为默认值 |
| `alg_imu_ekf_init(me, &config)` | 初始化 EKF 对象 |
| `alg_imu_ekf_reset(me, &q, gyro_bias[2])` | 重置到指定四元数和零偏 |
| `alg_imu_ekf_reset_from_accelerometer(me, accel[3])` | 从加速度计计算 Roll/Pitch 并重置（需静止） |
| `alg_imu_ekf_predict(me, gyro[3], dt)` | 仅执行预测步（陀螺仪积分） |
| `alg_imu_ekf_correct_accelerometer(me, accel[3], dt)` | 仅执行校正步（加速度计观测） |
| `alg_imu_ekf_update(me, gyro[3], accel[3], dt, &used)` | **推荐**：完整预测+校正，输出是否使用加速度 |
| `alg_imu_ekf_get_quaternion(me, &q)` | 获取当前姿态四元数 |
| `alg_imu_ekf_get_euler(me, &euler)` | 获取当前欧拉角（ZYX 顺序） |
| `alg_imu_ekf_get_continuous_yaw(me, &yaw)` | 获取连续偏航角（无 +/-pi 跳变） |
| `alg_imu_ekf_get_gyro_bias(me, bias[3])` | 获取 X/Y 零偏（Z=0） |
| `alg_imu_ekf_get_corrected_gyroscope(me, raw[3], corr[3])` | 获取零偏校正后角速度 |
| `alg_imu_ekf_get_diagnostics(me, &diag)` | 获取运行时诊断信息 |
| `alg_imu_ekf_get_gravity_body(me, grav[3])` | 获取机体系重力向量（m/s²） |
| `alg_imu_ekf_get_linear_acceleration_body(me, accel[3], lin[3])` | 获取机体系线加速度（测量-重力） |
| `alg_imu_ekf_get_linear_acceleration_world(me, accel[3], lin[3])` | 获取世界系线加速度 |

---

## 使用示例

### 基础用法

```c
#include "alg_imu_ekf.h"

static alg_imu_ekf_t ekf;   // 约 2.5KB，建议 static/全局分配

void init_ekf(void)
{
    alg_imu_ekf_config_t cfg;
    alg_imu_ekf_config_init(&cfg);

    // 根据传感器标定微调
    cfg.gyro_noise_std_rad_s = 0.003f;             // BMI088 标称
    cfg.accelerometer_direction_noise_std = 0.05f;
    cfg.accelerometer_lpf_cutoff_hz = 20.0f;       // 20Hz 低通滤振动
    cfg.accelerometer_rejection_threshold_g = 0.20f;
    cfg.chi_square_rejection_threshold = 11.3f;    // 3-DOF 卡方 99%
    cfg.chi_square_adaptation_threshold = 5.0f;    // 3-DOF 卡方 ~85%
    cfg.maximum_measurement_noise_scale = 20.0f;
    cfg.gyro_bias_fading_factor = 1.001f;          // 微弱的零偏渐消

    alg_imu_ekf_init(&ekf, &cfg);

    // 静止水平放置，用加速度计初始化 Roll/Pitch
    float rest_accel[3] = {0.0f, 0.0f, 9.81f};
    alg_imu_ekf_reset_from_accelerometer(&ekf, rest_accel);
}

// IMU 中断回调（1kHz 定时器）
void imu_callback(float gyro[3], float accel[3])
{
    bool accel_used;
    alg_imu_ekf_status_t status = alg_imu_ekf_update(
        &ekf, gyro, accel, 0.001f, &accel_used);

    // 即使 accel_used=false，姿态仍由陀螺积分提供
    if (status != ALG_IMU_EKF_STATUS_OK &&
        status != ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED) {
        // 真正的错误处理（数值错误等）
        handle_ekf_error(status);
    }

    // 获取姿态用于控制
    alg_imu_ekf_euler_t euler;
    alg_imu_ekf_get_euler(&ekf, &euler);

    // 或获取连续 yaw（用于定位/导航）
    float continuous_yaw;
    alg_imu_ekf_get_continuous_yaw(&ekf, &continuous_yaw);
}
```

### 仅预测模式（离地/空中状态）

```c
// 机器人离地或跳跃时，加速度计不可信
void imu_callback_air(float gyro[3], float accel[3])
{
    // 方式一：只用 predict
    alg_imu_ekf_predict(&ekf, gyro, 0.001f);
    // 姿态仅靠陀螺积分，零偏不再更新
}
```

### 线加速度提取（运动检测）

```c
float linear_accel[3];
alg_imu_ekf_get_linear_acceleration_body(&ekf, raw_accel, linear_accel);
// linear_accel 为去除重力后的真实运动加速度
// 可用于碰撞检测、加速度阈值判断等
```

### 运行时诊断

```c
alg_imu_ekf_diagnostics_t diag;
alg_imu_ekf_get_diagnostics(&ekf, &diag);

if (!diag.was_accelerometer_used) {
    // 加速度观测被拒绝，设备可能在快速运动中
}
if (diag.normalized_innovation_squared > 11.3f) {
    // NIS 异常高，可能加速度计故障或设备剧烈运动
}
if (!diag.is_stable && diag.has_converged) {
    // 收敛但正在运动中，姿态可信但精度略降
}
```

---

## 注意事项

1. **Yaw 不可观测**：六轴 IMU 只能长期校正 Roll/Pitch。Z 轴零偏恒为 0（不在状态中），Yaw 完全依赖 Z 轴陀螺积分，会随时间持续漂移。需要磁力计（九轴）、视觉里程计、UWB 或其他外部航向参考来校正 Yaw。

2. **加速度模长拒绝**：加速度计观测仅在静止或匀速运动时可信。`accelerometer_rejection_threshold_g` 控制硬拒绝阈值：设 0.20 表示模长超过 0.8g~1.2g 即拒绝。建议设 0.15~0.30，不推荐超过 0.5。

3. **零偏渐消**：`gyro_bias_fading_factor` 默认 1.0（不渐消）。若温度变化导致陀螺零偏漂移，可设 1.001~1.01。因子过大（如 > 1.05）会导致协方差过快膨胀，零偏估计抖动。

4. **内存分配**：EKF 对象约 2.5KB（含内部工作区），使用 `static` 或全局变量分配，避免栈溢出。

5. **初始化后不可复制**：对象包含指向内部数组的指针和通用 EKF 框架的引用。按值拷贝会导致指针悬空。

6. **卡方阈值标定**：实际 NIS 通常远小于理论卡方分位值（因模型误差和线性化近似），推荐通过实际飞行数据标定合适的阈值，不要盲目使用理论值。

7. **加速度计低通滤波**：由 EKF 内部管理，无需单独调用。截止频率用于滤除高频振动（如电机振动），设 20~50Hz。过高则起不到滤波作用，过低则校正延迟增大。

8. **微分 EKF 形式**：本实现直接在四元数上执行前向积分，雅可比在线计算。相比于 MEKF（误差状态 EKF），代码更简洁但需要归一化+协方差投影步骤来维护单位四元数约束。
