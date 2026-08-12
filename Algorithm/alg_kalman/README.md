# alg_kalman -- 通用卡尔曼滤波器库

## 功能概述

纯 C11 卡尔曼滤波库，不依赖 HAL、CMSIS 或 RTOS，不使用动态内存分配。提供三种递推贝叶斯状态估计器：

1. **标量卡尔曼滤波器**（`alg_kalman_scalar_t`） -- 单变量极简实现，适合单传感器平滑
2. **线性卡尔曼滤波器**（`alg_kalman_linear_t`） -- 矩阵形式的经典 KF，适合线性高斯系统
3. **扩展卡尔曼滤波器**（`alg_kalman_extended_t`） -- 处理非线性系统，用户提供状态转移/观测函数及雅可比矩阵

**被依赖**：`alg_imu_ekf` 模块作为 EKF 的上层封装使用本库。

---

## 三种形式的使用场景

| 形式 | 适用场景 | 典型应用 |
|------|---------|---------|
| **标量 KF** | 单个传感器值的低通等效，无状态转移模型 | 电池电压滤波、温度平滑 |
| **线性 KF** | 系统模型和观测模型均为线性 | 目标跟踪（CV/CA 模型）、组合导航误差状态 |
| **扩展 KF** | 系统或观测为非线性 | IMU 姿态估计、SLAM、GNSS/INS 融合 |

选择建议：能用线性 KF 就不用 EKF。线性 KF 的数值性质最优（协方差更新精确），EKF 引入了线性化误差且需要用户提供雅可比，调试复杂度显著增高。

---

## 完整数学原理

### 1. 标量卡尔曼滤波

**模型**：x = x + delta（预测），z = x + v（观测，v ~ N(0, R)）

**预测步**：
```
x_pred = x + state_delta
P_pred = P + Q
```

**校正步**（Joseph 形式）：
```
K = P_pred / (P_pred + R)
x = x_pred + K * (z - x_pred)
P = (1 - K)^2 * P_pred + K^2 * R
```

注意：标量 KF 的协方差更新也使用 Joseph 形式（约瑟夫形式在标量下退化为 `(1-K)^2*P + K^2*R`），并非简化的 `P = (1-K)*P`，以提高数值稳定性。

### 2. 线性卡尔曼滤波

**状态方程**：x_k = F * x_{k-1} + B * u_k + w_k（w ~ N(0, Q)）
**观测方程**：z_k = H * x_k + v_k（v ~ N(0, R)）

**预测步**：
```
x_pred = F * x + B * u
P_pred = F * P * F^T + Q
```

**校正步**：
```
y = z - H * x_pred                    (创新)
S = H * P_pred * H^T + R              (创新协方差)
K = P_pred * H^T * S^{-1}             (卡尔曼增益)
x = x_pred + K * y                    (状态更新)
P = (I-KH) * P_pred * (I-KH)^T +
    K * R * K^T                       (Joseph 协方差更新)
```

### 3. 扩展卡尔曼滤波（EKF）

**状态方程**：x_k = f(x_{k-1}, u_k, dt) + w_k（非线性，f 由回调提供）
**观测方程**：z_k = h(x_k) + v_k（非线性，h 由回调提供）

**预测步**：
```
x_pred = f(x, u, dt)                  (非线性状态转移 via 回调)
F = ∂f/∂x(x, u, dt)                  (雅可比 via 回调)
P_pred = F * P * F^T + Q
```

**校正步**：
```
z_pred = h(x_pred)                    (非线性观测预测 via 回调)
H = ∂h/∂x(x_pred)                    (雅可比 via 回调)
y = z - z_pred                        (创新)
S = H * P_pred * H^T + R              (创新协方差)
K = P_pred * H^T * S^{-1}             (卡尔曼增益)
x = x_pred + K * y                    (状态更新)
P = (I-KH) * P_pred * (I-KH)^T +
    K * R * K^T                       (Joseph 协方差更新)
```

### 4. Joseph 协方差更新

```
P = (I - K*H) * P * (I - K*H)^T + K * R * K^T
```

**为什么不用简化形式 P = (I-KH)*P？**

简化形式在数学上等价（取 K = P*H^T * S^{-1} 时），但在浮点运算中：
- 舍入误差可能导致 P 失去对称性
- 累积的舍入误差可能导致 P 失去正定性（对角线出现负值）
- 失去正定性的协方差会导致卡尔曼增益计算错误，最终滤波器发散

Joseph 形式：
- 对更新项做平方操作（类似 P = A*A^T），强制对称
- 即使 K 不是最优的（有舍入误差），结果仍保持对称正定
- 代价是额外 2 次 n*n 矩阵乘法（约多 50% 计算量）

对于嵌入式应用（n 通常在 6~20 范围），这个计算代价远小于数值发散的风险。

---

## 工作区大小计算

```c
#define ALG_KALMAN_WORKSPACE_SIZE(n, m)  \
    (n + 3*n*n + 4*n*m + 2*m + 2*m*m)
```

| 项 | 大小 | 用途 |
|----|------|------|
| n | n | 预测后的状态向量 |
| 3*n*n | 3n² | 雅可比矩阵 + 临时协方差 + 预测协方差 |
| 4*n*m | 4nm | H*P(m*n) + P*H^T(n*m) + K(n*m) + I-KH(n*n→实际为 n*n) |
| 2*m | 2m | 预测测量值 + 创新残差 |
| 2*m*m | 2m² | 创新协方差 S + S 的逆 |

**注意**：校正核心内部还需要额外的 n*n 临时矩阵（Joseph 计算），这在上述公式中已通过公式整体覆盖。实际使用时用 `ALG_KALMAN_WORKSPACE_SIZE(n, m)` 宏即可。

---

## 核心结构体

### `alg_kalman_status_t` -- 状态码

| 枚举值 | 含义 |
|--------|------|
| `ALG_KALMAN_STATUS_OK` | 操作成功 |
| `ALG_KALMAN_STATUS_INVALID_ARGUMENT` | 参数非法（空指针等） |
| `ALG_KALMAN_STATUS_OUT_OF_RANGE` | 参数超出范围（非有限数、负方差等） |
| `ALG_KALMAN_STATUS_INSUFFICIENT_WORKSPACE` | 工作区大小不够 |
| `ALG_KALMAN_STATUS_NOT_INITIALIZED` | 对象未初始化 |
| `ALG_KALMAN_STATUS_SINGULAR_MATRIX` | 矩阵奇异（行列式为 0，不可求逆） |
| `ALG_KALMAN_STATUS_MODEL_ERROR` | EKF 模型回调函数返回错误 |
| `ALG_KALMAN_STATUS_NUMERICAL_ERROR` | 数值错误（溢出、非有限结果） |

### `alg_kalman_scalar_t` -- 标量卡尔曼

| 字段 | 含义 |
|------|------|
| `process_noise` | 过程噪声方差 Q（>= 0） |
| `measurement_noise` | 测量噪声方差 R（> 0） |
| `estimate` | 当前估计值 |
| `covariance` | 当前协方差 P |
| `gain` | 最近一次的卡尔曼增益 K（只读调试用） |
| `is_initialized` | 是否已初始化 |

### `alg_kalman_linear_config_t` -- 线性 KF 配置

| 字段 | 含义 |
|------|------|
| `state_dimension` | 状态维度 n |
| `measurement_dimension` | 测量维度 m |
| `control_dimension` | 控制维度 c（0 = 无控制输入） |
| `state` | 状态向量指针（n 个 float） |
| `covariance` | 协方差矩阵指针（n*n 个 float，行优先） |
| `transition_matrix` | 状态转移矩阵 F（n*n，行优先） |
| `control_matrix` | 控制矩阵 B（n*c），c=0 时可为 NULL |
| `process_noise` | 过程噪声矩阵 Q（n*n） |
| `measurement_matrix` | 观测矩阵 H（m*n） |
| `measurement_noise` | 测量噪声矩阵 R（m*m） |
| `workspace` | 工作区指针（调用者分配） |
| `workspace_size` | 工作区大小（float 元素数，需 >= ALG_KALMAN_WORKSPACE_SIZE(n,m)） |

**重要**：所有矩阵指针（state, covariance, F, B, H, Q, R, workspace）由调用者提供并维护。滤波器只保存指针（不内部复制）。这些内存必须在滤波器生命周期内保持有效。

### `alg_kalman_linear_t` -- 线性 KF 实例

| 字段 | 含义 |
|------|------|
| `config` | 配置结构体（包含所有矩阵指针的拷贝） |
| `is_initialized` | 是否已初始化 |

### EKF 回调函数类型

| 回调类型 | 签名 | 用途 |
|---------|------|------|
| `alg_kalman_state_function_t` | `f(state, n, u, c, dt, pred, ctx)` | 非线性状态转移 x_pred = f(x, u, dt) |
| `alg_kalman_state_jacobian_function_t` | `jac(state, n, u, c, dt, F, ctx)` | 状态雅可比 F = df/dx（n*n 输出） |
| `alg_kalman_measurement_function_t` | `h(state, n, m, pred, ctx)` | 非线性观测 z_pred = h(x) |
| `alg_kalman_measurement_jacobian_function_t` | `jac(state, n, m, H, ctx)` | 观测雅可比 H = dh/dx（m*n 输出） |

所有回调：
- 返回 `ALG_KALMAN_STATUS_OK` 表示成功，其他值导致预测/校正终止
- state 是当前线性化点（只读，不可修改）
- 输出数组由调用者预分配（大小由维度决定）
- user_context 由 config 中设置，透明传递

### `alg_kalman_extended_config_t` -- EKF 配置

| 字段 | 含义 |
|------|------|
| `state_dimension` | 状态维度 n |
| `measurement_dimension` | 测量维度 m |
| `control_dimension` | 控制维度 c（0 = 无控制） |
| `state` | 状态向量指针（n 个 float） |
| `covariance` | 协方差矩阵指针（n*n 个 float） |
| `process_noise` | 过程噪声矩阵 Q（n*n，可运行时更新） |
| `measurement_noise` | 测量噪声矩阵 R（m*m，可运行时更新） |
| `workspace` | 工作区指针 |
| `workspace_size` | 工作区大小 |
| `state_function` | 状态转移函数回调 |
| `state_jacobian_function` | 状态雅可比回调 |
| `measurement_function` | 测量函数回调 |
| `measurement_jacobian_function` | 测量雅可比回调 |
| `user_context` | 用户上下文（透明传递给所有回调） |

### `alg_kalman_extended_t` -- EKF 实例

| 字段 | 含义 |
|------|------|
| `config` | 配置结构体 |
| `is_initialized` | 是否已初始化 |

---

## 完整 API（16 个函数）

### 标量卡尔曼（6 个函数）

| 函数 | 功能 |
|------|------|
| `alg_kalman_scalar_init(me, Q, R, x0, P0)` | 初始化（Q>=0, R>0, P0>=0） |
| `alg_kalman_scalar_set_noise(me, Q, R)` | 运行时修改噪声参数 |
| `alg_kalman_scalar_reset(me, x0, P0)` | 重置状态和协方差 |
| `alg_kalman_scalar_predict(me, state_delta)` | 预测步（x += delta, P += Q） |
| `alg_kalman_scalar_correct(me, z, &output)` | 校正步（使用 Joseph 协方差更新） |
| `alg_kalman_scalar_update(me, z, &output)` | 便捷函数：predict(0) + correct() |

### 线性卡尔曼（6 个函数）

| 函数 | 功能 |
|------|------|
| `alg_kalman_linear_init(me, &config)` | 初始化（验证配置、对称化协方差） |
| `alg_kalman_linear_reset(me, x0, P0)` | 重置状态和协方差 |
| `alg_kalman_linear_predict(me, &u)` | 预测步（u 可为 NULL，c=0） |
| `alg_kalman_linear_correct(me, &z)` | 校正步（Joseph 协方差更新） |
| `alg_kalman_linear_get_state(me)` | 获取当前状态（只读指针） |
| `alg_kalman_linear_get_covariance(me)` | 获取当前协方差（只读指针） |

### 扩展卡尔曼（4 个函数）

| 函数 | 功能 |
|------|------|
| `alg_kalman_extended_init(me, &config)` | 初始化（验证配置和回调非空） |
| `alg_kalman_extended_reset(me, x0, P0)` | 重置状态和协方差 |
| `alg_kalman_extended_predict(me, &u, dt)` | 预测（调用 f 和 df/dx 回调） |
| `alg_kalman_extended_correct(me, &z)` | 校正（调用 h 和 dh/dx 回调） |
| `alg_kalman_extended_get_state(me)` | 获取当前状态（只读指针） |
| `alg_kalman_extended_get_covariance(me)` | 获取当前协方差（只读指针） |

---

## 使用示例

### 标量卡尔曼 -- 单传感器滤波

```c
#include "alg_kalman.h"

alg_kalman_scalar_t kf;
// Q = 0.01, R = 1.0, x0 = 0, P0 = 1.0
alg_kalman_scalar_init(&kf, 0.01f, 1.0f, 0.0f, 1.0f);

float filtered;
alg_kalman_scalar_update(&kf, raw_value, &filtered);
// filtered 随迭代收敛到真实值的加权平均
```

带状态变化的版本：

```c
// 例：电机转速估计（预测步加转速变化量）
alg_kalman_scalar_predict(&kf, speed_delta);   // x += delta
alg_kalman_scalar_correct(&kf, encoder_speed, &filtered);
```

### 线性卡尔曼 -- 2 状态速度估计器

```c
#include "alg_kalman.h"

// 状态：[位置, 速度]，测量：[位置]，无控制
static float state[2] = {0.0f, 0.0f};
static float cov[4] = {1.0f, 0.0f, 0.0f, 1.0f};
// F: 匀速模型 dt=1ms
static float F[4] = {1.0f, 0.001f, 0.0f, 1.0f};
// H: 只测位置
static float H[2] = {1.0f, 0.0f};
// Q: 过程噪声（速度噪声主导）
static float Q[4] = {0.001f, 0.0f, 0.0f, 0.01f};
// R: 测量噪声
static float R[1] = {0.1f};
static float ws[ALG_KALMAN_WORKSPACE_SIZE(2, 1)];

alg_kalman_linear_t kf;
alg_kalman_linear_config_t cfg = {
    .state_dimension = 2,
    .measurement_dimension = 1,
    .control_dimension = 0,            // 无控制
    .state = state,
    .covariance = cov,
    .transition_matrix = F,
    .control_matrix = NULL,
    .process_noise = Q,
    .measurement_matrix = H,
    .measurement_noise = R,
    .workspace = ws,
    .workspace_size = sizeof(ws) / sizeof(ws[0]),
};
alg_kalman_linear_init(&kf, &cfg);

// 每周期
alg_kalman_linear_predict(&kf, NULL);
float z[1] = {measured_position};
alg_kalman_linear_correct(&kf, z);

// 读取：x[0]=位置, x[1]=速度
const float *x = alg_kalman_linear_get_state(&kf);
```

### 扩展卡尔曼 -- 自定义非线性系统

```c
// 例：1 维匀加速模型，测量距离（非线性）
// 状态：[位置, 速度]，控制：加速度，测量：位置（平方和开方模拟距离传感器）

static alg_kalman_status_t state_func(const float *x, size_t n,
                                       const float *u, size_t c,
                                       float dt, float *x_pred, void *ctx)
{
    (void)ctx;
    if (n != 2 || c != 1) return ALG_KALMAN_STATUS_MODEL_ERROR;
    x_pred[0] = x[0] + x[1]*dt + 0.5f*u[0]*dt*dt;  // 位置 = x + v*dt + ½a*dt²
    x_pred[1] = x[1] + u[0]*dt;                      // 速度 = v + a*dt
    return ALG_KALMAN_STATUS_OK;
}

static alg_kalman_status_t state_jac(const float *x, size_t n,
                                      const float *u, size_t c,
                                      float dt, float *F, void *ctx)
{
    (void)x; (void)u; (void)ctx;
    if (n != 2) return ALG_KALMAN_STATUS_MODEL_ERROR;
    // F = [1, dt; 0, 1]
    F[0] = 1.0f; F[1] = dt;
    F[2] = 0.0f; F[3] = 1.0f;
    return ALG_KALMAN_STATUS_OK;
}

static alg_kalman_status_t meas_func(const float *x, size_t n,
                                      size_t m, float *z_pred, void *ctx)
{
    (void)ctx;
    if (n != 2 || m != 1) return ALG_KALMAN_STATUS_MODEL_ERROR;
    z_pred[0] = x[0];   // 观测 = 位置（如果是线性观测，建议直接用线性 KF）
    return ALG_KALMAN_STATUS_OK;
}

static alg_kalman_status_t meas_jac(const float *x, size_t n,
                                     size_t m, float *H, void *ctx)
{
    (void)x; (void)ctx;
    if (n != 2 || m != 1) return ALG_KALMAN_STATUS_MODEL_ERROR;
    H[0] = 1.0f; H[1] = 0.0f;   // H = [1, 0]
    return ALG_KALMAN_STATUS_OK;
}

// 初始化
static float ekf_state[2] = {0.0f, 0.0f};
static float ekf_cov[4] = {1.0f, 0.0f, 0.0f, 1.0f};
static float ekf_Q[4] = {0.001f, 0.0f, 0.0f, 0.01f};
static float ekf_R[1] = {0.1f};
static float ekf_ws[ALG_KALMAN_WORKSPACE_SIZE(2, 1)];

alg_kalman_extended_t ekf;
alg_kalman_extended_config_t ekf_cfg = {
    .state_dimension = 2,
    .measurement_dimension = 1,
    .control_dimension = 1,
    .state = ekf_state,
    .covariance = ekf_cov,
    .process_noise = ekf_Q,
    .measurement_noise = ekf_R,
    .workspace = ekf_ws,
    .workspace_size = sizeof(ekf_ws) / sizeof(ekf_ws[0]),
    .state_function = state_func,
    .state_jacobian_function = state_jac,
    .measurement_function = meas_func,
    .measurement_jacobian_function = meas_jac,
    .user_context = NULL,
};
alg_kalman_extended_init(&ekf, &ekf_cfg);

// 运行
float u[1] = {measured_acceleration};
float z[1] = {measured_position};
alg_kalman_extended_predict(&ekf, u, 0.001f);   // dt = 1ms
alg_kalman_extended_correct(&ekf, z);
```

---

## 注意事项

### 工作区分配

1. 使用 `ALG_KALMAN_WORKSPACE_SIZE(n, m)` 宏计算所需 float 元素数量。
2. 工作区由调用者以 `float workspace[SIZE]` 方式静态分配。不足时返回 `INSUFFICIENT_WORKSPACE`。
3. 工作区的具体内存布局是实现细节，不可跨版本依赖。

### 矩阵生命周期

1. 线性 KF 的所有矩阵指针（F, B, H, Q, R, state, covariance, workspace）由滤波器保存，必须保证在滤波器生命周期内有效。
2. EKF 的状态和协方差指针同理。
3. **时变模型支持**：可在 `predict()` 前直接修改 F, B, H, Q, R 矩阵内容（例如更新 F 中的 dt 项）。滤波器在 `predict()` 中读取这些矩阵。线程安全需由调用者保证。

### 矩阵存储格式

所有矩阵采用**行优先**（row-major）连续存储。例如 2x3 矩阵：
```
A = [a, b, c]
    [d, e, f]
```
存储为 `float A[6] = {a, b, c, d, e, f}`。

访问元素 A[i][j] 即 `A[i * cols + j]`。

### Joseph 协方差更新

协方差更新统一使用 Joseph 形式 `P = (I-KH)*P*(I-KH)^T + K*R*K^T`，而非简化形式 `P = (I-KH)*P`：
- 优势：强制对称正定，对舍入误差鲁棒
- 代价：多约 2 次 n*n 矩阵乘法
- 对于嵌入式应用（n 通常 <= 20），推荐保留 Joseph 形式

### EKF 回调注意事项

1. **雅可比准确性**：错误的雅可比比错误的噪声协方差更快导致发散。建议对雅可比做有限差分验证。
2. **线性化点**：预测雅可比在预测前的状态处计算，测量雅可比在校正前的状态处计算。
3. **回调返回值**：返回非 OK 时预测/校正终止并传播错误码给调用者。
4. **输出清零**：雅可比回调必须先将输出矩阵清零，再填充非零元素。框架不代清零。

### 数值稳定性

1. 内部在每次校正后对称化协方差：`P = (P + P^T) / 2`。
2. 矩阵求逆使用带部分主元选择的 Gauss-Jordan 消元法，行列式 < 1e-12 时判定奇异。
3. 所有输入和中间结果均检查 `isfinite()`。

### 标量卡尔曼特殊说明

`alg_kalman_scalar_update()` 内部先执行 `predict(0)` 再执行 `correct()`，适合稳态无控制输入的单传感器滤波。如果状态有确定的变化量（如积分），使用 `predict(delta)` + `correct(z)` 分步调用。
