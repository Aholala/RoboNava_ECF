# alg_lqr -- 固定增益线性二次型状态反馈控制器

## 功能概述

车载端执行固定增益 LQR 状态反馈控制。增益矩阵 K 由离线工具（如 MATLAB、Python Control Library）根据车型和工作点求解 Riccati 方程得到，以常量形式嵌入固件。模块不在 MCU 上求解 Riccati 方程，不包含模型离散化。

典型应用场景：平衡小车（倒立摆）的平衡控制、底盘速度跟踪、云台角度控制等——任何适合状态反馈的线性化系统。

## 核心结构体

### `alg_lqr_status_t` -- 状态码枚举

| 枚举值 | 含义 |
|--------|------|
| `ALG_LQR_STATUS_OK` | 执行成功 |
| `ALG_LQR_STATUS_INVALID_ARGUMENT` | 无效参数（NULL 指针等） |
| `ALG_LQR_STATUS_OUT_OF_RANGE` | 参数超出合法范围 |
| `ALG_LQR_STATUS_NOT_INITIALIZED` | 对象未初始化 |
| `ALG_LQR_STATUS_NUMERICAL_ERROR` | 数值计算错误（NaN 或 Inf） |

### `alg_lqr_config_t` -- 控制器配置

| 字段 | 类型 | 含义 |
|------|------|------|
| `state_dimension` | `size_t` | 状态向量维度 n |
| `control_dimension` | `size_t` | 控制向量维度 m |
| `gain_matrix` | `const float *` | 增益矩阵 K（行优先，m x n） |
| `control_min` | `const float *` | 控制下限（长度 m），NULL 表示无限幅 |
| `control_max` | `const float *` | 控制上限（长度 m），NULL 表示无限幅 |

### `alg_lqr_t` -- 控制器实例

| 字段 | 类型 | 含义 |
|------|------|------|
| `config` | `alg_lqr_config_t` | 配置副本 |
| `is_initialized` | `bool` | 是否已初始化 |

## 数学原理

### 控制律

```
u = u_ff - K * (x - x_ref)
```

其中：
- `u`：控制输出向量（m x 1）
- `u_ff`：前馈控制量（m x 1），用于补偿已知扰动
- `K`：增益矩阵（m x n），离线求解
- `x`：当前状态向量（n x 1）
- `x_ref`：参考状态向量（n x 1），通常为零状态

### LQR 问题的标准形式

在 MATLAB / Python 中求解：

```matlab
% 连续时间 LQR
sys = ss(A, B, C, D);
[K, S, e] = lqr(sys, Q, R, N);

% 或直接用 CARE 方程
[K, S, e] = lqr(A, B, Q, R, N);
```

最小化性能指标：

```
J = integral(x'*Q*x + u'*R*u + 2*x'*N*u) dt
```

增益矩阵：

```
K = R^(-1) * (B'*S + N')
```

其中 S 是连续代数 Riccati 方程（CARE）的解：

```
A'*S + S*A - (S*B + N)*R^(-1)*(B'*S + N') + Q = 0
```

### 离散化注意

离线求解的是**连续时间** LQR。车载端以固定周期调用 `alg_lqr_update`，实际构成零阶保持离散闭环。系统带宽远高于控制频率（10x 以上）时近似有效。如需精确离散控制，应在离线工具中使用 `dlqr` 并将离散 A、B 验证后再导入。

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_lqr_init(me, config)` | 初始化 LQR 控制器 | `alg_lqr_status_t` |
| `alg_lqr_update(me, reference, state, feedforward, output)` | 计算控制输出 | `alg_lqr_status_t` |

## 使用示例

### 平衡小车（倒立摆）-- 2 状态 1 输出

```c
#include "alg_lqr.h"

// 状态：[角度偏差, 角速度]，单位 rad, rad/s
// 输出：电机电压

// 离线求解的增益矩阵（使用连续时间 LQR）
static const float gain[2] = {-45.0f, -8.5f};  // K = [k1, k2]
static const float ctrl_min[1] = {-12.0f};     // 电压下限
static const float ctrl_max[1] = {12.0f};       // 电压上限

alg_lqr_t balance_ctrl;
alg_lqr_config_t cfg = {
    .state_dimension = 2,
    .control_dimension = 1,
    .gain_matrix = gain,
    .control_min = ctrl_min,
    .control_max = ctrl_max,
};

void init_controller(void)
{
    alg_lqr_init(&balance_ctrl, &cfg);
}

float compute_motor_voltage(float angle_rad, float angular_velocity_rad_per_s)
{
    float state[2] = {angle_rad, angular_velocity_rad_per_s};
    float output[1] = {0.0f};

    // 零参考 + 零前馈
    alg_lqr_update(&balance_ctrl, NULL, state, NULL, output);
    return output[0];
}
```

### 速度跟踪 -- 含前馈

```c
// 状态：[速度误差, 位置误差]
float state[2] = {speed_error, position_error};
float reference[2] = {0.0f, 0.0f};       // 零误差为参考
float feedforward[1] = {1.2f};           // 平衡所需基值

alg_lqr_update(&ctrl, reference, state, feedforward, output);
```

## 注意事项

1. **增益离线求解**：模块**不**在校车端求解 Riccati 方程。K 矩阵必须在 MATLAB、Python 等工具中提前计算，并以常量写入代码。
2. **连续 vs 离散**：离线求解的 K 是连续 LQR 结果。离散化误差在控制频率远高于系统带宽时（10x 以上）可忽略，否则应使用 `dlqr` 并在离线阶段验证。
3. **增益矩阵布局**：`gain_matrix` 为行优先存储，尺寸为 `control_dimension x state_dimension`。例如 1x2 的增益 `{k1, k2}` 对应的输出为 `k1*state[0] + k2*state[1]`。
4. **参考状态**：`reference` 可为 NULL（即零参考）。所有状态和参考使用**绝对单位**（非归一化），确保与离线设计一致。
5. **输出限幅**：`control_min` / `control_max` 对每个控制通道独立限幅。不提供限幅后的反计算或积分抗饱和——饱和时控制输出直接被截断。
6. **NaN/Inf 拒绝**：输入出现非有限值时返回 `NUMERICAL_ERROR`。
