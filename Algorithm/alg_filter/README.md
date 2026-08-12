# alg_filter -- 数字滤波器库

## 功能概述

纯 C11 数字滤波器库，提供 8 种常用滤波器，覆盖从简单平滑到通用 IIR/FIR 设计。所有滤波器共享统一的设计模式：

- **无动态内存**：缓冲区由调用者静态分配
- **显式时间步长**：`delta_time_s` 由调用者传入，不依赖系统时钟
- **独立实例**：每个对象拥有独立状态，支持任意数量的静态实例
- **统一错误码**：所有 API 返回 `alg_filter_status_t`

### 滤波器一览

| 滤波器 | 结构体 | 典型用途 |
|--------|--------|----------|
| 一阶低通 | `alg_filter_low_pass_t` | 平滑传感器噪声、加速度计滤波 |
| 一阶高通 | `alg_filter_high_pass_t` | 去除直流分量、提取 AC 信号 |
| 指数移动平均 | `alg_filter_exponential_t` | 极简平滑、极低计算开销 |
| 滑动平均 | `alg_filter_moving_average_t` | 固定窗口均值、数据平滑 |
| 中值滤波 | `alg_filter_median_t` | 去除脉冲噪声（如编码器尖刺） |
| FIR 滤波器 | `alg_filter_fir_t` | 自定义任意系数线性相位滤波 |
| Biquad 滤波器 | `alg_filter_biquad_t` | IIR 二阶（低通/高通/带通/陷波） |
| 互补滤波器 | `alg_filter_complementary_t` | 融合高频通道和低频通道 |

## 核心结构体

### `alg_filter_status_t` -- 公共状态码

| 枚举值 | 含义 |
|--------|------|
| `ALG_FILTER_STATUS_OK` | 操作成功 |
| `ALG_FILTER_STATUS_INVALID_ARGUMENT` | 参数非法（空指针等） |
| `ALG_FILTER_STATUS_OUT_OF_RANGE` | 参数超出范围 |
| `ALG_FILTER_STATUS_NOT_INITIALIZED` | 对象未初始化 |
| `ALG_FILTER_STATUS_NUMERICAL_ERROR` | 数值错误（溢出、非有限数等） |

### 各滤波器结构体字段

#### `alg_filter_low_pass_t` -- 一阶低通

| 字段 | 类型 | 含义 | 约束 |
|------|------|------|------|
| `cutoff_frequency_hz` | `float` | 截止频率（Hz） | > 0 |
| `output` | `float` | 当前输出值 | -- |
| `is_initialized` | `bool` | 是否已初始化 | -- |
| `has_previous_sample` | `bool` | 是否有上次采样（首次直接赋值） | -- |

#### `alg_filter_high_pass_t` -- 一阶高通

| 字段 | 类型 | 含义 | 约束 |
|------|------|------|------|
| `cutoff_frequency_hz` | `float` | 截止频率（Hz） | > 0 |
| `previous_input` | `float` | 上次输入值 | -- |
| `output` | `float` | 当前输出值 | -- |
| `is_initialized` | `bool` | 是否已初始化 | -- |
| `has_previous_sample` | `bool` | 是否有上次采样 | -- |

#### `alg_filter_exponential_t` -- 指数移动平均

| 字段 | 类型 | 含义 | 约束 |
|------|------|------|------|
| `smoothing_factor` | `float` | 平滑因子 alpha | (0, 1] |
| `output` | `float` | 当前输出值 | -- |
| `is_initialized` | `bool` | 是否已初始化 | -- |
| `has_previous_sample` | `bool` | 是否有上次采样 | -- |

#### `alg_filter_moving_average_t` -- 滑动平均

| 字段 | 类型 | 含义 | 约束 |
|------|------|------|------|
| `sample_buffer` | `float *` | 样本缓冲区（调用者分配） | -- |
| `capacity` | `size_t` | 缓冲区容量 | > 0 |
| `sample_count` | `size_t` | 当前有效样本数 | 0 ~ capacity |
| `write_index` | `size_t` | 写入位置 | 0 ~ capacity-1 |
| `sum` | `float` | 窗口内样本和 | -- |
| `is_initialized` | `bool` | 是否已初始化 | -- |

#### `alg_filter_median_t` -- 中值滤波

| 字段 | 类型 | 含义 | 约束 |
|------|------|------|------|
| `sample_buffer` | `float *` | 样本缓冲区（调用者分配） | -- |
| `sort_buffer` | `float *` | 排序工作区（调用者分配） | -- |
| `capacity` | `size_t` | 缓冲区容量 | > 0 |
| `sample_count` | `size_t` | 当前有效样本数 | 0 ~ capacity |
| `write_index` | `size_t` | 写入位置 | 0 ~ capacity-1 |
| `is_initialized` | `bool` | 是否已初始化 | -- |

#### `alg_filter_fir_t` -- FIR 滤波器

| 字段 | 类型 | 含义 | 约束 |
|------|------|------|------|
| `coefficients` | `const float *` | FIR 系数数组（调用者提供） | -- |
| `state_buffer` | `float *` | 状态缓冲区（调用者提供） | -- |
| `tap_count` | `size_t` | 抽头数（滤波器阶数 + 1） | > 0 |
| `write_index` | `size_t` | 写入位置 | 0 ~ tap_count-1 |
| `is_initialized` | `bool` | 是否已初始化 | -- |

#### `alg_filter_biquad_t` -- Biquad 滤波器

| 字段 | 类型 | 含义 |
|------|------|------|
| `b0, b1, b2` | `float` | 分子系数（零点） |
| `a1, a2` | `float` | 分母系数（极点，注意符号约定） |
| `state_1, state_2` | `float` | 直接 II 型转置的状态变量 |
| `is_initialized` | `bool` | 是否已初始化 |

#### `alg_filter_biquad_type_t` -- Biquad 响应类型

| 枚举值 | 含义 |
|--------|------|
| `ALG_FILTER_BIQUAD_LOW_PASS` | 低通 |
| `ALG_FILTER_BIQUAD_HIGH_PASS` | 高通 |
| `ALG_FILTER_BIQUAD_BAND_PASS` | 带通 |
| `ALG_FILTER_BIQUAD_NOTCH` | 陷波（带阻） |

#### `alg_filter_complementary_t` -- 互补滤波器

| 字段 | 类型 | 含义 | 约束 |
|------|------|------|------|
| `prediction_weight` | `float` | 预测权重（值越大越信任积分预测） | [0, 1] |
| `output` | `float` | 当前输出值 | -- |
| `is_initialized` | `bool` | 是否已初始化 | -- |

## 数学原理

### 一阶低通滤波器

模拟 RC 电路的差分方程实现：

```
tau = 1 / (2 * pi * fc)
alpha = dt / (tau + dt)
y[n] = y[n-1] + alpha * (x[n] - y[n-1])
```

传递函数（连续域）：

```
H(s) = 1 / (tau * s + 1)
```

**首次调用**：`has_previous_sample == false` 时，输出直接赋值为输入（跳过瞬态响应）。

### 一阶高通滤波器

```
tau = 1 / (2 * pi * fc)
alpha = tau / (tau + dt)
y[n] = alpha * (y[n-1] + x[n] - x[n-1])
```

传递函数（连续域）：

```
H(s) = tau * s / (tau * s + 1)
```

### 指数移动平均（EMA）

```
y[n] = y[n-1] + alpha * (x[n] - y[n-1])
```

alpha 与等效截止频率 fc 的关系（在给定采样频率 fs 下）：

```
alpha = 1 - exp(-2 * pi * fc / fs)
```

alpha 越大响应越快（平滑越少），alpha = 1 时输出等于输入（无滤波）。

### 滑动平均（SMA）

```
y[n] = (1 / N) * sum(x[n], x[n-1], ..., x[n-N+1])
```

使用环形缓冲区在线更新，维护窗口内所有样本的和以避免重复求和。时间复杂度 O(1)。

### 中值滤波

取窗口内所有样本的中位数。使用排序工作区（不改变样本缓冲区），每次更新 O(N log N)。

### FIR 滤波器

```
y[n] = sum(b[k] * x[n-k], k=0..M-1)
```

其中 M = tap_count，b[0] 对应当前输入。可设计任意频率响应，保证线性相位（对称系数时）。

### Biquad 滤波器

传递函数（Z 域）：

```
H(z) = (b0 + b1*z^(-1) + b2*z^(-2)) / (1 + a1*z^(-1) + a2*z^(-2))
```

使用**转置直接 II 型结构**：

```
w1 = x + (-a1)*w1 + (-a2)*w2  // 注意：这里的 a1, a2 已取负号
y  = b0*x + b1*w1 + b2*w2
```

系数设计公式（连续时间原型，双线性变换）：

```
K = tan(pi * fc / fs)
```

**低通**：

```
omega = 2 * pi * fc / fs
alpha = sin(omega) / (2 * Q)
b0 = (1 - cos(omega)) / 2
b1 = 1 - cos(omega)
b2 = (1 - cos(omega)) / 2
a0 = 1 + alpha
a1 = -2 * cos(omega)
a2 = 1 - alpha
归一化：b /= a0, a1 /= a0, a2 /= a0
```

**高通**：类似地将 `cos(omega)` 取反。

**带通**：

```
b0 = alpha
b1 = 0
b2 = -alpha
```

**陷波**：

```
b0 = 1
b1 = -2 * cos(omega)
b2 = 1
```

### 互补滤波器

```
y[n] = weight * (y[n-1] + rate * dt) + (1 - weight) * measured
```

典型应用：陀螺仪（高频通道，rate 积分） + 加速度计（低频通道，measured 直接角度）。

## API 速查

### 一阶低通

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_filter_low_pass_init(me, cutoff_hz)` | 初始化 | `alg_filter_status_t` |
| `alg_filter_low_pass_set_cutoff(me, cutoff_hz)` | 运行时修改截止频率 | `alg_filter_status_t` |
| `alg_filter_low_pass_reset(me, initial_output)` | 重置输出到指定值 | `alg_filter_status_t` |
| `alg_filter_low_pass_update(me, input, dt, &output)` | 更新 | `alg_filter_status_t` |

### 一阶高通

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_filter_high_pass_init(me, cutoff_hz)` | 初始化 | `alg_filter_status_t` |
| `alg_filter_high_pass_set_cutoff(me, cutoff_hz)` | 运行时修改截止频率 | `alg_filter_status_t` |
| `alg_filter_high_pass_reset(me, initial_input)` | 重置 | `alg_filter_status_t` |
| `alg_filter_high_pass_update(me, input, dt, &output)` | 更新 | `alg_filter_status_t` |

### 指数移动平均

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_filter_exponential_init(me, factor)` | 初始化（factor 为 alpha） | `alg_filter_status_t` |
| `alg_filter_exponential_set_factor(me, factor)` | 运行时修改 alpha | `alg_filter_status_t` |
| `alg_filter_exponential_reset(me, initial_output)` | 重置 | `alg_filter_status_t` |
| `alg_filter_exponential_update(me, input, &output)` | 更新（无 dt 参数） | `alg_filter_status_t` |

### 滑动平均

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_filter_moving_average_init(me, buffer, capacity)` | 初始化（buffer 由调用者分配） | `alg_filter_status_t` |
| `alg_filter_moving_average_reset(me)` | 清空历史 | `alg_filter_status_t` |
| `alg_filter_moving_average_update(me, input, &output)` | 更新 | `alg_filter_status_t` |

### 中值滤波

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_filter_median_init(me, sample_buf, sort_buf, capacity)` | 初始化 | `alg_filter_status_t` |
| `alg_filter_median_reset(me)` | 清空历史 | `alg_filter_status_t` |
| `alg_filter_median_update(me, input, &output)` | 更新 | `alg_filter_status_t` |

### FIR 滤波器

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_filter_fir_init(me, coefficients, state_buf, tap_count)` | 初始化 | `alg_filter_status_t` |
| `alg_filter_fir_reset(me)` | 清空状态缓冲 | `alg_filter_status_t` |
| `alg_filter_fir_update(me, input, &output)` | 更新 | `alg_filter_status_t` |

### Biquad 滤波器

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_filter_biquad_init(me, type, fs, fc, Q)` | 初始化（自动计算系数） | `alg_filter_status_t` |
| `alg_filter_biquad_set_coefficients(me, b0, b1, b2, a1, a2)` | 手动设置系数 | `alg_filter_status_t` |
| `alg_filter_biquad_reset(me)` | 清空状态 | `alg_filter_status_t` |
| `alg_filter_biquad_update(me, input, &output)` | 更新 | `alg_filter_status_t` |

### 互补滤波器

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_filter_complementary_init(me, weight, initial_output)` | 初始化 | `alg_filter_status_t` |
| `alg_filter_complementary_set_weight(me, weight)` | 运行时修改权重 | `alg_filter_status_t` |
| `alg_filter_complementary_reset(me, initial_output)` | 重置 | `alg_filter_status_t` |
| `alg_filter_complementary_update(me, measured, rate, dt, &output)` | 更新 | `alg_filter_status_t` |

## 使用示例

### 传感器低通滤波（加速度计 50Hz 截止）

```c
#include "alg_filter.h"

alg_filter_low_pass_t accel_lpf[3];  // 三轴各一个

void init(void)
{
    for (int i = 0; i < 3; i++) {
        alg_filter_low_pass_init(&accel_lpf[i], 50.0f);  // 50Hz 截止
    }
}

void imu_callback(float ax, float ay, float az)
{
    float filtered[3];
    alg_filter_low_pass_update(&accel_lpf[0], ax, 0.001f, &filtered[0]);
    alg_filter_low_pass_update(&accel_lpf[1], ay, 0.001f, &filtered[1]);
    alg_filter_low_pass_update(&accel_lpf[2], az, 0.001f, &filtered[2]);
    // 使用 filtered 进行后续处理
}
```

### 互补滤波 -- 陀螺 + 加速度计角度融合

```c
alg_filter_complementary_t angle_filter;

// 初始化，陀螺权重 98%（信任积分），加速度权重 2%（长期校正）
alg_filter_complementary_init(&angle_filter, 0.98f, 0.0f);

// 每周期调用
void update_angle(float gyro_rate_rad_per_s, float accel_angle_rad, float dt)
{
    float fused_angle;
    // measured = 加速度计直接角度
    // rate     = 陀螺仪角速度
    alg_filter_complementary_update(&angle_filter,
        accel_angle_rad,           // 测量值（低频参考）
        gyro_rate_rad_per_s,       // 变化率（高频积分）
        dt, &fused_angle);
    // fused_angle 短期跟踪陀螺、长期收敛到加速度计角度
}
```

### 指数平滑 -- 极简编码器速度滤波

```c
alg_filter_exponential_t speed_filter;
alg_filter_exponential_init(&speed_filter, 0.1f);  // alpha = 0.1，强平滑

float raw_speed = compute_encoder_speed();
float filtered_speed;
alg_filter_exponential_update(&speed_filter, raw_speed, &filtered_speed);
```

### Biquad 陷波器 -- 去除 50Hz 工频干扰

```c
alg_filter_biquad_t notch;
// 采样率 1000Hz，中心频率 50Hz，Q 值 30
alg_filter_biquad_init(&notch, ALG_FILTER_BIQUAD_NOTCH, 1000.0f, 50.0f, 30.0f);

// 每周期更新
float clean;
alg_filter_biquad_update(&notch, noisy_signal, &clean);
```

### 中值滤波 -- 编码器脉冲去尖刺

```c
float sample_buf[7];
float sort_buf[7];
alg_filter_median_t median;

alg_filter_median_init(&median, sample_buf, sort_buf, 7);

float clean;
alg_filter_median_update(&median, raw_encoder, &clean);
// 窗口大小 7，取中位数，有效去除偶尔的尖刺脉冲
```

### 组合使用 -- 级联滤波链

```c
// 原始信号 -> 中值（去尖刺） -> 低通（平滑） -> 输出
alg_filter_median_t median;
alg_filter_low_pass_t lpf;

float step1, final;
alg_filter_median_update(&median, raw, &step1);
alg_filter_low_pass_update(&lpf, step1, 0.001f, &final);
```

## 注意事项

1. **缓冲区所有权**：滑动平均和中值滤波器的 sample_buffer 和 sort_buffer 由调用者分配和管理，对象只保存指针。调用者必须保证缓冲区的生命周期覆盖对象使用期。
2. **Biquad 系数符号**：`a1` 和 `a2` 的符号约定为 `H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)`。当通过 `set_coefficients` 手动设置时，确保系数符号与此一致。
3. **指数平滑无 dt**：`alg_filter_exponential_update` 不需要 `delta_time_s` 参数，因为 alpha 本身已包含时间尺度信息。
4. **互补滤波初始化**：`alg_filter_complementary_init` 需要传入 `initial_output`，作为首次融合的基准。
5. **首次瞬态**：低通和高通滤波器在首次调用时直接赋值输入（跳过瞬态），避免初始响应尖峰。
6. **在线修改参数**：低通、高通、指数平滑和互补滤波均支持 `set_*` 运行时修改关键参数，无需重新初始化。
7. **FIR 系数索引**：`coefficients[0]` 对应当前输入 `x[n]`，`coefficients[k-1]` 对应 `x[n-(k-1)]`。
