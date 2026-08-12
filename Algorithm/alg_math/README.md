# alg_math -- 基础数学算法库

## 功能概述

纯 C11 数学工具库，为项目中所有上层算法模块提供基础数学运算。覆盖四大领域：

1. **标量工具**：限幅、线性插值、区间映射、死区、角度回绕、安全平方根/除法
2. **在线统计**：基于 Welford 算法的均值、方差、标准差、最小/最大值（无需存储历史样本）
3. **向量/四元数**：2D/3D 向量的加/减/缩放/点积/叉积/模长/归一化，四元数的构造/乘法/共轭/归一化/SLERP 球面插值/旋转向量/欧拉角互转
4. **动态矩阵**：矩阵初始化/清零/单位化/加减乘/转置/向量乘/求逆（Gauss-Jordan）/线性方程组求解（Gauss 消元）/Cholesky 分解

**不依赖**：HAL、CMSIS、RTOS、动态内存。所有数据缓冲区由调用者提供。

## 核心结构体

### `alg_math_status_t` -- 状态码

| 枚举值 | 含义 |
|--------|------|
| `ALG_MATH_STATUS_OK` | 操作成功 |
| `ALG_MATH_STATUS_INVALID_ARGUMENT` | 参数非法（空指针、内存复用冲突等） |
| `ALG_MATH_STATUS_OUT_OF_RANGE` | 参数超出范围（NaN、Inf、非严格递增等） |
| `ALG_MATH_STATUS_SIZE_MISMATCH` | 矩阵维度或工作区大小不匹配 |
| `ALG_MATH_STATUS_SINGULAR` | 奇异（零向量归一化、奇异矩阵、非正定矩阵） |
| `ALG_MATH_STATUS_NUMERICAL_ERROR` | 数值错误（溢出、非有限结果） |

### `alg_math_vector2_t` -- 二维向量

| 字段 | 类型 | 含义 |
|------|------|------|
| `x` | `float` | X 分量 |
| `y` | `float` | Y 分量 |

### `alg_math_vector3_t` -- 三维向量

| 字段 | 类型 | 含义 |
|------|------|------|
| `x` | `float` | X 分量 |
| `y` | `float` | Y 分量 |
| `z` | `float` | Z 分量 |

### `alg_math_quaternion_t` -- 四元数

| 字段 | 类型 | 含义 |
|------|------|------|
| `w` | `float` | 标量（实部） |
| `x` | `float` | X 轴虚部分量 |
| `y` | `float` | Y 轴虚部分量 |
| `z` | `float` | Z 轴虚部分量 |

表示主动旋转，实部在前，虚部在后。w=1, x=y=z=0 为单位四元数（恒等旋转）。

### `alg_math_matrix_t` -- 动态矩阵描述符

| 字段 | 类型 | 含义 |
|------|------|------|
| `rows` | `size_t` | 行数 |
| `columns` | `size_t` | 列数 |
| `data` | `float *` | 数据指针（行优先，大小 = rows * columns） |

矩阵描述符只持有数据指针，不拥有数据内存。支持任意尺寸矩阵。

### `alg_math_statistics_t` -- 在线统计

| 字段 | 类型 | 含义 |
|------|------|------|
| `sample_count` | `uint32_t` | 累计样本数 |
| `mean` | `float` | 当前均值 |
| `sum_of_squared_deviations` | `float` | 离差平方和（用于方差计算） |
| `minimum` | `float` | 历史最小值 |
| `maximum` | `float` | 历史最大值 |

### 工作区大小宏

```c
// 矩阵求逆工作区
ALG_MATH_MATRIX_INVERSE_WORKSPACE_SIZE(order)  // = 2 * order * order

// 线性方程组求解工作区
ALG_MATH_MATRIX_SOLVE_WORKSPACE_SIZE(order)    // = order * (order + 1)
```

### 数学常量

| 宏 | 值 | 含义 |
|----|-----|------|
| `ALG_MATH_PI_F` | 3.14159265358979f | 圆周率 |
| `ALG_MATH_TWO_PI_F` | 6.28318530717959f | 2 倍圆周率 |
| `ALG_MATH_HALF_PI_F` | 1.57079632679490f | 半圆周率 |
| `ALG_MATH_DEG_TO_RAD_F` | PI/180 | 度转弧度系数 |
| `ALG_MATH_RAD_TO_DEG_F` | 180/PI | 弧度转度系数 |

## 数学原理

### Welford 在线统计算法

单程、数值稳定的在线方差算法：

```
count = count + 1
delta = x - mean
mean = mean + delta / count
M2 = M2 + delta * (x - mean)   // M2 = sum_of_squared_deviations
```

方差：

```
population_variance = M2 / N
sample_variance = M2 / (N - 1)   （N >= 2）
```

### 四元数运算

**乘法**（组合旋转）：

```
q_result = q_left * q_right
```

其中 (q * p) 表示先应用 p 旋转再应用 q 旋转（主动旋转约定）。

**旋转向量**：

```
v_rotated = q * v * q_conjugate
```

**欧拉角 -> 四元数**（ZYX 顺序）：

```
q = q_yaw * q_pitch * q_roll
```

其中：

```
q_roll  = [cos(r/2), sin(r/2), 0, 0]
q_pitch = [cos(p/2), 0, sin(p/2), 0]
q_yaw   = [cos(y/2), 0, 0, sin(y/2)]
```

**SLERP（球面线性插值）**：

```
cos_theta = dot(q_start, q_end)
theta = acos(|cos_theta|)
if |cos_theta| < 0.999:
    q_result = (sin((1-t)*theta)*q_start + sin(t*theta)*q_end) / sin(theta)
else:
    q_result = lerp(q_start, q_end, t)  // 接近平行时退化为线性插值
```

### 一维分段线性插值

给定严格递增的 x 表 `x[0..n-1]` 和 `y[0..n-1]`，对输入 `x_in`：

```
找到区间 i 使得 x[i] <= x_in < x[i+1]
ratio = (x_in - x[i]) / (x[i+1] - x[i])
output = y[i] + ratio * (y[i+1] - y[i])
```

外推处理（由 `clamp_to_table` 控制）。

### 矩阵求逆（Gauss-Jordan 消元）

```
1. 构建增广矩阵 [A | I]
2. 部分主元：选列中绝对值最大的行交换
3. 前向消元：将主元归一化，消去其他行的该列元素
4. 反向替换：右侧 I 部分即为 A 的逆矩阵
```

### Cholesky 分解

对称正定矩阵 A 分解为：

```
A = L * L^T
```

其中 L 为下三角矩阵。逐列递推：

```
L[j][j] = sqrt(A[j][j] - sum(L[j][k]^2, k=0..j-1))
L[i][j] = (A[i][j] - sum(L[i][k]*L[j][k], k=0..j-1)) / L[j][j]   (i > j)
```

## API 速查

### 标量工具

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_math_is_finite_array(values, count)` | 检查数组是否全部有限 | `bool` |
| `alg_math_clamp(value, lo, hi, &result)` | 限幅到 [lo, hi] | `alg_math_status_t` |
| `alg_math_lerp(start, end, ratio, &result)` | 线性插值 | `alg_math_status_t` |
| `alg_math_map_range(value, in_lo, in_hi, out_lo, out_hi, clamp, &result)` | 区间映射 | `alg_math_status_t` |
| `alg_math_apply_deadband(value, deadband, rescale, &result)` | 死区处理 | `alg_math_status_t` |
| `alg_math_wrap(value, lo, hi, &result)` | 回绕到 [lo, hi) | `alg_math_status_t` |
| `alg_math_wrap_angle_pi(angle_rad, &result)` | 角度回绕到 [-pi, pi) | `alg_math_status_t` |
| `alg_math_angle_difference(target, current, &diff)` | 最短角度差 | `alg_math_status_t` |
| `alg_math_degrees_to_radians(deg)` | 度转弧度 | `float` |
| `alg_math_radians_to_degrees(rad)` | 弧度转度 | `float` |
| `alg_math_safe_sqrt(value, &result)` | 安全平方根（检查非负） | `alg_math_status_t` |
| `alg_math_safe_divide(num, den, min_den, &result)` | 安全除法（防除零） | `alg_math_status_t` |

### 在线统计

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_math_statistics_init(me)` | 初始化统计对象 | `alg_math_status_t` |
| `alg_math_statistics_update(me, sample)` | 喂入新样本 | `alg_math_status_t` |
| `alg_math_statistics_get_population_variance(me, &var)` | 总体方差（分母 N） | `alg_math_status_t` |
| `alg_math_statistics_get_sample_variance(me, &var)` | 样本方差（分母 N-1） | `alg_math_status_t` |
| `alg_math_statistics_get_standard_deviation(me, sample_std, &std)` | 标准差 | `alg_math_status_t` |
| `alg_math_array_mean(values, count, &mean)` | 数组均值 | `alg_math_status_t` |
| `alg_math_array_rms(values, count, &rms)` | 数组均方根（RMS） | `alg_math_status_t` |

### 插值

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_math_interpolate_linear1_d(x_tab, y_tab, n, input, clamp, &output)` | 一维分段线性插值 | `alg_math_status_t` |
| `alg_math_interpolate_bilinear(xr, yr, v00, v10, v01, v11, &output)` | 双线性插值（单位正方形） | `alg_math_status_t` |

### 二维向量运算

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_math_vector2_add(&a, &b, &result)` | 向量加法 | `alg_math_status_t` |
| `alg_math_vector2_subtract(&a, &b, &result)` | 向量减法 | `alg_math_status_t` |
| `alg_math_vector2_scale(&v, scale, &result)` | 向量缩放 | `alg_math_status_t` |
| `alg_math_vector2_dot(&a, &b, &result)` | 点积 | `alg_math_status_t` |
| `alg_math_vector2_norm(&v, &norm)` | 模长 | `alg_math_status_t` |
| `alg_math_vector2_normalize(&v, &result)` | 归一化 | `alg_math_status_t` |

### 三维向量运算

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_math_vector3_add(&a, &b, &result)` | 向量加法 | `alg_math_status_t` |
| `alg_math_vector3_subtract(&a, &b, &result)` | 向量减法 | `alg_math_status_t` |
| `alg_math_vector3_scale(&v, scale, &result)` | 向量缩放 | `alg_math_status_t` |
| `alg_math_vector3_dot(&a, &b, &result)` | 点积 | `alg_math_status_t` |
| `alg_math_vector3_cross(&a, &b, &result)` | 叉积 | `alg_math_status_t` |
| `alg_math_vector3_norm(&v, &norm)` | 模长 | `alg_math_status_t` |
| `alg_math_vector3_normalize(&v, &result)` | 归一化 | `alg_math_status_t` |

### 四元数运算

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_math_quaternion_identity(&result)` | 获取单位四元数 | `alg_math_status_t` |
| `alg_math_quaternion_normalize(&q, &result)` | 归一化 | `alg_math_status_t` |
| `alg_math_quaternion_conjugate(&q, &result)` | 共轭 | `alg_math_status_t` |
| `alg_math_quaternion_multiply(&a, &b, &result)` | 乘法（左乘） | `alg_math_status_t` |
| `alg_math_quaternion_from_euler(roll, pitch, yaw, &result)` | 欧拉角 -> 四元数（ZYX） | `alg_math_status_t` |
| `alg_math_quaternion_to_euler(&q, &euler)` | 四元数 -> 欧拉角（输出为 vector3） | `alg_math_status_t` |
| `alg_math_quaternion_rotate_vector(&q, &v, &result)` | 旋转向量 | `alg_math_status_t` |
| `alg_math_quaternion_slerp(&start, &end, ratio, &result)` | 球面线性插值 | `alg_math_status_t` |

### 动态矩阵运算

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_math_matrix_init(&m, data, rows, cols)` | 初始化矩阵描述符 | `alg_math_status_t` |
| `alg_math_matrix_zero(&m)` | 矩阵清零 | `alg_math_status_t` |
| `alg_math_matrix_identity(&m)` | 设置单位矩阵（方阵） | `alg_math_status_t` |
| `alg_math_matrix_copy(&src, &dst)` | 复制矩阵 | `alg_math_status_t` |
| `alg_math_matrix_add(&a, &b, &result)` | 矩阵加法 | `alg_math_status_t` |
| `alg_math_matrix_subtract(&a, &b, &result)` | 矩阵减法 | `alg_math_status_t` |
| `alg_math_matrix_scale(&in, scale, &result)` | 矩阵缩放 | `alg_math_status_t` |
| `alg_math_matrix_multiply(&a, &b, &result)` | 矩阵乘法（禁内存复用） | `alg_math_status_t` |
| `alg_math_matrix_transpose(&in, &result)` | 矩阵转置（方阵支持原地） | `alg_math_status_t` |
| `alg_math_matrix_multiply_vector(&m, vec, vlen, result, rlen)` | 矩阵乘向量 | `alg_math_status_t` |
| `alg_math_matrix_invert(&in, &inv, ws, ws_sz)` | 矩阵求逆（Gauss-Jordan） | `alg_math_status_t` |
| `alg_math_matrix_solve(&A, b, x, ws, ws_sz)` | 线性方程组求解 Ax = b | `alg_math_status_t` |
| `alg_math_matrix_cholesky(&A, &L)` | Cholesky 分解 A = L*L^T | `alg_math_status_t` |

## 使用示例

### 标量工具

```c
#include "alg_math.h"

// 限幅
float result;
alg_math_clamp(1500.0f, -1000.0f, 1000.0f, &result);  // result = 1000.0f

// 区间映射
alg_math_map_range(512, 0, 1023, -1.0f, 1.0f, false, &result);
// ADC 值 512 映射到 0.0

// 角度回绕
alg_math_wrap_angle_pi(3.5f, &result);  // result ≈ -2.78（3.5 - 2*pi）

// 最短角度差
alg_math_angle_difference(3.0f, -3.0f, &result);  // result ≈ -0.283（最短路径）

// 安全除法
alg_math_safe_divide(1.0f, 0.001f, 1e-6f, &result);  // result = 1000.0
```

### Welford 在线统计

```c
alg_math_statistics_t stats;
alg_math_statistics_init(&stats);

// 逐样本喂入
for (int i = 0; i < 1000; i++) {
    alg_math_statistics_update(&stats, sensor_read());
}

// 查询
float mean = stats.mean;
float pop_var, sample_var, std_dev;
alg_math_statistics_get_population_variance(&stats, &pop_var);
alg_math_statistics_get_standard_deviation(&stats, true, &std_dev);

// 直接获取 min/max
float sensor_min = stats.minimum;
float sensor_max = stats.maximum;
```

### 四元数 -- 姿态合成

```c
// 从欧拉角构建
alg_math_quaternion_t q;
alg_math_quaternion_from_euler(0.1f, -0.2f, 0.5f, &q);  // roll, pitch, yaw

// 旋转向量
alg_math_vector3_t body_vec = {1.0f, 0.0f, 0.0f};
alg_math_vector3_t world_vec;
alg_math_quaternion_rotate_vector(&q, &body_vec, &world_vec);

// SLERP 插值（两帧姿态间平滑）
alg_math_quaternion_t q_interp;
alg_math_quaternion_slerp(&q_prev, &q_curr, 0.5f, &q_interp);
// q_interp 为两帧之间的中间姿态
```

### 查表插值

```c
// 温度-电阻 校准表
float temp_c[] = {0, 25, 50, 75, 100};
float resist_kohm[] = {32.6f, 10.0f, 3.6f, 1.5f, 0.68f};

float reading = 5.2f;  // kOhm
float temperature;
alg_math_interpolate_linear1_d(resist_kohm, temp_c, 5, reading, true, &temperature);
// temperature ≈ 38.5°C（5.2k 在 10k(25°C) 和 3.6k(50°C) 之间）
```

### 矩阵运算 -- 最小二乘求解

```c
float A_data[6] = {1, 0, 0, 1, 1, 1};  // 2x3 矩阵
float b_data[2] = {5.0f, 8.0f};
float x[3] = {0};

alg_math_matrix_t A, AT, ATA;
// 构建 A^T * A * x = A^T * b
// ...（省略构建步骤）

float ws[ALG_MATH_MATRIX_SOLVE_WORKSPACE_SIZE(3)];
alg_math_matrix_solve(&ATA, ATb_data, x, ws, sizeof(ws)/sizeof(float));
```

### Cholesky 分解 -- 对称正定系统

```c
float A_data[9] = {4, 2, -2, 2, 5, 3, -2, 3, 9};
float L_data[9];

alg_math_matrix_t A, L;
alg_math_matrix_init(&A, A_data, 3, 3);
alg_math_matrix_init(&L, L_data, 3, 3);

alg_math_matrix_cholesky(&A, &L);
// L 为下三角矩阵，满足 A = L * L^T
```

## 注意事项

1. **矩阵内存复用**：`alg_math_matrix_multiply` 禁止 result 与输入共享数据指针。转置在方阵时支持原地操作。求逆时输入和输出必须独立。
2. **四元数欧拉角转换**：使用 ZYX 旋转顺序。`alg_math_quaternion_to_euler` 输出为 `alg_math_vector3_t`，其中 x=roll, y=pitch, z=yaw。
3. **插值查表严格递增**：`alg_math_interpolate_linear1_d` 要求 `x_values` 严格递增，否则返回 `OUT_OF_RANGE` 或产生未定义行为。
4. **Cholesky 条件**：要求输入矩阵对称正定（或近似对称，仅做轻量检查）。非正定矩阵会导致 NaN 或溢出。
5. **安全函数**：`alg_math_safe_sqrt` 和 `alg_math_safe_divide` 在输入非法时返回错误码而非 NaN，适合嵌入式安全关键路径。
6. **统计计数溢出**：`sample_count` 为 `uint32_t`，在极高频率长时间累加时（如 1kHz 运行 1000 小时以上）可能溢出，届时行为未定义。
