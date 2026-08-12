# app_imu -- 基于扩展卡尔曼滤波的 IMU 姿态估计

## 功能概述

IMU 姿态模块读取 BMI088 六轴 IMU 传感器（加速度计 + 陀螺仪）的原始数据，通过扩展卡尔曼滤波（EKF）融合估计机器人的三轴欧拉角（偏航/俯仰/横滚）以及修正后的角速率。姿态快照通过交换层发布，供云台控制、视觉通信和底盘参考系转换等模块消费。

**数据流向：** BMI088（SPI） --> `module_bmi088`（读取原始数据） --> `app_imu`（EKF 姿态估计） --> `app_exchange`（imu_snapshot） --> `app_gimbal`, `app_vision`, `app_command`

## 核心结构体

### 配置结构体 `app_imu_config_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `sensor` | `module_bmi088_t *` | BMI088 传感器实例（需已初始化） |
| `ekf_config` | `const alg_imu_ekf_config_t *` | EKF 调参配置，`NULL` 时使用默认参数 |

### 运行时实例 `app_imu_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `sensor` | `module_bmi088_t *` | BMI088 传感器句柄 |
| `ekf` | `alg_imu_ekf_t` | 扩展卡尔曼滤波器状态 |
| `snapshot` | `app_imu_snapshot_t` | 最新姿态快照（每周期更新） |
| `attitude_initialized` | `bool` | 俯仰/横滚已由加速度计成功初始化 |
| `initialized` | `bool` | 初始化阶段已成功完成 |

### 交换数据类型 `app_imu_snapshot_t`（定义在 `app_types.h`）

| 字段 | 类型 | 说明 |
|------|------|------|
| `yaw_rad` | `float` | 欧拉偏航角 [rad] |
| `pitch_rad` | `float` | 欧拉俯仰角 [rad] |
| `roll_rad` | `float` | 欧拉横滚角 [rad] |
| `angular_velocity_rad_per_s` | `float[3]` | 修正后的陀螺仪角速率 (x, y, z) [rad/s] |
| `sample_count` | `uint32_t` | 累计传感器采样计数 |
| `valid` | `bool` | 姿态估计有效标志（传感器正常 + EKF 收敛时置 `true`） |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `app_imu_init(me, config)` | 初始化 IMU 实例和 EKF，传感器未就绪或 EKF 初始化失败时返回错误 | `BSP_STATUS_OK` / `BSP_STATUS_INVALID_ARGUMENT` |
| `app_imu_update(me, delta_time_s)` | 执行一个姿态估计周期：读取传感器 -> EKF 更新 -> 发布快照到交换层 | `void` |

## 姿态初始化与时序

```
上电
 |
 v
app_imu_init()  --> EKF 初始化
 |
 v
app_imu_update() 首次调用
 |
 +-- 传感器读取失败？ ---> snapshot.valid = false, publish, return
 |
 +-- 传感器在线但数据无效？ ---> snapshot.valid = false, publish, return
 |
 +-- attitude_initialized == false？
 |      |
 |      +-- EKF 从加速度计初始化俯仰/横滚
 |      |   alg_imu_ekf_reset_from_accelerometer()
 |      |   成功: attitude_initialized = true
 |      |   失败: 下周期重试
 |      |
 |      v
 +-- attitude_initialized == true？
        |
        +-- EKF 更新（融合陀螺仪 + 加速度计）
        |   alg_imu_ekf_update()
        |
        +-- 提取欧拉角 + 修正角速率
        |   alg_imu_ekf_get_euler()
        |   alg_imu_ekf_get_corrected_gyroscope()
        |
        +-- 填充 snapshot, valid = true
        +-- 发布到交换层
```

**注意：** 偏航角（yaw）无法仅通过加速度计初始化（需要磁力计或其他绝对参考），因此 EKF 的 yaw 状态从 0 开始。

## 使用示例

```c
#include "app_imu.h"
#include "app_exchange.h"
#include "module_bmi088.h"
#include "alg_imu_ekf.h"

/* --- 初始化阶段 --- */

module_bmi088_t bmi088;
// 假设 bmi088 已通过 SPI 初始化并配置为 1kHz ODR ...

app_imu_t imu;

// 方式一：使用默认 EKF 参数
app_imu_config_t config_default = {
    .sensor     = &bmi088,
    .ekf_config = NULL,    // 使用 alg_imu_ekf_config_init() 的默认值
};

// 方式二：自定义 EKF 参数
alg_imu_ekf_config_t ekf_cfg;
alg_imu_ekf_config_init(&ekf_cfg);
ekf_cfg.gyroscope_noise = 0.001f;       // 根据需要调参
ekf_cfg.accelerometer_noise = 0.01f;
app_imu_config_t config_custom = {
    .sensor     = &bmi088,
    .ekf_config = &ekf_cfg,
};

bsp_status_t rc = app_imu_init(&imu, &config_custom);
if (rc != BSP_STATUS_OK) {
    // 错误处理：检查 BMI088 是否已初始化
}

/* --- 周期性任务中（与 BMI088 ODR 匹配，如 1kHz） --- */

void imu_task(float dt_s)
{
    app_imu_update(&imu, dt_s);

    // 可选：外部读取 IMU 快照用于日志/遥测
    app_imu_snapshot_t snap;
    app_exchange_read_imu(&snap);
    if (snap.valid) {
        // snap.pitch_rad, snap.yaw_rad, snap.roll_rad 可用
    }
}
```

## 注意事项

1. **`delta_time_s` 必须 > 0**：EKF 需要积分步长。传入 0 或负值会导致 update 函数直接返回，不执行任何更新。
2. **传感器必须先初始化**：`config->sensor->is_initialized` 必须在 `app_imu_init` 调用前已为 `true`，否则初始化会失败。
3. **偏航角漂移**：EKF 仅通过加速度计修正俯仰和横滚，偏航角无绝对参考，长时间运行会因陀螺仪零漂产生累积误差。需要外部绝对参考（如视觉、磁力计）来校正偏航角。
4. **首次有效数据决定俯仰/横滚初值**：上电后第一次有效的加速度计数据会被用于初始化俯仰和横滚角。确保上电时机器人处于静止或准静止状态，否则初值会有较大偏差。
5. **传感器离线处理**：传感器读取失败或数据无效时，`snapshot.valid` 置为 `false` 并发布。消费者模块（如 `app_gimbal`）会回退到编码器反馈模式。
6. **EKF 默认参数可能不匹配机械结构**：默认 EKF 参数是通用值，如需更高精度建议根据实际机械振动和运动特性调参。
