# app_gimbal -- 云台俯仰/偏航双轴角度控制

## 功能概述

云台控制模块驱动俯仰（pitch）和偏航（yaw）两个电机轴到目标角度位置。云台指令和 IMU 快照由更新参数显式传入，最新反馈保存在实例中。

**数据流向：** `command + imu_snapshot` --> `app_gimbal` --> `module_motor` --> `app_gimbal_get_feedback()`

## 核心结构体

### 配置结构体 `app_gimbal_config_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `pitch_motor` | `module_motor_t *` | 俯仰轴电机实例 |
| `yaw_motor` | `module_motor_t *` | 偏航轴电机实例 |
| `target_tolerance_rad` | `float` | 判定目标已锁定的位置误差阈值 [rad] |

### 运行时实例 `app_gimbal_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `config` | `app_gimbal_config_t` | 静态配置的副本 |
| `initialized` | `bool` | 初始化阶段已成功完成 |

### 交换数据类型（定义在 `app_types.h`）

**`app_gimbal_command_t`** -- 命令层发布的云台运动指令：

| 字段 | 类型 | 说明 |
|------|------|------|
| `yaw_target_rad` | `float` | 期望偏航角 [rad] |
| `pitch_target_rad` | `float` | 期望俯仰角 [rad] |
| `feedback_mode` | `app_gimbal_feedback_mode_t` | 选定的反馈源（编码器或 IMU） |
| `enabled` | `bool` | 指令有效标志，`false` 时电机断电 |
| `sequence` | `uint32_t` | 单调递增的帧序号 |

**`app_gimbal_feedback_t`** -- 每周期发布的云台反馈：

| 字段 | 类型 | 说明 |
|------|------|------|
| `yaw_rad` | `float` | 实测偏航角 [rad] |
| `pitch_rad` | `float` | 实测俯仰角 [rad] |
| `yaw_velocity_rad_per_s` | `float` | 实测偏航角速率 [rad/s] |
| `pitch_velocity_rad_per_s` | `float` | 实测俯仰角速率 [rad/s] |
| `motors_online` | `bool` | 俯仰和偏航双电机均在线 |
| `target_locked` | `bool` | 双轴位置误差均在 `target_tolerance_rad` 内 |
| `imu_valid` | `bool` | 本周期 IMU 姿态数据有效 |

### 反馈模式枚举 `app_gimbal_feedback_mode_t`

| 枚举值 | 说明 |
|--------|------|
| `APP_GIMBAL_FEEDBACK_ENCODER` | 使用电机编码器作为位置反馈源 |
| `APP_GIMBAL_FEEDBACK_IMU` | 使用 IMU 姿态融合（`imu.pitch_rad`, `imu.yaw_rad`）作为位置反馈源。IMU 无效时自动回退到编码器 |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `app_gimbal_init(me, config)` | 初始化云台实例，校验参数并拷贝配置 | `BSP_STATUS_OK` / `BSP_STATUS_INVALID_ARGUMENT` |
| `app_gimbal_update(me, command, imu, delta_time_s)` | 执行一个云台控制周期：显式指令/IMU -> 选反馈源 -> 更新电机 -> 保存反馈 | `bsp_status_t` |
| `app_gimbal_get_feedback(me)` | 读取最近云台反馈 | 只读指针或 `NULL` |

## 反馈源选择逻辑

```
                    command.feedback_mode
                           |
            +--------------+--------------+
            |                             |
    APP_GIMBAL_FEEDBACK_IMU    APP_GIMBAL_FEEDBACK_ENCODER
            |                             |
            v                             v
     imu.valid == true?            使用编码器反馈
       |         |              pitch_feedback->position_rad
       v         v              yaw_feedback->position_rad
      YES       NO
       |         |
       v         v
   使用 IMU    回退到编码器
   imu.pitch_rad
   imu.yaw_rad
```

## 使用示例

```c
#include "app_gimbal.h"
#include "module_motor.h"

/* --- 初始化阶段 --- */

// 获取俯仰和偏航电机实例（假设 board_config 已初始化）
module_motor_t *pitch_motor = board_config_get_motor(BOARD_CONFIG_MOTOR_GIMBAL_PITCH);
module_motor_t *yaw_motor   = board_config_get_motor(BOARD_CONFIG_MOTOR_GIMBAL_YAW);

app_gimbal_t gimbal;
app_gimbal_config_t config = {
    .pitch_motor          = pitch_motor,
    .yaw_motor            = yaw_motor,
    .target_tolerance_rad = 0.0175f,       // 1 度
};

bsp_status_t rc = app_gimbal_init(&gimbal, &config);
if (rc != BSP_STATUS_OK) {
    // 错误处理
}

/* --- 周期性任务中（如 1kHz） --- */

void gimbal_task(float dt_s)
{
    app_gimbal_update(&gimbal, &command, &imu_snapshot, dt_s);

    // 可选：读取反馈用于调试/日志
    app_gimbal_feedback_t fb;
fb = *app_gimbal_get_feedback(&gimbal);
    if (fb.target_locked) {
        // 云台已锁定目标
    }
}
```

## 注意事项

1. **电机必须配置为角度模式**：云台电机需在 `module_motor` 层配置为角度（位置）PID 控制模式，速度/力矩模式无法正确跟踪目标角。
2. **IMU 反馈模式需先初始化 IMU**：使用 `APP_GIMBAL_FEEDBACK_IMU` 时，必须确保 `app_imu` 已初始化并正常运行，否则 `imu.valid` 始终为 `false`，模块会自动回退到编码器模式。
3. **`target_tolerance_rad` 影响射击窗口**：此容差同时用于反馈中的 `target_locked` 标志，该标志被 `app_shooter` 用来判断是否允许自动连发。设置过大可能导致未对准时误发射，设置过小则射击窗口过窄。
4. **`command.enabled == false` 时电机断电**：遥控断连或安全急停时，云台电机会立即断电（`module_motor_disable`），不受控的云台可能因重力或惯性漂移。
5. **角度模式不处理多圈累积**：如果偏航电机是连续旋转的，编码器反馈会溢出。需要在上层处理多圈累积逻辑，或将电机配置为有限行程。
