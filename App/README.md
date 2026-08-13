# App -- 可复用机器人控制流程

## 功能概述

App 层只组织机器人控制逻辑，不绑定 HAL、FreeRTOS、CAN ID 或具体任务。每个模块都是普通实例，由调用者持有；输入通过参数显式传入，输出通过 getter 读取，不依赖任何全局交换层。

**数据流向：** `remote + imu + vision` --> `app_command` --> `{chassis, gimbal, shooter}` --> `module_motor / module_shooter` --> 各 getter 读取反馈

## 模块清单

| 模块 | 职责 | 主要接口 |
|------|------|----------|
| `app_command` | 通用遥控输入转换为底盘、云台和发射命令 | `init` / `update` / `get_output` |
| `app_chassis` | 麦轮、全向轮和舵轮底盘控制 | `init` / `update` / `get_feedback` |
| `app_gimbal` | 双轴云台控制，支持编码器或 IMU 反馈 | `init` / `update` / `get_feedback` |
| `app_shooter` | 摩擦轮、拨弹和自动开火逻辑 | `init` / `update` / `get_feedback` |
| `app_imu` | BMI088 采样和 EKF 姿态解算 | `init` / `calibrate` / `update` / `get_snapshot` / `get_state` |
| `app_vision` | USB VCP 视觉目标通信 | `init` / `set_mode` / `update` / `get_target` |
| `app_safety` | 心跳、失联和整机输出门控 | `init` / `register` / `process` / `output_allowed` |
| `app_types.h` | App 之间显式传递的命令和反馈类型 | — |

## 共享类型（`app_types.h`）

各模块通过 `app_types.h` 中定义的结构体显式交换数据，避免耦合到具体遥控器、板间协议或传感器。

### 通用输入与枚举

| 类型 | 说明 |
|------|------|
| `app_switch_t` | 三段开关状态（`INVALID` / `UP` / `DOWN` / `MIDDLE`） |
| `app_remote_input_t` | 项目适配层提供的通用遥控输入（通道、拨杆、鼠标、拨轮） |
| `app_chassis_mode_t` | 底盘驱动模式（无力 / 正常 / 自旋 / 跟随云台） |
| `app_gimbal_feedback_mode_t` | 云台反馈源（编码器 / IMU） |

### 命令类型（`app_command` 发布）

| 类型 | 说明 |
|------|------|
| `app_chassis_command_t` | 底盘运动指令（速度、角速率、模式、自锁） |
| `app_gimbal_command_t` | 云台运动指令（目标角、反馈源） |
| `app_shooter_command_t` | 射击器指令（摩擦轮、单发、连发） |

### 反馈 / 快照类型（各模块发布）

| 类型 | 发布者 | 说明 |
|------|--------|------|
| `app_imu_snapshot_t` | `app_imu` | 姿态快照（欧拉角、修正角速率） |
| `app_gimbal_feedback_t` | `app_gimbal` | 实测角度、角速率、锁定标志 |
| `app_chassis_feedback_t` | `app_chassis` | 指令速度、模式、在线标志 |
| `app_shooter_feedback_t` | `app_shooter` | 状态机、卡弹计数、开火许可 |
| `app_vision_target_t` | `app_vision` | 视觉目标角度、有效性、跟踪标志 |

## API 速查

### `app_command`

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `app_command_init(me, config)` | 初始化命令模块 | `BSP_STATUS_OK` / `BSP_STATUS_INVALID_ARGUMENT` |
| `app_command_update(me, remote, gimbal_feedback, vision_target, dt_s)` | 遥控输入 -> 底盘/云台/发射命令 | `bsp_status_t` |
| `app_command_get_output(me)` | 读取最近输出指令 | 只读指针或 `NULL` |

`remote`、`gimbal_feedback`、`vision_target` 均允许传 `NULL`（失联 / 无反馈 / 无视觉时安全降级）。

### `app_chassis`

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `app_chassis_init(me, config)` | 初始化底盘（麦轮 / 全向轮 / 舵轮） | `bsp_status_t` |
| `app_chassis_update(me, command, dt_s)` | 计算并设置电机目标 | `bsp_status_t` |
| `app_chassis_get_feedback(me)` | 读取底盘反馈 | 只读指针或 `NULL` |

底盘类型仅在初始化时选择，周期任务始终调用 `update()`。

### `app_gimbal`

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `app_gimbal_init(me, config)` | 初始化云台 | `bsp_status_t` |
| `app_gimbal_update(me, command, imu, dt_s)` | 驱动俯仰/偏航到目标角 | `bsp_status_t` |
| `app_gimbal_get_feedback(me)` | 读取云台反馈 | 只读指针或 `NULL` |

`imu` 允许传 `NULL`，此时回退到编码器反馈。

### `app_shooter`

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `app_shooter_init(me, config)` | 初始化射击器 | `bsp_status_t` |
| `app_shooter_update(me, command, gimbal, dt_s)` | 摩擦轮 / 单发 / 连发控制 | `bsp_status_t` |
| `app_shooter_get_feedback(me)` | 读取射击器反馈 | 只读指针或 `NULL` |

### `app_imu`

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `app_imu_init(me, config)` | 初始化 IMU 与 EKF | `bsp_status_t` |
| `app_imu_calibrate(me)` | 执行传感器标定 | `bsp_status_t` |
| `app_imu_update(me, dt_s)` | 执行一个姿态估计周期 | `bsp_status_t` |
| `app_imu_get_snapshot(me)` | 读取姿态快照 | 只读指针或 `NULL` |
| `app_imu_get_state(me)` | 读取运行状态 | `app_imu_state_t` |

### `app_vision`

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `app_vision_init(me, config)` | 初始化视觉模块 | `bsp_status_t` |
| `app_vision_set_mode(me, mode)` | 设置工作模式（手动 / 自动） | `bsp_status_t` |
| `app_vision_update(me, imu, elapsed_ms)` | 收发 12 字节视觉帧 | `bsp_status_t` |
| `app_vision_get_target(me)` | 读取视觉目标 | 只读指针或 `NULL` |

### `app_safety`

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `app_safety_init(me, watchdog)` | 初始化安全模块并关闭输出 | `bsp_status_t` |
| `app_safety_monitor_init(me, config)` | 初始化心跳监控器 | `bsp_status_t` |
| `app_safety_register(me, monitor)` | 注册监控器 | `bsp_status_t` |
| `app_safety_notify_online(monitor, now_ms)` | 上报监控器在线 | — |
| `app_safety_process(me, now_ms)` | 刷新状态并更新输出门 | — |
| `app_safety_set_output_enabled(me, enabled)` | 操作者输出请求 | — |
| `app_safety_output_allowed(me)` | 查询整机输出门 | `bool` |

## 完整使用示例

```c
#include "app_command.h"
#include "app_chassis.h"
#include "app_gimbal.h"
#include "app_shooter.h"
#include "app_imu.h"
#include "app_vision.h"
#include "app_safety.h"

app_command_t command;
app_chassis_t chassis;
app_gimbal_t   gimbal;
app_shooter_t  shooter;
app_imu_t      imu;
app_vision_t   vision;
app_safety_t   safety;
app_safety_monitor_t remote_monitor;

/* --- 初始化阶段 --- */

void app_init(void)
{
    /* 各模块使用静态配置（内部拷贝）初始化，省略配置细节 */
    app_imu_init(&imu, &imu_config);
    app_vision_init(&vision, &vision_config);
    app_command_init(&command, &command_config);
    app_chassis_init(&chassis, &chassis_config);
    app_gimbal_init(&gimbal, &gimbal_config);
    app_shooter_init(&shooter, &shooter_config);

    /* 安全门控：注册必需监控器，上电默认关闭整机输出 */
    app_safety_init(&safety, &watchdog);
    app_safety_monitor_init(&remote_monitor, &(app_safety_monitor_config_t){
        .name = "remote", .timeout_ms = 100, .required = true,
    });
    app_safety_register(&safety, &remote_monitor);
    app_safety_set_output_enabled(&safety, true);
}

/* --- 周期性任务中（如 1kHz） --- */

void robot_task(float dt_s, uint32_t now_ms)
{
    /* 1. 传感器与视觉 */
    app_imu_update(&imu, dt_s);
    app_vision_update(&vision, app_imu_get_snapshot(&imu), now_ms);

    /* 2. 命令转换：遥控 + 云台反馈 + 视觉目标 */
    app_command_update(&command, &remote_input,
                       app_gimbal_get_feedback(&gimbal),
                       app_vision_get_target(&vision), dt_s);

    /* 3. 下发执行 */
    const app_command_output_t *output = app_command_get_output(&command);
    app_chassis_update(&chassis, &output->chassis, dt_s);
    app_gimbal_update(&gimbal, &output->gimbal,
                      app_imu_get_snapshot(&imu), dt_s);
    app_shooter_update(&shooter, &output->shooter,
                       app_gimbal_get_feedback(&gimbal), dt_s);

    /* 4. 安全门控 */
    app_safety_notify_online(&remote_monitor, now_ms);
    app_safety_process(&safety, now_ms);

    /* 5. 电机总线统一刷新 */
    module_dji_motor_bus_update(&dji_bus, dt_s);
    module_dm_motor_bus_update(&dm_bus, dt_s);
}
```

## 注意事项

1. **App 不持有 HAL**：模块只通过指针接收电机、传感器、USB 等实例，不主动分配资源；调用者负责实例的生命周期。
2. **电机总线统一刷新**：`app_chassis_update()` 等只计算并设置目标，调用方必须在所有 App 更新完成后，对每条 DJI/DM 总线各调用一次 `module_*_motor_bus_update(bus, dt_s)`，不要单独调用 `module_motor_update()`。
3. **`delta_time_s` 必须 > 0**：涉及积分（IMU EKF、云台目标累积）的模块，传入 0 或负值会直接返回错误、不执行更新。
4. **允许传 `NULL` 的输入需注意语义**：`app_command_update()` 的 `remote`/`gimbal_feedback`/`vision_target` 与 `app_gimbal_update()` 的 `imu` 允许为 `NULL`，此时模块安全降级（失联、编码器反馈、跳过姿态发送）。
5. **多任务同步由机器人项目负责**：使用目标 RTOS 的队列、任务通知或项目局部快照传递 `app_command_output_t` / `app_imu_snapshot_t` 等结构；App 层不提供全局交换层。
6. **安全门控是最后一道闸**：必需监控器失联会关闭全局电机输出门并撤销操作者请求，恢复在线后需重新 `app_safety_set_output_enabled(true)`。
