# app_safety -- 安全/心跳看门狗监控

## 功能概述

安全模块管理最多 16 个软件心跳监控器，每个监控器跟踪一个子系统（如电机 CAN 总线、遥控接收机、IMU 等）的在线状态。当某个监控器在配置的超时时间内未收到心跳，模块将其标记为离线并触发用户回调。此外，模块在对所有监控器执行完状态评估后刷新硬件看门狗，形成软件+硬件双重保护。

**数据流向：** 各子系统任务/ISR 调用 `app_safety_notify_online()` 喂狗 --> `app_safety_process()` 周期性评估超时并刷新硬件看门狗

## 核心结构体

### 枚举 `app_safety_state_t`

| 值 | 说明 |
|------|------|
| `APP_SAFETY_STATE_STARTING` | 已注册但尚未收到首次心跳 |
| `APP_SAFETY_STATE_ONLINE` | 心跳在超时阈值内到达 |
| `APP_SAFETY_STATE_OFFLINE` | 超时已过期，系统认为该子系统故障 |

### 回调函数类型 `app_safety_callback_t`

```c
typedef void (*app_safety_callback_t)(void *user_context);
```

离线/在线回调签名，`user_context` 由注册时提供。

### 配置结构体 `app_safety_monitor_config_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `name` | `const char *` | 人类可读的名称（用于日志输出，如 "DR16", "CAN1"） |
| `timeout_ms` | `uint32_t` | 心跳超时阈值 [ms] |
| `required` | `bool` | 是否参与 `app_safety_all_required_online()` 的统计 |
| `offline_callback` | `app_safety_callback_t` | 转入离线状态时调用的回调（可为 NULL） |
| `online_callback` | `app_safety_callback_t` | 转入在线状态时调用的回调（可为 NULL） |
| `user_context` | `void *` | 传递给回调的不透明指针 |

### 运行时状态 `app_safety_monitor_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `config` | `app_safety_monitor_config_t` | 静态配置的副本 |
| `last_online_tick` | `volatile uint32_t` | 最后一次心跳的 FreeRTOS 滴答值 |
| `heartbeat_received` | `volatile bool` | 至少收到过一次心跳 |
| `offline_time_ms` | `uint32_t` | 距上次在线的经过时间 [ms] |
| `state` | `app_safety_state_t` | 当前在线/离线状态 |
| `is_registered` | `bool` | 监控器已注册 |

### 配置常量

| 宏 | 值 | 说明 |
|------|------|------|
| `APP_SAFETY_MAX_MONITOR_COUNT` | `16` | 最大可注册的心跳监控器数量 |
| `APP_SAFETY_DEFAULT_TASK_PERIOD_MS` | `5` | 推荐的安全处理任务周期 [ms] |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `app_safety_init(watchdog)` | 初始化安全管理器单例，可选绑定硬件看门狗 | `BSP_STATUS_OK` |
| `app_safety_set_watchdog(watchdog)` | 运行时替换硬件看门狗实例 | `BSP_STATUS_OK` / `BSP_STATUS_NOT_INITIALIZED` |
| `app_safety_monitor_init(me, config)` | 初始化监控器结构体（注册前必须调用） | `BSP_STATUS_OK` / `BSP_STATUS_INVALID_ARGUMENT` |
| `app_safety_register(monitor)` | 将已初始化的监控器注册到管理器 | `BSP_STATUS_OK` / `BSP_STATUS_NOT_INITIALIZED` / `BSP_STATUS_INVALID_ARGUMENT` / `BSP_STATUS_BUSY` / `BSP_STATUS_NO_RESOURCE` |
| `app_safety_notify_online(monitor)` | 为指定监控器记录一次心跳 | `void` |
| `app_safety_process()` | 执行安全处理周期：评估超时 -> 触发回调 -> 刷新硬件看门狗 | `void` |
| `app_safety_get_state(monitor)` | 查询监控器的当前状态（NULL 返回 OFFLINE） | `app_safety_state_t` |
| `app_safety_get_offline_time_ms(monitor)` | 查询监控器已离线时长（NULL 返回 UINT32_MAX） | `uint32_t` |
| `app_safety_all_required_online()` | 检查所有 `required` 标记的监控器是否均在线 | `bool` |

## 状态转换图

```
                      app_safety_monitor_init()
                               |
                               v
                    +-------------------+
                    |    STARTING        |
                    | (已注册，未收到心跳) |
                    +--------+----------+
                             |
              +--------------+--------------+
              | heartbeat_received          | 超时
              | 且 elapsed <= timeout_ms    | elapsed > timeout_ms
              v                             v
    +------------------+         +------------------+
    |     ONLINE        |         |     OFFLINE      |
    | 心跳正常           | ------> | 超时，触发回调    |
    +------------------+         +------------------+
              ^                             |
              |                             |
              +-----------------------------+
                 心跳恢复 (elapsed <= timeout_ms)
                       触发 online_callback
```

状态转换仅在 `app_safety_process()` 中发生，且仅当新状态不同于当前状态时触发回调（防止重复触发）。

## 使用示例

```c
#include "app_safety.h"
#include "bsp_watchdog.h"
#include "FreeRTOS.h"
#include "task.h"

/* ==== 离线/在线回调 ==== */

static void on_dr16_offline(void *ctx)
{
    // 遥控掉线：禁用所有动力输出
    // ctx 可能指向底盘实例等
}

static void on_dr16_online(void *ctx)
{
    // 遥控恢复：重新使能控制
}

/* ==== 监控器定义 ==== */

static app_safety_monitor_t monitor_dr16;
static app_safety_monitor_t monitor_can1;
static app_safety_monitor_t monitor_imu;

/* ==== 初始化（在 main 或 init 任务中） ==== */

void safety_init(void)
{
    bsp_watchdog_t wdog;
    bsp_watchdog_init(&wdog, &iwdg_handle);  // 绑定 STM32 IWDG

    // 1. 初始化安全管理器
    app_safety_init(&wdog);

    // 2. 初始化并注册监控器
    app_safety_monitor_config_t cfg_dr16 = {
        .name             = "DR16",
        .timeout_ms       = 500,          // 500ms 无心跳视为掉线
        .required         = true,         // 参与全局必须在线检查
        .offline_callback = on_dr16_offline,
        .online_callback  = on_dr16_online,
        .user_context     = NULL,
    };
    app_safety_monitor_init(&monitor_dr16, &cfg_dr16);
    app_safety_register(&monitor_dr16);

    app_safety_monitor_config_t cfg_can1 = {
        .name       = "CAN1",
        .timeout_ms = 100,
        .required   = true,
    };
    app_safety_monitor_init(&monitor_can1, &cfg_can1);
    app_safety_register(&monitor_can1);

    app_safety_monitor_config_t cfg_imu = {
        .name       = "IMU",
        .timeout_ms = 50,
        .required   = false,             // 可选，不影响全局检查
    };
    app_safety_monitor_init(&monitor_imu, &cfg_imu);
    app_safety_register(&monitor_imu);
}

/* ==== 各模块喂狗 ==== */

void dr16_isr_handler(void)
{
    // 收到 DR16 帧
    app_safety_notify_online(&monitor_dr16);
}

void can1_task(void)
{
    // 收到 CAN 消息
    app_safety_notify_online(&monitor_can1);
}

void imu_task(void)
{
    app_safety_notify_online(&monitor_imu);
    app_imu_update(&imu, dt);
}

/* ==== 安全处理任务（5ms 周期） ==== */

void safety_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        app_safety_process();

        // 可选：检查全局状态
        if (!app_safety_all_required_online()) {
            // 有必须在线模块掉线 -> 执行安全策略
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(APP_SAFETY_DEFAULT_TASK_PERIOD_MS));
    }
}
```

## 注意事项

1. **`app_safety_process()` 必须周期性调用**：超时判断和状态转换都在此函数内完成。建议使用独立任务以 5ms（`APP_SAFETY_DEFAULT_TASK_PERIOD_MS`）周期调用。
2. **喂狗可从 ISR 调用**：`last_online_tick` 和 `heartbeat_received` 声明为 `volatile`，ISR 中调用 `app_safety_notify_online` 是安全的（仅写入滴答值和一个 bool）。
3. **监控器数量上限 16**：超限时 `app_safety_register` 返回 `BSP_STATUS_NO_RESOURCE`。
4. **已注册的监控器不可重复注册**：重复注册返回 `BSP_STATUS_BUSY`。
5. **`required` 标记的使用场景**：将安全关键的子系统（如遥控、底盘 CAN）标记为 `required`，通过 `app_safety_all_required_online()` 统一判断是否所有关键链路在线。
6. **硬件看门狗刷新**：`app_safety_process()` 在遍历完所有监控器后刷新硬件看门狗。如果安全任务因死锁或高优先级任务不释放 CPU 而无法执行，硬件看门狗将超时复位系统。
7. **离线回调不会重复触发**：内置防重入逻辑，仅在状态从非离线转为离线时调用 `offline_callback`。
8. **`timeout_ms` 不能为 0**：初始化校验会拒绝超时为 0 的配置。
