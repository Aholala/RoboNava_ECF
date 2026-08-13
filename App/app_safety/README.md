# app_safety -- 心跳、失联与整机输出门控

## 功能概述

`app_safety` 管理固定容量的心跳监控器、可选硬件看门狗和整机电机输出门。它是普通实例，不依赖 RTOS。只有存在必需（`required`）监控器、所有必需监控器在线且操作者已请求输出时，`app_safety_output_allowed()` 才为 `true`。

**数据流向：** `notify_online` / `set_output_enabled` --> `app_safety` --> `module_motor_set_output_allowed()`

## 核心结构体

### 监控器配置 `app_safety_monitor_config_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `name` | `const char *` | 监控器名称（用于日志） |
| `timeout_ms` | `uint32_t` | 心跳超时时间 [ms] |
| `required` | `bool` | 是否必需：失联时关闭整机输出 |
| `offline_callback` | `app_safety_callback_t` | 失联回调，可为 `NULL` |
| `online_callback` | `app_safety_callback_t` | 在线回调，可为 `NULL` |
| `user_context` | `void *` | 回调用户上下文 |

### 监控器实例 `app_safety_monitor_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `config` | `app_safety_monitor_config_t` | 静态配置的副本 |
| `last_online_time_ms` | `volatile uint32_t` | 上次收到心跳的时刻 [ms] |
| `heartbeat_received` | `volatile bool` | 是否已收到心跳 |
| `offline_time_ms` | `uint32_t` | 已失联时长 [ms] |
| `state` | `app_safety_state_t` | 当前在线状态 |
| `is_registered` | `bool` | 已注册到安全实例 |

### 安全实例 `app_safety_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `monitors[16]` | `app_safety_monitor_t *[16]` | 已注册监控器数组 |
| `monitor_count` | `size_t` | 已注册监控器数量 |
| `watchdog` | `bsp_watchdog_t *` | 可选硬件看门狗 |
| `output_enabled` | `bool` | 操作者已请求输出 |
| `output_allowed` | `bool` | 整机输出门 |
| `initialized` | `bool` | 初始化阶段已成功完成 |

### 状态枚举 `app_safety_state_t`

| 枚举值 | 说明 |
|--------|------|
| `APP_SAFETY_STATE_STARTING` | 启动中，尚未收到首个心跳 |
| `APP_SAFETY_STATE_ONLINE` | 在线 |
| `APP_SAFETY_STATE_OFFLINE` | 失联 |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `app_safety_init(me, watchdog)` | 初始化安全模块并关闭整机输出 | `bsp_status_t` |
| `app_safety_set_watchdog(me, watchdog)` | 运行时更换看门狗 | `bsp_status_t` |
| `app_safety_monitor_init(me, config)` | 初始化心跳监控器 | `bsp_status_t` |
| `app_safety_register(me, monitor)` | 注册监控器 | `bsp_status_t` |
| `app_safety_notify_online(monitor, now_ms)` | 上报监控器在线 | — |
| `app_safety_process(me, now_ms)` | 刷新状态并更新输出门 | — |
| `app_safety_set_output_enabled(me, enabled)` | 设置操作者输出请求 | — |
| `app_safety_output_allowed(me)` | 查询整机输出门 | `bool` |
| `app_safety_get_state(monitor)` | 查询监控器状态 | `app_safety_state_t` |
| `app_safety_get_offline_time_ms(monitor)` | 查询失联时长 [ms] | `uint32_t` |
| `app_safety_all_required_online(me)` | 必需监控器全在线 | `bool` |

## 输出门逻辑

```
output_allowed = output_enabled
                 && (存在 required 监控器)
                 && (所有 required 监控器均 ONLINE)

required 监控器失联 -> output_enabled = false
                    -> output_allowed = false
                    -> module_motor_set_output_allowed(false)
```

## 使用示例

```c
app_safety_t safety;
app_safety_monitor_t remote_monitor;

app_safety_init(&safety, &watchdog);
app_safety_monitor_init(&remote_monitor, &(app_safety_monitor_config_t){
    .name = "remote",
    .timeout_ms = 100,
    .required = true,
});
app_safety_register(&safety, &remote_monitor);

void safety_task(uint32_t now_ms)
{
    app_safety_notify_online(&remote_monitor, now_ms); /* 有效帧到达 */
    app_safety_set_output_enabled(&safety, true);      /* 操作者请求 */
    app_safety_process(&safety, now_ms);               /* 周期调用 */

    if (app_safety_output_allowed(&safety)) {
        // 允许输出，正常执行控制逻辑
    }
}
```

## 注意事项

1. **上电默认关闭输出**：`app_safety_init()` 会立即调用 `module_motor_set_output_allowed(false)`，操作者确认后才能请求输出。
2. **必需监控器失联撤销操作者请求**：`required` 对象失联会关闭全局电机输出门，并把 `output_enabled` 置 `false`；恢复在线后需重新 `app_safety_set_output_enabled(true)`。
3. **心跳上报时机**：`app_safety_notify_online()` 应在收到有效帧时调用，`app_safety_process()` 按固定周期（如 `APP_SAFETY_DEFAULT_TASK_PERIOD_MS`）刷新判定。
4. **看门狗可选**：`watchdog` 可传 `NULL`；非空时 `app_safety_process()` 每次刷新看门狗，作为安全任务的最后兜底。
5. **无必需监控器时输出门不放开**：`app_safety_all_required_online()` 在没有 `required` 监控器时返回 `false`，因此 `output_allowed` 恒为 `false`。
