# app_safety

`app_safety` 负责心跳超时、状态转换、硬件看门狗刷新和全局电机输出门控。它不读取 HAL 或 RTOS 时间，所有时间均由调用者以毫秒传入。

## 放行条件

电机输出仅在以下条件同时满足时放行：

1. 已调用 `app_safety_set_output_enabled(true)`；
2. 至少注册了一个 `required` 监控器；
3. 所有 `required` 监控器均为 `ONLINE`。

任意 required 监控器离线时，Motor 门控先关闭并撤销人工解锁，再执行离线回调。通信恢复后必须再次显式解锁；初始化和人工禁止也会立即关闭门控。

## 使用

```c
static app_safety_monitor_t remote_monitor;

void safety_init(void)
{
    const app_safety_monitor_config_t config = {
        .name = "remote",
        .timeout_ms = 100U,
        .required = true,
        .offline_callback = NULL,
        .online_callback = NULL,
        .user_context = NULL,
    };

    (void)app_safety_init(NULL); /* 无硬件看门狗时传 NULL */
    (void)app_safety_monitor_init(&remote_monitor, &config);
    (void)app_safety_register(&remote_monitor);
    app_safety_set_output_enabled(true);
}

void on_remote_frame(uint32_t now_ms)
{
    app_safety_notify_online(&remote_monitor, now_ms);
}

void safety_step(uint32_t now_ms)
{
    app_safety_process(now_ms);
}
```

## API

| API | 作用 |
|---|---|
| `app_safety_init(watchdog)` | 清空管理器并默认禁止输出 |
| `app_safety_monitor_init(me, config)` | 初始化一个监控器 |
| `app_safety_register(me)` | 注册监控器，最多 16 个 |
| `app_safety_notify_online(me, now_ms)` | 记录一次有效心跳 |
| `app_safety_process(now_ms)` | 计算超时、转换状态、更新门控、刷新硬件看门狗 |
| `app_safety_set_output_enabled(enabled)` | 操作员请求解锁或立即禁止输出 |
| `app_safety_output_allowed()` | 查询最终输出许可 |
| `app_safety_all_required_online()` | 查询 required 监控器是否全部在线 |

`uint32_t` 时间差使用无符号减法，正常支持毫秒计数器自然回绕。调用者应保证 `notify_online` 和 `process` 的并发访问符合目标平台的同步要求；完整移植示例见 [使用与移植指南](../../Docs/USAGE_AND_PORTING.md)。
