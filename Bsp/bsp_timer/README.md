# bsp_timer — 通用定时器抽象层

`bsp_timer` 提供对基本硬件定时器的统一抽象：启动/停止、计数器读写、周期设置、时钟频率查询和周期到期通知。用于固定周期任务调度和通用计数。

**数据流向：** `bsp_timer_config_t` --> `init/set_period/start` --> ISR `notify_elapsed` --> 到期回调

## 核心结构体

| 结构体 | 角色 | 关键字段 |
|--------|------|---------|
| `bsp_timer_t` | 基类（App 持有） | `callback`、`user_context` |
| `bsp_timer_device_t` | 派生设备 | `super`、`driver_ops` |
| `bsp_timer_driver_ops_t` | 平台驱动表 | `start`/`stop`/`set_counter`/`get_counter`/`set_period`/`get_period`/`get_frequency` |
| `bsp_timer_config_t` | 初始化配置 | `device_handle`、`driver_ops`、`callback`、`user_context` |
| `bsp_timer_callback_t` | 到期回调 | `void(me, user_context)` |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `bsp_timer_init(me, cfg)` | 初始化定时器设备 | OK / INVALID_ARGUMENT |
| `bsp_timer_as_base(me)` | 向上转型 | 基类指针或 NULL |
| `bsp_timer_set_callback(me, cb, ctx)` | 设置到期回调 | OK / NOT_INITIALIZED |
| `bsp_timer_start(me)` / `bsp_timer_stop(me)` | 启动/停止定时器 | OK / NOT_INITIALIZED |
| `bsp_timer_reset(me)` | 复位计数器为 0 | OK / NOT_INITIALIZED |
| `bsp_timer_set_counter(me, ticks)` | 设置计数器值 | 状态码 |
| `bsp_timer_get_counter(me, &ticks)` | 获取计数器值 | 状态码 |
| `bsp_timer_set_period(me, ticks)` | 设置周期（逻辑 tick） | OK / OUT_OF_RANGE（0） |
| `bsp_timer_get_period(me, &ticks)` | 获取周期值 | 状态码 |
| `bsp_timer_get_frequency(me, &hz)` | 获取时钟频率 | 状态码 |
| `bsp_timer_notify_elapsed(me)` | 到期通知（ISR 调用） | void |

## 使用示例

```c
static bsp_timer_device_t s_timer;
static bsp_timer_t *s_timer_ptr;

void board_timer_init(void) {
    bsp_timer_config_t cfg = {
        .device_handle = &htim3,
        .driver_ops = &stm32_timer_driver,
        .callback = control_tick_callback,
        .user_context = NULL,
    };
    bsp_timer_init(&s_timer, &cfg);
    s_timer_ptr = bsp_timer_as_base(&s_timer);
    bsp_timer_set_period(s_timer_ptr, 1000);   // 逻辑周期 1000 tick
    bsp_timer_start(s_timer_ptr);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim3) bsp_timer_notify_elapsed(s_timer_ptr);
}

void control_tick_callback(bsp_timer_t *me, void *ctx) {
    // ISR 上下文：只置标志、累加计数、释放信号量
}
```

## 注意事项

1. **`period_ticks` 是逻辑周期**：`周期(s) = period_ticks / frequency_hz`；平台驱动负责逻辑值到硬件 ARR（`ARR = period - 1`）的转换。
2. **回调在 ISR 上下文**：只允许置标志、累加计数、释放信号量，禁止阻塞与长循环。
3. **时钟树与预分频由平台配置**：本层不管理，只通过 `get_frequency` 查询。
4. **用途边界**：PWM 输出用 `bsp_pwm`，正交编码器用 `bsp_encoder`，微秒短延时用 `bsp_dwt`。
