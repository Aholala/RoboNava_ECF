# module_ws2812 — WS2812 灯带驱动

基于 SPI 编码波形的 WS2812 LED 灯带驱动模块。支持像素缓冲、全局亮度、异步刷新，以及闪烁、流水、呼吸、彩虹、剧院追逐五种内置灯光效果。

## 功能概述

- 通过 SPI 输出编码波形驱动 WS2812（无需专用 PWM/DMA 通道）
- 像素颜色缓冲（调用者分配），全局亮度软件缩放
- 支持阻塞/中断/DMA 三种 SPI 传输模式，异步发送时通过通知回调清除忙标志
- 内置效果引擎：`update()` 周期性推进状态机，效果自动调用 `show()` 刷新

## 核心结构体

### 状态码枚举 `module_ws2812_status_t`

| 枚举值 | 含义 |
|--------|------|
| `MODULE_WS2812_STATUS_OK` | 操作成功 |
| `MODULE_WS2812_STATUS_INVALID_ARGUMENT` | 参数非法 |
| `MODULE_WS2812_STATUS_NOT_INITIALIZED` | 对象未初始化 |
| `MODULE_WS2812_STATUS_NOT_STARTED` | 未启动 |
| `MODULE_WS2812_STATUS_BUSY` | SPI 传输忙 |
| `MODULE_WS2812_STATUS_TRANSPORT_ERROR` | SPI 传输错误 |

### 颜色结构体 `module_ws2812_color_t`

| 成员 | 类型 | 说明 |
|------|------|------|
| `red` | `uint8_t` | 红色分量（0~255） |
| `green` | `uint8_t` | 绿色分量（0~255） |
| `blue` | `uint8_t` | 蓝色分量（0~255） |

> 注意：结构体按 RGB 存储，但 SPI 编码时按 WS2812 要求的 GRB 顺序输出。

### 效果类型枚举 `module_ws2812_effect_t`

| 枚举值 | 含义 |
|--------|------|
| `MODULE_WS2812_EFFECT_NONE` | 无效果（手动控制） |
| `MODULE_WS2812_EFFECT_BLINK` | 闪烁（亮灭交替） |
| `MODULE_WS2812_EFFECT_COLOR_WIPE` | 流水（逐个点亮） |
| `MODULE_WS2812_EFFECT_BREATH` | 呼吸（亮度渐变） |
| `MODULE_WS2812_EFFECT_RAINBOW` | 彩虹（颜色循环） |
| `MODULE_WS2812_EFFECT_THEATER_CHASE` | 剧院追逐（间隔亮灯） |

### 效果状态结构体 `module_ws2812_effect_state_t`

| 成员 | 类型 | 说明 |
|------|------|------|
| `type` | `module_ws2812_effect_t` | 当前效果类型 |
| `color` | `module_ws2812_color_t` | 效果使用的主颜色 |
| `step_time_ms` | `uint32_t` | 每步时间（毫秒） |
| `elapsed_time_ms` | `uint32_t` | 已累积时间 |
| `led_index` | `size_t` | 流水效果当前 LED 索引 |
| `color_offset` | `uint16_t` | 彩虹效果颜色偏移 |
| `phase` | `uint8_t` | 剧院追逐/闪烁相位 |
| `brightness` | `uint8_t` | 呼吸效果当前亮度 |
| `brightness_direction` | `int8_t` | 呼吸方向（+1 变亮，-1 变暗） |
| `is_enabled` | `bool` | 效果是否启用 |

### 配置结构体 `module_ws2812_config_t`

| 成员 | 类型 | 说明 |
|------|------|------|
| `spi` | `bsp_spi_t *` | SPI BSP 基类，必须已初始化 |
| `pixels` | `module_ws2812_color_t *` | 像素颜色数组（调用者分配，长度 = led_count） |
| `led_count` | `size_t` | LED 数量 |
| `transmit_buffer` | `uint8_t *` | 编码发送缓冲区（调用者分配） |
| `transmit_buffer_size` | `size_t` | 发送缓冲区大小，须 >= `REQUIRED_BUFFER_SIZE(led_count, reset_byte_count)` |
| `reset_byte_count` | `size_t` | 复位低电平字节数（至少 1） |
| `transmit_timeout_ms` | `uint32_t` | SPI 传输超时（毫秒） |
| `transfer_mode` | `bsp_transfer_mode_t` | 传输模式（BLOCKING / INTERRUPT / DMA） |

### 设备对象 `module_ws2812_t`

| 成员 | 类型 | 说明 |
|------|------|------|
| `is_initialized` | `bool` | 是否已初始化 |
| `spi` | `bsp_spi_t *` | SPI BSP 基类 |
| `pixels` | `module_ws2812_color_t *` | 像素数组（引用外部） |
| `led_count` | `size_t` | LED 数量 |
| `transmit_buffer` | `uint8_t *` | 发送缓冲区（引用外部） |
| `transmit_buffer_size` | `size_t` | 实际发送缓冲区大小 |
| `reset_byte_count` | `size_t` | 复位字节数 |
| `transmit_timeout_ms` | `uint32_t` | SPI 超时 |
| `transfer_mode` | `bsp_transfer_mode_t` | 传输模式 |
| `brightness` | `uint8_t` | 全局亮度（0~255，默认 255） |
| `effect` | `module_ws2812_effect_state_t` | 效果状态机 |
| `is_busy` | `volatile bool` | SPI 发送忙标志（ISR/任务共享） |
| `is_started` | `bool` | 是否已启动 |

## 辅助宏

| 宏 | 说明 |
|----|------|
| `MODULE_WS2812_ENCODED_BYTES_PER_LED` | 每颗 LED 编码后占 9 字节 |
| `MODULE_WS2812_REQUIRED_BUFFER_SIZE(led_count, reset_byte_count)` | 计算发送缓冲区最小字节数 |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `module_ws2812_init(me, config)` | 初始化设备，清空像素和发送缓冲区 | `module_ws2812_status_t` |
| `module_ws2812_start(me)` | 启动灯带（允许 show 和效果运行） | `module_ws2812_status_t` |
| `module_ws2812_stop(me)` | 停止灯带，中止 SPI 传输并禁用效果 | `module_ws2812_status_t` |
| `module_ws2812_set_pixel(me, index, color)` | 设置单个 LED 颜色 | `module_ws2812_status_t` |
| `module_ws2812_fill(me, color)` | 将所有 LED 填充为同一颜色 | `module_ws2812_status_t` |
| `module_ws2812_clear(me)` | 熄灭所有 LED | `module_ws2812_status_t` |
| `module_ws2812_set_brightness(me, brightness)` | 设置全局亮度（软件缩放，0~255） | `module_ws2812_status_t` |
| `module_ws2812_show(me)` | 编码像素缓冲并通过 SPI 发送 | `module_ws2812_status_t` |
| `module_ws2812_update(me, elapsed_ms)` | 周期更新效果状态机（需在任务中循环调用） | `module_ws2812_status_t` |
| `module_ws2812_start_blink(me, color, half_period_ms)` | 启动闪烁效果 | `module_ws2812_status_t` |
| `module_ws2812_start_color_wipe(me, color, step_time_ms)` | 启动流水效果（逐个点亮，完成后自动停止） | `module_ws2812_status_t` |
| `module_ws2812_start_breath(me, color, step_time_ms)` | 启动呼吸效果 | `module_ws2812_status_t` |
| `module_ws2812_start_rainbow(me, step_time_ms)` | 启动彩虹效果 | `module_ws2812_status_t` |
| `module_ws2812_start_theater_chase(me, color, step_time_ms)` | 启动剧院追逐效果 | `module_ws2812_status_t` |
| `module_ws2812_stop_effect(me)` | 停止当前效果（恢复手动控制） | `module_ws2812_status_t` |
| `module_ws2812_notify_transmit_complete(me, status)` | SPI 传输完成回调（清除 busy，在 ISR 中调用） | `void` |
| `module_ws2812_is_busy(me)` | 查询是否正在发送 | `bool` |
| `module_ws2812_make_color(red, green, blue)` | 构造 RGB 颜色 | `module_ws2812_color_t` |
| `module_ws2812_color_wheel(position)` | 色环生成（0~255 映射到彩虹色） | `module_ws2812_color_t` |

## 使用示例

```c
// 1. 分配缓冲区
#define LED_NUM 8
static module_ws2812_color_t pixels[LED_NUM];
static uint8_t tx_buf[MODULE_WS2812_REQUIRED_BUFFER_SIZE(LED_NUM, 2)];

module_ws2812_t leds;

// 2. 初始化
module_ws2812_config_t config = {
    .spi                 = &spi,
    .pixels              = pixels,
    .led_count           = LED_NUM,
    .transmit_buffer     = tx_buf,
    .transmit_buffer_size = sizeof(tx_buf),
    .reset_byte_count    = 2,
    .transmit_timeout_ms = 100,
    .transfer_mode       = BSP_TRANSFER_MODE_BLOCKING,
};
module_ws2812_init(&leds, &config);
module_ws2812_start(&leds);

// 3.1 手动控制
module_ws2812_color_t red = module_ws2812_make_color(255, 0, 0);
module_ws2812_set_pixel(&leds, 0, red);
module_ws2812_set_pixel(&leds, 1, module_ws2812_make_color(0, 255, 0));
module_ws2812_show(&leds);

// 3.2 启动内置效果
module_ws2812_start_rainbow(&leds, 50);

// 4. 周期更新效果（放在定时任务中）
void task_led_update(void) {
    if (!module_ws2812_is_busy(&leds)) {
        module_ws2812_update(&leds, 10);  // 距上次调用 10ms
    }
}

// 5. SPI 中断回调中通知完成
void spi_tx_complete_callback(void) {
    module_ws2812_notify_transmit_complete(&leds, BSP_STATUS_OK);
}

// 6. 停止
module_ws2812_stop(&leds);
```

## 注意事项

- 像素数组和发送缓冲区均由调用者静态分配，模块不管理其生命周期。发送缓冲区大小必须通过 `MODULE_WS2812_REQUIRED_BUFFER_SIZE` 宏计算。
- 全局亮度（`brightness`）在 `module_ws2812_show` 内对每个像素按分量软件缩放，不影响 `pixels` 数组原值。
- 效果引擎通过 `module_ws2812_update` 推进，内部按 `step_time_ms` 计时，达到步长后执行一步并自动调用 `show`。流水效果完成后会自动停止（`is_enabled = false`）。
- 若使用中断/DMA 模式，`show()` 标记 `is_busy` 后立即返回，传输完成后须在 SPI 回调中调用 `module_ws2812_notify_transmit_complete` 清除忙标志。
- 模块依赖 `math.h`。
