# module_oled — I2C 单色 OLED 显示屏驱动

基于 I2C 的页式 OLED 帧缓冲驱动，适配 SSD1306 等常见控制器。提供像素、直线、矩形、位图绘制及整帧刷新。

## 功能概述

- 页式帧缓冲（调用者静态分配），最大 128x64 像素
- 像素级绘图：点、直线（Bresenham）、矩形（空心/填充）、位图
- 整帧刷新到屏幕，支持对比度调节
- 所有 I2C 通信均通过 `bsp_i2c_t` 抽象层

## 核心结构体

### 状态码枚举 `module_oled_status_t`

| 枚举值 | 含义 |
|--------|------|
| `MODULE_OLED_STATUS_OK` | 操作成功 |
| `MODULE_OLED_STATUS_INVALID_ARGUMENT` | 参数非法 |
| `MODULE_OLED_STATUS_NOT_INITIALIZED` | 对象未初始化 |
| `MODULE_OLED_STATUS_NOT_STARTED` | 未启动 |
| `MODULE_OLED_STATUS_TRANSPORT_ERROR` | I2C 传输错误 |

### 配置结构体 `module_oled_config_t`

| 成员 | 类型 | 说明 |
|------|------|------|
| `i2c` | `bsp_i2c_t *` | I2C BSP 基类，必须已初始化 |
| `address_7bit` | `uint16_t` | 7 位 I2C 地址（0~0x7F） |
| `width_pixels` | `uint16_t` | 屏幕宽度（像素），最大 128 |
| `height_pixels` | `uint16_t` | 屏幕高度（像素），最大 64，须为 8 的倍数 |
| `frame_buffer` | `uint8_t *` | 帧缓冲区（调用者静态分配） |
| `frame_buffer_size` | `size_t` | 缓冲区大小，须 >= width x height / 8 |
| `timeout_ms` | `uint32_t` | I2C 传输超时（毫秒） |

### 设备对象 `module_oled_t`

| 成员 | 类型 | 说明 |
|------|------|------|
| `is_initialized` | `bool` | 是否已初始化 |
| `i2c` | `bsp_i2c_t *` | I2C BSP 基类 |
| `address_7bit` | `uint16_t` | I2C 地址 |
| `width_pixels` | `uint16_t` | 屏幕宽度 |
| `height_pixels` | `uint16_t` | 屏幕高度 |
| `frame_buffer` | `uint8_t *` | 帧缓冲区指针 |
| `frame_buffer_size` | `size_t` | 帧缓冲大小（字节） |
| `timeout_ms` | `uint32_t` | I2C 超时 |
| `is_started` | `bool` | 是否已启动 |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `module_oled_init(me, config)` | 初始化设备，校验参数，清零帧缓冲 | `module_oled_status_t` |
| `module_oled_start(me)` | 启动 OLED，发送 SSD1306 初始化序列并刷新 | `module_oled_status_t` |
| `module_oled_stop(me)` | 停止 OLED，发送显示关闭命令 | `module_oled_status_t` |
| `module_oled_clear(me, is_on)` | 清屏，`is_on=true` 全亮，`false` 全灭 | `module_oled_status_t` |
| `module_oled_set_pixel(me, x, y, is_on)` | 设置单个像素 | `module_oled_status_t` |
| `module_oled_draw_line(me, x1, y1, x2, y2, is_on)` | 绘制直线（Bresenham 算法） | `module_oled_status_t` |
| `module_oled_draw_rectangle(me, x, y, w, h, filled, is_on)` | 绘制矩形（空心或填充） | `module_oled_status_t` |
| `module_oled_draw_bitmap(me, x, y, w, h, bitmap, size, is_on)` | 绘制位图，超出屏幕部分自动裁剪 | `module_oled_status_t` |
| `module_oled_flush(me)` | 将帧缓冲逐页发送到屏幕（I2C 整帧刷新） | `module_oled_status_t` |
| `module_oled_set_contrast(me, contrast)` | 设置对比度（0~255） | `module_oled_status_t` |

## 使用示例

```c
// 1. 分配帧缓冲
static uint8_t fb[128 * 64 / 8];
module_oled_t oled;

// 2. 初始化
module_oled_config_t config = {
    .i2c               = &i2c,
    .address_7bit      = 0x3C,
    .width_pixels      = 128,
    .height_pixels     = 64,
    .frame_buffer      = fb,
    .frame_buffer_size = sizeof(fb),
    .timeout_ms        = 100,
};
module_oled_init(&oled, &config);

// 3. 启动（发送 SSD1306 初始化序列）
module_oled_start(&oled);

// 4. 绘制内容
module_oled_clear(&oled, false);
module_oled_draw_line(&oled, 0, 0, 127, 63, true);
module_oled_draw_rectangle(&oled, 10, 10, 50, 30, false, true);
module_oled_set_pixel(&oled, 64, 32, true);

// 5. 刷新到屏幕
module_oled_flush(&oled);

// 6. 停止
module_oled_stop(&oled);
```

## 注意事项

- 帧缓冲由调用者静态分配，大小为 `width * height / 8` 字节，模块不管理其生命周期。
- `module_oled_start` 会发送满量 SSD1306 初始化序列（电荷泵、时钟分频、COM 引脚等），其中 COM 引脚配置自动根据 `height_pixels` 选择（64 行用 0x12，否则 0x02）。
- 所有绘图函数只修改帧缓冲，必须调用 `module_oled_flush` 才会实际更新屏幕。
- 该模块不包含字库或字符串绘制，字符显示需自行实现或通过位图方式绘制。
- `module_oled_draw_bitmap` 采用逐行高位在前（MSB first）的位图格式。
