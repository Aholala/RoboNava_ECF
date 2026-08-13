# bsp_adc — ADC 通用抽象层

`bsp_adc` 屏蔽轮询、中断、DMA 三种读取模式的差异，向上提供统一的 ADC 数值获取接口。通过 `bsp_adc_t`（基类）操作，`bsp_adc_device_t`（派生类）持有平台驱动句柄。

**数据流向：** `bsp_adc_config_t` --> `init` --> `read_raw/read_normalized/read_voltage` / `start_dma` --> 调用者缓冲区

## 核心结构体

| 结构体 | 角色 | 关键字段 |
|--------|------|---------|
| `bsp_adc_t` | 基类（App 持有） | `callback`、`user_context`、`reference_voltage_v`、`maximum_raw_value` |
| `bsp_adc_device_t` | 派生设备 | `super`、`driver_ops`、`channel` |
| `bsp_adc_driver_ops_t` | 平台驱动表 | `start`/`stop`/`calibrate`/`read_raw`/`start_dma`/`stop_dma`/`init`/`deinit` |
| `bsp_adc_config_t` | 初始化配置 | `device_handle`、`driver_ops`、`channel`、`resolution_bits`、`reference_voltage_v` |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `bsp_adc_init(me, cfg)` | 初始化设备 | OK / INVALID_ARGUMENT |
| `bsp_adc_as_base(me)` | 向上转型 | `bsp_adc_t *` |
| `bsp_adc_set_callback(me, cb, ctx)` | 设置事件回调 | OK / NOT_INITIALIZED |
| `bsp_adc_calibrate(me)` | 硬件校准 | OK / 平台错误 |
| `bsp_adc_start(me)` / `bsp_adc_stop(me)` | 启动/停止转换 | 状态码 |
| `bsp_adc_read_raw(me, &raw, ms)` | 阻塞读原始值 | OK / TIMEOUT |
| `bsp_adc_read_normalized(me, &norm, ms)` | 读归一化值 (0~1) | 状态码 |
| `bsp_adc_read_voltage(me, &v, ms)` | 读电压值 | 状态码 |
| `bsp_adc_start_dma(me, buf, n)` / `bsp_adc_stop_dma(me)` | DMA 批量采样 | OK / UNSUPPORTED |
| `bsp_adc_notify(me, event, status, size)` | ISR 通知入口 | void |

## 使用示例

```c
static bsp_adc_device_t s_adc_dev;
static bsp_adc_t *s_adc_ptr;

void board_adc_init(void) {
    bsp_adc_config_t cfg = {
        .device_handle = &hadc1,
        .driver_ops = &stm32_adc_driver,   // 平台驱动表
        .channel = 5,
        .resolution_bits = 12,
        .reference_voltage_v = 3.3f,
    };
    bsp_adc_init(&s_adc_dev, &cfg);
    s_adc_ptr = bsp_adc_as_base(&s_adc_dev);
    bsp_adc_calibrate(s_adc_ptr);
    bsp_adc_start(s_adc_ptr);
}

void adc_task(void) {
    float voltage;
    if (bsp_adc_read_voltage(s_adc_ptr, &voltage, 100) == BSP_STATUS_OK) {
        // voltage 为 ADC 引脚电压，外部分压需在 Module 层换算
    }
}
```

## 注意事项

1. **驱动必填项**：`start`/`stop`/`calibrate`/`read_raw` 必须实现；`init`/`deinit` 可为 NULL，`start_dma`/`stop_dma` 不支持则置 NULL（返回 `BSP_STATUS_UNSUPPORTED`）。
2. **DMA 缓冲区所有权**：缓冲区由调用者持有，完成/停止前不可释放或复用；带 D-Cache 时需 Clean/Invalidate。
3. **电压换算**：`read_voltage` 是引脚电压，分压电阻换算与非线性校准应在 Module 层处理。
