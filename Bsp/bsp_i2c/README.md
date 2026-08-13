# bsp_i2c — I2C 主机抽象层

`bsp_i2c` 提供对 I2C 主机接口的统一抽象：直接收发、8/16 位寄存器地址访问、设备就绪探测、中止和忙状态查询，支持阻塞/中断/DMA 三种传输模式。所有公共接口使用 **7 位设备地址**，调用方不预先左移。

**数据流向：** `bsp_i2c_config_t` --> `init` --> `transmit/receive/memory_read/memory_write` --> 调用者缓冲区

## 核心结构体

| 结构体 | 角色 | 关键字段 |
|--------|------|---------|
| `bsp_i2c_t` | 基类（App 持有） | `callback`、`user_context` |
| `bsp_i2c_device_t` | 派生设备 | `super`、`driver_ops` |
| `bsp_i2c_driver_ops_t` | 平台驱动表 | `transmit`/`receive`/`memory_write`/`memory_read`/`is_device_ready`/`abort`/`get_busy` |
| `bsp_i2c_config_t` | 初始化配置 | `device_handle`、`driver_ops`、`callback`、`user_context` |
| `bsp_i2c_memory_address_size_t` | 寄存器地址宽度 | 8_BIT / 16_BIT |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `bsp_i2c_init(me, cfg)` | 初始化 I2C 设备 | OK / INVALID_ARGUMENT |
| `bsp_i2c_as_base(me)` | 向上转型 | `bsp_i2c_t *` |
| `bsp_i2c_set_callback(me, cb, ctx)` | 设置事件回调 | OK / NOT_INITIALIZED |
| `bsp_i2c_transmit(me, addr, data, n, mode, ms)` | 发送（7 位地址） | OK / UNSUPPORTED / 平台错误 |
| `bsp_i2c_receive(me, addr, data, n, mode, ms)` | 接收（7 位地址） | OK / UNSUPPORTED / 平台错误 |
| `bsp_i2c_memory_write(me, addr, reg, size, data, n, mode, ms)` | 写寄存器（8/16 位地址） | 状态码 |
| `bsp_i2c_memory_read(me, addr, reg, size, data, n, mode, ms)` | 读寄存器（8/16 位地址） | 状态码 |
| `bsp_i2c_is_device_ready(me, addr, trials, ms)` | 设备就绪探测 | OK / TIMEOUT |
| `bsp_i2c_abort(me, addr)` | 中止当前事务 | OK / UNSUPPORTED |
| `bsp_i2c_get_busy(me, &busy)` | 查询总线忙状态 | OK / UNSUPPORTED |
| `bsp_i2c_notify(me, event, status, size)` | ISR 通知入口 | void |

## 使用示例

```c
static bsp_i2c_device_t s_i2c;
static bsp_i2c_t *s_i2c_ptr;

void board_i2c_init(void) {
    bsp_i2c_config_t cfg = {
        .device_handle = &hi2c1,
        .driver_ops = &stm32_i2c_driver,
    };
    bsp_i2c_init(&s_i2c, &cfg);
    s_i2c_ptr = bsp_i2c_as_base(&s_i2c);
}

void sensor_read(void) {
    uint8_t reg;
    bsp_i2c_memory_read(s_i2c_ptr, 0x68, 0x0F, BSP_I2C_MEMORY_ADDRESS_8_BIT,
                        &reg, 1, BSP_TRANSFER_MODE_BLOCKING, 10);
}
```

## 注意事项

1. **7 位地址**：调用方传 7 位地址（0x00~0x7F），平台驱动负责左移 1 位加 R/W 位。
2. **异步缓冲区所有权**：中断/DMA 模式下，用户缓冲区在传输完成/中止前必须保持有效。
3. **必须实现**：`transmit`、`receive`；其余可选，未实现返回 `BSP_STATUS_UNSUPPORTED`。
4. **总线是共享资源**：多任务共享同一 I2C 对象时需上层串行化（总线管理器或互斥锁）。
