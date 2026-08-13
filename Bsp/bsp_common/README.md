# bsp_common — BSP 通用基础设施

`bsp_common` 是整个 BSP 抽象层的基石，为所有外设对象提供统一的状态码、事件模型、设备基类和容器宏。它不依赖任何硬件，仅提供纯软件层面的类型与工具。

## 核心类型

| 类型 | 作用 |
|------|------|
| `bsp_status_t` | 统一错误码（OK / INVALID_ARGUMENT / OUT_OF_RANGE / NOT_INITIALIZED / BUSY / TIMEOUT / IO_ERROR / NO_RESOURCE / UNSUPPORTED） |
| `bsp_transfer_mode_t` | 传输模式（阻塞 / 中断 / DMA） |
| `bsp_event_t` | 事件类型（发送/接收/传输完成、中止、错误） |
| `bsp_event_callback_t` | 回调原型 `void(event, status, transferred_size, user_context)` |
| `bsp_device_t` | 设备基类：`vptr`、`device_handle`、`object_magic`、`is_initialized` |
| `bsp_device_ops_t` | 基类虚表（仅 `deinit`） |
| `bsp_error_t` | 全局错误寄存器（code / source / detail / is_valid） |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `bsp_device_init(me, ops, handle)` | 初始化基类，绑定虚表和句柄 | OK / INVALID_ARGUMENT |
| `bsp_device_deinit(me)` | 反初始化，调用虚析构并清空字段 | 状态码 |
| `bsp_device_is_initialized(me)` | 综合检查魔数/状态/虚表/句柄 | bool |
| `bsp_device_get_handle(me)` | 返回设备句柄 | 指针或 NULL |
| `bsp_transfer_mode_is_valid(mode)` | 校验传输模式 | bool |
| `bsp_error_record(code, source, detail)` | 写入全局错误寄存器 | void |
| `bsp_error_read()` | 读取错误寄存器 | `const bsp_error_t *` |
| `bsp_status_to_string(code)` | 状态码转字符串 | `const char *` |

## 容器宏

派生类必须把 `bsp_device_t super` 作为**第一个成员**，用容器宏从基类指针找回派生对象：

```c
#define BSP_CONTAINER_OF(pointer, type, member) \
    ((type *)((uint8_t *)(pointer) - offsetof(type, member)))

// 虚函数实现中
static bsp_status_t bsp_uart_send(bsp_uart_t *base, ...) {
    bsp_uart_device_t *dev = BSP_CONTAINER_OF(base, bsp_uart_device_t, super);
    return dev->driver_ops->send(dev->super.device_handle, ...);
}
```

## 设计模式

| 模式 | 适用 | 结构 |
|------|------|------|
| 完整 OOP | CAN/USART/SPI/USB/WDT（有多态需求） | `bsp_device_t super` + `vptr` + `container_of` |
| 单例 dispatcher | GPIO/EXTI/PWM/DWT（永远单一实现） | `bind_platform()` + 全局 ops 指针 + 平铺结构体 |

## 注意事项

1. **零动态内存**：对象、虚表、缓冲区全部静态分配，禁止 `malloc`。
2. **初始化失败留确定状态**：`is_initialized` 保持 `false`，避免半初始化对象。
3. **回调非阻塞**：回调可能运行在 ISR 上下文，禁止延时、阻塞与长循环。
4. **`device_handle` 只用于平台适配器**：Module/App 不应把它强转为 STM32 HAL 类型。
