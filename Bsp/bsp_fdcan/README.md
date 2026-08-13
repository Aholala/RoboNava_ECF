# bsp_fdcan — CAN FD 抽象层与 Classic 适配器

`bsp_fdcan` 提供厂商无关的 CAN FD 通用抽象，并在同目录提供 Classic CAN 适配器，使 FDCAN 硬件能无缝兼容上层 Classic CAN 模块。它是可选扩展：需要与 F405 bxCAN 共用的协议应依赖 `bsp_can_t`，只有 CAN FD 长帧、BRS 和协议状态能力才依赖 `bsp_fdcan_t`。

**数据流向：** `bsp_fdcan_config_t` --> `init/start` --> `transmit/receive/get_protocol_status` --> 帧 / 状态

## 核心结构体

| 结构体 | 角色 | 关键字段 |
|--------|------|---------|
| `bsp_fdcan_frame_t` | CAN FD 帧 | `identifier`、`id_type`、`frame_type`、`format`、`data_length`、`data[64]` |
| `bsp_fdcan_protocol_status_t` | 协议状态 | `is_bus_off`、`is_error_passive`、`has_warning`、错误计数 |
| `bsp_fdcan_t` | 基类 | `super`、`callback`、`user_context` |
| `bsp_fdcan_device_t` | 派生设备 | `super`、`driver_ops` |
| `bsp_fdcan_driver_ops_t` | 平台驱动表 | `start`/`stop`/`configure_filter`/`transmit`/`receive`/`get_protocol_status`/`get_transmit_free_level` |
| `bsp_fdcan_classic_adapter_t` | Classic 适配器 | 组合 `bsp_fdcan_t *`，实现 `bsp_can_t` 接口 |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `bsp_fdcan_init(me, cfg)` | 初始化 FDCAN 设备 | OK / INVALID_ARGUMENT |
| `bsp_fdcan_as_base(me)` | 向上转型 | `bsp_fdcan_t *` |
| `bsp_fdcan_set_callback(me, cb, ctx)` | 设置事件回调 | OK / NOT_INITIALIZED |
| `bsp_fdcan_start(me)` / `bsp_fdcan_stop(me)` | 启动/停止总线 | 状态码 |
| `bsp_fdcan_configure_filter(me, filter)` | 配置硬件过滤器 | OK / INVALID_ARGUMENT |
| `bsp_fdcan_transmit(me, frame, ms)` | 发送帧（阻塞） | OK / OUT_OF_RANGE / 平台错误 |
| `bsp_fdcan_receive(me, fifo, frame)` | 从 FIFO 接收一帧 | OK / IO_ERROR |
| `bsp_fdcan_get_protocol_status(me, &st)` | 查询协议状态 | OK / UNSUPPORTED |
| `bsp_fdcan_get_transmit_free_level(me, &n)` | 查询发送邮箱余量 | OK / UNSUPPORTED |
| `bsp_fdcan_notify(me, event, status, size)` | ISR 通知入口 | void |
| `bsp_fdcan_classic_adapter_init(me, cfg)` | 初始化 Classic 适配器 | OK / INVALID_ARGUMENT |
| `bsp_fdcan_classic_adapter_as_can(me)` | 获取 `bsp_can_t *` | 指针或 NULL |

## 使用示例

```c
static bsp_fdcan_device_t s_fdcan;
static bsp_fdcan_t *s_fdcan_ptr;

void board_fdcan_init(void) {
    bsp_fdcan_config_t cfg = {
        .device_handle = &hfdcan1,
        .driver_ops = &platform_fdcan_driver_ops,
    };
    bsp_fdcan_init(&s_fdcan, &cfg);
    s_fdcan_ptr = bsp_fdcan_as_base(&s_fdcan);
    bsp_fdcan_start(s_fdcan_ptr);
}

// 旧模块需要 Classic CAN 接口时
static bsp_fdcan_classic_adapter_t s_adapter;
bsp_fdcan_classic_adapter_init(&s_adapter, &(bsp_fdcan_classic_adapter_config_t){
    .fdcan = s_fdcan_ptr,
});
bsp_can_t *can = bsp_fdcan_classic_adapter_as_can(&s_adapter);
```

## 注意事项

1. **帧格式三种**：Classic（≤8 字节）、FD 无 BRS、FD 带 BRS；`data_length` 只能是 0~8、12、16、20、24、32、48、64 之一。
2. **Classic 适配器只接受 Classic 语义**：`data_length > 8` 或 FD 帧返回 `BSP_STATUS_UNSUPPORTED`，不能借适配器发 64 字节 FD 帧。
3. **通用层只报告协议状态，不擅自复位硬件**：Bus-Off 恢复流程由上层健康管理器执行（禁止输出 → 记录 → stop → 重配 → 恢复 → 重新 start）。
4. **适配器不拥有 `bsp_fdcan_t`**：其生命周期由调用者管理，不可先销毁 FDCAN 对象。
