# module_usb_comm — USB 虚拟串口通信协议

基于 USB CDC（Virtual COM Port）的双向固定帧协议，用于与上位机视觉系统通信。帧格式为 `[0xA5][0x5A][mode][id][extra_data][CRC8]`，ID 范围 1~7。

## 功能概述

- 固定帧协议，模式（1B）+ 目标 ID（1B）+ 可选扩展数据 + CRC8 校验
- 接收端状态机自动帧同步，同时校验 ID 范围和 CRC8
- 分别统计三类错误（CRC 错误、无效帧），便于调试
- 发送前检查 USB VCP 忙状态，忙时返回 `BUSY` 供上层重试

## 协议帧

| 字节偏移 | 内容 |
|----------|------|
| 0 | 0xA5（帧头首字节） |
| 1 | 0x5A（帧头次字节） |
| 2 | mode（模式字节） |
| 3 | id（目标 ID，范围 1~7） |
| 4 ~ N+3 | extra_data（`MODULE_USB_COMM_EXTRA_DATA_SIZE`，默认 0） |
| N+4 | CRC8（多项式 0x8C，初值 0xFF） |

完整帧大小 = `MODULE_USB_COMM_FRAME_SIZE` = `MODULE_USB_COMM_EXTRA_DATA_SIZE + 5`

## 核心结构体

### 状态码枚举 `module_usb_comm_status_t`

| 枚举值 | 含义 |
|--------|------|
| `MODULE_USB_COMM_STATUS_OK` | 操作成功 |
| `MODULE_USB_COMM_STATUS_INVALID_ARGUMENT` | 参数非法（含 ID 不在 1~7 范围） |
| `MODULE_USB_COMM_STATUS_NOT_INITIALIZED` | 对象未初始化 |
| `MODULE_USB_COMM_STATUS_TRANSPORT_ERROR` | USB 传输错误 |
| `MODULE_USB_COMM_STATUS_BUSY` | 发送忙（上一帧未发完） |
| `MODULE_USB_COMM_STATUS_INVALID_FRAME` | 无效帧（ID 不在 1~7 范围） |
| `MODULE_USB_COMM_STATUS_CHECKSUM_ERROR` | CRC8 校验失败 |
| `MODULE_USB_COMM_STATUS_NO_DATA` | 尚无有效数据 |

### 协议数据结构体 `module_usb_comm_data_t`

| 成员 | 类型 | 说明 |
|------|------|------|
| `mode` | `uint8_t` | 模式字节 |
| `id` | `uint8_t` | 目标 ID（范围 1~7） |
| `extra_data` | `uint8_t[EXTRA_DATA_SIZE]` | 扩展数据区（仅当 EXTRA_DATA_SIZE > 0 时存在） |

### 解析数据结构体 `module_usb_comm_process_data_t`

| 成员 | 类型 | 说明 |
|------|------|------|
| `data` | `module_usb_comm_data_t` | 解析出的协议数据 |
| `update_count` | `uint32_t` | 数据更新计数（每次成功接收递增，饱和上限） |
| `is_valid` | `bool` | 当前数据是否有效 |

### 配置结构体 `module_usb_comm_config_t`

| 成员 | 类型 | 说明 |
|------|------|------|
| `usb_vcp` | `bsp_usb_vcp_t *` | USB VCP BSP 基类，必须已初始化 |
| `transmit_timeout_ms` | `uint32_t` | 发送超时时间（毫秒） |

### 设备对象 `module_usb_comm_t`

| 成员 | 类型 | 说明 |
|------|------|------|
| `usb_vcp` | `bsp_usb_vcp_t *` | USB VCP BSP 基类 |
| `transmit_timeout_ms` | `uint32_t` | 发送超时 |
| `transmit_frame` | `uint8_t[FRAME_SIZE]` | 发送帧缓冲区（内部使用） |
| `stream_frame` | `uint8_t[FRAME_SIZE]` | 流式接收组装缓冲区 |
| `stream_size` | `size_t` | 流缓冲区中已累积字节数 |
| `received_data` | `module_usb_comm_process_data_t` | 最近一次成功接收的数据 |
| `valid_frame_count` | `uint32_t` | 有效帧接收总数（饱和计数） |
| `invalid_frame_count` | `uint32_t` | 无效帧计数（ID 非法等，饱和计数） |
| `checksum_error_count` | `uint32_t` | CRC 校验错误总数（饱和计数） |
| `is_initialized` | `bool` | 是否已初始化 |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `module_usb_comm_init(me, config)` | 初始化通信模块，清零所有状态 | `module_usb_comm_status_t` |
| `module_usb_comm_send(me, data)` | 编码完整帧（帧头 + mode + id + extra + CRC8）并发送 | `module_usb_comm_status_t` |
| `module_usb_comm_feed_data(me, bytes, size)` | 注入接收字节，状态机解析帧 | `module_usb_comm_status_t` |
| `module_usb_comm_get_data(me, process_data)` | 获取最近一次成功接收的帧数据 | `module_usb_comm_status_t` |
| `module_usb_comm_crc8(data, data_size)` | 计算 CRC8 校验值 | `uint8_t` |

## 使用示例

```c
// 编译时可配置扩展数据长度（默认 0）
// #define MODULE_USB_COMM_EXTRA_DATA_SIZE 4

module_usb_comm_t comm;
module_usb_comm_process_data_t rx_data;

// 1. 初始化
module_usb_comm_config_t config = {
    .usb_vcp             = board_config_get_usb_vcp(),
    .transmit_timeout_ms = 100,
};
module_usb_comm_init(&comm, &config);

// 2. 接收（在 USB 接收回调中注入字节）
void on_usb_rx(const uint8_t *bytes, size_t count) {
    module_usb_comm_feed_data(&comm, bytes, count);
}

// 3. 读取解析后的数据
void task_vision(void) {
    if (module_usb_comm_get_data(&comm, &rx_data) == MODULE_USB_COMM_STATUS_OK) {
        if (rx_data.is_valid) {
            uint8_t mode = rx_data.data.mode;
            uint8_t id   = rx_data.data.id;
            // 根据 mode/id 处理视觉指令
        }
    }
}

// 4. 发送数据到上位机
module_usb_comm_data_t tx_data = {
    .mode = 0x01,
    .id   = 1,
};
module_usb_comm_status_t status = module_usb_comm_send(&comm, &tx_data);
if (status == MODULE_USB_COMM_STATUS_BUSY) {
    // 对方忙，稍后重试
}
```

## 注意事项

- 扩展数据长度通过编译宏 `MODULE_USB_COMM_EXTRA_DATA_SIZE` 配置（默认 0），范围 0~250。超出范围触发编译错误。
- ID 字段必须在 1~7 范围内，发送时 `module_usb_comm_send` 会校验（`INVALID_ARGUMENT`），接收时 `module_usb_comm_feed_data` 也会校验（`INVALID_FRAME`）。
- `module_usb_comm_send` 为非阻塞设计：先查询 VCP 忙状态，忙则立即返回 `BUSY`，调用者需稍后重试；不忙时编码并发送。
- `module_usb_comm_feed_data` 同时校验 ID 和 CRC8，二者均通过才算有效帧。仅 ID 非法时递增 `invalid_frame_count`，仅 CRC 错误时递增 `checksum_error_count`。
- 所有计数器均为饱和计数（到达 UINT32_MAX 后不再递增），不会溢出回绕。
- CRC8 算法与 `module_uart_comm` 相同，多项式 0x8C，初值 0xFF。
