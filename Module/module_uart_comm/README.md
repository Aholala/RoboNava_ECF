# module_uart_comm — UART 固定帧协议

基于 USART 的固定帧长点对点通信协议。帧格式为 `[0xA5][0x5A][data x N][CRC8]`，接收端通过字节流状态机自动帧同步。

## 功能概述

- 固定帧长协议，数据长度通过 `MODULE_UART_COMM_DATA_SIZE` 宏配置（默认 8 字节）
- 接收端状态机自动搜索帧头 `0xA5/0x5A`，帧头不匹配时自动丢弃并重新同步
- CRC8 尾校验（多项式 0x8C，初值 0xFF）
- 统计有效帧计数和校验错误计数，便于监控链路质量

## 协议帧

| 字节偏移 | 内容 |
|----------|------|
| 0 | 0xA5（帧头首字节） |
| 1 | 0x5A（帧头次字节） |
| 2 ~ N+1 | 用户数据（N = `MODULE_UART_COMM_DATA_SIZE`，1~252） |
| N+2 | CRC8（多项式 0x8C，初值 0xFF） |

完整帧大小 = `MODULE_UART_COMM_FRAME_SIZE` = `MODULE_UART_COMM_DATA_SIZE + 3`

## 核心结构体

### 状态码枚举 `module_uart_comm_status_t`

| 枚举值 | 含义 |
|--------|------|
| `MODULE_UART_COMM_STATUS_OK` | 操作成功 |
| `MODULE_UART_COMM_STATUS_INVALID_ARGUMENT` | 参数非法 |
| `MODULE_UART_COMM_STATUS_NOT_INITIALIZED` | 对象未初始化 |
| `MODULE_UART_COMM_STATUS_TRANSPORT_ERROR` | 串口传输错误 |
| `MODULE_UART_COMM_STATUS_BUSY` | 发送忙（上一帧未发完） |
| `MODULE_UART_COMM_STATUS_INVALID_FRAME` | 无效帧（当前未使用） |
| `MODULE_UART_COMM_STATUS_CHECKSUM_ERROR` | CRC8 校验失败 |
| `MODULE_UART_COMM_STATUS_NO_DATA` | 尚无有效数据 |

### 解析数据结构体 `module_uart_comm_process_data_t`

| 成员 | 类型 | 说明 |
|------|------|------|
| `data` | `uint8_t[MODULE_UART_COMM_DATA_SIZE]` | 用户数据负载 |
| `update_count` | `uint32_t` | 数据更新计数（每次成功接收递增，饱和上限） |
| `is_valid` | `bool` | 当前数据是否有效 |

### 配置结构体 `module_uart_comm_config_t`

| 成员 | 类型 | 说明 |
|------|------|------|
| `usart` | `bsp_usart_t *` | USART BSP 基类，必须已初始化 |
| `transmit_mode` | `bsp_transfer_mode_t` | 发送传输模式（阻塞/中断/DMA） |
| `transmit_timeout_ms` | `uint32_t` | 发送超时时间（毫秒） |

### 设备对象 `module_uart_comm_t`

| 成员 | 类型 | 说明 |
|------|------|------|
| `usart` | `bsp_usart_t *` | USART BSP 基类 |
| `transmit_mode` | `bsp_transfer_mode_t` | 发送传输模式 |
| `transmit_timeout_ms` | `uint32_t` | 发送超时 |
| `transmit_frame` | `uint8_t[FRAME_SIZE]` | 发送帧缓冲区（内部使用） |
| `stream_frame` | `uint8_t[FRAME_SIZE]` | 流式接收组装缓冲区 |
| `stream_size` | `size_t` | 流缓冲区中已累积字节数 |
| `received_data` | `module_uart_comm_process_data_t` | 最近一次成功接收的数据 |
| `valid_frame_count` | `uint32_t` | 有效帧接收总数（饱和计数） |
| `checksum_error_count` | `uint32_t` | CRC 校验错误总数（饱和计数） |
| `is_initialized` | `bool` | 是否已初始化 |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `module_uart_comm_init(me, config)` | 初始化通信模块，清零所有状态 | `module_uart_comm_status_t` |
| `module_uart_comm_send(me, data, data_size)` | 构建帧（帧头 + 数据 + CRC8）并发送 | `module_uart_comm_status_t` |
| `module_uart_comm_feed_data(me, bytes, size)` | 注入接收字节，状态机解析帧 | `module_uart_comm_status_t` |
| `module_uart_comm_get_data(me, process_data)` | 获取最近一次成功接收的帧数据 | `module_uart_comm_status_t` |
| `module_uart_comm_crc8(data, data_size)` | 计算 CRC8 校验值 | `uint8_t` |

## 使用示例

```c
// 编译时可配置数据长度（默认 8）
// #define MODULE_UART_COMM_DATA_SIZE 16

module_uart_comm_t comm;
module_uart_comm_process_data_t rx_data;

// 1. 初始化
module_uart_comm_config_t config = {
    .usart               = &usart,
    .transmit_mode       = BSP_TRANSFER_MODE_BLOCKING,
    .transmit_timeout_ms = 100,
};
module_uart_comm_init(&comm, &config);

// 2. 接收（在 UART 接收回调或轮询任务中注入字节）
void on_uart_rx(const uint8_t *bytes, size_t count) {
    module_uart_comm_feed_data(&comm, bytes, count);
}

// 3. 读取解析后的数据
void task_process(void) {
    if (module_uart_comm_get_data(&comm, &rx_data) == MODULE_UART_COMM_STATUS_OK) {
        if (rx_data.is_valid) {
            // rx_data.data[0..N-1] 包含有效负载
            // rx_data.update_count 可用于判断是否有新数据
        }
    }
}

// 4. 发送数据
uint8_t payload[MODULE_UART_COMM_DATA_SIZE] = {0x01, 0x02, 0x03, 0x04,
                                                0x05, 0x06, 0x07, 0x08};
module_uart_comm_send(&comm, payload, sizeof(payload));
```

## 注意事项

- 数据长度通过编译宏 `MODULE_UART_COMM_DATA_SIZE` 配置（默认 8），有效范围 1~252。超出范围触发编译错误。
- `module_uart_comm_send` 的 `data_size` 参数必须等于 `MODULE_UART_COMM_DATA_SIZE`，否则返回 `INVALID_ARGUMENT`。
- `module_uart_comm_feed_data` 接收原始字节流，内部状态机自动搜索帧头并校验 CRC8。帧头不匹配时状态机重置，无累积错误风险。
- `valid_frame_count` 和 `checksum_error_count` 采用饱和计数（到达 UINT32_MAX 后不再递增），可用于诊断链路质量。
- CRC8 算法使用 `alg_crc` 库，多项式 0x8C，初值 0xFF。
