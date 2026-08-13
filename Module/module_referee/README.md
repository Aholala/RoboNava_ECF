# module_referee — RoboMaster 裁判系统协议

`module_referee` 是裁判系统串口协议的流式收发框架：帧同步、CRC8/CRC16 双重校验、分包/粘包处理、命令路由分发、在线超时检测、发送序号和运行统计。基于 `bsp_usart`，与具体 MCU 解耦。

**数据流向：** USART ISR（拷贝 + 重启 DMA）--> `module_referee_update()`（流解析 + 校验 + 路由）--> 命令处理器 / 数据仓库

## 文件组成

| 文件 | 说明 |
|------|------|
| `module_referee.h/.c` | 核心框架：流解析、路由、发送、生命周期 |
| `module_referee_crc.h/.c` | CRC 验证/追加（底层调用 `alg_crc`） |
| `module_referee_data.h/.c` | 强类型比赛/机器人/功率/射击数据仓库 |
| `module_referee_ui.h/.c` | 客户端图形编码、队列、批量发送 |

## 帧格式

```
SOF(0xA5) | DataLen(LE,2) | Seq(1) | CRC8(1) | CmdID(LE,2) | Payload | CRC16(LE,2)
```

- CRC8 校验帧头（初值 0xFF，poly 0x8C）；CRC16 校验整帧（初值 0xFFFF，poly 0x8408）。

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `module_referee_init(me, cfg)` | 初始化（缓冲区、路由、超时、模式） | 状态码 |
| `module_referee_start(me)` | 注册回调并启动接收 | 状态码 |
| `module_referee_stop(me)` | 停止接收 | 状态码 |
| `module_referee_update(me, dt_ms)` | 周期更新：解析、校验、路由、在线检测 | 状态码 |
| `module_referee_transmit(me, cmd, payload, n, mode)` | 发送帧 | 状态码 |
| `module_referee_is_online(me)` | 查询在线状态 | bool |
| `module_referee_get_statistics(me, &stats)` | 读取运行统计 | 状态码 |
| `module_referee_read_uint16_le(p)` | 小端读 uint16 | uint16_t |
| `module_referee_data_reset(me)` / `data_has_update(me, cmd)` / `data_clear_updates(me)` | 数据仓库生命周期 | — |

## 使用示例

```c
static module_referee_t ref;
static uint8_t rx[256], proc[256], stream[512], tx[256];

void referee_init(void) {
    module_referee_config_t cfg = {
        .usart = usart_ptr,
        .receive_buffer = rx,      .receive_capacity = sizeof(rx),
        .processing_buffer = proc, .processing_capacity = sizeof(proc),
        .stream_buffer = stream,   .stream_capacity = sizeof(stream),
        .transmit_buffer = tx,     .transmit_capacity = sizeof(tx),
        .receive_timeout_ms = 100, .offline_timeout_ms = 500,
        .receive_mode = BSP_TRANSFER_MODE_DMA,
    };
    module_referee_init(&ref, &cfg);
    module_referee_start(&ref);
}

void referee_task(uint32_t dt_ms) {
    module_referee_update(&ref, dt_ms);
    if (module_referee_is_online(&ref)) {
        // 裁判系统在线
    }
}
```

## 注意事项

1. **缓冲区约束**：`processing_capacity >= receive_capacity`，`stream_capacity >= 最大帧大小`。
2. **接收模式**：不支持阻塞，必须用 `INTERRUPT` 或 `DMA`。
3. **`payload` 仅在回调期间有效**：需长期保存的数据必须复制。
4. **不硬编码协议结构**：命令 ID、长度、字节序、字段缩放按当前赛季手册核对，不把官方 packed 结构直接强转到接收缓冲区。
5. **在线超时**：`offline_timeout_ms` 应大于 2 倍帧间隔，避免误判离线。
6. **功率/安全数据需同时检查在线**：读取前先看对应更新位，涉及安全和功率限制还要确认 `is_online()`。
