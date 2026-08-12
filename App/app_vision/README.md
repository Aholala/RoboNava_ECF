# app_vision -- 视觉通信模块（USB VCP 与外部视觉系统通信）

## 功能概述

视觉通信模块通过 USB 虚拟串口（VCP）与外部视觉处理系统（如 miniPC 运行 OpenCV）交换数据。模块以固定的 12 字节帧协议接收视觉跟踪目标（目标俯仰角、偏航角），同时按配置周期回传 IMU 姿态数据给视觉系统。支持手动和自动两种工作模式，自动模式下视觉目标数据用于辅助瞄准。

**数据流向：**
- **接收方向：** USB VCP --> `app_vision`（帧校验、解析） --> `app_exchange`（vision_target） --> `app_command`（视觉辅助超控）
- **发送方向：** `app_exchange`（imu_snapshot） --> `app_vision`（组帧、CRC8） --> USB VCP --> 外部视觉系统

## 帧协议

### 接收帧（视觉系统 --> MCU，12 字节）

| 字节偏移 | 长度 | 字段 | 说明 |
|---------|------|------|------|
| 0 | 1 | 帧头1 | 固定 `0xA5` |
| 1 | 1 | 帧头2 | 固定 `0x5A` |
| 2 | 1 | 模式 | `0x00` = 手动（无跟踪），`0x01` = 自动（跟踪中） |
| 3-6 | 4 | pitch | 目标俯仰角 (float32, little-endian) [rad] |
| 7-10 | 4 | yaw | 目标偏航角 (float32, little-endian) [rad] |
| 11 | 1 | CRC8 | 字节 0-10 的 CRC8 校验值（多项式 0x31，初始值 0xFF） |

### 发送帧（MCU --> 视觉系统，12 字节）

| 字节偏移 | 长度 | 字段 | 说明 |
|---------|------|------|------|
| 0 | 1 | 帧头1 | 固定 `0xA5` |
| 1 | 1 | 帧头2 | 固定 `0x5A` |
| 2 | 1 | 模式 | 当前工作模式（手动/自动） |
| 3-6 | 4 | pitch | IMU 俯仰角 (float32, little-endian) [rad] |
| 7-10 | 4 | yaw | IMU 偏航角 (float32, little-endian) [rad] |
| 11 | 1 | CRC8 | 字节 0-10 的 CRC8 校验值 |

## 核心结构体

### 枚举 `app_vision_mode_t`

| 值 | 说明 |
|------|------|
| `APP_VISION_MODE_MANUAL` | 视觉系统未跟踪目标 |
| `APP_VISION_MODE_AUTOMATIC` | 视觉系统正在跟踪目标 |

### 配置结构体 `app_vision_config_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `usb_vcp` | `bsp_usb_vcp_t *` | USB VCP 实例 |
| `target_timeout_ms` | `uint32_t` | 目标数据有效超时 [ms]，超过此时间未收到新帧则 `target_valid` 置 `false` |
| `transmit_period_ms` | `uint32_t` | IMU 数据回传周期 [ms] |

### 交换数据类型 `app_vision_target_t`（定义在 `app_types.h`）

| 字段 | 类型 | 说明 |
|------|------|------|
| `target_yaw_rad` | `float` | 视觉系统报告的偏航角 [rad] |
| `target_pitch_rad` | `float` | 视觉系统报告的俯仰角 [rad] |
| `update_count` | `uint32_t` | 累计有效目标更新次数 |
| `target_valid` | `bool` | 当前目标数据有效（未超时） |
| `tracking_ready` | `bool` | 视觉系统报告已锁定跟踪 |

### 帧协议常量

| 宏 | 值 | 说明 |
|------|------|------|
| `APP_VISION_FRAME_SIZE` | `12` | 帧固定总长度 [字节] |
| `APP_VISION_HEADER_FIRST` | `0xA5` | 帧头第一字节 |
| `APP_VISION_HEADER_SECOND` | `0x5A` | 帧头第二字节 |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `app_vision_init(config)` | 初始化视觉模块单例，校验配置 | `BSP_STATUS_OK` / `BSP_STATUS_INVALID_ARGUMENT` |
| `app_vision_set_mode(mode)` | 设置回传给视觉系统的工作模式 | `void` |
| `app_vision_update(elapsed_time_ms)` | 执行一个视觉处理周期：接收+解析目标帧 -> 管理目标超时 -> 按周期回传 IMU 数据 | `void` |

## 数据处理流程

```
app_vision_update(elapsed_time_ms)
  |
  +-- 累积目标超时计时器和发送周期计时器
  |
  +-- 尝试从 USB VCP 接收 12 字节帧
  |     |
  |     +-- 帧头校验 (0xA5 0x5A)
  |     +-- 模式域校验 (0x00 或 0x01)
  |     +-- CRC8 校验
  |     |
  |     +-- 全部通过 --> 解析 pitch/yaw --> 更新 vision_target
  |     +-- 任一失败 --> 丢弃该帧
  |
  +-- 目标超时检查
  |     elapsed > target_timeout_ms？
  |       是 --> target_valid = false, tracking_ready = false
  |
  +-- 发布 vision_target 到交换层
  |
  +-- 发送周期检查
        elapsed >= transmit_period_ms 且 USB 空闲？
         是 --> 读取 IMU 快照 --> 组发送帧 --> CRC8 --> bsp_usb_vcp_transmit()
```

## 使用示例

```c
#include "app_vision.h"
#include "app_exchange.h"
#include "bsp_usb_vcp.h"

/* --- 初始化阶段 --- */

bsp_usb_vcp_t vcp;
// 假设 USB VCP 已初始化（USB CDC 枚举完成）...

app_vision_config_t config = {
    .usb_vcp             = &vcp,
    .target_timeout_ms   = 100,    // 100ms 无新帧则目标失效
    .transmit_period_ms  = 10,     // 每 10ms 回传一次 IMU 数据
};

bsp_status_t rc = app_vision_init(&config);
if (rc != BSP_STATUS_OK) {
    // 错误处理：检查 USB VCP 是否为空
}

/* --- 周期性任务中（如 1kHz） --- */

void vision_task(uint32_t elapsed_ms)
{
    app_vision_update(elapsed_ms);

    // 可选：外部读取视觉目标用于日志
    app_vision_target_t target;
    app_exchange_read_vision_target(&target);
    if (target.target_valid) {
        // target.target_yaw_rad, target.target_pitch_rad 可用
    }
}
```

## 外部视觉系统对接示例（伪代码）

```python
# 视觉系统（如 miniPC 上的 Python 程序）对接示例
import serial
import struct

SERIAL_PORT = '/dev/ttyACM0'
HEADER = b'\xA5\x5A'

def crc8(data):
    """CRC8 多项式 0x31, 初始值 0xFF"""
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) if (crc & 0x80) else (crc << 1)
    return crc & 0xFF

# 发送目标帧
def send_target(ser, pitch_rad, yaw_rad, tracking):
    frame = bytearray(12)
    frame[0:2] = HEADER
    frame[2] = 0x01 if tracking else 0x00
    struct.pack_into('<f', frame, 3, pitch_rad)
    struct.pack_into('<f', frame, 7, yaw_rad)
    frame[11] = crc8(frame[:11])
    ser.write(frame)

# 接收 IMU 数据
def receive_imu(ser):
    data = ser.read(12)
    if data[0:2] != HEADER:
        return None
    if crc8(data[:11]) != data[11]:
        return None
    pitch = struct.unpack_from('<f', data, 3)[0]
    yaw = struct.unpack_from('<f', data, 7)[0]
    return pitch, yaw
```

## 注意事项

1. **单例模式**：视觉模块为全局单例，仅支持一路 USB VCP 连接。
2. **USB VCP 必须已初始化**：`app_vision_init` 要求 `config->usb_vcp` 不为 NULL，且 USB CDC 已在 BSP 层枚举完成。未就绪时初始化返回错误。
3. **目标超时保护**：视觉系统死机或 USB 断开后，目标数据在 `target_timeout_ms` 内自动失效。消费方（`app_command`）通过检查 `vision_target.target_valid` 决定是否使用视觉数据。
4. **CRC8 算法必须匹配**：外部视觉系统必须使用相同的 CRC8 参数（多项式 0x31，初始值 0xFF，不取反输出），否则所有帧校验失败。
5. **float 字节序**：帧内浮点数为 little-endian（与 ARM Cortex-M 一致）。如果视觉系统运行在 x86（也是 little-endian），无需额外转换。
6. **发送反压**：发送前检查 `bsp_usb_vcp_get_busy`，USB 忙时不发送（跳过该周期），避免阻塞或丢帧。
7. **`elapsed_time_ms` 累加保护**：模块内置了 `UINT32_MAX` 溢出检查，长时间运行不会因计时器溢出而异常。
8. **工作模式影响视觉系统行为**：回传给视觉系统的 `mode` 字段告诉视觉系统当前 MCU 期望的工作状态。视觉系统可能根据此字段决定是否启用跟踪算法以节省计算资源。
