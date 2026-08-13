# app_vision -- USB VCP 视觉目标通信

## 功能概述

`app_vision` 通过 USB VCP 收发固定 12 字节视觉帧：接收方向解析视觉目标，发送方向周期回传 IMU 姿态。它是普通 `app_vision_t` 实例，不依赖 RTOS 或 `app_exchange`。

**数据流向：** 视觉端 <-- `bsp_usb_vcp` --> `app_vision` --> `app_vision_get_target()`

## 核心结构体

### 配置结构体 `app_vision_config_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `usb_vcp` | `bsp_usb_vcp_t *` | USB VCP 实例 |
| `target_timeout_ms` | `uint32_t` | 目标超时时间 [ms]，超时后目标置为无效 |
| `transmit_period_ms` | `uint32_t` | 姿态发送周期 [ms] |

### 运行时实例 `app_vision_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `config` | `app_vision_config_t` | 静态配置的副本 |
| `target` | `app_vision_target_t` | 最近一次视觉目标 |
| `receive_frame` / `transmit_frame` | `uint8_t[12]` | 收发帧缓冲区 |
| `target_elapsed_ms` | `uint32_t` | 距上次有效目标的经过时间 [ms] |
| `transmit_elapsed_ms` | `uint32_t` | 距上次发送姿态的经过时间 [ms] |
| `mode` | `app_vision_mode_t` | 当前工作模式 |
| `initialized` | `bool` | 初始化阶段已成功完成 |

### 工作模式枚举 `app_vision_mode_t`

| 枚举值 | 说明 |
|--------|------|
| `APP_VISION_MODE_MANUAL` | 手动模式 |
| `APP_VISION_MODE_AUTOMATIC` | 自动/跟踪模式 |

### 交换数据类型 `app_vision_target_t`（定义在 `app_types.h`）

| 字段 | 类型 | 说明 |
|------|------|------|
| `target_yaw_rad` | `float` | 视觉系统报告的偏航角 [rad] |
| `target_pitch_rad` | `float` | 视觉系统报告的俯仰角 [rad] |
| `update_count` | `uint32_t` | 累计有效目标更新次数 |
| `target_valid` | `bool` | 当前目标数据有效（未超时） |
| `tracking_ready` | `bool` | 视觉系统报告已锁定跟踪 |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `app_vision_init(me, config)` | 初始化视觉实例 | `BSP_STATUS_OK` / `BSP_STATUS_INVALID_ARGUMENT` |
| `app_vision_set_mode(me, mode)` | 设置工作模式 | `bsp_status_t` |
| `app_vision_update(me, imu, elapsed_ms)` | 收发一个通信周期 | `bsp_status_t` |
| `app_vision_get_target(me)` | 读取最近视觉目标 | 只读指针或 `NULL` |

## 12 字节协议

| 偏移 | 长度 | 内容 |
|---:|---:|---|
| 0 | 1 | `0xA5` |
| 1 | 1 | `0x5A` |
| 2 | 1 | `0` 手动，`1` 自动/跟踪 |
| 3 | 4 | pitch，little-endian float32，rad |
| 7 | 4 | yaw，little-endian float32，rad |
| 11 | 1 | 字节 0–10 的 CRC8，poly `0x31`，init `0xFF` |

接收方向中 pitch/yaw 表示视觉目标；发送方向中表示 IMU 姿态。

## 帧处理流程

```
app_vision_update()
    |
    +-- 接收：读取并校验帧
    |    有效 -> 更新 target，复位 target_elapsed_ms
    |    无效 / 超时 -> target_valid = false
    |
    +-- 发送：未到周期 / imu 为 NULL / USB 忙 -> 跳过
    |    否则组装帧（mode + imu 姿态 + CRC8）并发送
```

## 使用示例

```c
app_vision_t vision;
app_vision_config_t config = {
    .usb_vcp = &usb_vcp,
    .target_timeout_ms = 100,
    .transmit_period_ms = 10,
};
app_vision_init(&vision, &config);

void vision_task(uint32_t elapsed_ms)
{
    app_vision_set_mode(
        &vision,
        command->automatic_vision_requested
            ? APP_VISION_MODE_AUTOMATIC
            : APP_VISION_MODE_MANUAL);

    app_vision_update(&vision, app_imu_get_snapshot(&imu), elapsed_ms);
    const app_vision_target_t *target = app_vision_get_target(&vision);
}
```

## 推荐控制周期

```c
app_imu_update(&imu, dt_s);
app_vision_update(&vision, app_imu_get_snapshot(&imu), elapsed_ms);

app_command_update(&command_app, &remote_input,
                   app_gimbal_get_feedback(&gimbal),
                   app_vision_get_target(&vision), dt_s);
```

## 注意事项

1. **IMU 参数允许为 `NULL`**：仍会接收和更新视觉目标，只跳过本周期姿态发送。
2. **目标自动超时清除**：超过 `target_timeout_ms` 没有收到有效帧后，`target_valid` 和 `tracking_ready` 自动清零。
3. **小端浮点**：pitch/yaw 使用 little-endian float32，需保证两端字节序一致。
4. **USB 忙时跳过发送**：发送前检查 `bsp_usb_vcp_get_busy()`，忙或未到周期则跳过本帧，不阻塞调用。
5. **多任务传递目标**：跨任务传递时在项目层复制 `app_vision_target_t`；框架不强制使用某种队列或锁。
