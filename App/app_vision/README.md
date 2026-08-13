# app_vision

`app_vision` 通过 USB VCP 收发固定 12 字节视觉帧。它是普通 `app_vision_t` 实例，不依赖 RTOS 或 `app_exchange`。

## 初始化

```c
app_vision_t vision;
app_vision_config_t config = {
    .usb_vcp = &usb_vcp,
    .target_timeout_ms = 100,
    .transmit_period_ms = 10,
};
app_vision_init(&vision, &config);
```

## Task 调用

```c
app_vision_set_mode(
    &vision,
    command->automatic_vision_requested
        ? APP_VISION_MODE_AUTOMATIC
        : APP_VISION_MODE_MANUAL);

app_vision_update(&vision, app_imu_get_snapshot(&imu), elapsed_ms);
const app_vision_target_t *target = app_vision_get_target(&vision);
```

IMU 参数允许为 `NULL`：仍会接收和更新视觉目标，只跳过本周期姿态发送。

## 12 字节协议

| 偏移 | 长度 | 内容 |
|---:|---:|---|
| 0 | 1 | `0xA5` |
| 1 | 1 | `0x5A` |
| 2 | 1 | `0` 手动，`1` 自动/跟踪 |
| 3 | 4 | pitch，little-endian float32，rad |
| 7 | 4 | yaw，little-endian float32，rad |
| 11 | 1 | 字节 0–10 的 CRC8，poly `0x31`，init `0xFF` |

接收方向中 pitch/yaw 表示视觉目标；发送方向中表示 IMU 姿态。目标超过 `target_timeout_ms` 没有收到有效帧后，`target_valid` 和 `tracking_ready` 自动清零。

## 推荐控制周期

```c
app_imu_update(&imu, dt_s);
app_vision_update(&vision, app_imu_get_snapshot(&imu), elapsed_ms);

app_command_update(&command_app, &remote_input,
                   app_gimbal_get_feedback(&gimbal),
                   app_vision_get_target(&vision), dt_s);
```

多任务工程如果需要跨任务传递目标，在项目层复制 `app_vision_target_t`；框架不强制使用某种队列或锁。
