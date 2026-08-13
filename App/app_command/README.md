# app_command -- 遥控输入到底盘/云台/发射命令的转换

## 功能概述

`app_command` 把与设备无关的遥控输入转换成底盘、云台和发射命令。它不依赖 DR16、板间通信、视觉模块或任何全局交换层，所有输入通过参数显式传入。

**数据流向：** `remote + gimbal_feedback + vision_target` --> `app_command` --> `app_command_get_output()`

## 核心结构体

### 配置结构体 `app_command_config_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `channel_maximum_offset` | `int16_t` | 通道量程（减中值后的最大值），典型 660 |
| `maximum_yaw_rate_rad_per_s` | `float` | 偏航角速率上限 [rad/s] |
| `maximum_pitch_rate_rad_per_s` | `float` | 俯仰角速率上限 [rad/s] |
| `minimum_pitch_rad` | `float` | 俯仰角下限 [rad] |
| `maximum_pitch_rad` | `float` | 俯仰角上限 [rad] |
| `maximum_chassis_velocity_m_per_s` | `float` | 底盘平移速度上限 [m/s] |
| `maximum_chassis_spin_rad_per_s` | `float` | 底盘自旋角速率上限 [rad/s] |
| `friction_velocity_rad_per_s` | `float` | 摩擦轮目标转速 [rad/s] |

### 运行时实例 `app_command_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `config` | `app_command_config_t` | 静态配置的副本 |
| `output` | `app_command_output_t` | 最近一次发布的输出指令 |
| `yaw_target_rad` | `float` | 累积的偏航目标角 [rad] |
| `pitch_target_rad` | `float` | 累积的俯仰目标角 [rad] |
| `sequence` | `uint32_t` | 单调递增的帧序号 |
| `initialized` | `bool` | 初始化阶段已成功完成 |

### 输出结构体 `app_command_output_t`

| 字段 | 类型 | 说明 |
|------|------|------|
| `chassis` | `app_chassis_command_t` | 底盘运动指令 |
| `gimbal` | `app_gimbal_command_t` | 云台运动指令 |
| `shooter` | `app_shooter_command_t` | 射击器指令 |
| `automatic_vision_requested` | `bool` | 请求切换视觉自动跟踪模式 |

### 输入结构体 `app_remote_input_t`（定义在 `app_types.h`）

| 字段 | 类型 | 说明 |
|------|------|------|
| `channel[4]` | `int16_t` | 原始通道减中值后的整数，DR16 典型 `[-660, 660]` |
| `left_switch` / `right_switch` | `app_switch_t` | 左右三段拨杆 |
| `mouse_*` | `int16_t` / `bool` | 鼠标位移与按键 |
| `keyboard` | `uint16_t` | 键盘按键位图 |
| `dial` | `int16_t` | 拨轮原始值减中值后的整数 |
| `sequence` | `uint32_t` | 单调递增的帧序号 |
| `online` | `bool` | 遥控在线标志 |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `app_command_init(me, config)` | 初始化命令实例，校验配置 | `BSP_STATUS_OK` / `BSP_STATUS_INVALID_ARGUMENT` |
| `app_command_update(me, remote, gimbal_feedback, vision_target, dt_s)` | 执行一个命令转换周期 | `bsp_status_t` |
| `app_command_get_output(me)` | 读取最近输出指令 | 只读指针或 `NULL` |

## 拨杆与模式映射

```
左拨杆 DOWN  ----------------------> 底盘无力（NO_FORCE），enabled = false
左拨杆 UP    ----------------------> 底盘自旋（SPIN），最大角速率
左拨杆 MIDDLE + 右拨杆 DOWN -------> 底盘跟随云台（FOLLOW_GIMBAL）
左拨杆 MIDDLE + 右拨杆 其它 -------> 底盘正常（NORMAL）

右拨杆 MIDDLE ---------------------> 云台 IMU 反馈，否则编码器反馈
鼠标右键按下 + 视觉目标有效 --------> 视觉自动跟踪，覆盖云台目标并切换 IMU 反馈
```

## 使用示例

```c
app_command_t command_app;
app_command_config_t config = {
    .channel_maximum_offset = 660,
    .maximum_yaw_rate_rad_per_s = 3.14f,
    .maximum_pitch_rate_rad_per_s = 2.09f,
    .minimum_pitch_rad = -0.52f,
    .maximum_pitch_rad = 0.35f,
    .maximum_chassis_velocity_m_per_s = 3.0f,
    .maximum_chassis_spin_rad_per_s = 6.28f,
    .friction_velocity_rad_per_s = 500.0f,
};
app_command_init(&command_app, &config);

void command_task(float dt_s)
{
    app_command_update(&command_app, &remote_input,
                       app_gimbal_get_feedback(&gimbal),
                       app_vision_get_target(&vision), dt_s);

    const app_command_output_t *output = app_command_get_output(&command_app);
    app_chassis_update(&chassis, &output->chassis, dt_s);
    app_gimbal_update(&gimbal, &output->gimbal, &imu_snapshot, dt_s);
    app_shooter_update(&shooter, &output->shooter,
                       app_gimbal_get_feedback(&gimbal), dt_s);
}
```

## 注意事项

1. **通道与拨轮需先减中值**：DR16 适配层直接复制减去 1024 后的 `channel[]` 和 `dial` 整数，典型范围为 `[-660, 660]`。`channel_maximum_offset` 需与之一致。
2. **`remote` / `gimbal_feedback` / `vision_target` 允许传 `NULL`**：`remote == NULL` 或 `remote->online == false` 时视为失联，通道归零并发布无力命令；无云台反馈时 `gimbal_yaw_rad` 取 0。
3. **拨轮阈值决定射击**：`dial > 100` 使能摩擦轮，`dial > 500` 触发单发；拨轮原始值需减中值后才能正确比较。
4. **`automatic_vision_requested` 只是项目层请求**：它表示鼠标右键按下，项目自行决定是否切换到视觉模式并调用 `app_vision_set_mode()`。
5. **多任务传递输出**：多任务工程可把 `app_command_output_t` 复制到自己的 RTOS 队列或项目快照；单任务工程直接使用返回的只读快照即可。
