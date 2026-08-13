# app_command -- 通用遥控输入到机器人命令

`app_command` 不直接读取 DR16，也不读写板间通信。项目适配层把本地 DR16、远端 CAN 或其他输入转换成 `app_remote_input_t`，本模块再生成底盘、云台和发射机指令。

## 遥控输入

`channel[4]` 和 `dial` 使用原始值减中值后的 `int16_t`；DR16 典型范围约为 `[-660, 660]`。不需要预先归一化成小数，`app_command` 只在计算 SI 目标时临时除以 `channel_maximum_offset`。超出范围的值会被限制。

```c
const module_dr16_process_data_t *dr16 = module_dr16_get_data(&remote);
app_remote_input_t input = {0};

if (dr16 != NULL && dr16->is_online) {
    for (size_t i = 0; i < 4; ++i) input.channel[i] = dr16->channel[i];
    input.left_switch = (app_switch_t)dr16->left_switch;
    input.right_switch = (app_switch_t)dr16->right_switch;
    input.mouse_x = dr16->mouse_x;
    input.mouse_y = dr16->mouse_y;
    input.mouse_z = dr16->mouse_z;
    input.mouse_left_pressed = dr16->mouse_left_pressed;
    input.mouse_right_pressed = dr16->mouse_right_pressed;
    input.keyboard = dr16->keyboard;
    input.dial = dr16->dial;
    input.sequence = dr16->valid_frame_count;
    input.online = true;
}

app_command_update(&input, dt_s);
```

如果遥控数据来自 `module_board_comm`，转换工作同样放在具体项目中；ECF App 不需要知道当前是单板还是多板。

## 配置与映射

```c
app_command_init(&(app_command_config_t){
    .channel_maximum_offset = 660,
    .maximum_yaw_rate_rad_per_s = 3.14F,
    .maximum_pitch_rate_rad_per_s = 2.09F,
    .minimum_pitch_rad = -0.52F,
    .maximum_pitch_rad = 0.35F,
    .maximum_chassis_velocity_m_per_s = 3.0F,
    .maximum_chassis_spin_rad_per_s = 6.28F,
});
```

- `channel[0]`：云台 yaw 速率。
- `channel[1]`：云台 pitch 速率。
- `channel[2]`：底盘 Y 速度。
- `channel[3]`：底盘 X 速度。
- 左开关 DOWN/UP：无力/小陀螺；右开关 DOWN：跟随云台。
- 拨轮 `>100`：启动摩擦轮；`>500`：请求发射。这些阈值与去中值后的整数同单位。

`remote == NULL` 或 `remote->online == false` 时，本模块发布禁用指令。当前仍使用 `app_exchange` 发布命令，该边界将在后续阶段单独处理。
