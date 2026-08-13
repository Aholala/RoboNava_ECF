# RoboNava ECF

> 面向 RoboMaster 的可移植 C11 控制框架。ECF 提供算法、硬件抽象、设备协议和可复用控制流程；具体机器人的 HAL 句柄、CAN ID、机械参数、任务和比赛策略由使用者工程负责。

ECF 不代表某一台完整机器人，也不绑定 STM32 型号或 FreeRTOS。建议将它作为只读子目录放入机器人项目，通过少量工程适配代码完成硬件连接和整机装配。

## 快速导航

- [框架与机器人项目的边界](#框架与机器人项目的边界)
- [整体架构](#整体架构)
- [目录与组件](#目录与组件)
- [快速开始](#快速开始)
- [典型控制周期](#典型控制周期)
- [关键设计约定](#关键设计约定)
- [文档索引](#文档索引)
- [验证状态](#验证状态)

## 框架与机器人项目的边界

推荐的新工程结构：

```text
your_robot/
├─ ECF/                       本仓库，跨项目复用
├─ Core/                      CubeMX、HAL 和启动文件
└─ User/
   ├─ robot_config.h/.c       实例、CAN ID、机械参数、PID、初始化
   ├─ robot_port.h/.c         STM32 HAL 到 ECF BSP 的驱动适配
   ├─ robot_adapter.h/.c      DR16/裁判数据转换和项目安全策略
   └─ robot_tasks.c           FreeRTOS 或裸机周期调度
```

移植新机器人时主要修改 `User/`，不修改 ECF：

| ECF 负责 | 机器人项目负责 |
| --- | --- |
| PID、滤波、EKF、底盘运动学 | 真实机械尺寸和控制参数 |
| CAN/UART/SPI 等通用 BSP 接口 | HAL 句柄、GPIO、DMA、IRQ 和 Cache |
| DJI/DM、BMI088、DR16、裁判协议 | CAN ID、设备实例和板级资源选择 |
| 底盘、云台、发射、IMU 等通用 App | 任务周期、优先级、跨任务同步和整机模式 |
| CRC、协议解析、在线检测 | 当赛季功率/热量规则和故障处置策略 |

ECF 不提供全局数据交换层。单任务直接传递指针；多任务使用目标 RTOS 的队列、任务通知或项目局部快照。

## 整体架构

```text
Robot Project   板级绑定、实例装配、参数、任务和整机策略
      │
      ▼
App             command / chassis / gimbal / shooter / imu / vision / safety
      │
      ▼
Module          电机、传感器、通信协议和功能状态机
      │
      ▼
BSP             CAN / SPI / USART / USB / PWM 等厂商无关接口
      │
      ▼
Platform        STM32 HAL 或其他平台驱动

Algorithm       由 App/Module 使用的纯算法，不依赖上述硬件层
```

依赖只能向下：

- `Algorithm` 不依赖 HAL、RTOS、BSP、Module 或 App。
- `BSP` 不保存某个工程的 HAL 全局句柄，通过 `driver_ops` 和平台句柄接入硬件。
- `Module` 负责设备协议和功能状态机，不决定机器人模式。
- `App` 负责可复用控制流程，不选择 CAN ID、HAL 外设或任务。
- 机器人项目负责把以上对象装配起来，并最终决定是否允许动力输出。

典型数据流：

```text
硬件接收
  → BSP 搬运数据
  → Module 校验协议并形成设备数据
  → App/Algorithm 解算和决策
  → Module 设置执行器目标
  → 电机总线统一更新并发送
```

## 目录与组件

### Algorithm

纯 C11 算法，使用 SI 单位和显式时间步长。

| 分类 | 组件 |
| --- | --- |
| 数学与信号处理 | `alg_math`、`alg_filter`、`alg_crc` |
| 状态与姿态估计 | `alg_kalman`、`alg_attitude`、`alg_imu_ekf` |
| 控制与规划 | `alg_pid`、`alg_lqr`、`alg_trajectory` |
| 底盘运动学 | `alg_chassis`、`alg_mecanum`、`alg_omni`、`alg_swerve` |

详见 [Algorithm 使用指南](Algorithm/README.md)。

### BSP

厂商无关的外设接口。机器人项目只需实现实际用到的 driver ops。

| 分类 | 组件 |
| --- | --- |
| 通信 | `bsp_can`、`bsp_fdcan`、`bsp_usart`、`bsp_spi`、`bsp_i2c`、`bsp_usb_vcp` |
| IO 与执行 | `bsp_gpio`、`bsp_exti`、`bsp_pwm`、`bsp_adc` |
| 时间与安全 | `bsp_timer`、`bsp_encoder`、`bsp_dwt`、`bsp_watchdog`、`bsp_log` |

详见 [BSP 移植说明](Bsp/README.md)。

### Module

设备协议、通信解析和可复用功能状态机。

| 分类 | 组件 |
| --- | --- |
| 电机与机构 | `module_motor`、`module_dji_motor`、`module_dm_motor`、`module_swerve`、`module_shooter` |
| 传感器与输入 | `module_bmi088`、`module_dr16` |
| 通信 | `module_referee`、`module_board_comm`、`module_uart_comm`、`module_usb_comm`、`module_nrf24l01` |
| 外围设备 | `module_servo`、`module_buzzer`、`module_oled`、`module_ws2812` |

DJI 使用一个 `module_dji_motor_t`，通过型号配置区分 M2006、M3508 和 GM6020；DM 使用独立 `module_dm_motor_t`，通过枚举选择 MIT、速度、位置速度或力位模式。两者协议边界明确，但对上层都暴露 `module_motor_t` 的共享控制和反馈行为。

详见 [Module 使用指南](Module/README.md)。

### App

App 是可复用控制流程，不是完整机器人程序。

| 组件 | 职责 |
| --- | --- |
| `app_command` | 将项目无关的遥控和视觉输入转成控制命令 |
| `app_chassis` | 统一驱动麦轮、三/四轮全向轮和四舵轮 |
| `app_gimbal` | 双轴云台，支持编码器或 IMU 反馈 |
| `app_shooter` | 摩擦轮、拨弹、单发和自动开火流程 |
| `app_imu` | BMI088 静止标定、加热、温稳判定和姿态解算 |
| `app_vision` | USB 视觉目标通信 |
| `app_safety` | 多实例心跳、失联检测和整机输出许可 |

详见 [App 使用说明](App/README.md)。

## 快速开始

1. 使用 CubeMX 或现有 BSP 创建 MCU 工程。
2. 将本仓库放入工程，例如 `your_robot/ECF/`。
3. 只加入机器人实际使用的 ECF 源文件和对应头文件目录。
4. 在项目中实现 CAN、SPI、USART、PWM 等所需的 BSP driver ops。
5. 静态创建 Module/App 对象、DMA 缓冲区和配置；填入 CAN ID、机械参数与 PID。
6. 按 `BSP → Module → App → Safety → 接收启动` 的顺序初始化。
7. IMU 保持静止完成标定，所有 required 心跳在线后再允许动力输出。
8. 用真实单调时钟计算 `dt`，在任务中周期调用更新函数。

完整文件划分、裁判适配和可复制代码见 [移植手册](移植手册.md)。

## 典型控制周期

下面只展示 ECF 调用关系。实际工程还应在进入 App 前完成遥控、裁判和 Safety 门控。

```c
const float dt_s = (float)elapsed_ms * 0.001F;

app_safety_process(&robot.safety, now_ms);
app_imu_update(&robot.imu, dt_s);
app_vision_update(&robot.vision,
                  app_imu_get_snapshot(&robot.imu), elapsed_ms);

app_command_update(&robot.command, &remote_input,
                   app_gimbal_get_feedback(&robot.gimbal),
                   app_vision_get_target(&robot.vision), dt_s);

const app_command_output_t *command =
    app_command_get_output(&robot.command);

app_chassis_command_t chassis_command = command->chassis;
app_gimbal_command_t gimbal_command = command->gimbal;
app_shooter_command_t shooter_command = command->shooter;

/* 项目根据 Safety 和裁判数据关闭不允许的命令。 */
robot_apply_output_gate(&chassis_command, &gimbal_command,
                        &shooter_command);

app_chassis_update(&robot.chassis, &chassis_command, dt_s);
app_gimbal_update(&robot.gimbal, &gimbal_command,
                  app_imu_get_snapshot(&robot.imu), dt_s);
app_shooter_update(&robot.shooter, &shooter_command,
                   app_gimbal_get_feedback(&robot.gimbal), dt_s);

module_dji_motor_bus_update(&robot.dji_bus, dt_s);
module_dm_motor_bus_update(&robot.dm_bus, dt_s);
```

App 和组合 Module 只设置电机目标。每条电机总线每周期只调用一次 `bus_update()`；项目不要再次调用 `module_motor_update()`，否则 PID、`dt` 和运行时间会重复累计。

## 关键设计约定

### 静态内存与所有权

- 不使用 `malloc`；实例、路由表、工作区和 DMA 缓冲区由调用者提供。
- 配置中被长期保存的指针必须覆盖对象的完整生命周期。
- 对象初始化后不要按值复制；使用对象指针和只读 getter。
- ISR 只搬运数据、记录长度或发送通知，协议解析和控制算法在任务上下文执行。

### 单位与时间

- 角度使用 `rad`，角速度使用 `rad/s`，距离使用 `m`，速度使用 `m/s`。
- 控制函数的 `dt_s` 来自真实单调时钟，不写死为 `1` 或 `0.001`。
- 硬件时间戳和任务周期必须明确单位，避免把毫秒直接传给秒接口。

### 电机使用

- App/组合 Module 设置目标，DJI/DM 总线负责唯一一次控制更新与报文发送。
- 电机应使用长期有效且可读的名字，例如 `gimbal_yaw`、`gimbal_pitch`、`shooter_motor`。
- 普通底盘按真实布局命名；舵轮推荐 `swerve_drive_1..4` 和 `swerve_steer_1..4`。
- 调试可读取电机名、最近 `dt`、累计使能运行时间、反馈和在线状态。

### IMU 与裁判系统

- BMI088 的解算流程参考验证过的 RoboMaster C Board INS 工程，并增加静止标定、温控和失败状态。
- 裁判 CRC 和常用字段与验证过的 ACE_ECF 实现交叉参考；本框架使用可移植 BSP 和流式解析，不绑定 H7/USART8。
- 裁判 Module 只解析协议。功率限制、枪口选择、每发热量和调试降级规则属于具体机器人项目。

### Safety

`app_safety_set_output_enabled()` 只是操作者请求。只有 required 监控全部在线，`app_safety_output_allowed()` 才允许输出；项目必须把该结果实际作用到 App 命令或电机总线，单独运行 Safety 任务不会自动切断硬件。

## 文档索引

- [移植手册：工程目录、初始化、任务与裁判调用](移植手册.md)
- [Algorithm 层](Algorithm/README.md)
- [BSP 层](Bsp/README.md)
- [Module 层](Module/README.md)
- [App 层](App/README.md)
- [底盘配置](App/app_chassis/README.md)
- [命令映射](App/app_command/README.md)
- [IMU 工程流程](App/app_imu/README.md)
- [裁判系统](Module/module_referee/README.md)
- [DJI 电机](Module/module_dji_motor/README.md)
- [DM 电机](Module/module_dm_motor/README.md)

每个组件的公开头文件是最终 API；同目录 README 说明初始化顺序、数据读取、内存要求和移植注意事项。

## 验证状态

PC 回归测试：

```powershell
./Tests/run_tests.ps1
```

当前测试覆盖控制链、安全门、电机总线、CRC、IMU EKF，以及麦轮、全向轮和舵轮黄金结果。移植到真实硬件后仍必须验证：

- 遥控/裁判失联后的 DJI 零电流与 DM 失能。
- BMI088 轴映射、静止标定、加热占空比和温度稳定状态。
- 真实任务周期、CAN/FDCAN Classic 帧、DMA 内存区域和 Cache 一致性。
- 轮序、转向符号、云台反馈方向、枪口热量和功率限制。

建议先悬空轮、使用限流电源和低目标值测试，再逐步开放整机输出。
