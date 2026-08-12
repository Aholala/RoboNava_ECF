# ROBONAVA_ECF — Embedded Control Framework for RoboMaster

ECF 是一套面向 RoboMaster 竞赛的**可移植嵌入式控制框架**，使用 C11 编写，不绑定任何具体机器人、MCU 型号或 FreeRTOS 实例。

核心原则：**静态内存、显式依赖、C 语言面向对象、算法与硬件解耦。**

---

## 目录

- [1. 架构总览](#1-架构总览)
- [2. C 语言面向对象设计](#2-c-语言面向对象设计)
- [3. 命名规范](#3-命名规范)
- [4. 分层详解](#4-分层详解)
  - [4.1 Algorithm — 算法层](#41-algorithm--算法层)
  - [4.2 Bsp — 板级支持包](#42-bsp--板级支持包)
  - [4.3 Module — 模块层](#43-module--模块层)
  - [4.4 App — 应用层](#44-app--应用层)
- [5. 数据流全景](#5-数据流全景)
- [6. 搭建一台机器人](#6-搭建一台机器人)
- [7. 移植到新平台](#7-移植到新平台)
- [8. Ozone 调试指南](#8-ozone-调试指南)
- [9. 设计决策与约束](#9-设计决策与约束)

---

## 1. 架构总览

```
┌─────────────────────────────────────────────────────────┐
│  App (应用层)                                            │
│  app_chassis  app_gimbal  app_shooter  app_command       │
│  app_imu      app_safety  app_vision   app_exchange      │
│  ┌─────────────────────────────────────────────────┐    │
│  │ app_exchange  (模块间数据交换 — 临界区保护)       │    │
│  └─────────────────────────────────────────────────┘    │
├─────────────────────────────────────────────────────────┤
│  Module (模块层)                                         │
│  module_swerve     module_dji_motor   module_dm_motor    │
│  module_motor (基类)  module_bmi088    module_dr16       │
│  module_board_comm  module_referee    module_buzzer      │
│  module_oled    module_ws2812  module_uart_comm ...      │
├─────────────────────────────────────────────────────────┤
│  Algorithm (算法层)                                      │
│  alg_attitude  alg_imu_ekf   alg_kalman   alg_filter     │
│  alg_pid       alg_math      alg_swerve   alg_mecanum    │
│  alg_trajectory  alg_lqr     alg_crc     alg_omni        │
├─────────────────────────────────────────────────────────┤
│  Bsp (板级支持包)                                        │
│  bsp_can  bsp_spi  bsp_i2c  bsp_usart  bsp_gpio         │
│  bsp_pwm  bsp_adc  bsp_timer  bsp_watchdog  bsp_log     │
│  bsp_dwt  bsp_exti  bsp_encoder  bsp_usb_vcp            │
│  bsp_can_dispatcher  bsp_fdcan  bsp_fdcan_classic_adapter│
├─────────────────────────────────────────────────────────┤
│  HAL / CMSIS / FreeRTOS (平台相关，不纳入 ECF)           │
└─────────────────────────────────────────────────────────┘
```

**层级依赖规则：**

```
App ──→ Module ──→ Algorithm
  │        │
  └────────┼──────→ Bsp
           │
Module ────┘
```

- **Algorithm** 层不依赖任何硬件，只依赖 C11 标准库 (`math.h`, `stdbool.h`, `stddef.h`)
- **Bsp** 层只封装 HAL/CMSIS，不包含任何算法或业务逻辑
- **Module** 层组合 Bsp + Algorithm，对外提供设备对象
- **App** 层编排 Module 和 Algorithm，实现具体的机器人行为

---

## 2. C 语言面向对象设计

ECF 使用三种核心 C 语言 OOP 模式。

### 2.1 虚表多态（基类 + 派生类）

**适用场景：** 同类外设存在多种具体实现（如 DJI-M2006、DJI-M3508、达妙-M4310 都是电机）。

**实现方式：**

```
module_motor_t (基类)              module_dji_motor_t (派生类)
┌──────────────────┐              ┌──────────────────────┐
│ vptr ─────────┐  │              │ super (第一个成员)     │  ← 偏移0
│ motor_name    │  │              │   .vptr ─────────┐   │
│ state         │  │              │   .motor_name    │   │
│ feedback      │  │              │   .state         │   │
│ ...           │  │              │   ...            │   │
└───────────────┘  │              │ motor_model      │   │ ← 派生专属
                   │              │ control_mode     │   │
                   └──→ module_dji_motor_ops_t       │   │
                        (虚表 — 函数指针数组)          │   │
                        ┌──────────────────┐         │   │
                        │ enable()         │         │   │
                        │ disable()        │         │   │
                        │ set_target()     │         │   │
                        │ update()         │         │   │
                        │ can_clear_fault()│         │   │
                        └──────────────────┘         │   │
                                                     │   │
                        module_dji_motor_t *get()     │   │
                        ← MODULE_MOTOR_CONTAINER_OF ──┘   │
                        └─────────────────────────────────┘
```

**关键宏：**

```c
// 1. 编译期断言：派生类第一成员必须是 super
MODULE_MOTOR_STATIC_ASSERT_SUPER_FIRST(module_dji_motor_t);

// 2. 从基类指针恢复派生类指针（等价于 C++ 的 static_cast<Derived*>(base)）
#define MODULE_MOTOR_CONTAINER_OF(ptr, type, member) \
    ((type *)((uint8_t *)(ptr) - offsetof(type, member)))

// 使用示例
static module_dji_motor_t *get_device(module_motor_t *base) {
    return MODULE_MOTOR_CONTAINER_OF(base, module_dji_motor_t, super);
}
```

**虚函数调用路径：**

```c
// 基类 API（module_motor.c）
module_motor_status_t module_motor_enable(module_motor_t *me) {
    // 参数校验 + 状态检查
    if (!me->is_registered) return MODULE_MOTOR_STATUS_NOT_REGISTERED;
    // 通过虚表调用派生类实现
    return me->vptr->enable(me);
}

// 派生类虚函数（module_dji_motor.c）
static module_motor_status_t dji_enable(module_motor_t *base) {
    module_dji_motor_t *me = get_device(base);  // 恢复派生类
    // DJI 特有的使能逻辑：复位 PID、设置默认命令等
}
```

### 2.2 驱动抽象表（OPS 表）

**适用场景：** 同一外设存在多种底层硬件实现（FDCAN vs bxCAN）。

**实现方式（bsp_can）：**

```c
// 平台无关的驱动操作表
typedef struct {
    bsp_status_t (*init)(void *);
    bsp_status_t (*deinit)(void *);
    bsp_status_t (*start)(void *);
    bsp_status_t (*stop)(void *);
    bsp_status_t (*configure_filter)(void *, const bsp_can_filter_t *);
    bsp_status_t (*transmit)(void *, const bsp_can_frame_t *, uint32_t);
    bsp_status_t (*receive)(void *, bsp_can_receive_fifo_t, bsp_can_frame_t *);
    bsp_status_t (*get_tx_free_level)(const void *, uint32_t *);
} bsp_can_driver_ops_t;

// 配置时绑定平台驱动
bsp_can_config_t cfg = {
    .device_handle = &hfdcan1,      // HAL 句柄
    .driver_ops    = &fdcan_ops,    // FDCAN 驱动表
};
bsp_can_init(&can, &cfg);

// 此后所有调用路径统一：
// bsp_can_transmit() → ops->transmit() → driver_ops->transmit() → HAL_FDCAN_xxx
```

### 2.3 回调注入（策略模式）

**适用场景：** 算法需要调用者提供模型函数（如 EKF 状态转移/观测函数）。

```c
// EKF 配置中包含四个回调函数指针
typedef struct {
    alg_kalman_state_function_t state_function;         // f(x, u, dt)
    alg_kalman_state_jacobian_function_t state_jacobian; // ∂f/∂x
    alg_kalman_measurement_function_t measurement_function; // h(x)
    alg_kalman_measurement_jacobian_function_t measurement_jacobian; // ∂h/∂x
    void *user_context;  // 透传上下文
} alg_kalman_extended_config_t;
```

---

## 3. 命名规范

### 3.1 前缀体系

| 前缀 | 含义 | 示例 |
|------|------|------|
| `alg_` | Algorithm — 纯算法 | `alg_pid_t`, `alg_filter_low_pass_init()` |
| `bsp_` | Board Support Package — 外设驱动 | `bsp_can_t`, `bsp_spi_transmit()` |
| `module_` | 设备模块 | `module_dji_motor_t`, `module_swerve_init()` |
| `app_` | Application — 应用逻辑 | `app_chassis_t`, `app_exchange_publish_imu()` |

### 3.2 类型命名

| 后缀 | 含义 | 示例 |
|------|------|------|
| `_t` | 类型（typedef） | `alg_pid_t`, `bsp_can_frame_t` |
| `_status_t` | 状态码枚举 | `bsp_status_t`, `module_motor_status_t` |
| `_config_t` | 初始化配置结构体 | `alg_pid_config_t`, `module_dji_motor_config_t` |
| `_ops_t` | 操作表（函数指针数组） | `module_motor_ops_t`, `bsp_can_driver_ops_t` |
| `_cb_t` / `_callback_t` | 回调函数指针 | `bsp_event_callback_t`, `bsp_can_frame_callback_t` |

### 3.3 函数命名

```
<prefix>_<module>_<verb>_<noun>()

示例：
  alg_pid_update()                   // 前缀_模块_动作
  module_dji_motor_bus_flush()       // 前缀_模块_子功能_动作
  bsp_can_dispatcher_add_route()     // 前缀_模块_子模块_动作_对象
  alg_imu_ekf_get_gravity_body()     // 前缀_模块_获取_对象_坐标系
```

**常用动词：**
- `init` — 初始化对象
- `reset` — 重置状态
- `update` — 周期更新（传入 dt）
- `get_*` — 读取值
- `set_*` — 设置值/配置
- `enable` / `disable` — 使能/禁用
- `start` / `stop` — 启动/停止硬件外设
- `send` / `receive` / `transmit` — 数据传输
- `publish_*` / `read_*` — 发布/读取（app_exchange）

### 3.4 常数与单位

- **角度：** 弧度（rad），命名后缀 `_rad`
- **角速度：** 弧度/秒（rad/s），命名后缀 `_rad_per_s`
- **加速度：** m/s²，命名后缀 `_m_per_s2`
- **温度：** 摄氏度（°C），命名后缀 `_c`
- **电流：** 安培（A），命名后缀 `_a`
- **频率：** 赫兹（Hz），命名后缀 `_hz`
- **时间：** 秒（s），命名后缀 `_s`；毫秒 `_ms`；微秒 `_us`

---

## 4. 分层详解

### 4.1 Algorithm — 算法层

**原则：** 纯 C11，不包含任何 `#include "bsp_*.h"`、`#include "module_*.h"`、HAL/CMSIS/FreeRTOS 头文件。不使用动态内存。所有时间通过 `delta_time_s` 参数显式传入。

| 模块 | 功能 | 核心结构体 |
|------|------|-----------|
| `alg_attitude` | IMU 姿态估计（Mahony/Madgwick） | `alg_attitude_t`, `alg_attitude_quaternion_t` |
| `alg_imu_ekf` | 6维四元数 EKF（陀螺仪+加速度计） | `alg_imu_ekf_t`, `alg_imu_ekf_config_t`, `alg_imu_ekf_diagnostics_t` |
| `alg_kalman` | 通用卡尔曼（标量/线性/EKF） | `alg_kalman_scalar_t`, `alg_kalman_linear_t`, `alg_kalman_extended_t` |
| `alg_filter` | 8种数字滤波器 | `alg_filter_low_pass_t`, `alg_filter_biquad_t`, `alg_filter_fir_t` 等 |
| `alg_pid` | PID（单环/串级/角度） | `alg_pid_t`, `alg_pid_config_t`, `alg_pid_cascade_t`, `alg_pid_angle_t` |
| `alg_math` | 数学库（向量/矩阵/四元数/统计/插值） | `alg_math_vector3_t`, `alg_math_matrix_t`, `alg_math_quaternion_t` |
| `alg_swerve` | 舵轮运动学（4模块矩形底盘） | `alg_swerve_t`, `alg_swerve_command_t`, `alg_swerve_module_target_t` |
| `alg_mecanum` | 麦克纳姆轮运动学 | `alg_mecanum_t` |
| `alg_omni` | 全向轮运动学（任意轮数） | `alg_omni_t` |
| `alg_trajectory` | 梯形/S曲线轨迹规划 | `alg_trajectory_t`, `alg_trajectory_group_t` |
| `alg_lqr` | 固定增益 LQR 状态反馈 | `alg_lqr_t` |
| `alg_crc` | CRC8/16/32 校验 | `alg_crc_t` |
| `alg_chassis` | 底盘公共接口+车轮里程计监测 | `alg_chassis_velocity_t`, `alg_chassis_wheel_monitor_t` |

**使用模式（以 alg_pid 为例）：**

```c
// 1. 静态分配对象
alg_pid_t speed_pid;
alg_pid_config_t config;

// 2. 获取默认配置
alg_pid_config_init(&config);
config.proportional_gain = 5.0f;
config.integral_gain     = 0.5f;
config.integral_min      = -2.0f;
config.integral_max      =  2.0f;
config.output_min        = -20.0f;
config.output_max        =  20.0f;

// 3. 初始化
alg_pid_init(&speed_pid, &config);
alg_pid_reset(&speed_pid, measured_speed, 0.0f);

// 4. 周期调用（通常在 1kHz 定时器中断或 RTOS 任务中）
float output;
alg_pid_status_t rc = alg_pid_update(&speed_pid, target_speed, measured_speed, 0.001f, &output);
if (rc == ALG_PID_STATUS_OK) {
    set_motor_current(output);
}
```

### 4.2 Bsp — 板级支持包

**原则：** 只封装 HAL/CMSIS 外设，提供平台无关的 API。每个外设一个 `bsp_xxx_t` 对象 + 对应的 `bsp_xxx_config_t` 配置结构体。

| 模块 | 功能 | 核心结构体 |
|------|------|-----------|
| `bsp_can` | Classic CAN（总线抽象+驱动表） | `bsp_can_t`, `bsp_can_frame_t`, `bsp_can_filter_t`, `bsp_can_driver_ops_t` |
| `bsp_can_dispatcher` | CAN 帧路由分发器 | `bsp_can_dispatcher_t`, `bsp_can_route_t` |
| `bsp_fdcan` | FDCAN（CAN-FD 支持） | `bsp_fdcan_t` |
| `bsp_fdcan_classic_adapter` | FDCAN→Classic CAN 适配器 | `bsp_fdcan_classic_adapter_t` |
| `bsp_spi` | SPI 外设抽象 | `bsp_spi_t` |
| `bsp_i2c` | I2C 外设抽象 | `bsp_i2c_t` |
| `bsp_usart` | 串口（阻塞/中断/DMA） | `bsp_usart_t` |
| `bsp_usb_vcp` | USB 虚拟串口 | `bsp_usb_vcp_t` |
| `bsp_gpio` | GPIO 读写 | `bsp_gpio_t` |
| `bsp_pwm` | PWM 输出（频率/脉宽/占空比） | `bsp_pwm_t` |
| `bsp_adc` | ADC 原始值/归一化/电压 | `bsp_adc_t` |
| `bsp_encoder` | 编码器（增量式/环形计数器） | `bsp_encoder_t` |
| `bsp_timer` | 硬件定时器（单次/周期/微秒延迟） | `bsp_timer_t` |
| `bsp_dwt` | DWT 周期计数器（微秒级精确定时） | `bsp_dwt_t` |
| `bsp_exti` | 外部中断（GPIO 上升/下降沿） | `bsp_exti_t` |
| `bsp_watchdog` | 独立/窗口看门狗 | `bsp_watchdog_t` |
| `bsp_log` | SEGGER RTT 日志输出 | 通过宏调用，无对象结构 |
| `bsp_common` | 公共类型：状态码、事件、传输模式 | `bsp_status_t`, `bsp_event_t`, `bsp_error_t` |

**BSP 层调用模式（以 CAN 为例）：**

```c
// 1. 在 board_config.c 中绑定平台硬件（仅此文件与 HAL 耦合）
bsp_can_config_t cfg = {
    .device_handle = &hfdcan1,          // HAL 句柄
    .driver_ops    = &fdcan_driver_ops, // 平台驱动表
};
bsp_can_init(&can1, &cfg);
bsp_can_start(&can1);

// 2. 应用层通过 board_config_get_can() 获取实例
bsp_can_t *can = board_config_get_can(BOARD_CONFIG_CAN_1);

// 3. 发送
bsp_can_frame_t frame = {
    .identifier  = 0x200,
    .id_type     = BSP_CAN_ID_STANDARD,
    .frame_type  = BSP_CAN_FRAME_DATA,
    .data_length = 8,
    .data        = {0, 1, 2, 3, 4, 5, 6, 7},
};
bsp_can_transmit(can, &frame, 10);  // 10ms 超时

// 4. 接收（轮询，在 RTOS 任务中）
bsp_can_frame_t rx_frame;
while (bsp_can_receive(can, BSP_CAN_RX_FIFO_0, &rx_frame) == BSP_STATUS_OK) {
    // 处理帧...
}
```

### 4.3 Module — 模块层

**原则：** 将外设和算法组合为设备对象。电机使用基类+派生+虚表的多态模型。

#### 4.3.1 电机子系统

```
module_motor_t (基类)
├── module_dji_motor_t    — DJI M2006/M3508/GM6020 (CAN 协议)
├── module_dm_motor_t     — 达妙 DM 系列电机 (RS485/CAN)
└── module_dm4310_t       — 达妙 DM-J4310-2EC
```

**电机控制链路（以角度控制为例）：**

```
module_motor_set_target(&motor.super, target_angle_rad)
  └── motor.super.vptr->set_target()  ← 虚函数，最终到 module_dji_motor
        └── motor.target_angle_rad = target_angle_rad

module_motor_update(&motor.super, dt)
  └── motor.super.vptr->update()      ← 虚函数
        └── [角度环 PID] → [速度环 PID] → [电流环 PID] → CAN 命令值

module_dji_motor_bus_flush(&bus)
  └── 将各电机 command_value 打包→ CAN 帧→发送
```

**级联 PID 结构：**

```
 target_angle ──→ [角度 PID] ──→ [速度 PID] ──→ [电流 PID] ──→ CAN 电流命令
                      ↑              ↑              ↑
                 position_fb    velocity_fb     current_fb
```

#### 4.3.2 舵轮模块

`module_swerve_t` 是一个**组合模块**：将两个电机（驱动+舵向）包装成一个物理舵轮。

```c
module_swerve_t module;
module_swerve_config_t cfg = {
    .drive_motor    = &dji_m3508.super,     // module_motor_t*（多态）
    .steering_motor = &dji_gm6020.super,    // module_motor_t*（多态）
    .wheel_radius_m = 0.076f,
    .drive_reduction_ratio = 19.2f,
    // ...
};
module_swerve_init(&module, &cfg);

// 底盘 App 下发运动学目标
alg_swerve_module_target_t target = {.velocity_m_per_s = 1.0f, .steering_angle_rad = 0.5f};
module_swerve_apply_target(&module, &target, dt);
// 内部自动：线速度→驱动电机目标 + 舵角→舵向电机目标
```

#### 4.3.3 其他模块

| 模块 | 功能 | 核心结构体 |
|------|------|-----------|
| `module_bmi088` | BMI088 IMU（SPI，加速度+陀螺） | `module_bmi088_t` |
| `module_dr16` | DJI DR16 遥控接收机 | `module_dr16_t` |
| `module_board_comm` | 云台-底盘板间 CAN 通信协议 | `module_board_comm_t` |
| `module_referee` | RoboMaster 裁判系统（解析+UI） | `module_referee_t` |
| `module_shooter` | 双摩擦轮+拨弹盘射击器 | `module_shooter_t` |
| `module_servo` | PWM 舵机 | `module_servo_t` |
| `module_buzzer` | PWM 蜂鸣器 | `module_buzzer_t` |
| `module_oled` | I2C SSD1306 128×64 OLED | `module_oled_t` |
| `module_ws2812` | WS2812B RGB LED 灯带 | `module_ws2812_t` |
| `module_nrf24l01` | NRF24L01 2.4G 无线模块 | `module_nrf24l01_t` |
| `module_uart_comm` | UART 固定帧协议 | `module_uart_comm_t` |
| `module_usb_comm` | USB CDC 自定义协议 | `module_usb_comm_t` |

### 4.4 App — 应用层

**原则：** 编排 Module 和 Algorithm，实现完整的机器人功能。通过 `app_exchange` 进行模块间通信。

| 模块 | 功能 | 核心结构体 |
|------|------|-----------|
| `app_exchange` | 模块间数据交换（临界区保护的单元素缓冲区） | 无对象，纯函数接口 |
| `app_command` | 遥控器→底盘/云台/射击器指令映射 | `app_command_config_t` |
| `app_chassis` | 底盘控制（4种模式+自锁） | `app_chassis_t` |
| `app_gimbal` | 云台双轴角度控制（IMU/编码器反馈） | `app_gimbal_t` |
| `app_shooter` | 摩擦轮+拨弹盘火控逻辑 | `app_shooter_t` |
| `app_imu` | IMU 驱动+EKF 姿态估计 | `app_imu_t` |
| `app_vision` | 视觉目标接收（USB CDC 协议） | `app_vision_t` |
| `app_safety` | 安全监控（16个软件心跳+硬件看门狗） | `app_safety_monitor_t` |
| `app_types.h` | 模块间共享数据类型 | `app_chassis_command_t`, `app_gimbal_command_t` 等 |

#### 4.4.1 app_types.h — 跨模块通信类型

```c
// 底盘指令（app_command → app_chassis）
app_chassis_command_t {
    velocity_x_m_per_s, velocity_y_m_per_s,  // 平面速度
    angular_velocity_rad_per_s,              // 角速度
    gimbal_yaw_rad,                          // 云台偏航（用于跟随模式）
    mode,         // NORMAL / SPIN / FOLLOW_GIMBAL / NO_FORCE
    self_lock_when_stopped,
    enabled, sequence
};

// 云台指令（app_command → app_gimbal）
app_gimbal_command_t {
    yaw_target_rad, pitch_target_rad,
    feedback_mode,  // ENCODER / IMU
    enabled, sequence
};

// 射击器指令（app_command → app_shooter）
app_shooter_command_t {
    friction_enabled, fire_requested,
    automatic_fire_enabled,
    friction_velocity_rad_per_s, sequence
};

// IMU 快照（app_imu → app_gimbal / app_command）
app_imu_snapshot_t {
    yaw_rad, pitch_rad, roll_rad,
    angular_velocity_rad_per_s[3],
    sample_count, valid
};

// 云台反馈（app_gimbal → app_shooter / 板间通信）
app_gimbal_feedback_t {
    yaw_rad, pitch_rad,
    yaw_velocity_rad_per_s, pitch_velocity_rad_per_s,
    motors_online, target_locked
};

// 视觉目标（app_vision → app_gimbal / app_shooter）
app_vision_target_t {
    target_yaw_rad, target_pitch_rad,
    update_count, target_valid, tracking_ready
};
```

#### 4.4.2 app_exchange — 发布/订阅数据交换

```c
// 初始化（清零所有缓冲区）
app_exchange_init();

// 生产者侧（命令发布者）
app_exchange_publish_chassis_command(&cmd);   // app_command 调用
app_exchange_publish_gimbal_command(&cmd);
app_exchange_publish_imu(&snapshot);           // app_imu 调用
app_exchange_publish_vision_target(&target);   // app_vision 调用
app_exchange_publish_gimbal_feedback(&fb);     // app_gimbal 调用

// 消费者侧（命令执行者）
app_exchange_read_chassis_command(&cmd);       // app_chassis 调用
app_exchange_read_gimbal_command(&cmd);        // app_gimbal 调用
app_exchange_read_imu(&snapshot);              // app_gimbal / app_command 调用
app_exchange_read_vision_target(&target);      // app_gimbal 调用
app_exchange_read_gimbal_feedback(&fb);        // app_shooter 调用
```

**设计要点：** 每类数据一个独立缓冲区；通过 `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()` 保证原子读写；单生产者单消费者，无锁。

---

## 5. 数据流全景

```
                    ┌─────────────┐
                    │   DR16 遥控   │
                    └──────┬──────┘
                           │ RC 数据
                    ┌──────▼──────┐
                    │ app_command │ ← 遥控→指令映射
                    └──┬──┬──┬───┘
                       │  │  │
          ┌────────────┘  │  └────────────┐
          ▼               ▼               ▼
   ┌─────────────┐ ┌─────────────┐ ┌─────────────┐
   │ publish_    │ │ publish_    │ │ publish_    │
   │ chassis_cmd │ │ gimbal_cmd  │ │ shooter_cmd │
   └──────┬──────┘ └──────┬──────┘ └──────┬──────┘
          │               │               │
   ┌──────▼──────┐ ┌──────▼──────┐ ┌──────▼──────┐
   │ read_       │ │ read_       │ │ read_       │
   │ chassis_cmd │ │ gimbal_cmd  │ │ shooter_cmd │
   └──────┬──────┘ └──────┬──────┘ └──────┬──────┘
          ▼               ▼               ▼
   ┌───────────┐   ┌───────────┐   ┌───────────┐
   │app_chassis│   │ app_gimbal│   │app_shooter │
   │  update() │   │  update() │   │  update() │
   └─────┬─────┘   └─────┬─────┘   └─────┬─────┘
         │               │               │
    ┌────▼────┐     ┌────▼────┐     ┌────▼────┐
    │ 舵轮×4  │     │电机×2   │     │摩擦轮×2 │
    │(module) │     │(module) │     │拨弹盘×1 │
    └─────────┘     └─────────┘     │(module) │
                                    └─────────┘

   ┌───────────┐          ┌───────────┐
   │ app_imu   │          │app_vision │
   │ EKF姿态   │          │ USB CDC   │
   └─────┬─────┘          └─────┬─────┘
         │ publish_imu()        │ publish_vision_target()
         ▼                      ▼
   ┌──────────────────────────────────┐
   │           app_exchange           │
   └──────────────────────────────────┘
         ▲                      ▲
         │ read_imu()           │ read_vision_target()
    ┌────┴────┐           ┌────┴────┐
    │app_gimbal│           │app_gimbal│ ← IMU 反馈 + 视觉目标
    └─────────┘           │app_shooter│ ← 自瞄射击判断
                          └─────────┘
```

---

## 6. 搭建一台机器人

### 6.1 整体步骤

```
1. 硬件选型 → 2. 板级配置 → 3. 创建模块实例 → 4. 编写 App 任务 → 5. 调试
```

### 6.2 第一步：board_config（板级配置）

创建 `board_config.h` / `board_config.c`，集中管理所有 HAL 外设句柄与 ECF 对象的绑定：

```c
// board_config.h — 枚举所有可以获取的外设实例
typedef enum {
    BOARD_CONFIG_CAN_CHASSIS = 0,  // 底盘 CAN
    BOARD_CONFIG_CAN_GIMBAL,       // 云台 CAN
    BOARD_CONFIG_SPI_IMU,          // BMI088 SPI
    BOARD_CONFIG_I2C_OLED,         // OLED I2C
    // ...
    BOARD_CONFIG_DEVICE_COUNT
} board_config_device_t;

// 获取 BSP 实例
bsp_can_t  *board_config_get_can(board_config_device_t id);
bsp_spi_t  *board_config_get_spi(board_config_device_t id);
bsp_i2c_t  *board_config_get_i2c(board_config_device_t id);
// ...

// board_config.c — 初始化所有外设并绑定 HAL 句柄
void board_config_init(void) {
    // CAN1 (FDCAN) — 底盘电机总线
    bsp_can_config_t can1_cfg = {
        .device_handle = &hfdcan1,
        .driver_ops    = &fdcan_driver_ops,
    };
    bsp_can_init(&can_devices[BOARD_CONFIG_CAN_CHASSIS], &can1_cfg);
    bsp_can_start(&can_devices[BOARD_CONFIG_CAN_CHASSIS]);

    // SPI2 — BMI088 IMU
    bsp_spi_config_t spi2_cfg = { .device_handle = &hspi2, /* ... */ };
    bsp_spi_init(&spi_devices[BOARD_CONFIG_SPI_IMU], &spi2_cfg);

    // ... 其他外设
}
```

### 6.3 第二步：创建机器人对象

```c
// robot.h
typedef struct {
    // === 电机总线 ===
    module_dji_motor_bus_t chassis_bus;  // 底盘 CAN 总线（4电机×5组=20槽位）
    module_dji_motor_bus_t gimbal_bus;   // 云台 CAN 总线

    // === 电机实例 ===
    // 底盘舵轮模块（4个，每个=驱动电机+舵向电机）
    module_swerve_t swerve_modules[4];

    // 底盘电机（派生对象）
    module_dji_motor_t drive_motors[4];     // M3508 驱动电机
    module_dji_motor_t steering_motors[4];  // GM6020 舵向电机

    // 云台电机
    module_dji_motor_t yaw_motor;           // GM6020 偏航
    module_dji_motor_t pitch_motor;         // GM6020 俯仰

    // 射击器
    module_shooter_t shooter;               // 摩擦轮+拨弹盘

    // === 传感器 ===
    module_bmi088_t imu;                    // BMI088

    // === 通信 ===
    module_dr16_t dr16;                     // 遥控接收机
    module_board_comm_t board_comm;         // 板间通信（云台↔底盘）

    // === App 对象 ===
    app_imu_t app_imu;
    app_chassis_t app_chassis;
    app_gimbal_t app_gimbal;
    app_shooter_t app_shooter;

    // === 算法对象 ===
    alg_swerve_t swerve_kinematics;         // 舵轮运动学
    alg_imu_ekf_t imu_ekf;                  // EKF 姿态估计

    bool initialized;
} robot_t;
```

### 6.4 第三步：初始化机器人

```c
bsp_status_t robot_init(robot_t *me) {
    // ===== 1. 初始化交换层 =====
    app_exchange_init();

    // ===== 2. 初始化电机总线 =====
    module_dji_motor_bus_init(&me->chassis_bus,
        board_config_get_can(BOARD_CONFIG_CAN_CHASSIS), 10);
    module_dji_motor_bus_init(&me->gimbal_bus,
        board_config_get_can(BOARD_CONFIG_CAN_GIMBAL), 10);

    // ===== 3. 初始化电机 =====
    // 底盘驱动电机 ×4 (M3508，速度环控制，CAN ID 1~4)
    for (int i = 0; i < 4; i++) {
        module_dji_motor_config_t cfg = {
            .motor_name      = drive_names[i],
            .motor_bus       = &me->chassis_bus,
            .motor_model     = MODULE_DJI_MOTOR_M3508,
            .control_mode    = MODULE_DJI_CONTROL_VELOCITY,
            .motor_identifier = (uint8_t)(i + 1),
            .direction_sign   = 1.0f,
            .velocity_pid_config = {.proportional_gain = 5.0f, .integral_gain = 0.5f, /*...*/},
            // ...
        };
        module_dji_motor_init(&me->drive_motors[i], &cfg);
        module_dji_motor_register(&me->drive_motors[i]);
    }
    // 类似地初始化 steering_motors、yaw_motor、pitch_motor...

    // ===== 4. 初始化舵轮模块（组合驱动+舵向电机） =====
    for (int i = 0; i < 4; i++) {
        module_swerve_config_t cfg = {
            .drive_motor    = &me->drive_motors[i].super,
            .steering_motor = &me->steering_motors[i].super,
            .wheel_radius_m = 0.076f,
            // ...
        };
        module_swerve_init(&me->swerve_modules[i], &cfg);
    }

    // ===== 5. 初始化运动学 =====
    alg_swerve_config_t swerve_cfg = {
        .module_position_x_m = {-0.15f, 0.15f, -0.15f, 0.15f},  // FL, FR, RL, RR
        .module_position_y_m = { 0.15f, 0.15f, -0.15f, -0.15f},
        .wheel_radius_m      = 0.076f,
        .maximum_velocity_m_per_s = 5.0f,
    };
    alg_swerve_init(&me->swerve_kinematics, &swerve_cfg);

    // ===== 6. 初始化传感器 =====
    module_bmi088_config_t imu_cfg = {
        .spi = board_config_get_spi(BOARD_CONFIG_SPI_IMU),
        // ...
    };
    module_bmi088_init(&me->imu, &imu_cfg);

    // ===== 7. 初始化 EKF =====
    alg_imu_ekf_config_t ekf_cfg;
    alg_imu_ekf_config_init(&ekf_cfg);
    // 根据实际 IMU 特性调整参数...
    alg_imu_ekf_init(&me->imu_ekf, &ekf_cfg);

    // ===== 8. 初始化通信模块 =====
    module_dr16_init(&me->dr16, &dr16_cfg);
    module_board_comm_init(&me->board_comm, &board_comm_cfg);

    // ===== 9. 初始化 App 模块 =====
    app_chassis_config_t chassis_cfg = {
        .kinematics    = &me->swerve_kinematics,
        .modules       = {&me->swerve_modules[0], &me->swerve_modules[1],
                          &me->swerve_modules[2], &me->swerve_modules[3]},
        .board_comm    = &me->board_comm,
        .follow_gain   = 3.0f,
        .stop_deadband = 0.05f,
    };
    app_chassis_init(&me->app_chassis, &chassis_cfg);

    // 类似初始化 app_gimbal、app_shooter、app_imu、app_command...

    me->initialized = true;
    return BSP_STATUS_OK;
}
```

### 6.5 第四步：创建 FreeRTOS 任务

```c
// 1kHz 实时控制任务（高优先级）
void task_control_1khz(void *pvParameters) {
    robot_t *robot = (robot_t *)pvParameters;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        // --- 传感器更新 ---
        module_bmi088_read(&robot->imu);  // 读取 IMU 原始数据
        app_imu_update(&robot->app_imu, 0.001f);  // EKF 姿态更新+发布

        // --- CAN 接收 ---
        bsp_can_frame_t frame;
        while (bsp_can_receive(chassis_can, BSP_CAN_RX_FIFO_0, &frame) == BSP_STATUS_OK) {
            module_dji_motor_bus_handle_feedback(&robot->chassis_bus, &frame);
        }

        // --- 运动学 + 电机控制 ---
        app_command_update(0.001f);  // 遥控→指令发布
        app_chassis_update(&robot->app_chassis, 0.001f);  // 底盘控制
        app_gimbal_update(&robot->app_gimbal, 0.001f);    // 云台控制

        // --- CAN 发送 ---
        module_dji_motor_bus_flush(&robot->chassis_bus);
        module_dji_motor_bus_flush(&robot->gimbal_bus);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}

// 射击器任务（更低的频率，如 100Hz）
void task_shooter_100hz(void *pvParameters) {
    // ...
    app_shooter_update(&robot->app_shooter, 0.01f);
    // ...
}

// 安全监控任务（5ms 周期）
void task_safety_5ms(void *pvParameters) {
    // ...
    app_safety_process();  // 评估心跳+喂狗
    // ...
}
```

---

## 7. 移植到新平台

### 7.1 移植清单

| 步骤 | 工作量 | 说明 |
|------|--------|------|
| 1. 实现 BSP 驱动表 | 中 | 为新 MCU 的 HAL 编写 `bsp_xxx_driver_ops_t` |
| 2. 编写 board_config | 中 | 绑定外设句柄到 ECF BSP 实例 |
| 3. 适配 `bsp_dwt` | 小 | DWT 是 Cortex-M 特有，其他架构需替换微秒定时器 |
| 4. 适配 `bsp_log` | 小 | SEGGER RTT 替换为对应平台的日志通道 |
| 5. 编写新模块派生类 | 中 | 如果有新类型电机/传感器，实现 `module_motor_ops_t` 虚表 |

### 7.2 不需要修改的部分

- **Algorithm 层全部**：纯 C11，无平台依赖
- **App 层全部**：只依赖 Module 和 Algorithm 接口
- **Module 层大部分**：只依赖 BSP 接口和 Algorithm，不直接依赖 HAL

### 7.3 BSP 驱动表实现示例

```c
// 为 STM32H723 的 FDCAN 实现 bsp_can_driver_ops_t
const bsp_can_driver_ops_t fdcan_ops = {
    .init             = fdcan_init,              // HAL_FDCAN_Init
    .deinit           = fdcan_deinit,            // HAL_FDCAN_DeInit
    .start            = fdcan_start,             // HAL_FDCAN_Start + 中断使能
    .stop             = fdcan_stop,              // HAL_FDCAN_Stop
    .configure_filter = fdcan_configure_filter,   // HAL_FDCAN_ConfigFilter
    .transmit         = fdcan_transmit,           // HAL_FDCAN_AddMessageToTxFifoQ
    .receive          = fdcan_receive,            // HAL_FDCAN_GetRxMessage
    .get_tx_free_level = fdcan_get_tx_free_level, // HAL_FDCAN_GetTxFifoFreeLevel
};
```

移植者只需实现这几个函数，模块层和应用层代码完全不用改。

---

## 8. Ozone 调试指南

### 8.1 关键变量监控

#### 电机状态

| 变量路径 | 含义 | 正常范围 |
|----------|------|----------|
| `robot.chassis_bus.motor_slots[n][m]` | 查看总线上的电机槽位 | 非 NULL 表示已注册 |
| `motor.super.state` | 电机状态 | `0=DISA, 1=ENA, 2=FAULT` |
| `motor.super.feedback.position_rad` | 位置反馈 | -π ~ +π（单圈），连续变化（多圈） |
| `motor.super.feedback.velocity_rad_per_s` | 速度反馈 | 取决于电机型号 |
| `motor.super.feedback.current_a` | 电流反馈 | 取决于负载 |
| `motor.super.feedback.motor_temperature_c` | 温度 | < 80°C 安全 |
| `motor.super.feedback.is_online` | 在线标志 | `true` = 最近收到反馈 |
| `motor.super.feedback.update_count` | 累计反馈次数 | 持续递增 |

#### PID 调试（如果电机使用 PID 控制）

| 变量路径 | 含义 |
|----------|------|
| `motor.current_pid.terms.proportional` | P 项输出 |
| `motor.current_pid.terms.integral` | I 项输出 |
| `motor.current_pid.terms.derivative` | D 项输出 |
| `motor.current_pid.terms.output` | PID 总输出 |
| `motor.current_pid.terms.unsaturated_output` | 限幅前输出 |

#### EKF 诊断

| 变量路径 | 含义 | 正常范围 |
|----------|------|----------|
| `robot.imu_ekf.state[0~3]` | 四元数 [w,x,y,z] | \|q\| ≈ 1.0 |
| `robot.imu_ekf.state[4~5]` | 陀螺零偏 X/Y (rad/s) | < 0.1 rad/s |
| `robot.imu_ekf.covariance[0]` | q_w 方差 | 应逐渐收敛、减小 |
| `robot.imu_ekf.last_normalized_innovation_squared` | NIS 值 | 正常 < 1e-5 |
| `robot.imu_ekf.was_accelerometer_used` | 加速度是否被使用 | 静止时 true，运动时可能 false |
| `robot.imu_ekf.rejection_count` | 连续拒绝计数 | 0 正常，>10 需检查 |
| `robot.imu_ekf.has_converged` | EKF 是否收敛 | true 表示初始化完成 |
| `robot.imu_ekf.is_stable` | IMU 是否静止 | true = 陀螺/加速度都接近零 |

#### 底盘/云台状态

| 变量路径 | 含义 |
|----------|------|
| `robot.app_chassis.initialized` | 底盘已初始化 |
| `robot.app_gimbal.initialized` | 云台已初始化 |
| `app_exchange` 对应缓冲区 | 查看当前指令值（如 `chassis_command.velocity_x_m_per_s`） |

#### CAN 通信

| 变量路径 | 含义 |
|----------|------|
| `can_dispatcher.received_frame_count` | 累计接收帧数 |
| `can_dispatcher.unmatched_frame_count` | 未匹配帧数（可能是路由表不对） |
| `can_dispatcher.receive_error_count` | 接收错误次数 |

### 8.2 Ozone 常用技巧

**1. Watch 窗口表达式：**

```
// 查看欧拉角（调用函数求值）
robot.imu_ekf.state[0]   // w
robot.imu_ekf.state[1]   // x
robot.imu_ekf.state[2]   // y
robot.imu_ekf.state[3]   // z

// 快速评估：q_w 接近 1 表示接近水平
```

**2. 条件断点：**

```
// 在 EKF 拒绝加速度时中断
alg_imu_ekf_correct_accelerometer return != ALG_IMU_EKF_STATUS_OK

// 在电机进入故障时中断
motor.state == MODULE_MOTOR_STATE_FAULT

// 在 NIS 突增时中断
me->last_normalized_innovation_squared > 1e-4
```

**3. 时间线分析（SWO/ETM）：**

如果有 SWO 或 ETM 跟踪，可以在关键路径打时间戳：
- `bsp_can_transmit()` 调用频率和耗时
- `app_chassis_update()` 单次执行时间（应 < 500μs）
- `alg_imu_ekf_update()` 单次执行时间

**4. bsp_log 实时输出：**

RTT Viewer 中可以直接看到安全事件、离线/恢复通知等日志。

---

## 9. 设计决策与约束

### 9.1 为什么用 C 而不是 C++

- arm-none-eabi-gcc 对 C 的编译更稳定，LTO 兼容性好
- Ozone/调试器对 C 结构体的可视化优于 C++ 类
- RoboMaster 生态中大部分开源代码为 C，保持一致性
- 虚表 + CONTAINER_OF 的多态模式在性能和可读性上足以覆盖需求

### 9.2 为什么用静态内存

- 堆碎片风险在长期运行的嵌入式系统中不可接受
- 编译期确定内存使用量，可精确规划
- `_Static_assert` 可以检查结构体大小和偏移

### 9.3 为什么时间显式传入

- 不依赖系统时钟或 `HAL_GetTick()`
- 同一套算法可以在仿真（非实时）、单元测试（自由控制时间）和真实硬件上运行
- `delta_time_s` 为 0 或不合法时，算法可以返回错误而不是静默失败

### 9.4 为什么不使用全局状态

- 每个对象独立持有自己的状态（`me` 参数）
- 同一算法可以实例化多个独立实例（如 4 个电机各有一个 PID 控制器）
- 单元测试时可以单独初始化、干预状态、验证输出

### 9.5 为什么 base + ops 而不是 switch(enum)

```c
// ❌ 不好：switch 分支会随着电机型号增加而膨胀
switch (motor->model) {
    case M2006: handle_m2006(); break;
    case M3508: handle_m3508(); break;
    // ...每加一种型号就要改这里
}

// ✅ 好：新增电机只需实现虚表，现有代码完全不动
me->vptr->enable(me);  // 自动调用对应派生类的实现
```

---

**项目仓库结构与以上各模块的详细信息（API、结构体、使用示例）请参阅各目录下的 `README.md`。**
