# app_exchange -- 模块间数据交换层（发布/读取模式）

## 功能概述

交换层是 App 层所有模块之间的数据中枢。它为每一类跨模块传递的数据（底盘指令、云台指令、射击器指令、IMU 快照、云台反馈、视觉目标）维护一个受临界区保护的单元素缓冲区。生产者模块通过 `publish` 写入，消费者模块通过 `read` 读走，两者通过 FreeRTOS 临界区保证多任务环境下的原子访问。

**数据流向全景图：**

```
                   app_command
                       |
          +------------+------------+
          |            |            |
          v            v            v
   chassis_cmd   gimbal_cmd   shooter_cmd
          |            |            |
          v            v            v
     app_chassis  app_gimbal   app_shooter
          |            |            |
          v            v            |
   chassis_fb    gimbal_fb          |
                       |            |
          +------------+            |
          |                         |
          v                         v
     app_command <----+      app_shooter

   app_imu -----> imu_snapshot ----> app_gimbal, app_vision, app_command

   app_vision --> vision_target ----> app_command
```

所有数据交换均通过 `app_exchange` 的中转缓冲完成，模块间无直接依赖。

## 交换协议（生产者-消费者对应表）

| 数据类型 | 生产者 | 消费者 |
|---------|--------|--------|
| `app_chassis_command_t` | `app_command` | `app_chassis` |
| `app_gimbal_command_t` | `app_command` | `app_gimbal` |
| `app_shooter_command_t` | `app_command` | `app_shooter` |
| `app_imu_snapshot_t` | `app_imu` | `app_gimbal`, `app_vision`, `app_command` |
| `app_gimbal_feedback_t` | `app_gimbal` | `app_command`, `app_shooter` |
| `app_vision_target_t` | `app_vision` | `app_command` |

## 核心结构体

本模块自身不定义结构体，所有交换的数据类型均在 `app_types.h` 中定义。详见各模块 README 中对应的交换数据结构体说明。

内部缓冲区为各类型的静态全局变量，通过 X-宏 `APP_EXCHANGE_DEFINE` 自动生成 publish/read 函数对：

```c
// X-宏展开示例（以 chassis_command 为例）
void app_exchange_publish_chassis_command(const app_chassis_command_t *value)
{
    if (value != NULL) {
        taskENTER_CRITICAL();
        app_exchange_chassis_command = *value;  // 值拷贝
        taskEXIT_CRITICAL();
    }
}
void app_exchange_read_chassis_command(app_chassis_command_t *value)
{
    if (value != NULL) {
        taskENTER_CRITICAL();
        *value = app_exchange_chassis_command;  // 值拷贝
        taskEXIT_CRITICAL();
    }
}
```

**原子性保证：** 在 `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()` 之间的结构体拷贝对单次读/写操作是原子的。

## API 速查

| 函数 | 功能 | 说明 |
|------|------|------|
| `app_exchange_init()` | 将所有交换缓冲区清零初始化 | 应在调度器启动前调用 |
| `app_exchange_publish_chassis_command(command)` | 发布底盘指令 | 生产者：`app_command` |
| `app_exchange_read_chassis_command(command)` | 读取底盘指令 | 消费者：`app_chassis` |
| `app_exchange_publish_gimbal_command(command)` | 发布云台指令 | 生产者：`app_command` |
| `app_exchange_read_gimbal_command(command)` | 读取云台指令 | 消费者：`app_gimbal` |
| `app_exchange_publish_shooter_command(command)` | 发布射击器指令 | 生产者：`app_command` |
| `app_exchange_read_shooter_command(command)` | 读取射击器指令 | 消费者：`app_shooter` |
| `app_exchange_publish_imu(snapshot)` | 发布 IMU 姿态快照 | 生产者：`app_imu` |
| `app_exchange_read_imu(snapshot)` | 读取 IMU 快照 | 消费者：多个模块 |
| `app_exchange_publish_gimbal_feedback(feedback)` | 发布云台反馈 | 生产者：`app_gimbal` |
| `app_exchange_read_gimbal_feedback(feedback)` | 读取云台反馈 | 消费者：`app_command`, `app_shooter` |
| `app_exchange_publish_vision_target(target)` | 发布视觉跟踪目标 | 生产者：`app_vision` |
| `app_exchange_read_vision_target(target)` | 读取视觉目标 | 消费者：`app_command` |

## 使用示例

```c
#include "app_exchange.h"
#include "app_types.h"

/* --- 初始化（在所有模块 init 之前调用一次） --- */
void app_layer_init(void)
{
    app_exchange_init();
    // 然后依次初始化各模块: app_command_init, app_chassis_init, ...
}

/* --- 生产者示例：发布 IMU 快照（在 app_imu_update 中） --- */
void imu_producer_example(const app_imu_snapshot_t *imu)
{
    app_exchange_publish_imu(imu);
}

/* --- 消费者示例：读取云台指令（在 app_gimbal_update 中） --- */
void gimbal_consumer_example(void)
{
    app_gimbal_command_t cmd;
    app_exchange_read_gimbal_command(&cmd);
    if (cmd.enabled) {
        // 执行云台控制
    }
}

/* --- 跨模块依赖检查示例 --- */
void check_dependencies(void)
{
    app_shooter_command_t shooter_cmd;
    app_gimbal_feedback_t gimbal_fb;

    app_exchange_read_shooter_command(&shooter_cmd);
    app_exchange_read_gimbal_feedback(&gimbal_fb);

    // 仅当云台锁定且摩擦轮就绪时才允许自动连发
    if (shooter_cmd.automatic_fire_enabled && gimbal_fb.target_locked) {
        // 触发自动连发
    }
}
```

## 注意事项

1. **Init 必须最先调用**：`app_exchange_init` 应在所有使用交换层的模块初始化之前调用，否则它们读到的初始值是未定义内存。
2. **临界区即关全局中断**：在 Cortex-M 上 `taskENTER_CRITICAL` 是关闭全局中断。拷贝操作极短，但仍需注意不要在高频 ISR 上下文中调用这些函数 —— 它们应在任务上下文中使用。
3. **单缓冲 = 最新值语义**：如果生产者写入两帧之间消费者没有读取，中间帧会被覆盖丢失。这适用于控制指令（只关心最新值），但如需历史记录需自行实现队列。
4. **没有通知机制**：消费者无法主动获知数据是否更新，只能通过 `sequence` 字段判断是否为同一帧。这也是为什么指令结构体中都带有 `sequence` 字段。
5. **NULL 安全**：所有 publish/read 函数内部已检查 NULL 指针，传入 NULL 时静默返回。
