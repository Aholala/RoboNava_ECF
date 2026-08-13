# module_motor — 通用电机基类

`module_motor` 统一电机生命周期、反馈、故障状态和注册表。M2006、M3508、GM6020、DM4310 等派生模块通过 `module_motor_t` 与操作表复用公共行为。

**数据流向：** 派生类解析反馈 --> `module_motor_notify_feedback()` --> `module_motor_enable/set_target/update()` --> 派生类虚函数

## 核心类型

| 类型 | 作用 |
|------|------|
| `module_motor_t` | 电机基类：虚表、状态、反馈、超时、累计运行时间 |
| `module_motor_feedback_t` | 位置/速度/扭矩/电流/温度及在线、有效性标志 |
| `module_motor_ops_t` | 派生类虚表：`enable`/`disable`/`can_clear_fault`/`set_target`/`update` |
| `module_motor_state_t` | DISABLED / ENABLED / FAULT |
| `module_motor_status_t` | 电机状态码 |
| `module_motor_pid_t` | 直接封装 `alg_pid_t`（`config` 与 `alg_pid_config_t` 相同） |

## 状态机

```text
DISABLED -> enable -> ENABLED
ENABLED  -> disable -> DISABLED
ENABLED  -> feedback timeout -> FAULT
FAULT    -> clear_fault -> DISABLED
```

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `module_motor_init_base(me, ops, name)` | 初始化基类（派生类构造时调用） | 状态码 |
| `module_motor_enable(me)` | 使能（未注册/故障/离线时拒绝） | 状态码 |
| `module_motor_disable(me)` | 禁用 | 状态码 |
| `module_motor_clear_fault(me)` | 清故障（回到 DISABLED） | 状态码 |
| `module_motor_set_target(me, value)` | 设置目标 | 状态码 |
| `module_motor_update(me, dt_s)` | 周期更新（离线且使能则进 FAULT） | 状态码 |
| `module_motor_set_feedback_timeout(me, ms)` | 设置反馈超时（0 禁用） | 状态码 |
| `module_motor_update_feedback_time(me, ms)` | 更新超时计时（周期调用） | 状态码 |
| `module_motor_notify_feedback(me)` | 通知反馈已更新（派生类调用） | 状态码 |
| `module_motor_get_feedback(me)` | 读取只读反馈 | 指针或 NULL |
| `module_motor_set_output_allowed(allowed)` / `module_motor_output_allowed()` | 全局输出门 | void / bool |
| `module_motor_get_name(me)` | 读取名称 | `const char *` |

## 使用示例

```c
// 派生类（如 module_dji_motor）内部：
// 1. 定义虚表，super 必须为第一个成员
// 2. 构造函数末尾调用 module_motor_init_base(&me->super, &ops, config->name);

// 业务层统一通过基类接口控制
module_motor_enable(&motor.super);
module_motor_set_target(&motor.super, 0.5f);
module_motor_update(&motor.super, 0.001f);

const module_motor_feedback_t *fb = module_motor_get_feedback(&motor.super);
if (fb->is_online) { /* 使用 fb->position_rad 等 */ }
```

## 注意事项

1. **派生类首成员必须是 `super`**：用 `MODULE_MOTOR_STATIC_ASSERT_SUPER_FIRST()` 编译期检查。
2. **反馈在线是使能前提**：没有有效反馈不能使能；反馈超时自动进 FAULT，需显式 `clear_fault` 再 `enable`。
3. **PID 统一**：不再区分位置式/增量式，通过 `module_motor_pid_init/update/reset/get_terms` 使用。
4. **分层约束**：本模块只提供通用电机对象，车型相关的模式切换与目标生成留在项目 App。
