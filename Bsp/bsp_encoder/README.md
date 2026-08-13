# bsp_encoder — 增量编码器抽象层

`bsp_encoder` 提供对增量编码器计数器的统一抽象：启动/停止计数、读写当前值、读取旋转方向，以及带模数回绕处理的增量计算。模块内部保存上次计数值以支持增量接口。

**数据流向：** `bsp_encoder_config_t` --> `init/start` --> `get_count/get_delta/get_direction` --> 上层速度计算

## 核心结构体

| 结构体 | 角色 | 关键字段 |
|--------|------|---------|
| `bsp_encoder_t` | 基类（App 持有） | `previous_count`、`counter_modulus` |
| `bsp_encoder_device_t` | 派生设备 | `super`、`driver_ops` |
| `bsp_encoder_driver_ops_t` | 平台驱动表 | `start`/`stop`/`set_count`/`get_count`/`get_direction`/`init`/`deinit` |
| `bsp_encoder_config_t` | 初始化配置 | `device_handle`、`driver_ops`、`counter_modulus` |
| `bsp_encoder_direction_t` | 方向枚举 | STOPPED / FORWARD / REVERSE |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `bsp_encoder_init(me, cfg)` | 初始化设备，配置模数并绑定驱动 | OK / INVALID_ARGUMENT |
| `bsp_encoder_as_base(me)` | 向上转型 | 基类指针或 NULL |
| `bsp_encoder_start(me)` / `bsp_encoder_stop(me)` | 启动/停止计数 | 状态码 |
| `bsp_encoder_reset(me)` | 计数归零并清除历史值 | 状态码 |
| `bsp_encoder_set_count(me, count)` | 设置计数值（不改历史值） | 状态码 |
| `bsp_encoder_get_count(me, &count)` | 读取当前计数值 | 状态码 |
| `bsp_encoder_get_delta(me, &delta)` | 读增量（带模数回绕）并更新历史值 | OK / OUT_OF_RANGE |
| `bsp_encoder_get_direction(me, &dir)` | 读取旋转方向 | 状态码 |

## 使用示例

```c
static bsp_encoder_device_t s_enc;
static bsp_encoder_t *s_enc_ptr;

void board_encoder_init(void) {
    bsp_encoder_config_t cfg = {
        .device_handle = &htim2,
        .driver_ops = &stm32_encoder_driver,
        .counter_modulus = 65536U,   // 16 位定时器
    };
    bsp_encoder_init(&s_enc, &cfg);
    s_enc_ptr = bsp_encoder_as_base(&s_enc);
    bsp_encoder_start(s_enc_ptr);
}

void control_loop(void) {
    int32_t delta;
    if (bsp_encoder_get_delta(s_enc_ptr, &delta) == BSP_STATUS_OK) {
        // 速度 = delta / 采样周期
    }
}
```

## 注意事项

1. **回绕前提**：单个采样周期内真实增量必须小于**半个模数**，否则无法区分回绕与真实大位移。
2. **`counter_modulus` 与硬件一致**：0 表示无回绕（自由运行计数器），否则须等于硬件计数范围（如 16 位为 65536）。
3. **`get_delta` 是有状态接口**：同一实例应只由一个周期任务调用，`set_count`/`reset` 与 `get_delta` 之间需串行化。
4. **速度计算在上层**：模块只提供增量，结合时间的速度换算应由 Module 层完成。
