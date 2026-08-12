# alg_crc -- 通用 CRC 校验算法

## 功能概述

纯算法模块，提供与硬件无关的通用 CRC 计算。通过单一接口 `alg_crc_calculate` 配合 `alg_crc_config_t` 参数化所有 CRC 变体，支持 CRC-8、CRC-16、CRC-32，以及 MSB-first / LSB-first 两种位序。预置三种常用配置，可直接使用或自定义。

**不依赖**：HAL、CMSIS、RTOS、硬件 CRC 外设。

## 核心结构体

### `alg_crc_bit_order_t` -- 位序枚举

| 枚举值 | 含义 |
|--------|------|
| `ALG_CRC_BIT_ORDER_MSB_FIRST` | 高位在前（先处理 MSB） |
| `ALG_CRC_BIT_ORDER_LSB_FIRST` | 低位在前（先处理 LSB） |

### `alg_crc_config_t` -- CRC 参数配置

| 字段 | 类型 | 含义 | 约束 |
|------|------|------|------|
| `width` | `uint8_t` | CRC 位宽 | 仅支持 8、16 或 32 |
| `polynomial` | `uint32_t` | 生成多项式 | LSB-first 时填写反射后的值 |
| `initial_value` | `uint32_t` | 初始寄存器值 | -- |
| `output_xor` | `uint32_t` | 输出异或掩码 | -- |
| `bit_order` | `alg_crc_bit_order_t` | 位处理顺序 | MSB-first 或 LSB-first |

### 预置配置

| 常量名 | 位宽 | 多项式 | 初值 | 位序 | 典型用途 |
|--------|------|--------|------|------|----------|
| `alg_crc8_0x8c_ff_config` | 8 | 0x8C | 0xFF | LSB-first | 视觉 / UART 协议 |
| `alg_crc16_ccitt_false_config` | 16 | 0x1021 | 0xFFFF | MSB-first | Modbus、CCITT-FALSE |
| `alg_crc16_0x8408_ff_config` | 16 | 0x8408（反射） | 0xFFFF | LSB-first | CRC-16/MODBUS 等价 |

## 数学原理

### CRC 计算流程

**MSB-first**：

```
1. crc = initial_value
2. 对每个字节 data[i]:
     crc ^= (data[i] << (width - 8))
     对每个 bit（共 8 位）:
         if (crc MSB == 1): crc = (crc << 1) ^ polynomial
         else:             crc = crc << 1
         保留低 width 位
3. crc ^= output_xor
```

**LSB-first**：

```
1. crc = initial_value
2. 对每个字节 data[i]:
     crc ^= data[i]
     对每个 bit（共 8 位）:
         if (crc LSB == 1): crc = (crc >> 1) ^ polynomial
         else:             crc = crc >> 1
         保留低 width 位
3. crc ^= output_xor
```

### LSB-first 多项式反射约定

对于 LSB-first 模式，`polynomial` 应当填写反射后的值。例如 CRC-16/MODBUS 的多项式名义值为 0x1021，反射后为 0x8408，因此 LSB-first 配置中 `polynomial = 0x8408`。

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `alg_crc_calculate(config, data, data_size, &result)` | 计算 CRC 校验值 | `true`=成功，`false`=参数无效 |

> **注意**：该模块只有一个公共函数，无 init/reset 操作（每次计算独立，无内部状态）。

## 使用示例

### 使用预置配置

```c
#include "alg_crc.h"

uint8_t packet[] = {0x01, 0x02, 0x03, 0x04};
uint32_t crc_result;

// CRC-8：视觉协议校验
alg_crc_calculate(&alg_crc8_0x8c_ff_config, packet, sizeof(packet), &crc_result);
uint8_t crc8 = (uint8_t)crc_result;

// CRC-16：Modbus 协议校验
alg_crc_calculate(&alg_crc16_ccitt_false_config, packet, sizeof(packet), &crc_result);
uint16_t crc16 = (uint16_t)crc_result;
```

### 自定义 CRC 配置

```c
// 自定义 CRC-32（Ethernet CRC32）
alg_crc_config_t my_crc32 = {
    .width = 32,
    .polynomial = 0x04C11DB7,
    .initial_value = 0xFFFFFFFF,
    .output_xor = 0xFFFFFFFF,
    .bit_order = ALG_CRC_BIT_ORDER_MSB_FIRST,
};

uint32_t result;
alg_crc_calculate(&my_crc32, data, length, &result);
```

### 在线追加计算（需自行维护状态）

CRC 模块本身是无状态的，每次 `alg_crc_calculate` 独立执行。如需对数据流分段计算，调用者自行维护并传入上一个块的 CRC 作为 `initial_value`，并在最后一段加上 `output_xor`。

## 注意事项

1. **位宽限制**：当前仅支持 8、16、32 位位宽，传入其他值返回 `false`。
2. **多项式反射**：LSB-first 模式下必须传入反射后的多项式值，否则计算结果与标准不匹配。
3. **性能**：纯软件逐位计算，适合配置帧等短数据包。若用于高速数据流的 CRC-32 校验，建议使用硬件 CRC 外设。
4. **参数有效性**：`data` 和 `result` 为 NULL 时返回 `false`。
