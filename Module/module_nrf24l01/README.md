# module_nrf24l01 — nRF24L01+ 无线收发

`module_nrf24l01` 是基于 SPI 的 nRF24L01(+) 2.4GHz 收发器驱动，支持地址、管道、速率、功率、自动应答、自动重发、发送轮询和接收。

**数据流向：** `bsp_spi + CE/CSN GPIO + delay_us` --> `init/start` --> `transmit/poll_transmit` / `receive` --> 载荷

## 核心结构体

| 结构体 | 角色 | 关键字段 |
|--------|------|---------|
| `module_nrf24l01_t` | 设备对象 | SPI/CE/CSN、channel、address_size、payload_size、mode、transmit_pending |
| `module_nrf24l01_config_t` | 初始化配置 | spi、CE/CSN、channel、link_address、payload_size、data_rate、output_power、自动应答/重发、delay_us |
| `module_nrf24l01_status_t` | 状态码 | OK / NO_DATA / BUSY / MAXIMUM_RETRANSMIT / ... |
| `module_nrf24l01_data_rate_t` | 数据率 | 1M / 2M / 250k |
| `module_nrf24l01_output_power_t` | 输出功率 | -18 / -12 / -6 / 0 dBm |
| `module_nrf24l01_mode_t` | 工作模式 | STANDBY / RECEIVE / TRANSMIT |

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `module_nrf24l01_init(me, cfg)` | 初始化设备 | 状态码 |
| `module_nrf24l01_start(me)` | 配置寄存器、验证 Chip ID、清 FIFO | 状态码 |
| `module_nrf24l01_stop(me)` | 关闭 CE、进入待机 | 状态码 |
| `module_nrf24l01_set_receive_address(me, pipe, addr, n)` | 设置接收地址 | 状态码 |
| `module_nrf24l01_set_receive_pipe_enabled(me, pipe, en)` | 启用/禁用接收管道 | 状态码 |
| `module_nrf24l01_set_transmit_address(me, addr, n)` | 设置发送地址 | 状态码 |
| `module_nrf24l01_start_receive(me)` | 启动接收模式 | 状态码 |
| `module_nrf24l01_transmit(me, payload, n)` | 发送数据 | 状态码 |
| `module_nrf24l01_poll_transmit(me)` | 轮询发送状态 | OK / BUSY / MAXIMUM_RETRANSMIT |
| `module_nrf24l01_receive(me, payload, cap, &pipe)` | 接收数据 | OK / NO_DATA |
| `module_nrf24l01_get_observe_transmit(me, &lost, &retx)` | 读取丢包/重发计数 | 状态码 |
| `module_nrf24l01_flush_transmit(me)` / `flush_receive(me)` | 清空收发 FIFO | 状态码 |

## 使用示例

```c
module_nrf24l01_t radio;
module_nrf24l01_config_t cfg = {
    .spi = spi_ptr,
    .chip_enable_gpio = &ce_pin,
    .chip_select_gpio = &csn_pin,
    .channel = 100, .address_size = 3, .payload_size = 16,
    .link_address = ace_link_address,
    .data_rate = MODULE_NRF24L01_DATA_RATE_2_MBPS,
    .automatic_acknowledge_enabled = true,
    .delay_us = dwt_delay_us,
};
module_nrf24l01_init(&radio, &cfg);
module_nrf24l01_start(&radio);

// 发送（轮询到非 BUSY）
module_nrf24l01_transmit(&radio, payload, 16);
module_nrf24l01_status_t rc;
do { rc = module_nrf24l01_poll_transmit(&radio); }
while (rc == MODULE_NRF24L01_STATUS_BUSY);

// 接收
uint8_t rx[32]; uint8_t pipe;
rc = module_nrf24l01_receive(&radio, rx, sizeof(rx), &pipe);
```

## 注意事项

1. **发送需轮询**：`transmit()` 启动发送后，需周期性调用 `poll_transmit()` 直到返回非 `BUSY`。
2. **固定载荷**：`payload_size` 收发双方必须一致（1~32 字节）。
3. **`delay_us` 回调**：用于 CE 脉冲和上电等待，需注入平台微秒延时。
4. **双方共用链路地址**：`link_address` 长度为 `address_size`，收发双方保持一致。
