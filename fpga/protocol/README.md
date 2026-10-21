# FPGA 协议 v2 模块

> 日期：2026-09-12
> 协议定义：`../通信协议.md`
> 仿真状态：✅ 全部通过

## 文件

| 文件 | 作用 |
|---|---|
| `crc16_byte.v` | CRC-16/CCITT-FALSE 单字节组合更新 |
| `protocol_rx.v` | 流式接收、帧头同步、长度检查、CRC 校验 |
| `protocol_tx.v` | ACK/STATUS 等帧的发送封装 |
| `comm_watchdog.v` | 独立通信看门狗 |
| `twist_to_wheel.v` | `(v, w)` 转左右轮目标速度 |
| `tb_protocol_v2.v` | 协议、看门狗、差速换算测试 |

## 协议关键点

```text
AA 55 | CMD | SEQ | LEN | PAYLOAD | CRC_L CRC_H
```

- CRC 初值：`0xFFFF`
- 多项式：`0x1021`
- 标准测试：CRC16(`123456789`) = `0x29B1`
- 所有多字节整数小端序
- 单帧 payload 最大 32 字节

## 仿真

使用项目一键脚本：

```bash
bash fpga/run_all_sim.sh
```

或手动：

```bash
cd fpga/protocol
iverilog -g2001 -o sim \
  crc16_byte.v protocol_rx.v comm_watchdog.v twist_to_wheel.v tb_protocol_v2.v
vvp sim
```

预期：

```text
===== 协议 v2 基础测试全部通过! =====
```

## 使用注意

1. `protocol_rx` 每检测到新的 `AA` 帧头都会重置 CRC、索引和 payload。
2. `comm_watchdog` 收到任意有效下行帧都会 kick。
3. `twist_to_wheel` 中 `w_mrad_s > 0` 表示逆时针左转。
4. `protocol_tx` 依赖 `uart_tx` 在停止位完整结束后才拉低 `tx_busy`。
