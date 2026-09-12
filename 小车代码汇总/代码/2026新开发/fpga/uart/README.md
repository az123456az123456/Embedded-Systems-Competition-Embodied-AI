# UART 通信模块（已仿真通过 ✅）

| 文件 | 说明 |
|---|---|
| `uart_tx.v` | UART 发送（8N1，可配置波特率）|
| `uart_rx.v` | UART 接收（8N1，16倍过采样）|
| `tb_uart.v` | 回环测试（50MHz/115200，4 字节验证）|

## 仿真
```bash
cd 开发/fpga/uart
source ../../工具/setup_env.sh
iverilog -o sim uart_tx.v uart_rx.v tb_uart.v
vvp sim
# 查看波形
gtkwave uart_wave.vcd
```

## 接口
- TX: `tx_start`(脉冲) + `tx_data[7:0]` → `tx`(串行), `tx_busy`
- RX: `rx`(串行) → `rx_valid`(脉冲) + `rx_data[7:0]`

## 踩坑记录
- 过采样相位计数器 `s_cnt` 必须 4 位（0~15），5 位会变成 32 拍=2 比特周期，导致隔位采样出错
- 时钟频率需满足 CLK/BAUD ≥ 16（否则 16 倍过采样分频为 0）

## 下一步
- 接入命令解析：按 FPGA↔RK3588 协议解析指令
- 多字节打包发送
