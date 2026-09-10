顶层模块：XR_uart
  ├── XR_uart_clkdiv  → 产生波特率9600的16倍频时钟（153600Hz）
  ├── XR_uart_RX      → 主串口接收（USB）
  ├── XR_uart_TX      → 主串口发送（USB）
  ├── XR_uart_RX0     → 串口0接收（带init_flag控制）
  ├── XR_uart_TX0     → 串口0发送（带send_flag控制）
  ├── XR_uart_RX1     → 串口1接收
  ├── XR_uart_TX1     → 串口1发送
  └── XR_uart_ctrl    → 发送控制状态机（字符串发送/数据上传）
