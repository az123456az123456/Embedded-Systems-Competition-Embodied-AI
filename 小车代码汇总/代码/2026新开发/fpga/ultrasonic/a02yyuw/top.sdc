// 50MHz 系统时钟，周期 20ns
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]

// 超声波串口输入是异步信号，已在逻辑里做了两级同步，不需要按普通时序约束
// （这里不必额外写 set_false_path，uart_rx 内部已同步）
