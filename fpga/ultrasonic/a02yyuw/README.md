# A02YYUW 三路超声波与电机控制接口

本模块接收安装在小车前方、左侧、右侧的三个 A02YYUW UART 自动输出传感器，为电机控制提供每个方向的障碍标志和距离。接入入口为 `a02yyuw_obstacle_if`；单独上板调试的顶层为 `top`。

## 1 接线与方向

| 方位 | 通道 | 模块 VCC | 模块 GND | 模块 TX | FPGA 引脚 |
|---|---|---|---|---|---|
| 前方 | 0 | P3.29，3.3V | P2.12 | P2.1 | Y18 |
| 左侧 | 1 | P10.24，3.3V | P10.23 | P2.3 | N20 |
| 右侧 | 2 | P2.29，3.3V | P2.30 | P2.5 | U20 |

P2、P3 属于 FMCtoGPIO 转接板，P10 属于 ACG138 主板。三个模块的 RX 功能引线均悬空并绝缘，选择处理值自动输出模式，手册给出的响应时间为 100～500ms。TX 是传感器输出、FPGA 输入，三根 TX 不得并接。

针脚按实物 Pin1 标记和模块功能标识确认，不能将原理图当成俯视接线图。FMC Bank5/8 随 VCCADJ 供电；`top.cst` 的 LVCMOS33 约束要求实际 Bank 电压适配 3.3V，并不会改变供电电压。VCC/GND 是硬件电源网络，不是 HDL 端口。

接线表依据项目外的 ACG138 硬件手册第30～32页和 FMCtoGPIO 原理图第1～2页。此前转接板已报告烧毁，软件修改不代表故障排除；硬件排查完成前不得给损坏板重新上电。

## 2 给电机控制的接口

所有输出与 `clk` 同步；`rst_n` 为低有效同步复位，至少跨过一个时钟上升沿。默认系统时钟 50MHz。三个传感器分别接 `us_rx_front`、`us_rx_left`、`us_rx_right`。

| 输出 | 宽度 | 含义 |
|---|---|---|
| `front_obstacle` | 1 | 前方已确认的障碍标志 |
| `left_obstacle` | 1 | 左侧已确认的障碍标志 |
| `right_obstacle` | 1 | 右侧已确认的障碍标志 |
| `front_distance_mm` | 16 | 前方最近一次有效距离，毫米 |
| `left_distance_mm` | 16 | 左侧最近一次有效距离，毫米 |
| `right_distance_mm` | 16 | 右侧最近一次有效距离，毫米 |
| `front_data_valid` | 1 | 前方距离当前可用，持续电平 |
| `left_data_valid` | 1 | 左侧距离当前可用，持续电平 |
| `right_data_valid` | 1 | 右侧距离当前可用，持续电平 |
| `distance_updated` | 3 | 每路接受新距离时拉高一个时钟周期 |
| `sensor_fault` | 3 | 每路数据无效标志，1表示上电未就绪、失联或量程异常 |
| `forward_stop_req` | 1 | 前方停车阈值触发，或任一路传感器故障 |
| `forward_slow_req` | 1 | 前方减速阈值触发，或已有前进停车请求 |
| `left_blocked` | 1 | 左侧停车阈值触发，或左传感器故障 |
| `right_blocked` | 1 | 右侧停车阈值触发，或右传感器故障 |

三位向量固定为 bit0=前、bit1=左、bit2=右。同一时刻可有多个方向检测到障碍，因此没有互斥的单一方向编码。

**电机控制必须同时检查数据有效状态。** 上电时距离寄存器为0；故障后保留最近一次有效距离作诊断。数据无效时，三个 `*_obstacle` 输出各自被有效状态屏蔽为0，但这不表示该方向安全；`sensor_fault` 指示未知，且会使 `forward_stop_req` 拉高。`distance_updated` 是新样本脉冲，不能代替持续有效状态。

## 3 判断阈值和故障行为

| 参数 | 默认值 | 用途 |
|---|---|---|
| `CLK_FREQ` | 50000000 | 系统时钟频率 Hz |
| `BAUD` | 9600 | 三路传感器 UART 波特率 |
| `DATA_TIMEOUT_MS` | 1000 | 距最后有效样本的失联期限 ms |
| `OBSTACLE_MM` | 1200 | 已确认障碍进入阈值 mm |
| `SLOW_MM` | 800 | 减速进入阈值 mm |
| `STOP_MM` | 300 | 停车进入阈值 mm |
| `HYST_MM` | 50 | 阈值释放滞回 mm |

有效样本 <= 阈值时置位，>= 阈值+50mm 时清零，中间区域保持上一状态。例如障碍标志在1200mm进入、1250mm释放。三路默认使用相同阈值；这些是工程默认值，需要按车速、安装位置、制动距离及传感器响应时间调整。

协议为9600 8N1，字节序列 `FF H L SUM`，`SUM=(FF+H+L)&FF`；距离为 `{H,L}`，单位毫米。默认接受30～4500mm。帧间字节间隔超时20ms后丢弃半帧。错误校验和错误停止位不更新距离，也不刷新失联计时；校验正确但量程外立即标为故障。恢复有效帧后按阈值和滞回重新更新状态。

任一路故障时，前进停车请求优先；只有侧面接近而三路数据均有效时，不会直接触发前进停车请求。左右受限标志不是允许转弯或旋转的许可。车宽、转弯扫掠空间、倒车以及电机闭环仍由运动控制负责。

三个探头自动发射，驱动只能独立接收，无法安排声波发射顺序。实际安装需验证串扰；有串扰时，应选用支持受控触发的型号并另行实现发射轮询。

## 4 在机器人顶层实例化

```verilog
wire front_obs, left_obs, right_obs;
wire [15:0] front_mm, left_mm, right_mm;
wire front_valid, left_valid, right_valid;
wire [2:0] distance_updated, sensor_fault;
wire forward_stop, forward_slow, left_blocked, right_blocked;

a02yyuw_obstacle_if #(.CLK_FREQ(50_000_000)) u_obstacle (
    .clk(clk), .rst_n(rst_n),
    .us_rx_front(us_rx_front),
    .us_rx_left(us_rx_left),
    .us_rx_right(us_rx_right),
    .front_obstacle(front_obs),
    .left_obstacle(left_obs),
    .right_obstacle(right_obs),
    .front_distance_mm(front_mm),
    .left_distance_mm(left_mm),
    .right_distance_mm(right_mm),
    .front_data_valid(front_valid),
    .left_data_valid(left_valid),
    .right_data_valid(right_valid),
    .distance_updated(distance_updated),
    .sensor_fault(sensor_fault),
    .forward_stop_req(forward_stop),
    .forward_slow_req(forward_slow),
    .left_blocked(left_blocked),
    .right_blocked(right_blocked)
);
```

机器人顶层需声明三个接收输入。综合至少加入：`a02yyuw_obstacle_if.v`、`ultrasonic_3ch.v`、`ultrasonic_driver.v`、`ultrasonic_sensor.v`、`a02_uart_rx.v`。这些UART模块带独立前缀，不与机器人通信串口的 `uart_rx`/`uart_tx` 重名。

板级调试的 `top.cst` 使用 `us_rx[0:2]` 端口名；若机器人顶层改用上述命名，合并约束时应使用：

```text
IO_LOC "us_rx_front" Y18;
IO_PORT "us_rx_front" IO_TYPE=LVCMOS33 PULL_MODE=UP;
IO_LOC "us_rx_left" N20;
IO_PORT "us_rx_left" IO_TYPE=LVCMOS33 PULL_MODE=UP;
IO_LOC "us_rx_right" U20;
IO_PORT "us_rx_right" IO_TYPE=LVCMOS33 PULL_MODE=UP;
```

合并到电机控制时，不要添加仅作演示的 `top.v` 和整份演示板约束，也不要把测试台加入综合。旧 `top_robot.v`、`top_robot_v2.v` 仍使用 HC-SR04 接口；本次交付是独立可复用接口，没有修改机器人运动控制和RK通信协议。迁移旧顶层时，需要替换旧 `ultrasonic` 实例、移除 `echo/trig`，并显式处理毫米到旧厘米状态字段的换算，不能直接连入旧 `distance_cm`。

## 5 单独上板调试

Gowin选择实际JTAG识别的芯片；当前硬件手册为GW5AT-LV60PG484A。顶层选择 `top`，加入 `top.v`、`a02_uart_rx.v`、`a02_uart_tx.v`、`ultrasonic_driver.v`、`ultrasonic_sensor.v`、`ultrasonic_3ch.v`，以及 `top.cst`、`top.sdc`。

J2 FPGA虚拟串口为115200 8N1，每100ms输出前、左、右三行快照：

```text
F:07A1 0
L:0258 3
R:00C8 7
```

四位十六进制数为毫米距离；例中前方1953mm、左侧600mm、右侧200mm。末尾状态是原始每路状态的十六进制值：bit3故障、bit2停车、bit1减速、bit0障碍。故障时状态可能为F，距离可能是旧值，不能单看距离作决策。原始内部障碍位在故障时保守置位，与电机接口中“已确认障碍”的屏蔽行为不同。

LED D0心跳；D1前在线；D2左在线；D3右在线；D4任一原始障碍；D5任一原始减速；D6任一原始停车；D7任一故障。演示顶层只作调试，电机接口输出需由机器人顶层实际调用。

## 6 文件与验证

| 文件 | 职责 |
|---|---|
| `a02yyuw_obstacle_if.v` | 电机控制接口，输出各方向障碍、距离、有效状态 |
| `ultrasonic_3ch.v` | 三路独立驱动、方位映射和辅助请求 |
| `ultrasonic_driver.v` | 单路UART与解析器组合 |
| `ultrasonic_sensor.v` | 字节流解析、量程、失联及阈值处理 |
| `a02_uart_rx.v` / `a02_uart_tx.v` | 独立命名的UART接收/调试发送 |
| `top.v` / `top.cst` / `top.sdc` | 板级演示和约束 |
| `tb_ultrasonic.v` | UART、协议、量程、滞回、超时回归 |
| `tb_ultrasonic_3ch.v` | 并发、方向映射、故障隔离、电机接口和调试串口验证 |
| `run_tests.sh` | 使用临时目录编译并运行测试，不保留仿真产物 |

在本目录执行 `bash run_tests.sh`。需要Icarus Verilog的 `iverilog`、`vvp`，可以通过 `IVERILOG`、`VVP` 指定程序路径；安装Verilator时同时执行静态检查，未安装时明确提示跳过。

已验证：手册1953mm样例、错误校验、错误停止位、量程上下界、滞回、失联和恢复；三路同时收帧且无数据互串；前方/左右障碍分别输出；侧面故障与已确认障碍分离；前进辅助请求；调试串口F/L/R标签。测试用较低的时钟参数缩短运行时间，生产默认时钟为50MHz。

验证范围不包含Gowin布局布线、实物电压、供电能力、声学串扰、制动距离和真实电机运动。
