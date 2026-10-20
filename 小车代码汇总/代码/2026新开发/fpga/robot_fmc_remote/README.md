# ACG138 四电机远控 + 双目摄像头 + MPU9250

## ⚡ 2026-10-16 帧率优化 v4

摄像头 UDP 发包后等待从 256 个时钟改为 16 个；仿真实际包间空闲由 260 缩短为 20 个字节时钟，仍满足至少 12 个字节的间隔。行拼接 FIFO 读使能采用下一状态预测后寄存输出，逐周期对比保持原行数据、错行恢复和配对语义，缩短到厂商 FIFO 存储器时钟使能的路径。相机复位/Init_Done 采用各本地时钟域同步释放；FIFO 首级指针同步路径保留明确 8ns 上限，未全局切断时钟域。

运行 `gw_sh build_fps.tcl` 生成 `impl/pnr/*_fps.fs`，原名称的旧位流保留。最终三个工程均为 0 setup / 0 hold 违例；逐脚核对已与 CST 一致，详见 `reports/fps/timing_summary.json` 和 `build.json`。这只证明已声明约束下的内部时序，外部相机/RGMII I/O 仍需实板验证。当前传感器名义目标仍约 15FPS；缩短发包间隔增加余量，不代表实际帧率自动成倍增加。

建议先更新配套 RK3588 `camera-fastpath` 接收端和 `perception-dashboard`，测量完整 RAW 接收、ISP 输出、CAP 和 AI 的速率，再替换对应功能的 FPGA 位流。图像仍为双目 960+960×1080 RAW8，UDP 6102/1922 字节行报文、IMU 接口和电机模式保持原工程语义。

以下旧版说明用于原接线与功能参照；本次构建以 `reports/fps/` 为准。


> 日期：2026-10-15
> 新工程：`robot_fmc_remote.gprj`；独立台架工程：`fmc_remote_motor_only.gprj`。
> 原摄像头、IMU、电机测试工程保持不变。此工程不会上电自动转动。

## 🔌 用途和接线

手机通过网络连接 RK3588，由 RK 发送左右 PWM 指令到板内 GMAC1/RGMII，再由 FPGA 驱动 TB6612。视频和 MPU9250 仍沿原 RGMII 发送链路到 RK；不新增手机直连 FPGA 的接线。

电机位置由用户确认：M1 左前、M2 左后、M3 右前、M4 右后。主机初始配置发送 `left_mask=0x03`、`invert_mask=0x0c`，补偿 M3/M4 电气方向相反。

| 电机 | PWM（P3 / FPGA） | IN1（P3 / FPGA） | IN2（P3 / FPGA） |
|---|---|---|---|
| M1 左前 | 1 / J15 | 2 / H15 | 5 / Y21 |
| M2 左后 | 6 / Y22 | 7 / AB21 | 9 / AA19 |
| M3 右前 | 10 / AB20 | 28 / M16 | 31 / K21 |
| M4 右后 | 16 / G20 | 19 / G17 | 39 / N18 |

MPU9250 保持 P10-1/D16 SCL、P10-2/E16 SDA、P10-22/3.3V、P10-21/GND。双目摄像头和相机诊断串口引脚保留原工程。相机的两份包装源增加各时钟域复位同步释放和 Init_Done 同步，图像格式、行拼接及网络数据协议保留。

TB6612：VM 使用外部 12V；逻辑 VCC 确保实际有 3.3V；STBY 由外部保持 3.3V。两套独立电源须共地：P3-30/GND 连接两块驱动 GND，电机功率回流仍由驱动 GND 直接回外部 12V 负极。P3-29 为 3.3V 供电针，使用前测量；板载稳压输出不可直接与另一路输出并接。FMC 信号 Bank 的 VCCADJ 必须实测符合 3.3V。

板内 RGMII 增加接收侧，1.8V Bank2：RX_CLK E19、RXD[0:3] F20/D20/A18/A19、RX_CTL C20；发送侧保持 E18/F19/B20/A20/C19/F18。无需额外飞线，不能将这组信号接 3.3V。

## 🎮 控制行为

- 20kHz PWM，四个载波相隔四分之一周期；左右归一化 PWM 的允许输入范围为 -1000..1000，实物输出限制为 ±600，即最高 60%。没有编码器测速，反馈表示实际 PWM 指令，不是实测速度。
- 每 10ms 增减 10 permille（1%），从零到 60% 约 0.6s。任一侧给 0、解除使能或看门狗到期，立即将对应 PWM 归零，不等待减速斜坡。
- 方向反转先 PWM=0，等待至少 1ms，再切方向并重新加速。
- 上电、断链超时后，必须先接收有效的 unarmed 且左右为 0 的 STOP 握手，才接受 armed 运动命令。运行时改变左右分组或方向反相配置必须先停止。
- 仅新序号且语义正确的有效电机命令刷新独立 500ms 看门狗；坏帧、重复帧、无关帧不刷新。即使 RK 的 RGMII 接收时钟停止，50MHz 电机时钟也会独立停车。
- FPGA 未收到对应命令确认、手机命令过期、目标丢失、IMU 失效等主机侧条件，由 RK 软件单独处理；FPGA 看门狗提供最终断链停车。

## 📡 MOT1 / ACK1 协议

使用独立 EtherType `0x88B6`。目的 FPGA MAC 为 `02:ac:13:80:00:01`，RK 发送源 MAC 使用其真实 GMAC1 地址。FPGA ACK 发到广播 MAC，源 MAC 为上述 FPGA MAC。无需为板内链路设置 IP；RK 需 root 或原始网络套接字权限。

| Payload 偏移 | 长度 | MOT1 命令 | ACK1 状态 |
|---|---|---|---|
| 0 | 4 | ASCII `MOT1` | ASCII `ACK1` |
| 4 | 1 | 版本 1 | 版本 1 |
| 5 | 1 | bit0 armed，其余 0 | bit0 armed、bit1 watchdog、bit2 configured，其余 0 |
| 6 | 4 | uint32 BE 序号 | 最近接受的命令序号 |
| 10 | 2 | int16 BE 左侧 permille | 加减速后的左侧实际 PWM permille |
| 12 | 2 | int16 BE 右侧 permille | 加减速后的右侧实际 PWM permille |
| 14 | 1 | 左侧电机掩码，1..14 | 当前配置回显 |
| 15 | 1 | 方向反相掩码，0..15 | 当前配置回显 |
| 16 | 2 | 前 16 字节 CRC16 BE | 前 16 字节 CRC16 BE |

CRC16 为 CCITT-FALSE，初值 `0xffff`，多项式 `0x1021`。Ethernet 14 字节头 + 18 字节 payload 后填充 28 个零，到 60 字节（不含 FCS）；网卡追加 4 字节 Ethernet FCS。FPGA 严格接收 64 字节含 FCS 的最小帧，检查目的 MAC、EtherType、版本、字段、零 padding、CRC16 和完整 FCS，全部正确才发布命令。不要添加 VLAN 标签。

新序号判定为 32 位差值非零且小于 `2^31`，支持 `0xffffffff→0` 回卷。超时清除序号会话，允许新主机用任意序号 STOP 重新握手。ACK 在新命令后及每 100ms 发布；RK 必须验证新鲜 ACK 和序号回显，才能确认载入了正确位流。ACK 的 PWM 数值没有轮速测量含义。

本协议独立于原冻结 UART v2；原 `SET_TWIST(mm/s,mrad/s)` 未被改义。

## 🧪 检查与编译

`make test` 运行故障注入和控制仿真，`make test-camera` 运行复位、真实三路报文和1080行拼接回归，`make test-real-clock` 验证实际50MHz/20kHz；`make test-gowin GOWIN_IDE=/实际高云安装目录/IDE` 使用高云官方 IDDR 功能模型验证 DDR 半字节顺序。

完整工程在高云 IDE 打开 `robot_fmc_remote.gprj`，或使用 `gw_sh build.tcl`。独立电机工程使用 `fmc_remote_motor_only.gprj` / `gw_sh build_motor_only.tcl`。命令行构建建议复制到纯 ASCII 临时路径，避免本机高云工具的中文路径问题。

完整位流保留双目和 IMU；**独立电机台架位流不包含摄像头和 IMU**。后者 LED D0 心跳、D1 已配置、D2 已使能、D3 看门狗。完整工程 LED 保持原 IMU 状态含义。

上板检查先悬空车轮，确认 STOP、正反转、左右转向和断网停车，再进行地面测试。软件仿真、编译和位流生成不代替实板验证。详细编译和时序结果见 `BUILD_STATUS.md`。
