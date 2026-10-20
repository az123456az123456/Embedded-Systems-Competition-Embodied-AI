# ACG138：双目摄像头、IMU与FMC四电机测试 v3

## ⚡ 2026-10-16 帧率优化 v4

摄像头 UDP 发包后等待从 256 个时钟改为 16 个；仿真实际包间空闲由 260 缩短为 20 个字节时钟，仍满足至少 12 个字节的间隔。行拼接 FIFO 读使能采用下一状态预测后寄存输出，逐周期对比保持原行数据、错行恢复和配对语义，缩短到厂商 FIFO 存储器时钟使能的路径。相机复位/Init_Done 采用各本地时钟域同步释放；FIFO 首级指针同步路径保留明确 8ns 上限，未全局切断时钟域。

运行 `gw_sh build_fps.tcl` 生成 `impl/pnr/*_fps.fs`，原名称的旧位流保留。最终三个工程均为 0 setup / 0 hold 违例；逐脚核对已与 CST 一致，详见 `reports/fps/timing_summary.json` 和 `build.json`。这只证明已声明约束下的内部时序，外部相机/RGMII I/O 仍需实板验证。当前传感器名义目标仍约 15FPS；缩短发包间隔增加余量，不代表实际帧率自动成倍增加。

建议先更新配套 RK3588 `camera-fastpath` 接收端和 `perception-dashboard`，测量完整 RAW 接收、ISP 输出、CAP 和 AI 的速率，再替换对应功能的 FPGA 位流。图像仍为双目 960+960×1080 RAW8，UDP 6102/1922 字节行报文、IMU 接口和电机模式保持原工程语义。

以下旧版说明用于原接线与功能参照；本次构建以 `reports/fps/` 为准。


> 2026-10-13 · 顶层robot_fmc_top · GW5AT-LV60PG484AC1/I0

打开robot_fmc.gprj或执行gw_sh build.tcl，共58个物理信号脚。保留双OV5640 1920×1080左右合成、MPU9250九轴和LED、原RGMII/UDP接收协议。

继续使用v2修正的四电机球位，依据FMCtoGPIO-241118.pdf的FMC接点与GPIO排针，结合本机ACG138原理图得到。保留v1推荐的P3针号，替换错误的FPGA球位。完整接线和诊断步骤见压缩包中fmc_four_test/README.md，表格见ACG138_四电机_IMU_引脚规划.xlsx。

IMU仍用P10-1/2。四电机依次及同时正反转，60%最大PWM带加减速，M3/M4方向取反、四路PWM相移12.5μs；下载约3秒后自动开始，约92秒后PWM保持0，STBY由外部保持高。本版本用于台架测试，不是RK遥控或闭环运动控制。测试时车轮悬空。

合并版本D0～D3仍显示IMU状态，不代表电机测试状态；排查电机建议先烧录独立电机工程。VCCADJ及驱动VCC/VM/STBY仍需实测，转接板11脚为5V、29脚为3.3V。

编译及剩余摄像头时序限制见BUILD_STATUS.md。原摄像头/IMU工程未修改。

v3不保证解决四轮持续卡顿；电源电流与负载VM电压仍待实测。完整新波形测试在fmc_four_test/tests中。
