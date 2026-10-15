# ACG138：双目摄像头、IMU与FMC四电机测试 v3

> 2026-10-13 · 顶层robot_fmc_top · GW5AT-LV60PG484AC1/I0

打开robot_fmc.gprj或执行gw_sh build.tcl，共58个物理信号脚。保留双OV5640 1920×1080左右合成、MPU9250九轴和LED、原RGMII/UDP接收协议。

继续使用v2修正的四电机球位，依据FMCtoGPIO-241118.pdf的FMC接点与GPIO排针，结合本机ACG138原理图得到。保留v1推荐的P3针号，替换错误的FPGA球位。完整接线和诊断步骤见压缩包中fmc_four_test/README.md，表格见ACG138_四电机_IMU_引脚规划.xlsx。

IMU仍用P10-1/2。四电机依次及同时正反转，60%最大PWM带加减速，M3/M4方向取反、四路PWM相移12.5μs；下载约3秒后自动开始，约92秒后PWM保持0，STBY由外部保持高。本版本用于台架测试，不是RK遥控或闭环运动控制。测试时车轮悬空。

合并版本D0～D3仍显示IMU状态，不代表电机测试状态；排查电机建议先烧录独立电机工程。VCCADJ及驱动VCC/VM/STBY仍需实测，转接板11脚为5V、29脚为3.3V。

编译及剩余摄像头时序限制见BUILD_STATUS.md。原摄像头/IMU工程未修改。

v3不保证解决四轮持续卡顿；电源电流与负载VM电压仍待实测。完整新波形测试在fmc_four_test/tests中。
