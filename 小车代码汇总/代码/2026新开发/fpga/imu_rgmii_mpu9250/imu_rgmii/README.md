# ACG138：MPU9250 九轴 → FPGA → RGMII → RK3588

这是一个独立测试工程：FPGA 通过 I²C 读取 MPU9250 的加速度、陀螺仪、温度，以及内部 AK8963 的三轴磁场；板载 LED 显示数据状态；RK3588 终端显示实际数值。

适用 ACG138 的 GW5AT-LV60PG484AC1/I0 和板内直连 GMAC1。已完成 RTL 仿真、Yosys GW5A 综合检查、Linux 接收程序测试与 ARM64 静态交叉编译。**尚未用 Gowin 布局布线、生成本工程位流或实板验证。** 原 RGMII 回环工程的上板通过记录不代表本工程已经上板通过。

## 1. 接线

IMU 接到 **P10**，不使用 P7。按丝印确认排针的 1 脚方向，下面的编号不是 FPGA 球位编号。

| MPU9250 模块 | ACG138 排针 | FPGA 球位 | 说明 |
|---|---|---|---|
| VCC（你写的 UCC） | P10-22 | 电源 | 3.3V |
| GND | P10-21 | 地 | 共地 |
| SCL | P10-1 | D16 | 3.3V I²C 时钟 |
| SDA | P10-2 | E16 | 3.3V I²C 数据 |

SCL、SDA 都是开漏输出，需要分别上拉到 **3.3V**。若模块已有上拉电阻可直接使用；没有时各加一个约 4.7kΩ 电阻。不能上拉到 5V。模块必须处于 I²C 模式；若模块引出了 CS，应按模块资料保持为高。程序自动尝试 MPU 地址 0x68、0x69。

这些脚与之前第二组电机测试的 P10-4/5/6/7 不重叠，但会占用之前四组电机规划中的第一组部分信号。以后四电机与 IMU 同时工作时需重新分配。

**加载这个独立工程会替换 FPGA 当前全部逻辑**，包括已占用 P7 的功能、电机或 PCIe 逻辑。它不保留那些功能，也不控制电机使能；独立测试时关闭电机驱动电源。需要同时运行时，应把这里的模块合并进现有顶层并统一引脚、时钟和 RGMII 发送仲裁。

## 2. LED 含义

| 板载灯 | 球位 | 含义 |
|---|---|---|
| D0 / LED0 | E13 | 每秒闪烁一次，说明 FPGA 的 50MHz 采集时钟在运行 |
| D1 / LED1 | E14 | 亮：最近 500ms 内读到有效加速度、陀螺仪数据 |
| D2 / LED2 | D14 | 亮：最近 500ms 内读到有效磁力计数据，且没有溢出 |
| D3 / LED3 | D15 | 亮：I²C/身份检查失败、磁场溢出，或初始化后数据失效 |

正常运行时 D0 闪烁、D1/D2 常亮、D3 熄灭。初始化及重试期间 D3 可能亮。D1/D2 只说明 FPGA 收到了传感器数据；RK3588 是否收到网络帧，要看接收程序的 `Link: RECEIVING`。

## 3. 编译 FPGA

建议将整个文件夹复制到简单英文路径，例如 `D:\imu_rgmii`，避免旧版工具对中文路径处理不一致。

1. Gowin IDE 打开 `imu_rgmii.gprj`。
2. 确认器件为 `GW5AT-LV60PG484AC1/I0`，顶层为 `imu_rgmii_top`。
3. 执行综合、布局布线、生成位流，检查报告中的引脚、电压、时钟和时序。
4. 使用 Gowin Programmer 下载本工程生成的 `impl/pnr/imu_rgmii.fs` 到 SRAM 进行测试。

也可在工程目录中执行 `gw_sh build.tcl`。本目录没有预生成的 `.fs`，不能把旧回环工程的位流当成本工程使用。

RGMII 球位和 ODDR 输出方式沿用资料包中的维护版回环工程：时钟输入 E19、输出 E18，Bank2 为 **1.8V**。仅使用 RK 输出的 125MHz 时钟，不接收 RK 发来的数据内容。传感器采集使用板载 F13 的 50MHz，因此 RK 网络时钟没有启动时，IMU 和 LED 仍可工作。

`imu_rgmii.sdc` 提供两个主时钟和跨时钟域约束，**不构成完整板级 RGMII 时序签核**。上板前还需要在 Gowin 中检查输出建立/保持时间和 mailbox 数据总线相对同步请求的稳定裕量。源数据保持到两级同步应答返回；整个 mailbox 只支持上电/重新配置时共同复位，不支持单独复位一侧。

## 4. 在 RK3588 上运行

前提：RK3588 已使用资料包 `rk3588_acg138/13_FPGA_RGMII` 对应的设备树，GMAC1 为板内直连的 fixed-link 1000/full，输出 125MHz 时钟。参考原包教程，不要直接替换当前 boot 或照搬普通外部 PHY 配置。旧工程使用的 TX delay=0x20、RX delay=0x48 可作起点，新位流仍须重新验证。

把 `build/imu-monitor.aarch64` 和 `software/find-interface.sh` 复制到 RK3588 同一个目录。在板端 root 终端执行：

```sh
chmod +x ./imu-monitor.aarch64
IFACE=$(sh ./find-interface.sh)
test -n "$IFACE" || exit 1
ip link set dev "$IFACE" up
./imu-monitor.aarch64 --interface "$IFACE"
```

先下载 FPGA，再启动网卡；若此前网卡因 FPGA 没有返回时钟而启动失败，需要在加载后重新将该接口 down/up。程序按控制器 `fe1c0000.ethernet` 自动识别 GMAC1，不能默认写死 eth0。`find-interface.sh` 找不到接口时，先检查设备树和驱动。

不需要设置 IP，不通过 ping 测试。传输使用原始以太网帧 `EtherType=0x88B5`，通过板内 RGMII 到 RK3588。程序需要 root 或 CAP_NET_RAW 权限。

程序每约 100ms 刷新终端，显示：

- 加速度 X/Y/Z，单位 g，量程 ±2g。
- 角速度 X/Y/Z，单位 °/s，量程 ±250°/s。
- 磁场 X/Y/Z，单位 μT，16 位模式，应用 ASA 工厂灵敏度修正。
- 芯片温度、器件身份、采样序号、数据年龄、I²C 错误及 FPGA mailbox 丢弃数。

加速度/陀螺仪和磁力计各自配置为约 100Hz，分别发布快照（正常最多约 200 帧/秒，外加每秒状态帧）。磁场与六轴不是同一时刻采样，分别给出序号和年龄。数据超过 500ms 不再作为有效测量显示；超过 2 秒没有新帧显示网络 TIMEOUT。无传感器时仍每秒发送状态，方便区别“传感器异常”和“链路未通”。

额外用法：

```sh
./imu-monitor.aarch64 --once              # 最多等10秒，打印收到的第一帧后退出
./imu-monitor.aarch64 --no-clear          # 保留终端历史，每秒输出
./imu-monitor.aarch64 --decode frame.bin  # 离线解析78字节帧或带FCS的82字节帧
```

九轴输出为各传感器原生坐标轴，**不是姿态角**。本工程没有实现姿态融合、磁场硬铁/软铁校准和安装坐标转换。磁力计尤其容易受电机、电流和磁性材料影响。

## 5. 文件与验证

| 文件 | 作用 |
|---|---|
| `rtl/i2c_reg_master.v` | 开漏 I²C、寄存器读写、连续读取、最终 NACK、每字节命令超时 |
| `rtl/mpu9250_reader.v` | MPU9250/AK8963 初始化、身份检查、ASA、九轴数据与错误重试 |
| `rtl/imu_ethernet_tx.v` | 跨时钟快照、以太网帧、CRC32、帧间隔 |
| `rtl/imu_rgmii_top.v` | FPGA 顶层、时间戳、LED、RGMII ODDR |
| `imu_rgmii.cst/.sdc/.gprj` | 引脚、时钟和 Gowin 工程 |
| `software/imu_monitor.c` | RK3588 接收显示程序源码，无第三方依赖 |
| `build/imu-monitor.aarch64` | 已编译的 ARM64 Linux 静态程序 |
| `tests/` | I²C 从机模型、网络/顶层仿真、C 解析校验 |

Linux 开发电脑上执行：

```sh
make test     # 需要 gcc、Icarus Verilog、Python3
make synth    # 需要支持GW5A的Yosys；输出在build/synthesis.log
make rk3588 CROSS_COMPILE=/你的工具链路径/aarch64-none-linux-gnu-
```

本机检查结果：上述仿真和综合通过；交叉编译产物为 ARM aarch64 静态 ELF，尚未在 RK3588 执行。综合识别出两个 IOBUF 和六个 ODDR；Yosys 对三态语法给出通用警告，实际 I²C 顶层已映射为 IOBUF。

仿真覆盖九轴大小端/符号、初始化寄存器、读结束 NACK、磁场溢出丢弃、磁力计缺失时六轴继续工作、DRDY 未就绪、SCL 卡低超时恢复、错误器件 ID、0x69 地址回退、跨时钟数据保持、停网络时钟、以太网 FCS/帧间隔、LED 数据过期、实际顶层帧字段和接收端单位换算。ODDR 使用功能模型，不能替代器件时序仿真或示波器验证。

I²C 超时会释放总线并重试；如果从机持续拉住 SDA，本实现没有额外的九脉冲 bus-clear，可能需要对传感器重新上电。`WHO_AM_I` 必须为 0x71、AK8963 `WIA` 必须为 0x48；不将其他型号或缺失的磁力计冒认为有效九轴设备。

## 6. 网络数据格式

目的 MAC `ff:ff:ff:ff:ff:ff`，源 MAC `02:ac:13:80:00:01`，类型 `0x88B5`。载荷固定64字节；以太网头14字节，FCS4字节。所有多字节字段均为网络大端，原始测量是16位二补码。FCS 是标准 Ethernet CRC32，低字节先发送；Linux 通常已剥除 FCS。

| 载荷偏移 | 长度 | 内容 |
|---|---|---|
| 0 | 4 | ASCII `IMU9` |
| 4 / 5 | 各1 | 协议版本1 / 型号标记0x92 |
| 6 | 2 | flags：bit0六轴有效，bit1磁场有效，bit2 MPU初始化，bit3 AK初始化，bit4通信/身份故障，bit5磁场溢出 |
| 8 / 12 | 各4 | 六轴采样序号 / 六轴采样时刻ms |
| 16 / 18 / 20 | 各2 | 加速度X/Y/Z |
| 22 | 2 | 温度原始值 |
| 24 / 26 / 28 | 各2 | 陀螺仪X/Y/Z |
| 30 / 32 / 34 | 各2 | 磁力计X/Y/Z，已转换成大端 |
| 36 | 3 | ASAX/ASAY/ASAZ |
| 39 / 40 / 41 | 各1 | MPU ID / AK ID / MPU I²C地址 |
| 42 | 2 | 磁场数据年龄ms，0xFFFF表示未收到或已饱和 |
| 44 / 48 | 各4 | I²C/身份错误累计 / mailbox丢弃累计（不是网络丢包数） |
| 52 | 4 | FPGA运行时间ms |
| 56 / 58 | 各2 | 六轴数据年龄ms / 标称采样率100Hz |
| 60 | 4 | 磁场采样序号 |

## 7. 依据

接线依据工作区 ACG138 硬件手册，RGMII 球位和 DDR 方式依据 `rk3588_acg138/13_FPGA_RGMII/01_FPGA工程/rgmii_l2_loopback` 的维护版源码/CST。

寄存器、身份及灵敏度依据 TDK 官方 [MPU9250 Register Map rev1.6](https://invensense.tdk.com/wp-content/uploads/2017/11/RM-MPU-9250A-00-v1.6.pdf) 和 [MPU9250 Product Specification rev1.1](https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf)。换算：加速度 raw/16384，角速度 raw/131，温度 raw/333.87+21，磁场 raw×0.15×(1+(ASA−128)/256)。
