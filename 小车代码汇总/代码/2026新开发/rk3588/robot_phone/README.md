# ACG138 安卓小车遥控：APK、RK3588 与 FPGA

> 日期：2026-10-15。安卓原生 APK；RK3588 Buildroot 原生 ARM64 服务；FPGA 四电机远控。硬件联调待完成。

## 📦 哪些文件放哪里

| 交付文件 | 放置位置／用途 |
|---|---|
| `ACG138-Robot-Control-debug.apk` | 复制到 Android 手机安装，最低 Android 8.0 |
| `robot-phone-rk3588-20261015.tar.gz` | 复制到 RK3588 的 `/root/`，按下方命令解压 |
| `robot_fmc_remote.fs` | 用电脑上的高云下载工具烧录 FPGA，包含双目摄像头、MPU9250、四电机远控 |
| `ACG138_FPGA_四电机远控_双目_IMU_20261015.zip` | FPGA 工程和验证报告，供修改、重新综合；不需要复制到手机 |
| `ACG138手机遥控_源码_20261015.zip` | Android、RK3588、FPGA 源码；不需要在板端编译 |

**旧的 `fmc_four_motor.fs` 是自动测试程序，不能接收手机控制，必须更换。** 独立电机版 `fmc_remote_motor_only.fs` 只供排查电机通信，不能同时提供摄像头和 IMU，不能代替完整版。

## 📡 无线连接

当前按你实际接线使用 **USB 无线网卡 → RK3588 USB 主机口**。应用通过普通局域网连接板子，无需绑定网卡型号，也无需更改 APK 或 FPGA 位流。USB 网卡是否可用取决于实际芯片及当前 Buildroot 内核的驱动、固件，先用诊断脚本确认设备和无线接口。

先让 USB 无线网卡和手机接入同一个无客户端隔离的 2.4GHz 路由器，也可以尝试板子连接 Android 手机热点；部分热点会限制宿主与客户端互访。这里不要求板子开热点。

完整步骤见 [USB 无线网卡联网说明](docs/USB无线网卡联网说明.md)。你之前提到的 AX210 型号还需和实际 USB 设备核对：Intel 原生 AX210 的 Wi-Fi 走 PCIe、蓝牙走 USB，只有普通 USB 转接不能据此认定已有 Wi-Fi 功能。旧 [AX210 资料说明](docs/AX210联网说明.md) 仅保留为历史参考，不是当前 USB 设备的安装步骤。

无线传手机数据和板内 RK3588↔FPGA 的 RGMII 是两条通路。不要为了 USB 无线上网修改内部 FPGA 网口；按设备身份识别接口，不猜固定 eth0/eth1 或 wlan0。

## 🔌 电机与 IMU 接线

默认轮组排列：从车体上方看，向车头方向为前。

| 电机 | 位置 | PWM | IN1 | IN2 |
|---|---|---|---|---|
| M1 | 左前 | P3-1（J15） | P3-2（H15） | P3-5（Y21） |
| M2 | 左后 | P3-6（Y22） | P3-7（AB21） | P3-9（AA19） |
| M3 | 右前 | P3-10（AB20） | P3-28（M16） | P3-31（K21） |
| M4 | 右后 | P3-16（G20） | P3-19（G17） | P3-39（N18） |

这里的 **P3 是 FMC 转 GPIO 板的 GPIO0 接口**，不是开发板上的同名连接器。上表沿用你已经跑通的电机信号位置。M1/M2 为左侧，默认 `left_mask=3`；针对你反馈的 M3/M4 方向相反，默认翻转 M3/M4，`invert_mask=12`。手机“设置”可修改轮组与方向；修改会停车并取消使能。

- 两块 TB6612：逻辑 **VCC 必须供 3.3V**，STBY 固定 3.3V；VM 接电机电源 12V。STBY 高电平不代替 VCC 供电。
- FPGA 与两块驱动的 GND 必须相连。电机功率回流分别回到外部电源负极，不能经过 FMC 细地线承载电机电流。
- 你使用两套独立电源，因此尤其需要明确接共地线。P3-29 为 3.3V、P3-30 为 GND；**P3-11 是 5V**。使用前测量 FMC VCCADJ／相关 Bank 电压确为 3.3V；不能只看标签推断实测电压。
- MPU9250 沿用原连接：P10-1 / D16 接 SCL，P10-2 / E16 接 SDA，VCC 接 3.3V、GND 共地。P7 摄像头继续使用。
- 航向控制约定 IMU **X 指向车头、Y 指向左侧、Z 向上**；装反会使修正方向错误。安装方向变化需要转换坐标，不能用“归零”修正轴向。

## 🚀 RK3588 启动

先停止旧的 `yolo`／YOLO+IMU 窗口程序，避免抢占摄像头。保留原 `/root/run-yolo.sh` 和 `/root/rknn_yolov8_demo`，新程序会优先复用已有模型和 RKNN 库。可复用已经运行的 `isp-probe`。

在板子 `/root/` 下解压：

```sh
cd /root
tar -xzf robot-phone-rk3588-20261015.tar.gz
cd /root/robot-phone
sh scripts/diagnose-network.sh
sh run-robot-phone.sh
```

启动器沿用你已跑通的 `/root/find-video.sh`、`/opt/fpga-camera-isp-1080/run_isp_camera.sh dual-1080 100000`，准备摄像头并等待 NV12 格式；不会启动第二个本地窗口。默认推理左右两个画面，显示拼接预览。视频节点自动发现，不写死 `/dev/video41`。该程序不依赖 Weston、Python 或 Qt。

终端将打印：

```text
Pairing token: <本机生成的配对码>
wlan...: http://<板子的无线IPv4地址>:8080
```

在手机 APK 的“设置”填入 **板子的无线 IPv4 地址和配对码**，点击连接。配对码首次生成并保存在 `/root/robot-phone/pair-token.txt`，下次启动沿用；不要填内部 FPGA 的地址。APK 只接收私有局域网 IPv4 地址，例如 `http://192.168.1.100:8080`。配对码传输使用局域网 HTTP，请在可信网络使用。

若自动识别不到内部 FPGA 网卡，先看诊断脚本中 `fe1c0000.ethernet` 对应的接口，再明确指定，例如：

```sh
sh run-robot-phone.sh --interface eth1
```

这里的 eth1 只是示例。内部接口需要处于 UP 状态；仅在确认名称后按需执行 `ip link set <内部接口> up`。不要求给原始以太网协议配置 IP。手册中的外部网口是 `fe1b0000.ethernet`。

若需要后台运行，确认前台已正常工作后再执行：

```sh
cd /root/robot-phone
nohup sh run-robot-phone.sh >/tmp/robot-phone.log 2>&1 &
```

暂不自动加入开机自启。启动时电机保持停用，需要手机明确“使能电机”。首次启动／静止校准需要约 200 个连续静止 IMU 样本，正常约 2 秒。

## 🎮 手机操作顺序

1. 首次把四轮架空，连接 APK，确认视频、姿态角、加速度及 FPGA 应答正常。
2. 按“使能电机”，轻推摇杆，检查前后与左右转向。松开摇杆立即 PWM=0；大“停车”按钮同时取消使能。
3. “航向保持”开启后，前后推摇杆以当前航向直行，左右操作可转向；松手停车。陀螺仪存在漂移，无编码器时不能保证两侧实际轮速相同。
4. 跟随时先使能，再点当前画面中的识别框，等待选中确认，最后按“跟随目标”。**选框本身不启动车辆。** 跟随中重新选框会先停车，需要再按跟随。
5. “姿态归零”停车并把当前显示 Roll/Pitch/Yaw 设为参考零点；“静止校准”停车并重新估计陀螺仪偏置。加速度以 m/s² 显示，包含重力，水平静止时 Z 轴通常约 9.81。

跟随按被选中摄像头那一半的框中心计算比例：目标偏左就左转，偏右就右转。框较近或目标丢失时停车，不自动换跟另一个同类目标。它是二维视觉跟随，**没有测距、避障或路径规划**；框高只作停止近似，不能当实际距离。当前姿态融合为加速度计＋陀螺仪六轴融合，尚未使用 MPU9250 磁力计作绝对航向修正。

## 🛑 控制与停止规则

| 条件 | 行为 |
|---|---|
| 正常摇杆松手 | 立即左右 PWM=0，保留使能 |
| 手机切后台、失焦、断网 | 尽力发送停车；不再续运动指令；恢复后需重新使能 |
| 350 ms 没有新运动指令 | RK3588 停车、取消使能；读取状态不会续期 |
| FPGA 应答过期／配置不一致 | RK3588 禁止使能或停车 |
| 500 ms 没有合法电机报文／接收时钟丢失 | FPGA 独立看门狗使 PWM=0；需停用握手后重新使能 |
| 航向保持时 IMU 超时／校准中 | 停车，不继续使用旧姿态 |
| 跟随时图像超时／选中目标失踪 | 停车，不自动恢复追赶 |

正常加减速有斜坡，停车零指令立即生效。FPGA 四路 20kHz PWM 分相，默认最大占空比 60%，跟随前进更低；这些限幅不能替代驱动供电和共地。TB6612 的 PWM=0 为短路制动状态，与 STBY 拉低的待机状态不同，参见 [东芝原厂真值表](https://toshiba.semicon-storage.com/info/datasheet_en_20141001.pdf?did=10660)。

## 🛠️ 源码、协议与验证

`backend/` 是 RK3588 C/C++ 服务，`android/` 是原生 Java+Canvas APK，配套 FPGA 工程在 `../../fpga/robot_fmc_remote/`。RK3588↔FPGA 电机控制使用新的 `MOT1/ACK1` 原始以太网协议（EtherType 0x88B6），IMU 继续 0x88B5；不改仓库冻结的 UART v2。详细格式见 FPGA 工程 README。

主机构建／验证：

```sh
make -j4
make test
make test-api
python3 tests/test_launcher.py
./android/test-host.sh
./scripts/build-arm64.sh
```

`make test-api` 仅使用明确标识的模拟模式，不会打开摄像头、NPU 或物理电机。需要测试界面而未接硬件时，也可以运行 `sh run-robot-phone.sh --demo`；手机上会显示演示标识，不能把此模式当作实物验证。

APK 已编译并检查签名；RK3588 ELF 已交叉编译，直接依赖 `librknnrt.so` 和系统标准 C 库，无 OpenCV／GStreamer 依赖。复用当前板子上已可运行的 `/root/rknn_yolov8_demo/lib/librknnrt.so`；若运行提示 GLIBC 版本不满足，需要按实际 Buildroot SDK 重编译，不能复制主机 glibc 进去。

测试覆盖差速混控、航向修正、框跟踪、失联停车、CRC/序号/应答、HTTP 会话隔离、JSON、视频帧与框同步、APK 选框确认竞态；具体结果见 `reports/` 和 `android/dist/verification.json`。**本环境未连接开发板、手机或 USB 无线网卡，实际无线延迟、触屏体验、相机/NPU帧率、电机方向和驱动负载仍需上板验收。**
