# RK3588 当前项目入口

> 2026-10-21 · ACG138 具身智能小车 · RK3588 / Buildroot / Linux 6.1.75

## 🧭 目录与入口

| 目录或文件 | 用途与入口 |
|---|---|
| [robot_perception_v12/](robot_perception_v12/README.md) | 原生 C/C++ 双目 YOLO＋IMU 窗口源码，程序版本 `v10.2-yolo11-30fps`；板端入口 `run-dashboard.sh -0` 或 `-90`。 |
| [camera-fastpath-30fps/](camera-fastpath-30fps/README.md) | 双目 RAW 行重组、RKAIQ ISP 与 NV12 输出；`make arm64-30fps` 构建目标 30FPS 接收程序。 |
| [acg138-yolo11-30fps-rk3588-20261019/](acg138-yolo11-30fps-rk3588-20261019/README.md) | YOLO11s INT8＋30FPS 板端更新目录；安装和回退入口在 `scripts/`，安装前按说明准备模型与校验文件。 |
| [robot_phone/](robot_phone/README.md) | Android APK 源码及原生遥控服务，入口 `run-robot-phone.sh`；配套 [FPGA 工程](../fpga/robot_fmc_remote/README.md)。 |
| [rtl8821cu-driver/](rtl8821cu-driver/README.md) | 当前 RTL8821CU USB 无线网卡的指定内核驱动。 |
| [sonar_nav_sim/](sonar_nav_sim/README.md) | 三路 A02YYUW、编码器与 IMU 的电脑端建图导航原型，入口 `python3 run_sim.py`。 |
| [imu_dashboard/](imu_dashboard/README.md) | 独立 IMU 诊断窗口，入口 `run-imu.sh`。 |
| `task6_mission.py`、`vision_demo.py`、`nav_demo.py` | 独立 Python 目标跟随、中文语义、虚拟 FPGA 与 A* 原型，使用 UART v2。详见 [虚拟 FPGA](虚拟FPGA联调说明.md) 与 [导航说明](导航与语义解析说明.md)。 |

当前双目链路为：FPGA 双 OV5640 → RGMII/UDP → RKAIQ ISP → 1920×1080 NV12 → 预览与识别。30FPS 摄像头配套源码在 [fpga/imu_camera_dual_1080_30fps](../fpga/imu_camera_dual_1080_30fps/README.md)；手机遥控和独立 Python 原型分别使用各自配套协议。

## 🔧 开发与验证

从仓库根目录激活已有 Python 环境：

```sh
source .venv/bin/activate
cd rk3588
```

| 模块 | 在模块目录执行 |
|---|---|
| 原生感知 | `make test test-launcher test-fast test-ui-render` |
| 手机遥控 | `make test test-api`、`python3 tests/test_launcher.py`、`sh android/test-host.sh` |
| 导航模拟 | `python3 -m unittest discover -s tests -v` |
| 本目录 Python 原型 | `python3 -m unittest test_fpga_protocol test_navigation test_semantic_parser test_virtual_fpga_link -v` |

交叉编译需要匹配板端的 BSP、ARM64 编译器、RKNN、RKAIQ 与 Wayland 开发文件。可用 `CROSS_COMPILE`、`RKNN_INCLUDE`、`RKNN_LIB`、`WAYLAND_INCLUDE`、`WAYLAND_LIB`、`BSP_SDK`、`RKAIQ_INCLUDE`、`RKAIQ_LIB` 指定位置；自动查找仍使用祖先目录中的 `YOLO/rknn_model_zoo`、`bsp_r4`、`gst-dev` 和原摄像头运行库。这些 SDK 与模型需要单独准备。

模型权重不进 Git。Python 原型默认使用本目录 `yolov8n.pt` 和 `yolov8_reid/ReID_resnet50_ibn_a.pth`；原生窗口默认使用 `yolo11s-int8-rk3588.rknn`，可用 `MODEL_PATH` 或 `--model` 覆盖。手机发布脚本仍需 `YOLO/model.rknn` 及其已验证 APK、ARM64 程序和 FPGA 位流，不能用缺失输入重新生成发布包。

## ✅ 已有证据与待验收项

- 20FPS 摄像头已有实板日志：`rx_fps/output_fps=20.00～20.05`，`incomplete=1` 连续保持不变，`malformed=0`、`rx_superseded=0`。
- YOLO11s INT8 在该 20FPS 输入下稳定识别约 19.5～20FPS，推理约 57～83ms；用户反馈效果好，尚无标注测试集精度结论。
- 当前窗口版本为 `v10.2-yolo11-30fps`，保留三核流水线、双目、IMU、全屏及 `-0/-90`。30FPS 位流和接收程序已构建，现有记录未包含其完整实板验收，不能把采集目标写成已实现的识别速度。

各模块 `reports/` 保留构建、测试和实板观察证据。主机测试不能代替摄像头、NPU、无线链路和电机的实车验收。
