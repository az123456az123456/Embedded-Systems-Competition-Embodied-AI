# ACG138 具身智能小车

2026嵌入式竞赛项目，平台为 RK3588 + 高云 GW5AT-60K FPGA。

```text
fpga/     FPGA工程、传感器驱动与仿真
rk3588/   感知、手机控制、导航与Python原型
docs/     比赛需求、开发环境和最新技术报告
```

## 工程入口

| 功能 | 代码 |
|---|---|
| 双目＋MPU9250，30FPS目标 | [fpga/imu_camera_dual_1080_30fps](fpga/imu_camera_dual_1080_30fps/README.md) |
| YOLO11s INT8感知窗口 | [rk3588/robot_perception_v12](rk3588/robot_perception_v12/README.md) |
| 摄像头RAW接收与ISP | [rk3588/camera-fastpath-30fps](rk3588/camera-fastpath-30fps/README.md) |
| 感知板端更新 | [rk3588/acg138-yolo11-30fps-rk3588-20261019](rk3588/acg138-yolo11-30fps-rk3588-20261019/README.md) |
| 手机控制与跟随 | [rk3588/robot_phone](rk3588/robot_phone/README.md) ＋ [fpga/robot_fmc_remote](fpga/robot_fmc_remote/README.md) |
| 电机诊断 | [最新方向修正](fpga/motor/fmc_four_load_diag_direction_v2_20261019/README.md)；[FMC引脚与顺序测试](fpga/motor/fmc_four_test/README.md) |
| 三向超声波导航 | [rk3588/sonar_nav_sim](rk3588/sonar_nav_sim/README.md) ＋ [A02YYUW驱动](fpga/ultrasonic/a02yyuw/README.md) |

高速感知与手机远控是独立工程，目前没有合成同一个30FPS四电机版本。30FPS感知保留源码和报告，工作区原本缺少对应位流与RKNN模型；完整部署需另外准备。Python任务4/6与UART v2仍是待集成原型，入口见 [RK3588说明](rk3588/README.md)。

## 开发验证

从仓库根目录运行；Python使用现有 `.venv`，主机测试需要编译器，FPGA仿真需要 `iverilog` 和 `vvp`。

```bash
source .venv/bin/activate
bash fpga/run_all_sim.sh
make -C fpga/robot_fmc_remote test test-camera
make -C rk3588/robot_phone test test-api
make -C rk3588/robot_perception_v12 test-ui-schedule test-fast
python -m unittest discover -s rk3588/sonar_nav_sim/tests -v
```

[比赛需求](docs/requirements.md) · [环境与依赖](docs/environment.md) · [最新技术报告](docs/技术报告_20261020.docx)
