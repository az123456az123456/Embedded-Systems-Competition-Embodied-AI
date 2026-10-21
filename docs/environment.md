# 开发与运行环境

> 路径均相对仓库根。源码在 `fpga/` 和 `rk3588/`，工具链与模型单独准备。

## 开发机

- Python 使用仓库根已有 `.venv`：`source .venv/bin/activate`；该环境不提交 Git。
- Python 原型依赖见 `rk3588/requirements.txt`。三向导航核心只需 Python 3.10+ 标准库；窗口另需 `rk3588/sonar_nav_sim/requirements.txt` 中的 PySide6。
- 主机 C/C++ 回归需要 `make`、C11 和 C++17 编译器。完整感知程序及窗口测试另需 Wayland client/server 开发库。
- RTL 回归需要 `iverilog` 和 `vvp`；GTKWave 用于查看波形。若工具在仓库外的 `~/tools/oss-cad-suite/`，将其 `bin/` 加入 PATH，或为支持的脚本设置 `IVERILOG`、`VVP`。
- FPGA 综合与烧录需要支持 `GW5AT-60B`、`GW5AT-LV60PG484AC1/I0` 的 Gowin EDA 和相应许可。使用各模块 `.gprj` / `build.tcl`；工具若不支持中文路径，可复制到纯 ASCII 临时目录构建。

## RK3588 构建与运行

板端运行 Buildroot 原生 ARM64 C/C++ 程序。交叉编译使用与实际 BSP 匹配的工具链、头文件和目标库，不用开发机 x86 共享库替代板端库。

| 工程 | 显式配置 |
|---|---|
| 感知与手机服务 `scripts/build-arm64.sh` | `CROSS_COMPILE`、`RKNN_INCLUDE`、`RKNN_LIB`；感知另需 `WAYLAND_INCLUDE`、`WAYLAND_LIB` |
| 摄像头接收 `camera-fastpath-30fps/scripts/build-arm64.sh` | `BSP_SDK`、`ARM64_CC`、`RKAIQ_INCLUDE`、`RKAIQ_LIB` |

RKNN 运行库应与板端 NPU 驱动和所用模型兼容。当前 YOLO11s INT8 转换元数据记录 Toolkit2 `2.0.0b0+9bab5682`、每眼 640×640 RGB、mean=0/std=255 和九路输出；完整身份与哈希见 [`模型元数据`](../rk3588/acg138-yolo11-30fps-rk3588-20261019/models/yolo11s-int8-rk3588.rknn.json)。模型、权重和外部 SDK 不提交 Git。

摄像头链路需要既有 FakeCam fastpath v4 隔离库，保留 5 ms pending-buffer 轮询及 RAW 拷贝/回调修复。隔离位置为 `/opt/fpga-camera-isp-1080/lib/librkaiq.so`，旧板端验证使用的参考 SHA256 为：

```text
bd20b922212d5b22cc155008772c5ee8efc49c34d031e35425d1691f3e8a27b6
```

这是历史已验证隔离库的文件身份，不代表所有重新构建都必须得到相同哈希。源码重建方法见 [`摄像头接收说明`](../rk3588/camera-fastpath-30fps/README.md)。不使用系统原 RKAIQ 库覆盖隔离库，不覆盖系统 IQ、内核或 v4l2loopback 模块。

感知安装、运行与回退见 [`统一安装入口`](../rk3588/acg138-yolo11-30fps-rk3588-20261019/README.md)；手机与 USB 无线网卡配置见 [`远控说明`](../rk3588/robot_phone/README.md)。当前 30 FPS FPGA 对应 `.fs` 原本未随仓库保存，模型文件也需另行准备；有源码与报告不等于当前目录已经可以直接部署。
