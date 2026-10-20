# RK3588 YOLO＋IMU 集中优化 v10

> 2026-10-19 · 基于最新 v9 三 NPU 核心流水线；适用于已运行的 `/root/perception-dashboard`。

## ✅ 改动与保留

本次针对实测采集14.71 FPS、识别12.59 FPS、颜色转换28.04ms、单目输入12～19ms优化。三个核心负载70%/64%/61%约平均65%，说明除了硬件推理，还值得缩短CPU处理和输入搬运。

| 环节 | 本版处理 |
|---|---|
| 摄像头颜色转换 | ARM64 NEON每批16像素，两个相邻行复用UV；缓存601/709、全/有限范围系数。保持NV12/NV21分平面、步长与尾部处理。 |
| 模型预处理 | 每个线程缓存letterbox采样位置和整数插值权重，直接写入目标缓冲；坐标与双线性插值保持v9语义。 |
| 模型输入 | 使用RKNN持久UINT8/NHWC输入内存，保留模型归一化；支持的运行库直接预处理到此缓冲并同步缓存，减少重复提交和中间拷贝。 |
| 兼容回退 | 可选API缺失、格式不支持、分配/绑定/缓存同步/绑定后运行失败时回退旧输入方式；需清理绑定时重建相同模型上下文并恢复核心。 |
| 统计 | 原PERF/STAGE保留，新增每核两秒窗口平均处理耗时，方便判断优化是否只是把input_ms搬到了run_ms。 |
| 采集时间 | frame_age/box_age优先采用可信V4L2单调时间戳，否则使用转换前出队时间，包含以前漏算的颜色转换耗时。 |

模型文件、模型输入尺寸、摄像头1920×1080采集、双目分别识别、三核绑定、独立预览和有限在途队列均保留。IMU校准与滤波解算、小车俯视动画、全屏和0/90度显示沿用v9；没有重新烧录FPGA或更换系统/运行库。

## 📦 安装

将 `perception-dashboard-fast-v10-arm64-20261019.tar.gz` 拷到板子的 `/root/`。先退出原窗口；安装器检测到该程序仍运行会拒绝覆盖。逐行执行：

```sh
cd /root
tar -xzf perception-dashboard-fast-v10-arm64-20261019.tar.gz
sh /root/perception-dashboard-v10-update/install.sh
cd /root/perception-dashboard
./run-dashboard.sh -0
```

90度横置使用同一个脚本：

```sh
./run-dashboard.sh -90
```

原启动时额外设置的 `--interface`、`--camera` 等参数继续加在后面。安装只更新 `perception-dashboard.aarch64` 和 `run-dashboard.sh`，保留板子上已跑通的 `run-perception.sh`、模型、库、ISP与桌面启动方式。若程序安装在别处，在安装/回退命令前设置 `DASHBOARD_DIR=/实际目录`。

终端开头应出现 `PERCEPTION fast-v10`。支持输入绑定时各核出现 `INPUT core=0/1/2 path=io-memory-u8`；若出现 `path=legacy reason=...`，说明已采用兼容路径，其他CPU优化仍有效。

## 🔎 运行检查与对比

保留相同场景、同样窗口方向和模型，运行稳定后记录30～60秒。可使用：

```sh
cd /root/perception-dashboard
./run-dashboard.sh -0 2>&1 | tee /tmp/yolo-v10.log
```

另一个终端查看NPU负载和运行状态：

```sh
cat /sys/kernel/debug/rknpu/load
sh /root/perception-dashboard-v10-update/diagnose-performance.sh
```

负载是整个NPU设备在驱动采样时间内的各核心忙碌比例；若还有其他NPU程序，也会计入。不要将Core0/1/2直接相加当成总百分比。频率、温度和可用内存由诊断脚本只读收集，不修改系统设置。

先退出窗口，然后可在同一新程序上关闭优化，做受控对比：

```sh
cd /root/perception-dashboard
PERCEPTION_CAMERA_SCALAR=1 PERCEPTION_PREPROCESS=legacy PERCEPTION_INPUT=legacy ./run-dashboard.sh -0 2>&1 | tee /tmp/yolo-reference.log
```

这会恢复旧颜色转换、旧预处理和旧输入API；时间戳及统计仍采用v10，方便直接比较。需要精确比较旧二进制时使用后面的回退功能。日志拷回电脑后：

```sh
python3 compare-performance.py yolo-reference.log yolo-v10.log --skip-perf 3
```

脚本在更新目录中，板端运行主程序和诊断脚本均不需要Python。对比优先看稳定的result_fps和每核mean_eye_ms，以及mean_preprocess_ms、mean_input_ms、mean_run_ms总和；不能仅看input_ms变小。输出p50/p95有对应的日志采样含义，不代表完整每帧测量。

frame_age/box_age现在包含之前漏算的转换时间，数字可能比旧版高，即使实际响应改善；应使用v10的参考模式做同口径延迟对比。检测框仍在350ms过期，无限排队不会用于提高表面帧率。

可选的纯CPU基准（合成图像，无需摄像头/NPU）：

```sh
cd /root/perception-dashboard-v10-update
./bench-camera.aarch64 60
./bench-preprocess.aarch64 --benchmark
```

## ↩️ 回退

退出窗口后：

```sh
sh /root/perception-dashboard-v10-update/rollback.sh
cd /root/perception-dashboard
./run-dashboard.sh -0
```

恢复安装前的程序和同名启动入口，备份路径由安装器输出，保存于 `.v10-last-backup`。如果安装前没有run-dashboard.sh，回退会移除新入口，届时使用原来的启动命令。

## 🧪 验证范围与预期

已验证缓存预处理与v9输出逐字节一致（含带步长的模型输入）；摄像头颜色公式遍历67,108,864组YUV，NEON定点公式与参考最大差1个色阶，覆盖704种帧布局和异常长度。已通过输入缓冲SDK模拟、缺API/绑定/同步/运行失败回退、三核完整双目流水线、IMU与窗口旋转回归、安装失败恢复测试。

ARM64程序使用当前BSP的GCC10.3.1编译；没有加入新的板端共享库。开发机CPU基准不是RK3588实测，本机没有执行ARM机器码，也没有连接实板测试FPS或识别精度。合理目标是缩短每组处理耗时、向现有约15FPS采集上限靠近，不能保证固定增幅。若input/run耗时或识别异常，可先设置PERCEPTION_INPUT=legacy比较，或完整回退。

工程源码和具体测试报告随源码ZIP交付。完整测试入口：make test test-fast test-input-memory test-stage-timing；图形回归make test-window；板端工具测试为tests/test_diagnostics.py、test_compare_performance.py与test_v10_installer.py。
