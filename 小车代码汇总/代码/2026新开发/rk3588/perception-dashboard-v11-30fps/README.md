> 本次发布默认使用 `model/yolo11s-int8-rk3588.rknn` 和摄像头目标30FPS；受控安装器同步部署同一份启动脚本。`MODEL_PATH`、`--model`、`-0/-90`接口保留。

# RK3588 YOLO＋IMU v10.2-yolo11-30fps

> 2026-10-19 · 基于 fast-v10.1。配套交付默认 YOLO11s INT8 模型和 FPGA 30 FPS 实验版；模型路径由更新包的启动器设置。

## 🖥️ 本次窗口变化

- 保持原有 33 ms 渲染间隔，窗口目标仍约 30 Hz，没有默认增加到 60 Hz。
- 将固定 10 ms 的空闲轮询等待改为根据下一次渲染期限缩短到 1～10 ms，避免 33 ms 期限因 10 ms 的 IMU 唤醒/轮询节奏反复延后到约 40 ms。
- 计算等待时间时重新读取当前单调时钟，包括刚发生的绘制、提交和 Wayland 事件处理耗时；等待配置或合成器回调时仍最多等待 10 ms。
- `--version`、帮助和启动提示统一显示 `v10.2-yolo11-30fps`。
- 检测器、IMU 解算、摄像头转换、三核队列与图像处理代码保持 fast-v10.1；本次仅改 UI 调度和版本文字。物理采集帧率由配套 FPGA 传感器配置决定，窗口不会补帧。

`make test-ui-schedule` 用 10 ms 的模拟 IMU 唤醒、17 ms 绘制耗时检查期限调度，固定轮询在该场景每秒调度 25 次，新方案约 30 次。该结果是调度逻辑测试，不是 RK3588 上的实测帧率。真实 `video_submit_fps` 仍取决于合成器回调和板端绘制耗时。

以下保留 fast-v10.1 的性能优化背景与旧版安装记录。本次应使用父目录交付的 YOLO11s/30 FPS 更新说明和安装器。

## 📚 fast-v10.1 背景

> 2026-10-19 · 基于 v10，模型、分辨率、三NPU核心、IMU解算与接线均沿用现有配置。

## 📊 本次实测结论

用户板子运行时NPU已是1,000,000,000Hz，系统可用最高频率也是1GHz。NPU温度约50.8°C，此次采样时没有发生NPU降频。performance governor只能请求当前系统限制内的最高档，不能把1GHz变得更高。

三核负载约58～70%，不表示应加入无用任务填到100%。目标是提高有效双目识别FPS并缩短延迟。按15FPS、每帧左右各一次、每个单目约75～80ms硬件工作粗算，三个核心平均约75～80%就可能处理完15FPS采集；更高频率也可能让同样任务的占用百分比下降。

v10的input_ms约0.1ms，但run_ms约93～98ms；切回旧输入后input_ms约19～25ms、run_ms约75～83ms。这说明部分工作可能换了计时位置。两次日志不是完全同场景、相同温度的长时间实验，不足以判定哪条输入路径一定更快。比较要看mean_eye_ms、稳定result_fps和box_age_ms。

CPU截图中policy4(大核4/5)当时600MHz，policy6(大核6/7)2.256GHz；这是瞬时读数，不能直接证明policy4造成持续瓶颈，因此大核性能模式作为可选对照提供。

## ✅ v10.1改动

- 窗口大面积不透明区域直接填色，只在圆角/边缘保留原抗锯齿计算；原尺寸0度显示直接复制。40种完整画面状态和8种尺寸×4种旋转均与v10逐像素一致。
- 模型预处理改为按像素共享RGB的采样位置/权重，缓存映射表，避免v10 ARM内核先逐通道收集到临时数组再加载。仍保留v9的letterbox坐标、11bit双线性舍入、114填充和目标步长语义。
- 保留v10的NEON颜色转换、自适应持久输入缓冲、原路径回退、有限队列与完整双目配对。
- 修正启动版本提示，并在INPUT行显示preprocess=cached-pixel/legacy/gather。
- 新增可选NPU与CPU大核性能脚本，均备份原governor并可恢复；默认安装不会执行这些调频脚本。

开发机代表性完整界面绘制约22.3ms→3.1ms，缓存预处理约2.7ms→1.7ms。以上是开发机合成测试，不是RK3588实测FPS。真实板端是否从约12FPS提高，仍须运行新版本观察。

## 📦 更新

退出当前窗口，将 `perception-dashboard-fast-v10.1-arm64-20261019.tar.gz` 拷到板子 `/root/`，执行：

```sh
cd /root
tar -xzf perception-dashboard-fast-v10.1-arm64-20261019.tar.gz
sh /root/perception-dashboard-v10.1-update/install.sh
cd /root/perception-dashboard
./run-dashboard.sh -0
```

横置90度仍用同一入口：

```sh
./run-dashboard.sh -90
```

原来的--interface等额外启动参数继续加在后面。安装器备份并替换二进制和run-dashboard.sh，保留原run-perception.sh、摄像头/ISP、模型与运行库。程序开头应显示PERCEPTION fast-v10.1，三个INPUT行显示实际输入路径及预处理方式。

## ⚡ 可选：CPU大核性能对照

先退出窗口，再用root执行：

```sh
sh /root/perception-dashboard-v10.1-update/cpu-big-performance.sh status
sh /root/perception-dashboard-v10.1-update/cpu-big-performance.sh enable
cd /root/perception-dashboard
./run-dashboard.sh -0
```

脚本只选择RK3588大核4/5及6/7的系统performance governor，不改小核0～3、不改min/max频率和温控。内核必须已提供performance，否则脚本报错并保留原设置。此模式是系统级临时设置，可能增加功耗与温度；可随时恢复：

```sh
sh /root/perception-dashboard-v10.1-update/cpu-big-performance.sh restore
```

可选地，在已确认有taskset命令的系统上用 `taskset f0 ./run-dashboard.sh -0` 将该程序及其线程限制在4～7号大核，单独比较此项；没有taskset时继续使用普通启动命令即可。不要同时改多项再把结果归因于单一改动。

## ⚡ 可选：NPU稳频

你当前运行时已经是系统最高1GHz，所以这项不能提高当前最高频率；仅用于后续发现频率波动时做对照：

```sh
sh /root/perception-dashboard-v10.1-update/npu-performance.sh status
sh /root/perception-dashboard-v10.1-update/npu-performance.sh enable
```

脚本只把已有NPU governor切到performance，保留当前min_freq/max_freq、系统OPP、电压与温控限制。不会写超出系统频点的数值。若系统没有performance governor，不修改任何NPU设置。恢复：

```sh
sh /root/perception-dashboard-v10.1-update/npu-performance.sh restore
```

两个脚本均保留首次备份，重复enable不会将原设置覆盖掉；备份位于/run/perception-npu-performance和/run/perception-cpu-big-performance。它们不加入开机启动。程序回退不会代替性能脚本的restore，如需完全恢复，请分别执行。

## 🔎 对比方法

保持相同场景、模型与窗口角度，每次运行至少30秒，跳过启动阶段前几组PERF。记录稳定PERF、STAGE camera_frame、STAGE npu_worker以及INPUT。建议先比较新旧程序，再单独比较CPU性能模式。

```sh
cd /root/perception-dashboard
./run-dashboard.sh -0 2>&1 | tee /tmp/yolo-v10.1.log
```

旧输入或旧预处理可单独对比：

```sh
PERCEPTION_INPUT=legacy ./run-dashboard.sh -0
PERCEPTION_PREPROCESS=legacy ./run-dashboard.sh -0
PERCEPTION_PREPROCESS=gather ./run-dashboard.sh -0
```

gather是v10的缓存内核，legacy是v9普通预处理，默认是新的cached-pixel。颜色转换继续沿用v10；不更换模型精度、模型文件或分辨率。

查看系统状态：

```sh
cat /sys/kernel/debug/rknpu/load
sh /root/perception-dashboard-v10.1-update/diagnose-performance.sh
```

NPU load是设备整体的各核心负载，含其他进程；每核busy_ms是工作线程流程耗时，不能当硬件负载。UI draw_ms下降若仅改善窗口而result_fps不变，也要如实区分。v10/v10.1的图像年龄包含转换前出队时间，不应与v9漏算转换的数字直接比较。

可选的独立板端合成基准，不连接传感器、摄像头或NPU：

```sh
cd /root/perception-dashboard-v10.1-update
./bench-ui-render.aarch64 --benchmark
./bench-preprocess.aarch64 --benchmark
./bench-camera.aarch64 60
```

## ↩️ 回退与验证

退出窗口后：

```sh
sh /root/perception-dashboard-v10.1-update/rollback.sh
```

恢复本次安装前的二进制和run-dashboard.sh。恢复性能governor请另执行对应restore命令。

本次完成ARM64交叉编译、RGB预处理逐字节对照、原三核流水线/IMU回归、UI逐像素对照及全屏旋转、安装回退、临时sysfs调频脚本测试。所有调频测试均使用临时目录，没有在开发机或实板写真实频率接口。本机没有执行ARM机器码，没有实板提速保证。
