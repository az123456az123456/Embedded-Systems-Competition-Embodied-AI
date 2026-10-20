# YOLO 与 IMU 统一窗口 · RK3588 / Buildroot

> 2026-10-16 · 零偏校准修复 v7 / 旋转显示、帧率优化、俯视小车及默认全屏 · ACG138 · 双目 OV5640 · MPU9250 · Weston / Wayland

## 🧭 v7 偏航漂移与校准进度修复（2026-10-16）

用户反馈：已经烧录 FPGA 六轴低通，回退后的窗口仍一直显示零偏校准，只有 YAW 约每秒漂 0.1°。旧版遇到任何大于 50ms 的新采样间隔便重置校准，所以会出现“静止但校准永远为 0%”；这并不能证明实板一定出现了同样间隔，新版每 2 秒打印接收间隔和实际零偏，便于确认。FPGA 的单位增益低通会保留常量零偏，增大低通强度不能解决这条校准链路。

v7 只更新 RK3588 窗口的姿态解算，继续使用已烧录的 IMU＋双目 FPGA，无需重新烧录。初始化或点击“重新校准”时，保持小车静止至少 2 秒且积累至少 32 个新样本。进度按 FPGA 采样时间推进，容忍 10～200ms 的采样间隔和序号跳变；重复包不计入校准，也不刷新新数据时间。断流超过 200ms、无效数据或明确运动会重启静止窗口。低采样率下可能需要更久，例如 5Hz 至少约 6.2 秒。

校准检查重力幅度、三轴变化范围以及陀螺仪/加速度方差，保留原先 3°/s 的合成角速度上限。校准期间**仍按原量程积分真实转动**，重新校准期间保留上次成功的零偏；校准完成后固定零偏，行驶时不自动学习，不锁定角度。启动和手动校准必须确实静止，六轴数据无法区别完全平稳的微速偏航与零偏，不应边转动车体边校准。

扣除零偏后，合成角速度不超过 **0.03°/s** 时视为零；超出阈值的角速度按完整比例积分，不减去阈值，不改变增益。可用 `./run-perception.sh --imu-deadband 0` 关闭该微小角速度死区；允许范围为 0～0.2°/s。默认死区会舍弃真实小于阈值的微速运动，运动控制若需要这部分信息应关闭。原始接收数据和加速度显示不改。

先关闭窗口，把 `perception-dashboard-imu-bias-arm64-v7-20261016.tar.gz` 放入板子 `/root/`：

```sh
cd /root
tar -xzf perception-dashboard-imu-bias-arm64-v7-20261016.tar.gz
cd perception-dashboard-imu-bias-update
sh install.sh
cd /root/perception-dashboard
./run-perception.sh
```

安装器备份原程序并保留原 `run-perception.sh`、桌面配置、模型、ISP 和运行库。两种方向入口仍为 `./run-normal.sh` 与 `./run-rotate90.sh`，默认全屏；方向相反时可用 `./run-perception.sh --rotate 270`。目录不同时使用 `DASHBOARD_DIR=/实际目录 sh install.sh`。

等状态栏显示“零偏校准完成”，再点击“航向归零”。上电后可先预热 30～60 秒再重新校准一次，减少升温导致的偏差。若仍未完成，请保留终端的 `IMU[v7]` 行：`fresh=0` 表示没有新鲜数据，`cal=motion/vibration` 表示角速度或振动超限，`dt_ms` 是新样本时间间隔，`gyro_dps/bias_dps` 是当前角速度/扣除的零偏。`cal=done progress=100%` 表示已完成。

`reports/imu_bias_v7_before_after.log` 使用同一组 60ms 间隔、0.09924°/s 恒定输入对照原 v4：旧版 0% 且 60 秒累积约 5.95°，新版完成校准且该合成输入的后续漂移为 0。其他测试涵盖 10～200ms 不规则间隔、量化噪声、振动拒绝、校准未完成时的 90° 转动、校准后的 ±0.05/±0.1/0.5/45°/s 转动、重新校准、原摄像头链路、全屏/旋转及 ARM64 构建。这些是合成数据与协议检查，**不是实板精度承诺**。当前仍为六轴相对航向，没有启用未标定磁力计；后续温漂和运动中的长期偏航误差仍可能存在。

## 🖥️ v6 正常与旋转显示（2026-10-16）

两个启动入口都沿用板上现有 `run-perception.sh`，默认全屏：

```sh
./run-normal.sh        # 原方向，旋转 0°
./run-rotate90.sh      # 整个窗口顺时针旋转 90°
```

可在原启动命令后直接追加 `--rotate 0` 或 `--rotate 90`。屏幕安装方向相反时用 `--rotate 270`；CLI 也支持 180°。默认 0°。示例：`./run-perception.sh --rotate 90 --no-detect`。全部内容一起旋转，视频、检测框、姿态角、俯视小车和按钮保持一致；鼠标及触摸坐标按同一变换换回原画布，黑边点击被忽略。只改变此应用窗口，不修改系统桌面或显示器配置。

画布按旋转后的宽高等比例适配，窗口模式最小尺寸也随 90°/270° 交换；缩放仍留黑边而不拉伸。绘制使用预计算索引及 16×16 分块，避免每像素旋转时计算三角函数。`--preview` 同样输出所选方向。

本版本恢复经过 v4 源码哈希核验的 IMU 解算，撤回 v5 的静止锁定、自动零偏学习和“校准未完成就屏蔽陀螺仪”的逻辑。此前 v5 静止过滤更新已撤回，勿再安装；旧测试与安装脚本仅归档在 `reports/withdrawn-v5/`。FPGA 端新加入的数据低通仍可配合原来的静止校准使用。

把 `perception-dashboard-rotation-arm64-v6-20261016.tar.gz` 放到板子 `/root/`，关闭当前窗口后执行：

```sh
cd /root
tar -xzf perception-dashboard-rotation-arm64-v6-20261016.tar.gz
cd perception-dashboard-rotation-update
sh install.sh
cd /root/perception-dashboard
./run-rotate90.sh
```

安装器备份并替换根目录 `perception-dashboard.aarch64`，增加两个入口脚本，保留你已经修好的原启动器、桌面配置、模型和运行库。目录不同时使用 `DASHBOARD_DIR=/实际目录 sh install.sh`。

验证包括四个旋转方向的逐像素结果、缩放/黑边、横屏/竖屏/缩放后的校准/归零/关闭按钮坐标、无输入设备的 Wayland 全屏和窗口模式、原摄像头和 IMU 回归、ASan/UBSan 及 ARM64 严格构建。原生演示预览为 `reports/rotation-normal.png`、`reports/rotation-90.png`，不是实板性能数据；尚未连接用户屏幕实测触摸。

## ⚡ v4 帧率优化（2026-10-16）

保留 v3 全屏和俯视小车。新增独立采集线程：仅保留最新一张完整 RGB 帧，左右眼推理共享同一张不可变图像；默认两个独立 RKNN 上下文分别请求 NPU0/NPU1 并行识别。第二上下文加载失败时回退串行；需要减少内存或比较速度时使用 `--serial-eyes`。不会把旧帧的框覆盖到新帧上。

图像补边缩放改用 11 位定点双线性插值，保持 RGB、补边和坐标映射；随机图像检查与浮点插值的最大差为 1/255。开发机单眼 960×1080→640×640 对比见 `reports/fps_preprocess_benchmark.log`；该结果不是板端帧率或识别准确率评测。

窗口右上角分别显示 `CAP`（V4L2 被本程序取到并转换的帧率）与 `AI`（完成整个所选视角检测的帧率）。它们都不是原始 RGMII 测量值，也不等于桌面刷新率。终端每约 2 秒输出 `PERF`，包含处理耗时与图像年龄。

```sh
./run-perception.sh --no-detect    # 摄像头＋IMU，不加载标签、模型或调用 NPU
./run-perception.sh               # 默认双眼并行识别
./run-perception.sh --serial-eyes # 对比旧串行双目模式，仍使用新的采集/缩放
./run-perception.sh --view left   # 全双目画面，仅左眼识别
```

配套 `../camera-fastpath` 优化 RAW UDP→ISP→NV12 接收端：独立持续收包、完整最新帧交接、批量接收、复用 RAW 内存、默认关闭调试落盘，并记录真实收到的完整帧与 ISP 输出速率。要改善单独预览帧率，应一起更新其 `isp-probe` 和专用 FakeCam 库。配套库把待处理 RAW 的定时通知从 100ms 缩短至 5ms，保留原 RAW 拷贝和回调修复。原 ISP 启动脚本、IQ 文件和摄像头节点发现方式沿用板上已跑通的版本；系统 RKAIQ 库不替换。

当前 FPGA 摄像头寄存器仍是约 15FPS 的名义目标。没有通过虚构视频格式的帧率、重复帧、降低分辨率或更换模型宣称达到 30FPS。尚未接实板，最终 `CAP / AI / FASTPATH rx_fps / output_fps` 均需实测。

## 🚗 v3 俯视小车与全屏升级

右侧原水平仪替换为简约四轮小车俯视图。车头带方向箭头，随相对 Yaw 旋转：归零后朝上，正角向左、负角向右；横滚和俯仰保留上方数值，不再倾斜动画视角。IMU 数据无效或超时时灰显并提示等待。圆环顶部是相对零方向，不表示磁北。

启动后默认请求全屏，按屏幕尺寸等比例显示，顶部右侧关闭按钮仍可退出。直接执行原来的 `./run-perception.sh` 即可，不必追加全屏参数；需要普通可拖动窗口时使用 `./run-perception.sh --windowed`。`--demo` 也默认全屏，并可演示小车航向变化。

已经运行 v2 的板子仅需替换窗口二进制。把 `perception-dashboard-top-view-arm64-v3-20261015.tar.gz` 复制到 `/root/`，先关闭旧窗口，再执行：

```sh
cd /root
tar -xzf perception-dashboard-top-view-arm64-v3-20261015.tar.gz
cd /root/perception-dashboard
cp -p perception-dashboard.aarch64 perception-dashboard.aarch64.bak
cp /root/perception-dashboard-top-view-update/perception-dashboard.aarch64 perception-dashboard.aarch64.new
chmod +x perception-dashboard.aarch64.new
mv -f perception-dashboard.aarch64.new perception-dashboard.aarch64
./run-perception.sh
```

若原窗口放在其他目录，将 `/root/perception-dashboard` 换成实际目录。必须更新启动器优先使用的项目根目录 `perception-dashboard.aarch64`，只更新 `build/` 下文件不会覆盖根目录的旧版本。原来的模型、运行库、启动参数可以沿用。

![俯视小车动画演示](reports/imu-top-view.gif)

本次严格编译、ARM64 交叉编译、原有单元检查和 Wayland 窗口检查已通过；检查了默认全屏请求、窗口模式回退，以及 0°、+90°、-90°、180° 和超时状态的原生渲染。本环境未连接实际 RK3588 屏幕。

## 🖥️ 在板上运行

先退出原来的 `yolo` 程序，释放摄像头。将发布包放到 RK3588 可写目录，在原先能够接收 IMU 的 root 终端执行：

```sh
tar -xzf perception-dashboard-buildroot-arm64-v2-20261015.tar.gz
cd perception-dashboard
./run-perception.sh
```

发布包已带当前工作区的 `YOLO/model.rknn` 和 COCO 80 类标签作为备用。不需要在板上装 Python、Qt、OpenCV 或 GStreamer。新版启动器优先复用你现有的 `/root/rknn_yolov8_demo/model/yolov8s.rknn` 及同目录标签，并把 `/root/rknn_yolov8_demo/lib` 加入动态库搜索路径。

根据你提供的 `/root/run-yolo.sh` 和运行日志，本版已适配以下启动条件：

| 项目 | 新版启动器的行为 |
|---|---|
| 摄像头 | 使用 `/root/find-video.sh` 自动查找，当前实测节点为 `/dev/video41`，重新准备 ISP 后再次查找 |
| ISP | 没有 `isp-probe` 时，以 `ISP_DISPLAY=0` 调用 `/opt/fpga-camera-isp-1080/run_isp_camera.sh dual-1080 100000` |
| 就绪检查 | 最多等待 30 次，确认摄像头节点返回 NV12；失败说明原因，不使用未就绪的图像 |
| 桌面 | 优先沿用当前会话，自动查找时包含 `/run`；默认 `/run/wayland-0` |
| 模型/运行库 | 优先复用原 YOLO 已跑通的模型、标签和 `lib` 目录 |

ISP 日志为 `/tmp/perception-isp.log`。本程序关闭后 ISP 采集服务按原方案保留。先退出旧的 YOLO 检测窗口，以免两个检测程序争用同一个输出节点。

可以显式指定你当前的设备和原 IMU 接口：

```sh
./run-perception.sh --camera /dev/video41 --interface eth1
```

`eth1` 只是示例，请使用原来接收 IMU 成功的接口；省略时沿用自动识别。错误设备不会被当成有效图像推理。启动准备失败时终端报错退出；窗口打开后的视觉处理错误会保留 IMU 显示。IMU 原始以太网接收需要 root 或 CAP_NET_RAW。

若从串口/SSH 启动且桌面连接失败，使用实际 Weston 会话，例如：

```sh
XDG_RUNTIME_DIR=/run WAYLAND_DISPLAY=wayland-0 ./run-perception.sh
./run-perception.sh --fullscreen
./run-perception.sh --demo
```

`--demo`、`--preview`、`--help` 跳过摄像头发现、ISP 启动和就绪等待。演示数据有明确标记，不调用摄像头、NPU 或原始网络。原有 `yolo` 命令保留；新版连接现有 Weston，不另启桌面。

需要手动管理 ISP 或连接其他相机时，可使用 `PERCEPTION_PREPARE_ISP=0 ./run-perception.sh --camera /dev/videoN`。`MODEL_PATH`、`LABELS_PATH`、`CAMERA_DEVICE` 可设置默认资源；显式命令行参数优先。板端目录变化可通过 `YOLO_DIR`、`FIND_VIDEO_SCRIPT`、`ISP_CAMERA_SCRIPT` 覆盖。

## 📊 一个窗口中的内容

- 左侧：双目视频、L/R 检测框、类别、置信度和当前显示处理速率。
- 右侧：横滚 Roll、俯仰 Pitch、相对偏航 Yaw、俯视线条小车航向动画、三轴加速度和短时趋势。
- 底部：静态零偏校准、航向归零。启动后保持 IMU 静止约 2 秒。
- 摄像头超过 1 秒没有新图像，停止显示实时检测框；IMU 超过 500ms 没有有效新六轴采样，数值显示横线。

左右眼分别检测，因此“检测结果 N 项”是两眼检测结果总数，**不是立体匹配后的独立物体数量**。两只眼看到同一个瓶子时出现两个框是正常的。

沿用已工作的 IMU9 原始以太网协议和四元数融合。加速度包含重力，单位 m/s²；Yaw 是相对角，会漂移，不是磁北航向。未把未经磁力计校准的数据用于强制校正偏航。校准和航向归零只影响显示解算。

![统一窗口演示预览](reports/window-preview.png)

## 🔧 本次针对识别问题的修改

| 原代码中确认的问题 | 本版处理 |
|---|---|
| 双目合成的 1920×1080 整体缩到 640×640，每只眼只剩约 320×360 的有效画面 | 默认左右 960×1080 分别等比例补边到模型尺寸；640 模型约为每眼 569×640，有效细节更多 |
| 固定使用请求尺寸与紧密 NV12 排列，忽略驱动协商结果 | 使用实际宽高、像素格式、行跨度、平面数、缓冲长度和颜色元数据 |
| 取帧重试超时后仍可能使用未取到的缓冲区 | 明确处理超时、EAGAIN、EINTR、错误帧和截断帧，不产生假图像 |
| NV12→RGB 失败后依然推理，CPU fallback 又不支持该转换 | 自带有边界检查的 NV12/NV21/YUYV/RGB/BGR 转换，失败即停止本次视觉处理 |
| 框去除 padding 后仍按模型尺寸裁剪 | 使用实际缩放比例映射回单眼原图，按 ROI 边界裁剪，再映射至显示画面 |
| 推理、显示、逐框日志和每 30 帧自动写 PNG 串在一起 | 推理独立线程、及时归还相机缓冲、取最新可用帧、复用内存，默认不保存图片 |
| 没有检查模型输出是否与解码器相符 | 初始化验证 Rockchip YOLOv8 的 6/9 路 DFL/分类输出；不兼容模型明确报错 |

分类头本来已经做过 sigmoid，本版不重复 sigmoid。DFL 使用稳定 softmax，NMS 按类别并在每个物理相机内分别执行。帧图像和它的框一起发布给窗口，避免把旧框画到另一帧上。

## 🎯 是否更新 YOLO 模型

先用本版沿用现有 YOLOv8s 模型，修正输入与坐标问题，再用实际拍摄的同一批图片比较。当前 `YOLO/convert.py` 明确使用 `do_quantization=False`；不能把原来的识别不准直接归因于 INT8 量化。

本次没有训练或更换模型权重，也没有你实际场景的标注验证集，**不能给出准确率提升百分比**。现有 COCO 模型只能识别其类别表里的类别；要识别特定竞赛物品，应按实际左右摄像头图像标注、微调并保留独立验证集，比较漏检、误检和框位置。换一个更新版本并不替代这一步。

可比较以下模式，左右模式仍显示整个双目画面，但只在指定眼上推理：

```sh
./run-perception.sh --view both       # 默认：两眼分别推理，工作量约两次推理
./run-perception.sh --view left       # 只识别左眼，通常更快
./run-perception.sh --view right      # 只识别右眼
./run-perception.sh --view single     # 输入本身是单目图，整个画面推理
```

置信度默认 0.35，比旧程序 0.25 更严格：低分误报会减少，也可能增加漏检。可调整比较，不代表模型本身变准：

```sh
./run-perception.sh --confidence 0.25
./run-perception.sh --confidence 0.45 --nms 0.45
```

后续替换模型使用 `--model` 和匹配顺序的 `--labels`。仅支持与当前 Rockchip YOLOv8 分支结构兼容的静态输入检测模型；标准单输出 `[1,84,8400]` ONNX、YOLO11/26、分割/姿态模型不能仅改文件名直接运行。转换前可用 `tools/convert_rknn.py` 在开发机检查并转换兼容 ONNX；它不会训练模型。

## 🛠️ 构建与运行库

预编译程序为 ARM64 glibc ELF，自身最高需要 **GLIBC_2.33**。依赖 `libwayland-client.so.0`、`librknnrt.so`、`libdl.so.2`、`libm.so.6`、`libpthread.so.0`、`libc.so.6`；RKNN 库还依赖板上的 C++ 运行库。必须使用与板上 NPU 驱动匹配的 RKNN 运行库。没有打包开发机的共享库。

启动器已自动加入 `/root/rknn_yolov8_demo/lib`；若旧 YOLO 移到了其他目录，设置 `YOLO_DIR` 或 `LD_LIBRARY_PATH`。不要用其他系统的 libc/Wayland 覆盖板上的库。glibc 过旧需用匹配的 Buildroot SDK 重编译。

开发机可构建窗口演示、运行单元测试和无屏幕窗口协议测试：

```sh
make
make test
make test-window
make test-launcher
./build/perception-dashboard --demo
```

主机默认不链接 ARM64 RKNN；真实推理会明确提示当前构建没有 RKNN 支持。开发机有相应 Python 依赖与本地 ONNX 时可运行 `make test-onnx`，这是离线模型链路检查，不调用 NPU。

工作区内交叉编译：

```sh
./scripts/build-arm64.sh
```

其他 SDK 通过 `CROSS_COMPILE`、`WAYLAND_INCLUDE`、`WAYLAND_LIB`、`RKNN_INCLUDE`、`RKNN_LIB` 指定路径。在 ARM64 板上已安装开发头文件时可使用 `make RKNN=1`。本程序要求线性 NV12/NV21（单/双内存平面）、YUYV、RGB24 或 BGR24 的 V4L2 streaming 采集；不将 MJPEG、压缩或 tiled 图像冒充 RGB。

## ✅ 验证范围

严格 C/C++ 编译；IMU 协议、姿态、校准、去重、超时；图像格式/stride/颜色/截断；DFL、模型结构、补边、坐标还原、类别 NMS 和非法数值；窗口配置、双缓冲、帧回调、缩放、关闭；ARM64 构建和动态依赖已检查。详情见 `reports/build_and_tests.log`。

v2 仅更新启动器及部署说明，原生窗口和推理 ELF 的 SHA256 保持 `c5257290a5239ccc038b5b0f4de143e6644f614032b939a2441c710a4f78c1b8`。模拟启动测试覆盖 ISP 复用/启动、动态发现、NV12 等待/超时、库路径、参数优先级和演示跳过；不会调用实物 ISP。测试记录见 `reports/launcher_tests_v2.log`。

**尚未连接你的 RK3588 实测摄像头、NPU、IMU 网络和屏幕，也没有实际场景精度评测。** 示例预览中的瓶子和数值是演示数据。

参考：[Rockchip YOLOv8 部署说明](https://github.com/airockchip/rknn_model_zoo/blob/main/examples/yolov8/README.md)、[Linux V4L2 行跨度与格式](https://cdn.kernel.org/doc/html/latest/userspace-api/media/v4l/pixfmt-v4l2.html)、[Ultralytics 模型验证说明](https://docs.ultralytics.com/modes/val)。字体许可证见 `assets/FONT-LICENSE`，Wayland 协议源码保留原许可证。
