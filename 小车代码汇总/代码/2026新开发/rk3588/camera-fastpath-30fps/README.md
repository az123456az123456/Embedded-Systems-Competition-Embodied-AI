# ACG138 摄像头接收端帧率优化

> 日期：2026-10-19 · 20FPS 实验版（基于最终 fastpath）· 双 OV5640 1920×1080 RAW8 → RGMII/UDP → RKAIQ ISP → NV12

## 🎥 20FPS 实验配套

本次只修改接收程序的 V4L2 标称帧间隔，配合 FPGA 中的 20FPS 传感器初始化表；实际帧率仍由 RAW 到达和 ISP 完成计数决定，不在接收端定时重复图像。不改模型、分辨率、UDP 线格式、RAW 交接/停止/超时行为。参考启动器仍创建 60/61/62 三个 loopback 节点，接收程序仍只向 `ISP_V4L2_OUTPUT` 指定的 combined 节点（默认 video60）发布 1920×1080 NV12；没有新增左右节点发布。

源码默认 `CAMERA_DEFAULT_FPS=15`，实验交付二进制以 `CAMERA_DEFAULT_FPS=20 sh scripts/build-arm64.sh` 构建。因此只替换 `isp-probe` 即可配合旧板端启动器，启动器不需要换名或包装。启动前的 `CAMERA_TARGET_FPS=15`、`20`、`30` 可覆盖编译默认值；只接受这三个完整字符串，非法或空值在 RKAIQ、V4L2 和网络操作前拒绝。**该变量不会配置 FPGA 摄像头，切回真实 15FPS 仍需恢复相应 FPGA 位流。**

`V4L2_INTERVAL requested_fps=20 negotiated_fps=20.000 nominal_only=1` 只表示向 loopback 请求及协商的帧间隔；不能作为 20FPS 验收。`FASTPATH rx_fps` 才是完整 RAW 接收速率，`FASTPATH output_fps` 是实际 ISP 完成并写出速率。程序在已验证 `FPGA_` 卡名后关闭 loopback 的 `sustain_framerate` 和 `timeout` 控制，避免之前会话遗留的重复帧和超时图像干扰测量。

FakeCam 隔离库沿用 v4 的 5ms pending-buffer 轮询、RAW_DATA 拷贝和回调修复；**本次只更新 `isp-probe` 的板端包要求现有隔离目录已安装最终 fastpath v4 专用库**，不能拿未启用 FakeCam 或仍为 100ms 轮询的系统原库代替。没有发现固定 15FPS 的接收限制，本次无需替换系统库或重编 FakeCam。BSP 的 `FakeSensorHw::get_sensor_fps` 仍固定返回 25，用于离线曝光描述及像素时钟推算，不是 RAW 输入节流器；本次不声称离线曝光时序描述与物理 20FPS 严格对齐。没有调用 `rk_aiq_uapi2_setFrameRate` 去改变传感器或伪造采集节奏。当前是固定 RAW 元数据与代理 IQ 链路，真实曝光、防闪烁和亮度应上板检查。

执行 `make test test-fps` 可验证真实行重组、mock V4L2 的 15/20/30 `VIDIOC_S_PARM`、NV12 格式、协商值记录、禁止重复帧控制，以及非法参数在设备/网络操作前退出。严格交叉构建结果与限制记录于 `reports/rk3588-20fps-validation.txt`。

## ⚡ 已修改

以仓库内 `rk3588_acg138/14_FPGA_摄像头1080p/project/tools/camera/` 的已交付 ISP 程序为基线建立独立新工程。旧资料包没有改动。

原循环接收一帧后才开始 ISP 处理、写调试文件和发布 NV12，这段期间停止收包。现在用独立接收线程、每次最多 32 个数据报的批量接收，持续按行重组；只有从 0 连续到 1079 行的完整帧才交接。接收使用两张有界 RAW 图像缓冲，消费者只拿最新完整帧，不积累旧视频；处理中被后来的完整帧替代会计入 `rx_superseded`。

ISP 原始输入包改为复用，仍在完成回调之后才覆盖；停止 AIQ 成功之后才释放。默认关闭每帧调试 RAW 文件和逐帧日志；需要原调试输出时，在启动 ISP 服务前设置 `ISP_SAVE_FRAMES=1`。保留代理 IQ、格式、色彩范围转换和单路 ISP 处理接口。配套重编译原 SDK 的 FakeCam 专用库：待处理 RAW 缓冲的轮询间隔从固定 100ms 缩短至 5ms，保留原来已交付的 RAW_DATA 拷贝与回调迭代器修复。原库的 100ms 定时通知会限制 RAW 提交速度；新定时器只唤醒实际排队的缓冲，不补帧。

每 30 张输出图像记录一行 `FASTPATH`：`rx_fps` 为接收线程重组成功的完整帧率，`output_fps` 为 ISP 处理并发布的帧率；`incomplete` 统计观察到的行跳变，`malformed` 统计长度错误，`rx_superseded` 是主动替换完整旧帧，并不等于网卡丢包。`wait_ms / isp_ms / output_ms` 是本轮平均耗时；`raw_age_ms` 从完整帧收到时计起，不是曝光时刻。

旧线协议只有行号，没有帧号，无法检测所有跨帧或跨相机错配；这一兼容升级不声称补上了不存在的帧号或做了硬件双目同步。

## 🔧 构建和更新

本机执行 `./scripts/build-arm64.sh`，输出 `build/isp-probe`。可用 `BSP_SDK / ARM64_CC / RKAIQ_INCLUDE / RKAIQ_LIB` 指定兼容 SDK。使用 `CMAKE=/兼容cmake路径 sh scripts/build-fakecam.sh` 重编译隔离的 FakeCam 库，输出 `build/lib/librkaiq.so`；源码补丁在 `scripts/patch-fakecam.py`，不修改原 SDK。仅更新 `/opt/fpga-camera-isp-1080/lib/librkaiq.so`，不替换系统 RKAIQ、IQ、内核或 v4l2loopback 模块。

本次板端发布包使用 `install-20fps.sh`，仅备份并原子替换 `isp-probe`，保留桌面程序、原启动器、模型和隔离库。`rollback-15fps.sh 备份目录` 恢复旧二进制；真实 15 FPS 还须恢复原 FPGA 位流。若隔离目录不同，用 `ISP_DIR` 指定。这两个脚本及 `fps-update-common.sh` 在板包根目录与二进制、SHA256SUMS 放在一起运行。

源码中保留的旧 `scripts/install-rk3588.sh` 对应历史桌面＋ISP 整包，不用于本次实验；本次交付板包不包含该旧安装器。

这里的 `run_isp_camera.sh` 是可选参考启动器，改为按 `fe1c0000.ethernet` 自动定位内部接口，或用 `FPGA_INTERFACE` 显式指定；不猜测 eth0。常规升级不覆盖原启动器，因此保留用户实板已跑通的节点配置。该参考脚本自身仍采用原资料包的 `/dev/media2 /dev/video22 /dev/video27 /dev/video60`，只应在设备布局一致时使用。

## 📊 实板对比

关闭旧窗口，更新后先执行 `./run-perception.sh --no-detect`，再执行普通 `./run-perception.sh`；观察终端 `PERF` 和 `/tmp/fpga-isp.log` 的 `FASTPATH`。

| 现象 | 下一步 |
|---|---|
| `rx_fps` 约 15，`output_fps` 明显低 | ISP 或 NV12 输出为瓶颈，查看 `isp_ms / output_ms` |
| `rx_fps` 约 15，`output_fps`、`CAP` 约 15，`AI` 约 5 | 识别计算为瓶颈，比较 `--serial-eyes`、`--view left`；需要更快模型时再做同场景精度验证 |
| `rx_fps` 也约 5 | 查摄像头串口诊断中的 VSYNC、曝光/光照、网卡丢包和双目配对；窗口线程优化不能补出未收到的图像 |
| `incomplete` / 网卡接收错误持续增加 | 查链路速率、MTU≥1950、接收缓冲和 RGMII 时序，不能把残缺帧计作成功 |
| `CAP` 高于 `output_fps` | 检查是否取错节点或 v4l2loopback 在重复帧；以接收和 ISP 输出计数为准 |

`scripts/diagnose-fps.sh` 默认仅读取进程、内部 GMAC1、UDP、温度/频率和测量日志。`capture /dev/videoN` 会暂时占用指定节点做无 YOLO/无 RGB 转换的 V4L2 取帧测试，必须先关闭其他视频消费者，保留 ISP 运行；限时 25 秒。

## ✅ 验证范围

新接收程序已做严格 ARM64 构建；专用库按原 SDK 编译选项构建，原厂源文件仍有编译警告。已检查所需动态库和导出 API；1920×1080 行数据接收、漏行/乱序/短包拒收、慢消费者期间连续接收、最新完整帧、步长填充和退出测试。未连接实际 RK3588、OV5640、ISP 和屏幕，没有实测加速倍数，也不保证 20FPS/30FPS 或 YOLO 同比例提升。本次实验 FPGA 初始化名义目标20FPS，须与相应位流一起验收。

技术参考：[Linux recvmmsg](https://man7.org/linux/man-pages/man2/recvmmsg.2.html)、[Linux V4L2 缓冲语义](https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/buffer.html)、[Rockchip NPU 核选择 API](https://github.com/airockchip/rknn_model_zoo/blob/main/3rdparty/rknpu2/include/rknn_api.h)。
