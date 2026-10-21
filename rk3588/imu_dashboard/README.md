# MPU9250 简洁窗口 · Buildroot / RK3588

> 2026-10-12 · ACG138 · Weston / Wayland

## 🖥️ 使用

FPGA 和原来的 IMU9 网络协议保持不变。将发布包解压到 RK3588 的可写目录，在已能接收 IMU 数据的 root 终端执行：

```sh
tar -xzf imu-dashboard-buildroot-arm64.tar.gz
cd imu-dashboard
chmod +x run-imu.sh imu-dashboard.aarch64
./run-imu.sh
```

程序自动识别 GMAC1。若之前接收程序需要指定接口，这里指定同一个：

```sh
./run-imu.sh --interface eth1  # eth1 只是例子，使用你实际的接口名
./run-imu.sh --fullscreen     # 铺满当前屏幕
./run-imu.sh --demo           # 不接传感器查看动态演示，右上角明确标记“演示数据”
```

窗口支持鼠标和触摸：拖动顶部标题区域移动窗口，右上角 × 退出；“重新校准”重新测量陀螺仪零偏；“航向归零”把当前偏航设为 0°。也可在启动终端按 Ctrl+C 退出。

启动后保持传感器静止约 **2 秒**，底部校准进度达到 100% 后自动显示温度及采样序号。若模块移动，校准会重新累计。校准针对陀螺仪静态零偏，不是加速度六面校准或磁力计校准。

## 📊 显示与姿态含义

| 显示 | 含义 |
|---|---|
| 横滚 Roll | 绕传感器 X 轴，单位 ° |
| 俯仰 Pitch | 绕传感器 Y 轴，单位 ° |
| 偏航 Yaw | 相对初始方向的航向，单位 °，可归零 |
| 加速度 X / Y / Z | 传感器机体坐标，m/s²，同时显示 g；**包含重力** |
| 姿态仪 | 实时横滚/俯仰示意 |
| 趋势 | 近约 6 秒三轴加速度 |

静放在水平面上时，朝上的轴通常接近 ±9.81m/s²，另外两轴接近 0；这是重力的正常读数，不是程序没有清零。

采用加速度与陀螺仪的四元数互补融合，按 FPGA 采样时间计算积分间隔。同一六轴序号只处理一次，磁力计更新帧和心跳不会重复积分。明显偏离 1g 时暂不使用加速度修正倾角。连续新样本累计静态陀螺仪零偏。

**偏航是相对角，会漂移，不是磁北或绝对航向。** 当前没有使用未经安装坐标确认、硬铁/软铁校准的磁力计去强行修正偏航。大幅持续加速、振动会影响倾角精度；本程序是数据显示和联调仪表，不是精密导航解算器。角度在传感器原生坐标系下，模块安装方向变化需另做坐标映射。

超过 500ms 没有有效六轴数据时，数字变为破折号，右上角显示超时，趋势断开；不把旧数值当成当前数据。采样时间间隔超过 200ms 时重新建立倾角参考，航向也重新开始。检测到 FPGA 运行时间回退时会重置姿态和零偏校准。

## 🔌 Buildroot 依赖

发布程序是 **ARM64 动态 ELF**，依赖板上的 `libwayland-client.so.0`、`libm.so.6`、`libc.so.6`。需要 glibc 的 ARM64 Buildroot、Wayland ≥ 1.20，以及支持 xdg-shell / wl_shm 的 Weston。工作区 BSP r4 的配置满足这些条件。无需 Python、Qt、浏览器或额外字体包；中文和数字已编入程序。

`run-imu.sh` 沿用当前图形会话变量；没有变量时寻找现存 Wayland socket。资料包中的默认路径为 `/var/run/wayland-0`。若从串口/SSH 启动后提示无法连接桌面，可明确指定现有桌面：

```sh
XDG_RUNTIME_DIR=/var/run WAYLAND_DISPLAY=wayland-0 ./run-imu.sh
```

如果实际 socket 不在这里，使用实际路径。脚本不会停止/启动 Weston、修改网卡、覆盖系统库或更换设备树。原始以太网接收需要 root 或 CAP_NET_RAW；图形会话也需要当前用户有权限访问。

常见问题：

- `Cannot connect to Weston`：核对上面的会话变量和 socket 权限。
- `libwayland-client.so.0` 缺失或 `wl_proxy_marshal_flags` 找不到：当前系统与此 BSP 的 Wayland 环境不同，需用匹配的 Buildroot SDK 重编译或启用对应软件包，不要复制电脑的系统库覆盖板端。
- 找不到网卡：使用 `--interface` 指定原接收程序成功使用的接口，确认它仍处于 UP 状态。
- 窗口等待数据：先确认原命令行程序仍能读到 IMU9 帧，再检查是否加载正确 FPGA 位流。
- 希望后台运行：在确认前台窗口正常后可执行 `./run-imu.sh >imu-window.log 2>&1 &`。

## 🛠️ 源码与编译

| 文件 | 作用 |
|---|---|
| `src/main.c` | 原始以太网接收、Wayland 窗口、双缓冲、鼠标/触摸、演示模式 |
| `src/imu_data.c` | IMU9 解码、四元数融合、去重、静态零偏、超时 |
| `src/dashboard.c` | 中文界面、姿态仪、曲线绘制 |
| `assets/font_atlas.h` | 内置中文/数字字形，来源 Noto Sans CJK；许可证见 FONT-LICENSE |
| `vendor/xdg-shell*` | Wayland 官方 xdg-shell 协议及生成接口，保留源许可证 |
| `tests/` | 融合/解析测试和无显示器 Wayland 协议测试 |

在 Linux 开发机上：

```sh
make                       # 本机程序，需要 Wayland 开发头文件和库
make test                  # 协议、姿态、校准、断流、预览
make test-window           # 无界面 Wayland 生命周期测试，需要 libwayland-server 和 Python3
./build/imu-dashboard --preview build/preview.ppm
```

用匹配的 Buildroot SDK 交叉编译：

```sh
make arm64 \
  CROSS_COMPILE=/path/to/host/bin/aarch64-buildroot-linux-gnu- \
  WAYLAND_INCLUDE=/path/to/staging/usr/include \
  WAYLAND_LIB=/path/to/staging/usr/lib
```

本次用工作区 ARM GNU 10.3 编译器和已有 ARM64 Wayland 开发文件编译；程序自身所需的 glibc 符号版本均为 2.17，没有嵌入工作区库路径。发布包不捆绑其他发行版的共享库。

## ✅ 验证范围

已通过：有符号单位转换、帧字段校验、已知 30° 横滚/俯仰、90°/s 偏航积分、重力修正收敛、陀螺仪零偏、重复帧去重、数据超时、自由落体拒绝初始化、动态加速度门限、长时间间隔、32位计数回绕、FPGA重启；无界面 Wayland 配置/应答、缓冲释放、帧回调、800×544 缩放和关闭；ARM64 编译及动态依赖检查。界面已检查实际渲染预览。

**尚未在你的 RK3588 上运行验证这个窗口。** 无界面协议测试不代替实际 Weston、鼠标/触摸和屏幕测试。你已确认的 FPGA→RK3588 采集链路保持原样。

姿态数学参考：[AHRS Mahony 四元数互补滤波说明](https://ahrs.readthedocs.io/en/latest/filters/mahony.html)。窗口协议来自 [Wayland 官方协议仓库镜像 1.32](https://github.com/wayland-mirror/wayland-protocols/blob/1.32/stable/xdg-shell/xdg-shell.xml)。
