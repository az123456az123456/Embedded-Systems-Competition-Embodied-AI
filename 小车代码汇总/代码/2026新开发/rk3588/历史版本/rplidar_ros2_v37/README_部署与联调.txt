ACG138 / RK3588 Buildroot：RPLIDAR C1 → ROS 2 → SLAM → FPGA 地图桥
交付版本：v37，2026-10-29

先读这一段
本包是可构建的源码、配置、部署脚本、主机验证记录，以及两个已交叉编译的
ARM64 小工具。完整 ROS 2 用户空间镜像尚未构建，ROS 节点尚未实际编译，雷达
实板 SLAM 尚未验收。原因是当前开发环境没有 Docker/ROS/CMake，分区仅剩约
3.7GiB；没有把静态检查或 FPGA 仿真冒充 ROS 端到端成功。

已完成：真实 SDK 扫描转换核心、现有编码器/IMU 协议解码和轮式里程计、启动
配置、slam_toolbox 参数、保存/加载地图流程、复用现有 FPGA NAV2 协议的地图桥、
Buildroot 隔离用户空间部署及自启动脚本。构建完整镜像后，需先运行随包 ROS
仿真，再进行本文的实板验收。软件包没有替换系统、旧 YOLO 或手机 APK。

雷达外参已按本次确认写入：base_link→laser = (0,0,0.175)m，yaw=0；
雷达旋转中心在车体中心，正前方朝车头。base_link 的参考点必须与原轮式里程计
参考点一致。若实车“中心”的定义与旧车体参数不同，应先统一参考点再行驶。

目录
  ros2_ws/src/acg138_rplidar           C1 SDK 驱动、LaserScan 转换、SDK 源码与许可
  ros2_ws/src/acg138_base_telemetry    SNS1/IMU9 只读接收、真实编码器里程计、可选融合健康门
  ros2_ws/src/acg138_bringup           TF、SLAM/定位/仿真 launch、YAML、无GUI工具
  ros2_ws/src/acg138_fpga_map_bridge   OccupancyGrid→局部阻塞图→RGMII→现有 FPGA A*
  deploy/                            构建、导出、chroot、服务、udev、交叉编译模板
  deploy/bin/                        已编译 ARM64 串口探测/挂载命名空间工具
  tools/、reports/                    主机测试、真实 BSP 判断与验证边界

A. ROS 2 选型与 Buildroot 部署

A1. 推荐 Jazzy，保留现有系统
Humble 对应 Ubuntu22.04，官方支持到2027年5月；Jazzy 对应 Ubuntu24.04，支持
到2029年5月。当前是2026年10月，新增工程优先 Jazzy 的剩余维护期。RK3588 的
aarch64 与 Linux6.1.75 可承载该 ARM64 用户空间；运行时 libc/Python 等留在独立
rootfs，不覆盖 Buildroot 系统库。没有建议刷 Ubuntu。

当前 BSP 内核配置 SHA 与用户已给出的运行内核配置 SHA 相同：
7114825305fb4b5f032a523d009ae4eaa321f1c816ff60f44eeaca3b9a0ec20f
支持 namespace、CP210x、seccomp、overlay；保存配置没有 Docker、VETH/BRIDGE、
MEMCG/CGROUP_PIDS。优先 chroot+私有挂载命名空间；Docker 只作板端运行时已验证
后的备选。内核功能判断来自本地 BSP 保存配置，最终以 preflight 实机输出为准。

两条部署路径对比：
  (a) Buildroot 原生交叉编译：最终占用可小、适合长期产品化；但需移植整个 ROS
      core/DDS/Python 生成器和 Ceres 依赖闭包，不是仅交叉编译一个 C++ 文件。
      本包的 BR2_EXTERNAL 是 overlay 接入模板，要求先有同 ABI 的 ROS underlay。
  (b) 官方 ARM64 ROS 二进制+隔离用户空间：本次优先，易复现版本、保留现系统；
      rootfs 较大，构建电脑需 Docker/buildx，RK 用 chroot 无需 Docker daemon。

最小组件：rclcpp/rclpy、sensor_msgs/nav_msgs/geometry_msgs、tf2、slam_toolbox、
nav2_map_server；加载已存图时用 AMCL+Lifecycle Manager。robot_localization
可选，只融合轮速和陀螺 yaw rate；CycloneDDS 只选一个 RMW 实现。RViz 放开发电脑，
不装进小车运行图。Cartographer 依赖更重，本次不选。整套 Nav2 控制器不自动启动。

A2. 在空间足够的 Linux 开发电脑构建（不能在 Buildroot 上 apt install）
以下假设本目录叫 rplidar-ros2-v37。开发电脑建议至少12GiB空闲；x86需已经可用的
ARM64 buildx/QEMU 或 ARM64 构建节点。先检查并记录真实基础镜像 digest：
  cd /实际路径/rplidar-ros2-v37
  docker buildx inspect --bootstrap
  docker buildx imagetools inspect ros:jazzy-ros-base-noble
  sh deploy/build-image.sh

Dockerfile 直接构建四个本地 ROS 包并安装依赖。成功后，先执行真正的 ROS 仿真：
  docker run --rm --platform linux/arm64 --network none --cap-drop ALL \
    acg138-ros2:jazzy-v37 ros2 run acg138_bringup ros-smoke.sh
要求输出：PASS simulated ROS2 LaserScan -> slam_toolbox -> OccupancyGrid -> saved map
这一步运行合成 LaserScan+真实 slam_toolbox/map_saver，不开串口、不发电机指令。
本机尚未执行这一步。若失败，先看容器内 /data/logs/ros-smoke-launch.log；需要保留
产物时，把开发电脑专用的空目录挂到 /data，不要挂用户原数据目录。

然后导出整套 ARM64 用户空间：
  sh deploy/export-userspace.sh acg138-jazzy-v37-rootfs.tar.gz
得到 rootfs.tar.gz 和同名 .sha256。首次构建的实际镜像 identity、包版本清单在
reports/image/。正式锁版用检查到的实际 ros@sha256:... 设置 ROS_BASE，并归档
本次成功的 rootfs+SHA 或镜像；浮动 tag+包版本清单不保证将来重建字节一致。

A3. RK 安装和启动（不更新系统）
把本源码包解到 /root/rplidar-ros2-v37，把上一步两个 rootfs 文件也拷到 /root。
不要把 Ubuntu 的 /lib、解释器、libstdc++、.deb 解进 Buildroot 的系统目录。
  cd /root/rplidar-ros2-v37
  sh deploy/preflight.sh
  sh deploy/install-userspace.sh /root/acg138-jazzy-v37-rootfs.tar.gz
默认只写 /userdata/acg138-ros2/rootfs，拒绝覆盖已有安装。至少留4GiB可用空间，
最终以构建 rootfs 的实际展开大小为准。运行时还需地图/日志空间。

首次仅检查雷达，先停止 ultra_simple 或其他占用同串口的程序：
  SERIAL_DEVICE=/dev/ttyUSB0 sh deploy/run-chroot.sh \
    ros2 launch acg138_bringup scan.launch.py
停止用 Ctrl+C。不要同时启动两次 run-chroot.sh；它是独占运行会话。
再启动完整建图（默认 dry_run 地图桥，不写 FPGA）：
  SERIAL_DEVICE=/dev/ttyUSB0 sh deploy/run-chroot.sh

在另一个 RK 终端执行 ROS CLI，必须加入已运行会话，不能在 Buildroot shell 直接 ros2：
  cd /root/rplidar-ros2-v37
  sh deploy/exec-chroot.sh ros2 topic hz /scan
  sh deploy/exec-chroot.sh ros2 topic echo /scan --once --field header
  sh deploy/exec-chroot.sh ros2 topic echo /map --once --field info

chroot内部串口统一别名为 /dev/rplidar_c1，不要求宿主先存在同名别名。
宿主 SERIAL_DEVICE 指定实际设备即可。RK→FPGA 默认内部 GMAC eth0；不是 USB Wi-Fi。
如果接口未 UP，在宿主确认后执行 ip link set eth0 up；不需要给 NAV2 配 IP 地址。
chroot不挂整个 /dev 或 /sys，不给 NPU/电机设备额外权限；只传串口和 NET_RAW。
完整隔离和 Docker 条件路径详见 deploy/部署路径与命令.txt。

A4. 原生交叉编译模板（不是首次 MVP 的前置）
需要 host/target 分开构建：CMake、Python/empy/lark/setuptools、ament/rosidl
生成器、rcutils/rcl/rclcpp、RMW/DDS、TinyXML2、yaml-cpp、Eigen、Boost、Ceres/
SuiteSparse、slam_toolbox。目标 Python C 扩展、DDS、C++ ABI 必须匹配 BSP sysroot。
GCC10.3/glibc2.33 的 Buildroot 不能直接运行 Noble glibc 编译的普通 ELF。
已有同 ABI ROS underlay 后：
  export ACG138_TOOLCHAIN_BIN=/实际BSP工具链/bin
  export ACG138_SYSROOT=/实际BSP/sysroot
  export ACG138_ROS_UNDERLAY=/同ABI的ARM64ROS安装前缀
  sh deploy/native/cross-build-overlay.sh
无 underlay 会明确失败。可选 BR2_EXTERNAL 示例及独立 O=输出命令见 deploy 文档，
不要覆盖原 BSP output。本交付不声称已完成整个 Buildroot 原生 ROS 依赖移植。

B. C1 驱动和真实里程计

B1. 雷达固定参数与修正点
源码：ros2_ws/src/acg138_rplidar/src/c1_node.cpp；同包含 CMakeLists.txt、package.xml、
config/c1.yaml 与附件 SDK 源码。没有解析 printf 文本，直接使用 createLidarDriver/
createSerialPortChannel/connect/startScan/grabScanDataHq/ascendScanData 等 SDK API。
附件主包 SDK 宏版本实际为2.1.0，给定说明书为2.0；API与已跑通 sl_lidar 路径一致。

  serial_port      /dev/ttyUSB0（独立节点默认；隔离启动改为/dev/rplidar_c1）
  baudrate         固定460800，启动参数错误会拒绝，不回退低波特率
  frame_id         laser
  scan_frequency   10.0Hz；规格8–12Hz，支持转速设置时才尝试设置，实测频率另外统计
  angle_min/max    -π 到 π-2π/500，500个均匀角格默认，360°覆盖
  range_min/max    0.05 / 12.0m；黑色10%反射率规格只到6m，不能承诺所有目标12m

关键：SDK角度公式是 angle_z_q14*90/16384 度，不能用 angle_z_q14/16384 度。
距离 dist_mm_q2/4 得毫米，再除1000变米；质量从 packed quality 右移2位取得。
雷达原坐标顺时针为正，ROS为逆时针为正，已转换。Q=0、零距、超出范围均置 inf；
不能把激光的零回波照搬超声波逻辑设成“已观测12m空地”。同一角格保留最近有效回波。
LaserScan各格为均匀重采样，不制造插值回波；range_min/max也受 SDK 实际能力限制。

header时间戳为主机估计扫描开始，使用ROS当前系统时钟；没有雷达硬件时间同步。
原始顺时针采样重排成逆时针均匀角格后，time_increment=0，避免发布错误的逐束
正向时间关系。scan_time仍记录实测周期。暂未做严格逐点运动畸变校正，初次建图慢行。
驱动超时会计数并在连续故障达到阈值时退出，由服务重启完整会话；串口只允许单进程占用。

不安装 ROS 也可先在 RK 跑随包工具（它会启动/停止雷达，不控制小车）：
  cp deploy/bin/lidar-scan-probe.aarch64 /root/
  chmod +x /root/lidar-scan-probe.aarch64
  /root/lidar-scan-probe.aarch64 --port /dev/ttyUSB0 --seconds 10
该工具已用 BSP 的 aarch64-none-linux-gnu GCC10.3 实际交叉编译，动态依赖仅
libm/libpthread/libc，最高 GLIBC2.17；不是 ROS 安装包，也不替代驱动节点验收。

B2. TF 与里程计（移动SLAM不能省略）
  map --slam_toolbox/AMCL--> odom --轮式里程计或EKF健康门--> base_link
                                                               ├── laser (0,0,0.175)
                                                               └── imu_link
base_telemetry_node 只读现有 SNS1(0x88b8)/IMU9(0x88b5)以太网帧，不改变 FPGA。
使用当前车体标定：编码器60527.7 count/rev，M1..M4方向(-1,+1,-1,+1)，
左轮mask10(M2/M4)，有效轮周长0.20507m，标定轮距0.180635m。
这些沿用当前配置，不把旧6.4cm几何直径覆盖成另一份标定。实际轮位必须与当前位流一致。

默认直接用四路编码器的两侧均值积分真实 /odom+TF。不会拿静态 odom→base_link
来假装移动定位。序号、计数回绕、设备时间检查已实现；编码器跳变/FPGA重启/长断流会
冻结里程计并报 diagnostics，防止把重启后的新计数当连续运动。修复来源后重新建图会话。

网络包使用 Linux 内核 SO_TIMESTAMPNS 接收时间；应用积压>250ms、缺时间戳、时间
倒退或未来包均拒绝，避免 CPU 忙完把旧数据重新标为“刚到”。IMU再扣除源数据年龄。
这是接收至应用排队修正，不是设备发送时钟同步，不能保证未知传输延迟为零。

IMU先保持静止约2s完成零偏校准；发布 SI单位 angular_velocity/linear_acceleration，
不把 MPU9250 积分 yaw 冒充磁北绝对航向。默认 use_ekf=false；验证模块+Z方向、yaw
正负和安装姿态后才试 use_ekf:=true。可选EKF只用轮式速度+陀螺Z角速度，不直接积分
加速度，不使用未验证磁力计航向。旧IMU位置TF来自原配置，其高度/姿态须实车复核。
EKF模式仅健康门发布 /odom+TF；不会与轮式TF同时发布同一变换。

验证（小车停车，手动低速测试另行在安全范围进行）：
  sh deploy/exec-chroot.sh ros2 topic echo /base/diagnostics --once
  sh deploy/exec-chroot.sh ros2 topic hz /wheel/odometry
  sh deploy/exec-chroot.sh ros2 run tf2_ros tf2_echo odom base_link
直行0.2–0.5m时 odom x应增加；逆时针转向时 yaw应增加。若轮位/符号错，先修参数。
不要自动关闭原编码器/近障保护去掩盖错误，也不要在此阶段开启路线执行。

C. slam_toolbox 建图、保存、加载

C1. 在线建图
  SERIAL_DEVICE=/dev/ttyUSB0 sh deploy/run-chroot.sh \
    ros2 launch acg138_bringup mapping.launch.py interface:=eth0
默认0.05m分辨率，地图约1s更新，回环检测开启，实际 map→odom 由 slam_toolbox 提供。
先用手机原有手动控制低速走小环路，保留近障保护；本 ROS 包不发布 /cmd_vel、MOT 或使能。
桥默认dry_run，和旧导航并行观察不会写FPGA地图。若旧程序会自动导航，先停自动导航。
仅干净雷达扫描还不等于 SLAM成功，需检查 /odom、TF、地图和闭环结果：
  sh deploy/exec-chroot.sh ros2 topic hz /scan
  sh deploy/exec-chroot.sh ros2 topic echo /map --once --field info
  sh deploy/exec-chroot.sh ros2 run tf2_ros tf2_echo map base_link
  sh deploy/exec-chroot.sh ros2 topic echo /fpga/map_metadata --once
地图发布若停车后不变不意味着雷达停止；观察扫描频率、时间戳与 TF新鲜度。

C2. 无GUI保存
在建图会话仍运行时，另一个终端：
  sh deploy/exec-chroot.sh ros2 run nav2_map_server map_saver_cli \
    -f /data/maps/site --ros-args -p save_map_timeout:=10.0
生成 /userdata/acg138-ros2/maps/site.yaml 和 site.pgm（宿主持久保存），可拷到电脑查看。
可选保存SLAM位姿图，便于以后继续建图；先 ros2 service list 核对服务存在：
  sh deploy/exec-chroot.sh ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph "{filename: '/data/maps/site'}"
栅格图用于 AMCL，位姿图用于 slam_toolbox 会话恢复，两者不能互相替代。

可选录包复现：
  sh deploy/exec-chroot.sh ros2 bag record -o /data/bags/site-run \
    /scan /tf /tf_static /wheel/odometry /odom /imu/data /map /base/diagnostics
主动停止录包，定期清理；2GiB RAM和eMMC容量有限，不长期无界录制。

C3. 无GUI加载和定位
先 Ctrl+C 停止建图会话，然后：
  SERIAL_DEVICE=/dev/ttyUSB0 sh deploy/run-chroot.sh ros2 launch \
    acg138_bringup localization.launch.py map:=/data/maps/site.yaml
此时 map_server加载图，AMCL等待真实初始位姿；不能默认车还在上次关机位置。
已知实际位姿例如地图(0,0)、朝向0°时，在另一个终端明确设置：
  sh deploy/exec-chroot.sh ros2 run acg138_bringup initial_pose.py \
    --x 0 --y 0 --yaw-deg 0
再检查 map→base_link 和定位一致性，不能只看程序无报错就执行行驶。
静态保存图允许 map_timeout_s=0；机器人TF仍必须新鲜，突然出现的障碍仍需实时安全层。

D. 通过现有 RGMII 把地图发给 FPGA

D1. 选择与范围
明确使用现有内部千兆RGMII链路。无线网卡仅手机使用；不新增SPI/UART/PCIe。
当前 FPGA 实际支持 NAV2 v1(0x88b9)和64×64 A*，本包按真实RTL/固件适配，
不发送一个旧FPGA根本不理解的“新地图头”。NAV2字符串是本项目协议名，不是ROS Nav2。

从 /map OccupancyGrid 和新鲜 map→base_link TF 截取局部6.4×6.4m，0.1m/格，
64×64、4096格。4096bit=512B；1阻塞，0已观测可通行。未知和不确定格按阻塞。
采用面积覆盖的保守降采样，避免细墙被采样漏掉；按车体最大外廓半径0.2163m加0.05m
膨胀，边界阻塞。不盲目清空自车周边未知区域。

已有线协议没有 resolution/origin/frame字段。因此这些米制元数据留在RK，和
generation/CRC一起发布到 /fpga/map_metadata 和 /tmp/acg138_ros2_fpga_map.json；
FPGA只处理该代图的格号。RK负责米制目标→格号、路径格号→米制路径。不能把不同代图的
origin和路径混用。如以后要求FPGA独立按米制定位，必须升级协议和FPGA，不是只改RK。

D2. 实际格式与接收状态机
每个Linux原始以太网发送帧80B，网卡自动追加FCS：
  目标MAC6 + 源MAC6 + EtherType2(0x88b9) + NAV2 body64 + CRC16 2
多字节整数字段大端，不发C结构体内存布局。body：
  [0:4] magic NAV2，4版本1，5操作，6状态，7数量
  [8:12] request_id，[12:16] generation
  [16:20] arg0，[20:24] arg1，[24:28] arg2，[28:32]保留0
  [32:64] data[32]
CRC16-CCITT(poly1021,initffff)校验body；整图用CRC32/IEEE。
bitmap格号=y*64+x，每字节低bit优先。一次事务：
  INFO同步旧序号/代号 → BEGIN(整图CRC32) → 16×CHUNK(32B) → COMMIT → ACK
逐条应答匹配操作/序号/代号；500ms超时最多同请求重发3次。全部片齐全且整图CRC通过
才把staging原子切换为active，少片/坏CRC不会安装半图。PLAN可选，PAGE取回路径。

RTL接口约定/状态机摘要：
  GMII RX→前导/MAC/TYPE→长度/FCS/CRC16→command_valid单脉冲
  →跨时钟mailbox→CPU操作/序号检查→分片staging→COMMIT验证→active→ACK
真实接口与完整字段/错误码见 acg138_fpga_map_bridge/README_PROTOCOL.txt 及
tests/reference/nav_packet_rx.v、nav_protocol.c；这些为现有工程冻结参考，未改旧位流。

默认1Hz，每次19个小请求（含INFO），含ACK和线速开销仍仅几十kbit/s；不挤满视频链路。
发送线程与ROS回调分离，只保留一个尚未开工的最新快照。map超时10s/pose超时0.25s
就拒绝上传。网络层通过 Channel接口封装，将来换传输只需替换send/receive适配。

D3. 先预览，再实发
默认建图入口已运行桥，查看：
  sh deploy/exec-chroot.sh ros2 topic echo /fpga/map_metadata --once
  sh deploy/exec-chroot.sh ros2 topic echo /fpga/local_costmap --once --field info
同一时刻只允许一个 RK程序写 FPGA NAV2地图。真上传前先停车、取消使能，停旧
robot-navigation及其自动重启入口；别只kill后让run-dashboard又拉起。核对：
  ps -ef | grep '[r]obot-navigation'
应无旧导航进程。新桥共享 /run/lock 并检查宿主进程；旧导航不认识新锁，仍必须由部署
关闭其自动重启，exclusive_nav2_owner是操作者的明确独占保证。

停止旧 ROS 建图会话，再实发（仍然不驱动电机）：
  SERIAL_DEVICE=/dev/ttyUSB0 sh deploy/run-chroot.sh ros2 launch \
    acg138_bringup mapping.launch.py interface:=eth0 \
    dry_run:=false exclusive_nav2_owner:=true
验收 metadata 中 uploaded=true、generation增长、CRC与geometry同代。没有INFO ACK
说明当前位流/链路不匹配，不得把无应答当成功。本次无需新增FPGA位流或资源。

可选让现有 FPGA A* 寻路，启动时额外 enable_plan:=true；然后另一个终端：
  sh deploy/exec-chroot.sh ros2 topic pub --once /fpga/goal geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}"
  sh deploy/exec-chroot.sh ros2 topic echo /fpga/planned_path --once
目标必须在本代局部图已观测可通行区域。超过6.4m窗口的全局目标需RK选择局部子目标。

边界：现有 FPGA 只做四连通 A*，不是完整轨迹跟随器。本交付不自动把 /fpga/planned_path
接入电机，也不替换手机导航。地图上传、算出Path、车辆安全沿Path行驶是三件需要分别
验收的事。后续路线执行应接当前授权/近障层并检查同代地图、TF、路径时效，不能旁路保护。

E. udev、开机服务、验收与排错

E1. 精确设备别名
  grep '^dialout:' /etc/group || addgroup -S dialout
  sh deploy/generate-udev-rule.sh /dev/ttyUSB0 > /tmp/99-rplidar-c1.rules
  cat /tmp/99-rplidar-c1.rules
核对唯一USB序列号后：
  cp /tmp/99-rplidar-c1.rules /etc/udev/rules.d/99-rplidar-c1.rules
  udevadm control --reload-rules
  udevadm trigger --subsystem-match=tty
  ls -l /dev/rplidar_c1
规则MODE0660/GROUPdialout，只匹配此一台C1，不把全部CP210x设0666。无唯一序列号时
生成器拒绝，可按真实 ID_PATH 固定USB口，或保持 SERIAL_DEVICE=/dev/ttyUSB0。
BusyBox stty无460800不影响SDK设置；不要用它改成其他波特率。

E2. SysV自启动（手动验收后再加）
把整个本包目录复制到 /opt/acg138-ros2，保持 deploy 相对路径。配置里使用真实串口：
  cp deploy/acg138-ros2.conf.example /etc/acg138-ros2.conf
  cp deploy/S80acg138-ros2 /etc/init.d/S80acg138-ros2
  chmod +x /etc/init.d/S80acg138-ros2
  /etc/init.d/S80acg138-ros2 start
  /etc/init.d/S80acg138-ros2 status
  tail -n 30 /userdata/acg138-ros2/logs/service.log
  /etc/init.d/S80acg138-ros2 stop
服务2/4/8/16/30s退避重启，5MiB日志轮转；只管理本会话，不杀YOLO，不自动使能电机。
驱动/底盘接收失败会退出整个会话，避免半死节点假装SLAM仍正常。逻辑故障冻结里程计
时先看 /base/diagnostics 并解决源故障，不能循环自动清零把地图漂移掩盖。

E3. 逐层验收
  雷达：10s观测 /scan 约8–12Hz（常用10），0.05–12m参数正确，墙方向/左右符合实物；
        只有SDK支持的回波视为观测，黑物体弱回波不能误当远处空旷。
  底盘：真实 /wheel/odometry 与 /odom 更新，前进/逆时针符号正确，无长时间旧TF；
        FPGA重启或计数跳变明确报错，不产生巨大位置跳跃。
  SLAM：/map 同时有free/occupied/unknown；慢速闭合路线后地图能正确重合，保存图可读；
        再加载并设初始位姿后 map→base_link 与真实位置相符。回环效果须实测，不预先保证。
  地图桥：dry_run元数据正确；live约1Hz完整ACK/CRC、uploaded=true；坏帧/断链不假成功；
          恢复后INFO重新同步，完整COMMIT，新generation路径不混旧geometry。
  并行负载：同时开YOLO时比较 /scan、TF年龄、内存、地图更新、原PERF。2GiB平台不跑RViz，
            初次小场地、限制地图增长/录包；不宣称ROS启动后YOLO仍无损满帧。

常见故障和原因：
  没有/scan：设备名/权限、串口被ultra_simple占用、USB供电、SDK连接/扫描模式失败。
  invalid baudrate：C1只准460800；节点参数不符即拒绝。
  TF extrapolation / map不出：没有真实odom、时钟倒退、旧包积压被拒绝、帧名不一致。
  odometry frozen：计数跳变、FPGA重启、非法编码器、时钟/设备间隔异常；看base/diagnostics。
  start/goal blocked：未知/薄墙/膨胀边界占据起终点；先补观测/核对外参，不盲目清格。
  NAV2 ACK timeout：错误GMAC、旧位流无接收器、链路未UP或FCS/CRC错误，不能用Wi-Fi接口。
  owner/process conflict：旧robot-navigation还运行或其自动重启没停、另一个地图桥写同接口。
  chroot lock：有会话在运行，或被强杀后留下锁；先确认PID及命名空间，不能盲删运行中锁。
  ELF/GLIBC错误：把Ubuntu ROS二进制在Buildroot宿主直接跑；应通过隔离rootfs启动。

已验证范围与复跑
详见 reports/交付验证.json 和各独立报告。主要测试覆盖真实SDK转换、协议解码、
轮式里程计、内核排队时间戳、地图保守转换、真实C固件事务和A*、真实FPGA接收RTL。
launch组合测试用窄适配器，不是编译ROS；ROS运行仿真脚本必须在构建后的环境实际执行。
本机/PC可复跑：
  python3 tools/test_base.py
  python3 tools/test_bringup.py
  python3 ros2_ws/src/acg138_rplidar/tests/run-host-tests.py
  python3 ros2_ws/src/acg138_fpga_map_bridge/tests/run_host_tests.py
需要Linux C/C++编译器、Python3/PyYAML、Icarus；时间戳测试需内核允许SO_TIMESTAMPNS。
安装运行ROS不需要这些主机仿真工具。部署脚本验证记录见reports/deploy-verification.json。

最小可行路径 MVP
  1. RK独占串口，跑随包ARM64探测，确认460800与扫描方向/频率。
  2. 空间足够的开发电脑构建Jazzy镜像，执行ros-smoke.sh；成功后导出rootfs+SHA。
  3. RK安装独立rootfs，先scan.launch.py验证/scan，再mapping.launch.py验证真实odom/TF。
  4. 手机原手动控制低速走小环路，观察/map并保存site.yaml/pgm。
  5. 停建图→加载site.yaml→设置真实initialpose→确认AMCL定位。
  6. 停旧FPGA地图写入者，再开启live地图桥；最后才测试FPGA A*返回路径。
  7. 路线执行/手机ROS地图展示另作显式集成和安全验收，不把本包规划路径直接自动开车。

主要依据（本包源码或附带资料优先）
  ROS发行版支持：https://github.com/ros-infrastructure/rep/blob/master/rep-2000.rst
  Jazzy发布：https://www.openrobotics.org/blog/2024/5/ros-jazzy-jalisco-released
  官方ARM64镜像：https://hub.docker.com/_/ros/
  官方交叉编译说明：https://github.com/ros2/ros2_documentation/blob/jazzy/source/How-To-Guides/Cross-compilation.rst
  Slamtec驱动：https://github.com/Slamtec/sllidar_ros2
  slam_toolbox：https://github.com/SteveMacenski/slam_toolbox/tree/jazzy
  附件C1规格书：SLAMTEC_rplidar_datasheet_C1_v1.1_cn.pdf（量程/频率/坐标）
  附件SDK手册：LR002_SLAMTEC_rplidar_sdk_v2.0_cn.pdf（SDK接口）
