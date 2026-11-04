ACG138 ROS 2 → FPGA RGMII 地图桥（v37，兼容现有 NAV2 v1）
======================================================

1. 实际核对的 FPGA 能力与边界
-----------------------------
本包读取并逐字节复用以下现有文件的 ABI，没有虚构 FPGA 已支持的新协议：
  fpga/robot_control_v36_raw/rtl/nav_packet_rx.v
  fpga/robot_control_v36_raw/softcore/firmware/nav_protocol.{c,h}
  fpga/robot_control_v36_raw/softcore/firmware/astar.{c,h}
  rk3588/robot_control_v36/navigation/protocol.{cc,h}
这些文件与项目当前 v29.1 RAW 路径的导航消息保持兼容。最终仍需实际核对
用户烧录的 .fs 是否含 NAV2 应答；没有 INFO ACK 时本包报错，不假装已上传。

容量是固定 64×64 格、4096 格、512 字节，不是 32×32。
每格只有一位，1=阻塞，0=已观测可通行。索引 cell=y*64+x，字节 bitmap[cell/8]
内 bit(cell%8)，LSB-first。FPGA 四连通 A*，不是 D*、8 邻接或整套 ROS Nav2。

FPGA 只接收局部阻塞图、start/goal 格号，返回路径格号。现有 NAV2 ABI 完全没有
分辨率、世界原点、frame_id、姿态或米制坐标字段。它也不含‘沿返回路线开车’指令。
这些 geometry 在 RK 保存，节点把米制 start/goal 转换成格号，收到路径后反变换成
/fpga/planned_path。把地图上传成功不等于车辆会自动启动；本节点从不发送 MOT1、
MOT2、MOT3、MOT4、使能或停车指令，现有手机控制也不会自动接上 ROS 规划路径。
后续路线跟随器必须显式接入，检查同一代地图、路径时效、TF、近障安全，再控制电机。

无需新增 FPGA BSRAM 或 LUT。本次桥复用已烧录的 NAV2/A*，不提供新位流。
原始 C 结构体 nav_server 的 host sizeof 见 tests/reports/host-verification.json。
该测试数值是宿主机结构布局证据，不替代 FPGA 综合 RAM 使用率报告。
真实 top 实例化 robot_nav_soc→nav_request_mailbox→nav_process→nav_message_gmii；
robot_nav_soc 的 16384×32bit RAM 从 softcore/firmware/firmware.hex 初始化。
v36 raw 与 v29.1 的该 firmware.hex SHA256相同，见tests/reference/ORIGIN.json。
已有 firmware.map 中 nav_process=0x0c9c、astar_plan=0x0750；代码/全局数据末端
0xce90，预留栈底0xf000（4KiB栈），其间剩8560B。本包没有增加固件数据结构。

2. 真实二进制格式（全部多字节整数字段为大端）
-----------------------------------------------
Linux AF_PACKET 发 80 字节：
  0..5   FPGA destination MAC = 02:ac:13:80:00:01（当前 RTL 固定）
  6..11  当前内部 GMAC 的 MAC，运行时 ioctl 读取
  12..13 EtherType 0x88b9
  14..77 64 字节 NAV2 body
  78..79 body 的 CRC16-CCITT，poly 0x1021，init 0xffff，不反射，无 xorout
网卡自动加 4 字节 Ethernet FCS；不能把 FCS/preamble/IFG 自己加入 send() 数据。
FPGA GMII RX 检查 Ethernet FCS 和长度 84 字节，再检查 body CRC16。
实际 RTL 接收流程可概括为：
  IDLE → 7×55/D5前导同步 → MAC/TYPE/BODY累积+CRC → 帧结束长度/FCS/CRC16校验
  → command_valid单脉冲 → 跨时钟mailbox → CPU校验request_id/generation/op
  → staging分片缓存 → 满16块+CRC32通过时COMMIT复制active图 → ACK。
任何校验失败在mailbox前丢弃，非法事务在CPU侧返回status；不把半幅图变成active。
Linux RX 通常已经去掉 FCS；桥接收 80 或 84 字节封包，验证来源 MAC、目的 MAC、
EtherType 和 body CRC16。目的允许本机或广播，与现有 RTL 回复一致。
不使用 IP/UDP/TCP，不需 IP 地址或网关；USB Wi-Fi 是手机链路，不能拿来发 NAV2。
auto 接口只选择 /sys/class/net/*/device 链接中 fe1c0000 的内部 GMAC1。

NAV2 body：
  byte  0..3  ASCII "NAV2"
  byte  4     版本 1
  byte  5     op=1 BEGIN,2 CHUNK,3 COMMIT,4 PLAN,5 PAGE,6 INFO
              应答 op=request_op|0x80
  byte  6     请求必须 0；应答 status
  byte  7     count；CHUNK=32；PAGE 请求=1..16；其它请求=0
  byte  8..11 request_id，除 INFO 外非零且按 int32 差值严格递增
  byte 12..15 generation，除 INFO 外非零
  byte 16..19 arg0
  byte 20..23 arg1
  byte 24..27 arg2，当前请求必须 0
  byte 28..31 reserved=0
  byte 32..63 data[32]
未知消息、错误 magic/版本/reserved、错误来源、错误 CRC 不能触发地图更新。

INFO（不消耗请求序号）：全部字段为 0，op=6；应答 arg0=当前 committed generation，
arg1=previous_request_id，arg2=有效 plan_id，data[0..3]=00 01 00 00。
桥每次上传前读 INFO，同步 next_id 和 generation；重启 RK 不硬编码从 1 开始。

事务：
  INFO → BEGIN → CHUNK(offset=0,32,...,480，共 16 块) → COMMIT
  BEGIN.arg0=完整 512 字节 bitmap 的 CRC32/IEEE：反射 poly0xedb88320，
             init0xffffffff，xorout0xffffffff。
  CHUNK.arg0=byte offset；count=32；data 恰好32字节。
  每一步等待 op、request_id、generation 严格匹配且 status=0 的 ACK。
  各步骤超时500 ms，同一个 request_id/完全相同 payload 最多发3次，固件缓存最近
  一条请求及应答，所以丢失 ACK 后重复 CHUNK/COMMIT 不会重复切换状态。
  所有16块齐全且 CRC32 匹配后，COMMIT 才原子复制 staging→active map。
  少块、坏CRC、超时或并发状态冲突：不宣称 uploaded，禁止该事务继续 PLAN。
  老的 active map 可仍存在 FPGA，但不得把本次新 geometry 错当成老图 geometry。
  发生错误后，下一次从 INFO/BEGIN 重新上传完整图，不盲目‘补发后面的块’。

可选 PLAN：arg0=start_id，arg1=goal_id；最长等10秒，同请求重传至多3次。
PLAN 应答 arg0=路径格数，arg1=expanded，arg2=cycles。PAGE 每次最多16格，
arg0=路径偏移，arg1=plan_id；应答返回 offset/total/plan_id 和大端 uint16 格号。
RK 验证 1..4096 长度、start/goal、每格可通行、无重复、四连通、所有页同一代计划。
PLAN 与 PAGE 不是电机动作。原点或局部窗口改变后必须以新 generation 重新寻路。

固件 status：0 OK；1 BAD_REQUEST；2 NO_MAP；3 GENERATION；4 INCOMPLETE；
5 MAP_CRC；6 NO_ROUTE；7 BLOCKED endpoint；8 STALE request；9 BUSY；10 INTERNAL。

3. 地图转换、车体与时效
-----------------------
默认每秒取一次 /map + map→base_link TF，以车为中心按0.1m网格对齐，生成6.4m见方
轴向与 map 一致的64×64局部图。示例 pose=(0,0) 时 origin=(-3.2,-3.2)。
并不按车头旋转窗口。原始 OccupancyGrid origin 可带 yaw；代码正确做逆旋转采样。

不能仅取目标格中心的源像素：那会让比0.1m窄的墙/立柱在降采样时消失。代码对目标格
覆盖的所有源格做面积重叠/SAT检查：任一重叠格为 -1、>free_threshold、出源图边界，
目标格均阻塞。默认 free_threshold=25，保留‘不确定’为阻塞，而不是当空地。
局部窗口外边界始终阻塞，防止 A* 越界绕过墙。

默认车体外廓前0.1454、后0.1673、左0.137、右0.115m，以最大角点半径约0.2163m
作各朝向安全包络，另加0.05m安全边距。对完整阻塞格做保守 Minkowski 膨胀，未知格
同样膨胀。不盲目清空自车区域或起点格。起点若是 blocked，先获取更多有效扫描/TF，
核对 SLAM origin/激光安装位和膨胀参数，不把 unknown 当 free 强行使A*成功。
footprint_radius 必须基于 base_link 的实际车体参考点确认；本包不自动校准外形。

map_timeout_s=10，pose_timeout_s=0.25。没有新鲜TF/地图就不上传；不伪造静态
odom→base_link来让移动建图‘跑起来’。仅单个外参 base_link→laser 可以是静态TF。
已有 root 包内 acg138_base_telemetry 的里程计/IMU融合负责真实 odom→base_link。
加载 map_server 保存的静态图时，可显式设置 map_timeout_s=0，允许地图本身不更新，
但 pose_timeout_s 不会因此取消，必须由 AMCL/SLAM定位等继续提供新鲜 map→base_link。
静态图不含新出现的障碍，运动执行仍需独立实时激光/超声波安全层。

上传线程与ROS回调分离，最多一个待处理最新快照，过载只覆盖未开工旧快照。
upload_period_s 允许0.5..10秒（0.1..2Hz），不随 /map 任意高频发包。
一轮地图19个80字节封包，不含ACK线速开销也仅1520 B/s@1Hz。
就算计入ACK/preamble/FCS/IFG，约几十kbit/s，对已有千兆摄像头通路很小。

4. 部署/命令（先完成 ROS 2 安装和 SLAM 的 parent MVP）
------------------------------------------------------
工作空间源码路径：
  rk3588/rplidar_ros2_v37/ros2_ws/src/acg138_fpga_map_bridge
在 Jazzy 环境中（安装目录按主报告选择），构建：
  . /opt/ros/jazzy/setup.sh
  cd /opt/acg138/ros2_ws
  colcon build --merge-install --packages-select acg138_fpga_map_bridge \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
  . install/setup.sh

配置默认位于 share/acg138_fpga_map_bridge/config/map_bridge.yaml。
mapping.launch.py与本包map_bridge.launch.py默认显式interface=eth0，可在没有/sys的chroot
中工作。单独ros2 run若直接使用yaml里的interface=auto，需要/sys；chroot应加
-p interface:=eth0。若真实内部GMAC并非eth0，请明确改为正确接口，不能选USB Wi-Fi。
默认仅预览派生图，不开raw socket、不发送任何FPGA或电机指令：
  ros2 launch acg138_fpga_map_bridge map_bridge.launch.py
  ros2 topic echo /fpga/map_metadata --once
  ros2 topic echo /fpga/local_costmap --field info --once
  cat /tmp/acg138_ros2_fpga_map.json

真下发前先停车/取消电机使能，并退出旧 robot-navigation 的启动入口，防止自动拉起。
本节点不擅自 kill 它。原 ABI 只有一个全局 request_id/地图/计划，不能双主机抢占。
  ps -ef | grep '[r]obot-navigation'
必须无该进程，再由已具 CAP_NET_RAW 或 root 的 ROS 环境运行：
  mkdir -p /run/lock
  ros2 launch acg138_fpga_map_bridge map_bridge.launch.py \
    dry_run:=false exclusive_nav2_owner:=true

桥通过 host_proc_root（native/chroot默认/proc）检测 robot-navigation，并且每次发消息前重检；同类新桥通过
/run/lock/acg138-nav2-<interface>.lock 排他。旧robot-navigation不理解新锁，
检测与发包之间仍可能有进程启动竞态，因此部署上必须关闭旧导航服务自动重启。
相同NAV2接口不能还有别的未识别客户端，exclusive_nav2_owner是操作者的明确保证。
Docker 方案把宿主 /proc 只读绑定为 /host_proc、共享宿主 /run/lock，并设
ACG138_HOST_PROC_ROOT=/host_proc。map_bridge.launch接受host_proc_root参数并默认读该环境变量，
直接ros2 run时应显式传-p host_proc_root:=/host_proc（YAML里默认值/proc不可用于隔离PID视图）。
所有权检查读取/proc/PID/comm，不依赖exe的ptrace访问权限，也不要求SYS_PTRACE；
识别Linux6.1截断主名robot-navigatio、完整robot-navigation及其.aarch64后缀。
找不到self/comm、没有数字PID、进程comm无权读取/格式异常或目录扫描错误均拒绝live发送，
不会把‘容器看不到宿主进程’误当作‘旧导航已退出’。使用真实GMAC需要host网络。
SPI/UART/PCIe 等替换只实现 wire::Channel::send/receive，Client和地图算法无需改。

可选仅寻路（仍不驱动车辆）：
  ros2 launch acg138_fpga_map_bridge map_bridge.launch.py \
    dry_run:=false exclusive_nav2_owner:=true enable_plan:=true
  ros2 topic pub --once /fpga/goal geometry_msgs/msg/PoseStamped \
    '{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}'
  ros2 topic echo /fpga/planned_path --once
  ros2 topic echo /fpga/map_metadata --once
目标必须位于当前6.4m窗口内且已观测可通行。超出窗口的全局目标必须由RK选局部子目标，
不能把地图clip后的目标偷偷映射到边界。复杂导航优先让Nav2 global planner产生子目标，
FPGA A*作为局部离散规划器；最后接入运动控制仍是后续显式集成工作。

5. 验证和验收
-------------
  python3 tests/run_host_tests.py
构建真正生产 map_core/nav2_wire/raw_ethernet，再链接 tests/reference 中从现有FPGA
冻结复制的 nav_protocol.c 与 astar.c，作为消息/地图/计划的真实 C 固件oracle。
检查包括：CRC金向量、大端/bitorder、薄障降采样、旋转原点、未知起点不盲清、边界封闭、
注入错ACK、丢Chunk ACK去重、重启恢复ID、完整16块原子COMMIT、不完整图和坏CRC拒绝、
失败上传后禁止PLAN、FPGA A*路径分页/端点/四连通。普通与 ASan+UBSan 两次运行。
另验证host_proc_root缺失、没有self/数字PID、不可读/空/过长comm时fail-closed，
旧导航截断名/完整名识别；不需要访问exe链接或提升SYS_PTRACE能力。
另用Icarus运行从真实工程冻结的nav_packet_rx.v，向125MHz GMII输入带完整前导/FCS的
BEGIN/CHUNK封包，验证正确payload、错误CRC16/FCS/MAC/magic丢弃。需主机已安装
cc、c++、python3、iverilog、vvp；这不是在ARM上运行仿真器的强制运行时依赖。
reports标明‘ROS节点未编译、实板未测试’，不以这些host测试冒充ROS/雷达/上板验收。

板端先验证 metadata dry_run=true free_cells合理、origin随车窗口变化；真发时
uploaded=true、generation增长且与bitmap_crc32/geometry同一条JSON绑定。
反复拔掉内部链路/停止FPGA应出现ACK timeout而非虚假成功。再恢复后新INFO/完整COMMIT
可恢复。不给MOT权限、不自动电机使能，验收阶段可把轮子架空并保持旧控制停用。

将来若扩展 MGP1 米制地图协议，需新FPGA接收器/ROM支持后再启用，不能只换RK脚本。
建议额外frame_id/geometry epoch/resolution/origin/fixed-point yaw/width/height/bitmap CRC，
分片序号与双buffer COMMIT，以epoch锁定PLAN；本包没有发送这种不受当前位流支持的协议。
