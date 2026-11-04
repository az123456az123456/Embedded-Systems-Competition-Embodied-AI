RK3588 只读软核状态监视器（2026-10-20）

softcore-monitor.aarch64 为 Linux aarch64 程序。它只接收 FPGA 状态，
没有发包、使能或电机控制操作。保留正在运行的统一 YOLO/IMU/手机程序，
另开一个 RK3588 终端执行：

  chmod +x ./softcore-monitor.aarch64
  ./softcore-monitor.aarch64

默认自动查找 /sys/class/net/*/device 中包含 fe1c0000 的板内 GMAC，
观察 10 秒。无法唯一确定时列出接口并报错，不会猜测 eth0/eth1 或 Wi-Fi。
可明确指定：

  ./softcore-monitor.aarch64 --interface eth0 --count 20 --timeout 10

--count 为最少有效 NAV1 帧数；即使数量已够，也必须观察到心跳向前变化，
防止把重复的静态数据当作软核执行成功。正常循环心跳预计约 100 毫秒变化一次，
A* 自测期间可能暂停，故停滞容限使用 3.5 秒。
--count 0（默认）观察整个 --timeout 窗口。--timeout 0 为无时间上限；
--count 0 --timeout 0 持续监视，Ctrl+C 退出。
live=unverified 表示尚未看到心跳变化；live=yes 表示 3.5 秒内看到前进；
live=stalled 表示之前前进过、现在已停滞或复位后尚未再次前进。
ready/selftest 和 live 分开显示，避免只看固定 ready 标志就误判通过。

退出码：0=最新状态 ready+selftest 通过、无错误且心跳正常；
1=参数、权限或系统错误；2=观察结束无有效数据或未满足指定帧数；
3=状态异常、心跳未确认或停滞。以 root 运行，或提供 CAP_NET_RAW。
旧位流不发送 NAV1，新位流的状态通常随现有控制 ACK 发出；没有流量时
先确认统一程序仍运行，不需要使能电机或移动小车来做这个检查。

协议：独立 Ethernet EtherType=0x88b7，源 MAC=02:ac:13:80:00:01。
Ethernet 首部之后为 28 字节 NAV1（不含以太网填充及 FCS）：
  0..3   ASCII NAV1
  4      版本 1
  5      bit0 ready；bit1 selftest_ok；bit2 hardware_trap；bit3 firmware_error
  6..7   path_length，uint16，大端
  8..11  heartbeat，uint32，大端
  12..15 astar_cycles，uint32，大端，50MHz 时钟，除以 50000 得毫秒
  16..19 expanded，uint32，大端
  20..23 软核已观察 IMU sequence，uint32，大端
  24..25 firmware_error code，uint16，大端
  26..27 CRC16-CCITT，init=ffff，poly=1021，覆盖前 26 字节，大端

该协议使用独立帧。原 EtherType=0x88b6 的 ACK1 格式、CRC 和零填充不变，
因此现有 RK3588 电机 ACK 解析器仍可使用。

本机编译/测试：
  make test
交叉编译：
  make arm64 CROSS_COMPILE=/path/to/aarch64-none-linux-gnu-
在板子上运行无网络、自包含解析测试：
  ./softcore-monitor.aarch64 --self-test

测试包含有效帧/字节序、所有截断长度、损坏 CRC、错误版本/源 MAC/
EtherType/魔数/保留位、错误状态、静态心跳、心跳前进、停滞、复位和回绕。
主机测试证明解析和状态判断，不代替 FPGA 实板启动和时序验证。
