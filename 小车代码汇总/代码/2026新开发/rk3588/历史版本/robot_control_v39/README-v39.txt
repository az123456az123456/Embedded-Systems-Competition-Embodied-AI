v39 RGB40：修复FPGA缩放横纹，保留v38控制与手机低延迟视频
2026-10-29

一、抓帧确认的原因
camera-frame-001左右眼原始RGB已包含横纹，PPM、左右拼图、尺寸和灰色补边一致。
v35有符号纵向双线性插值在高云综合时被映射为有错误扩展位的DSP预加法。
上下两行都是灰度100时，RTL输出100，正式交付综合网表却输出188、108、28等周期值，
与实拍每行的周期性偏移一致；这不是YOLO识别或手机解码造成的。
v39使用非负权重独立乘积，寄存后相加，避免错误的DSP预加法推断；接口不变，增加1PCLK延迟。
验证包含最终整工程两眼综合网表，以及DVP→Bayer→缩放→补边→跨时钟→RGP打包链。
另将FIFO占用计算、会话有效判定及发送预取等关键路径拆级，保持原时钟约束和40FPS吞吐。
不是在RK反向补偿条纹，没有更换当前训练好的INT8模型。新位流尚未在用户实体板实拍。

二、配套与视频线路
FPGA：robot_control_v39_rgb.fs。
RK：robot-control-v39-rk3588-20261029.tar.gz。
手机：沿用ACG138-Robot-Control-v38.apk；已经装v38不用重装，旧手机覆盖安装v38 APK即可。
主线路：双目RAW Bayer→FPGA去马赛克/缩放/114灰补边→每眼640×640 RGB888、双眼40FPS
RGP1/0x88ba→RK直接送模型输入。每眼有效区569×640，左右补边35/36列；传完整640×640。
RGB像素负载786.43Mbit/s，加包头/帧间隔约845Mbit/s，保留IMU/控制通道。
RGB路线不运行RK RAW ISP或重复缩放；40FPS采集不保证识别和手机也达到40FPS。
原RAW40/ISP路线保留兼容，默认自动探测实际协议。模型为原v35 best e7f26253 INT8，标签原样保留。

三、替换
先手机停车，退出RK统一窗口及导航（Ctrl+C）。将RK压缩包拷到板子/root后：
  cd /root
  tar -xzf robot-control-v39-rk3588-20261029.tar.gz
  cd robot-control-v39-rk3588-20261029
  sh stop-camera.sh
  sh install.sh --check-only
  sh install.sh
安装器检查原成套程序、生成备份，保留网卡驱动、配对码、车辆参数、地图及手机设置。
运行中的dashboard/navigation/ISP会拒绝更新，自动重启服务也需先停。未知或混装程序拒绝覆盖。
此包更新已有可运行v35/v36/v38基础环境，不是完整系统镜像；不要单独覆盖其中一个ELF。
烧录robot_control_v39_rgb.fs，沿用原器件和烧录方法，之后给摄像头重新上电。
仍用原统一入口：
  sh /root/perception-dashboard/run-dashboard.sh -0
横置用-90，手机仍连接原8080地址。启动版本应含v39，自动检测fpga-rgb（RGP1/0x88ba），
接收1280×640 RGB。RGB线路没有RAW/ISP的FASTPATH日志是正常的。
若显示v4l2/NV12，核对是否仍烧RAW位流。去掉PERCEPTION_CAMERA_SOURCE=v4l2或
--camera /dev/videoXX等旧强制覆盖；默认自动探测，显式协议不一致会报错。

四、核对与抓帧
  /root/robot-perception/perception-dashboard.aarch64 --version
  /root/robot-navigation/robot-navigation.aarch64 --version
  grep -E 'CAMERA_LINK|PERF |PHONE_PERF|PHONE_CTRL|SENSOR_NET' /tmp/robot-nav-dashboard.log | tail -n 12
先停车核对RK和手机颜色、物体边缘。再抓原始证据时，先退出统一窗口：
  /root/robot-perception/capture-fpga-rgb.aarch64 --interface eth0 --output /root/camera-frame-v39 --timeout 10
left.ppm/right.ppm/stereo.ppm可在电脑直接打开，.rgb和metadata.json用于复核。
工具会开启独立相机会话，避免与YOLO抢占。新位流尚未实拍，不能以仿真替代这一步。
手机视频保留v38三层最新帧优先、受限队列及慢流重连；实际热点时延仍需实测。

五、保留的控制修正
超声波按最后真实观测源年龄判断：≤1000ms不过期，>1000ms零速等待数据恢复。
短缺测只延续已确认安全的直行，不凭旧距离批准新启动/转向/加速；新近障立即停车。
坏校验和重复包不刷新测距年龄，合法零回波FF0000FF映射4500mm。
不因缺测5s自动撤销导航，ACK1→ACK4采用非阻塞零速等待后接续请求。
手动取消/停车、明确断开、30s没有新合法指令才撤销电机授权；真实故障仍可零速保护。
直行避障不依赖地图，导航只在绕障后回归路线时使用地图。
真实近障、IMU/编码器失联、软核陷阱、车身旋转空间不足保护保留。
当前车轮映射、编码器参数、电机/传感器接口及原NAV2地图协议不变。

六、回退
先停车并退出两RK程序及ISP：
  sh rollback.sh /root/安装时输出的robot-perception-v39-backup.XXXXXX
回退恢复旧整套模型/入口/ISP，不改当前配置、配对码和地图；位流也回到该备份对应版本。
若文件已被后续修改，回退拒绝覆盖。保留备份与报错，不能把混合文件当成功安装。
断电/SIGKILL无法运行shell回滚；保持程序关闭，按备份old.sha256核对恢复。

七、v37雷达/ROS源码
  sh install-ros2-addon.sh
仅安装原v37已校验源码与ARM小工具到/root/rplidar-ros2-v37，不启动雷达或电机。
完整ROS用户空间仍需依其README单独构建，本包不含完整ROS运行系统。
已有rootfs时可另终端运行：
  SERIAL_DEVICE=/dev/ttyUSB0 sh /root/rplidar-ros2-v37/deploy/run-chroot.sh
地图桥默认dry_run；实际写FPGA地图必须停旧导航写入者，遵守NAV2单写入者约定。

验证范围为真实综合/布局布线、综合网表原语仿真、RTL和软件回归。
实体摄像头、热点和机械车辆尚需上板验收。

开发电脑入口：python3 integration/robot_rgb_v39/rk/build_arm64.py
FPGA工程：fpga/robot_control_v39_rgb/robot_control_v39_rgb.gprj
打包：python3 integration/robot_rgb_v39/rk/package_release.py
历史回归脚本保留原版本文件名；继承v38验证与本次新测结果在reports中明确区分。
