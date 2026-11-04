FPC1 v40.1 原生 ARM64 校准工具，2026-10-29

本版本解决 RK3588 Buildroot 未安装 Python/Python3 时旧安装脚本退出的问题。
用现有 BSP aarch64-none-linux-gnu GCC10.3.1 编译，完全静态链接，不依赖
Python、libstdc++.so、动态加载器或 Ubuntu rootfs。无需更换 FPGA 位流。
应配 robot_image_v40.fs，不改变 YOLO/手机软件、模型、电机、导航或系统库。

板子安装（把此目录/压缩包拷到 /root）：
  cd /root
  tar -xzf image-cal-v40-native-rk3588-20261029.tar.gz
  cd image-cal-v40-native-rk3588-20261029
  sh install.sh

先做无网络自检及只读查询：
  /root/image-cal-v40/fpc1 --self-test
  /root/image-cal-v40/fpc1 --interface eth0 info --eye 0
  /root/image-cal-v40/fpc1 --interface eth0 info --eye 1
注意查询需要 v40 FPGA 配套。没有 ACK 是位流/接口/链路问题，不能用假成功绕过。

启用/关闭 Gamma（默认关闭；传感器内部也可能已启用Gamma，需实拍确认）：
  /root/image-cal-v40/fpc1 --interface eth0 gamma on
  /root/image-cal-v40/fpc1 --interface eth0 gamma off

校验/上传已有真实标定 JSON：
  /root/image-cal-v40/fpc1 --check /root/fpn-left.json --eye 0
  /root/image-cal-v40/fpc1 --interface eth0 upload --eye 0 /root/fpn-left.json
  /root/image-cal-v40/fpc1 --interface eth0 upload --eye 1 /root/fpn-right.json
  /root/image-cal-v40/fpc1 --interface eth0 info --eye 0
  /root/image-cal-v40/fpc1 --interface eth0 info --eye 1
恢复某眼 FPN 旁路：
  /root/image-cal-v40/fpc1 --interface eth0 bypass --eye 0

暗/亮RAW系数生成仍在有Python3的开发电脑执行现有 fpc1.py prepare，之后只需
把JSON传到RK；本原生程序没有另写简化算法替换相位感知标定。当前 RGB/RGP1
截图不能替代960×1080 RAW8暗/亮场；本工具不会自动采集RAW或锁定曝光。
支持原生 FPN-COLUMN-RAW8-Q2.8-v1 以及 fpc1-column-q8-v1 两种JSON。
上传前验证全部960列，再发BEGIN；超时重发完全相同的ID和内容；首次INFO同步
事务ID；逐列确认ACK；COMMIT后在下一SOF应用，需INFO确认。重启后重新上传。
仍使用板内 eth0、EtherType0x88BB、CRC16/FPC1，非手机无线接口。

验证：实际 AArch64 ELF 无 INTERP/NEEDED；QEMU执行正式二进制自检；原Python
协议oracle与主机/ARM64 C++逐字节比对、畸形ACK/JSON拒绝共118项通过。
尚未在真实RK3588/FPGA上运行。测试记录见 reports/verification.json。
源码 src/fpc1.cpp；PC构建 python3 build.py；PC测试 python3 tests/verify_native.py。
发行包内保留picojson原始许可；源码位于vendor/picojson.h。
