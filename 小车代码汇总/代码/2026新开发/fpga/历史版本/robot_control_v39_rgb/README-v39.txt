robot_control_v39_rgb — 真正的双目 FPGA RGB40 预处理 + v38 控制/传感器修复

主工程 robot_control_v39_rgb.gprj，器件 GW5AT-LV60PG484AC1/I0。
主输出 robot_control_v39_rgb.fs；新工程独立于 v35/v36/v38，不覆盖旧目录。
使用 Gowin IDE 打开工程后，source project_settings.tcl；命令行 gw_sh build.tcl。

图像：每眼960×1080 RAW8 BGGR，FPGA完成demosaic、569×640双线性缩放和左右35/36像素114灰补边。
RGP1线协议输出每眼完整640×640 RGB888（包括确定的114灰补边），不传原始RAW全幅。
双眼40FPS RGB像素载荷约786.43Mbit/s；加协议/FCS/前导码/IFG约844Mbit/s，仍需实板验证丢包与余量。
OV5640寄存器、双目采集相位、全部75管脚、RGB session协议与有界CDC FIFO容量保留v35。
这不是v38 RAW/ISP回退；RK默认auto，安装配套更新后按原命令启动即可：
  sh /root/perception-dashboard/run-dashboard.sh -0
手动覆盖相机时使用有效入口（仅排查时需要，默认auto推荐）：
  PERCEPTION_CAMERA_SOURCE=fpga-rgb sh /root/perception-dashboard/run-dashboard.sh -0
或在启动参数中给出 --camera fpga-rgb:eth0。
RGB40是一项工程目标；实际传感器帧率、颜色、链路丢包和NPU吞吐还需实机测量。

v35真综合错误：原vertical signed delta乘加被Gowin映射进MULTALU27X18预加器时，符号扩展和有效位宽不符。
均匀输入也会按垂直权重变亮/回绕，表现为彩色乱纹。
v39使用两个非负权重乘积分别寄存后再求和，避免有符号预加器路径；增加1个PCLK固定流水延迟，吞吐不变。
所有SOF/EOL/EOF、坐标与有效信号同步延迟。最终发布前以真实生产整体VG+官方原语重新验证，不仅测试RTL。
首轮独立STA拦下165个setup违例，因此首轮生成的.fs不是交付物。
追加六族结构时序优化（不改变控制策略、协议、像素、引脚和时钟约束）：
1. FIFO写侧Gray解码与占用减法分拍，计入当拍接受写入，保持计数保守上界，真正full仍独立。
2. 预先寄存epoch_active并与新会话同拍更新，切断宽epoch比较到Demosaic使能扇出。
3. sender仅在输入头缓存空时补充，切断消费状态到FIFO BRAM地址路径；每像素3拍的吞吐不变。
4. 双眼sequence/timestamp差值与skew判定分拍，元数据稳定后配对；完整线包逐字节与原逻辑相同。
5. 编码器平衡器私有mean/threshold寄存独立pending使能，外部trims/active/fault逐拍等价。
6. IMU以精确lookahead寄存ready，旧state==IDLE的ready/data/valid每拍等价。
SDC只扩大报告覆盖到1000paths与CE，原125MHz及其它时钟、所有CDC例外完全保持。
最终独立门级和STA必须绑定第二轮真实整体VG，不能沿用首轮网表证据。

执行器/传感器：完整保留v38的30秒外部授权、电机物理方向、编码器、IMU9、SNS1、NAV1/NAV2和A*。
四编码器CPR60527.7，符号(-1,+1,-1,+1)，左侧mask10(M2/M4)，车体几何保持原配置。
A02YYUW源龄1000ms以内仍可用，超过1000ms才过期；校验失败不刷新观测时间，无包不伪装远距离。
新近障、CPU trap、IMU失效、编码器异常的立即PWM=0保护保留；停车和取消外部授权分离。

v37 ROS2/雷达为RK侧独立工程，使用现有NAV2协议；本工程不额外虚构一个v37位流。
在线地图写入必须由单一拥有者执行；不要同时运行旧导航地图写入器和ROS地图桥。

验证入口（在workspace根目录）：
  python3 integration/robot_rgb_v39/integration/test_fpga.py
  python3 integration/robot_rgb_v39/integration/test_fpga_control.py
  make -C fpga/robot_control_v39_rgb test-v39
  python3 integration/robot_rgb_v39/integration/build_fpga.py
  python3 integration/robot_rgb_v39/integration/audit_fpga.py

最终报告位于 integration/robot_rgb_v39/integration/reports/。
过去版本README与历史脚本保留作参考，当前以本文件和v39源绑定报告为准。
仿真、门级原语模型和内部STA通过不等于实板DVP/RGMII物理I/O裕量或实车避障成功保证。
