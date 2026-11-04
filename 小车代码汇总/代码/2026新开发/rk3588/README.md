# RK3588 源码索引

当前为 v143 的原生感知/网关与板端 ROS 源码集合。保留独立摄像头、建图导航、语音与早期原型。历史 RK 内嵌 Android 已按平台移至上位机目录，原位置保留指引；旧联合测试的相对路径需按归档布局处理。

当前入口：[**v143_20261104**](v143_20261104/README.md)。详细状态见 [项目说明](../项目说明.md)，环境见 [构建与复现](../文档/构建与复现.md)。

## 版本目录

| 目录 | 类型 | 版本/说明 |
| --- | --- | --- |
| [v143_20261104](v143_20261104/README.md) | 完整源码集合 | 当前基线；原生主程序和网关由已匹配板端二进制的本地源码补齐，ROS/启动脚本来自板端。 |
| [HDMI无鼠标显示修复_v19.2](历史版本/HDMI无鼠标显示修复_v19.2/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [acg138-yolo11-30fps-rk3588-20261019](历史版本/acg138-yolo11-30fps-rk3588-20261019/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [camera-fastpath-30fps](历史版本/camera-fastpath-30fps/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [camera-fastpath-highfps](历史版本/camera-fastpath-highfps/README.md) | 配套模块源码 | 历史源文件；不表示当前版本的硬件验证状态 |
| [image_cal_v40_native](历史版本/image_cal_v40_native/README.md) | 配套模块源码 | 历史源文件；不表示当前版本的硬件验证状态 |
| [imu_dashboard](历史版本/imu_dashboard/README.md) | 配套模块源码 | 历史源文件；不表示当前版本的硬件验证状态 |
| [nav3_bridge_candidate26_20261101](历史版本/nav3_bridge_candidate26_20261101/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [nav3_bridge_stage0_20261101](历史版本/nav3_bridge_stage0_20261101/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [raw_calibration_standalone](历史版本/raw_calibration_standalone/README.md) | 配套模块源码 | 历史源文件；不表示当前版本的硬件验证状态 |
| [robot_control_v39](历史版本/robot_control_v39/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [robot_perception_v12](历史版本/robot_perception_v12/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [robot_perception_v34](历史版本/robot_perception_v34/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [robot_phone](历史版本/robot_phone/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [robot_reactive_v29](历史版本/robot_reactive_v29/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [robot_sonar_avoid_v30](历史版本/robot_sonar_avoid_v30/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [robot_sonar_avoid_v31](历史版本/robot_sonar_avoid_v31/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [robot_sonar_avoid_v32](历史版本/robot_sonar_avoid_v32/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [robot_sonar_avoid_v33](历史版本/robot_sonar_avoid_v33/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [robot_sonar_v29_1](历史版本/robot_sonar_v29_1/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [robot_vision_v35](历史版本/robot_vision_v35/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [ros_exploration_candidate26_20261101](历史版本/ros_exploration_candidate26_20261101/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [ros_localnav_v49_20261103](历史版本/ros_localnav_v49_20261103/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [ros_localnav_v50_los_20261103](历史版本/ros_localnav_v50_los_20261103/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [ros_localnav_v51_20261103](历史版本/ros_localnav_v51_20261103/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [ros_localnav_v53_20261103](历史版本/ros_localnav_v53_20261103/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [rplidar_ros2_v37](历史版本/rplidar_ros2_v37/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [rtl8821cu-driver](历史版本/rtl8821cu-driver/README.md) | 配套模块源码 | 历史源文件；不表示当前版本的硬件验证状态 |
| [sonar_nav_sim](历史版本/sonar_nav_sim/README.md) | 配套模块源码 | 历史源文件；不表示当前版本的硬件验证状态 |
| [v138_目标跟随_源码补丁](历史版本/v138_目标跟随_源码补丁/README.md) | 局部源码补丁 | 只保存了本版修改文件，不能独立代表该版本全量工程。 |
| [v139_稳定识别框_源码补丁](历史版本/v139_稳定识别框_源码补丁/README.md) | 局部源码补丁 | 只保存了本版修改文件，不能独立代表该版本全量工程。 |
| [v141_4dm_ROS源码补丁](历史版本/v141_4dm_ROS源码补丁/README.md) | 局部 ROS 源码补丁 | 只有 3 个 ROS 源文件，不代表完整 RK v141。 |
| [yolo_voice_integration_20261103](历史版本/yolo_voice_integration_20261103/README.md) | 配套模块源码 | 历史源文件；不表示当前版本的硬件验证状态 |
| [早期Python导航原型](历史版本/早期Python导航原型/README.md) | 历史 Python 原型 | 未纳入当前板端启动链。 |
| [模型转换工具_20261101](历史版本/模型转换工具_20261101/README.md) | 转换工具源码 | 仅转换/校验脚本与模型元数据；权重、模型、校准图像不入库。 |

## 归档边界

源码补丁不能独立代表完整版本；仅有输出的旧目录未以其他源码冒充，见 [版本沿革与恢复缺口](../文档/版本沿革.md)。各版本 README 记录来源。源码内原始说明如含旧报告/安装路径，保留在历史说明.md，当前入口以本页为准。
