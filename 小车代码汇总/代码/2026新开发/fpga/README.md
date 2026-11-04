# FPGA 源码索引

当前为 v54 的 150/70 mm 冻结交付源码。历史工程涵盖远控、传感器、软核导航、RGB 图像链、v42/candidate26 和后续导航；布局/布线尝试明确标为实验。内部沿用 v42 工程名。

当前入口：[**v54_150_70_20261104**](v54_150_70_20261104/README.md)。详细状态见 [项目说明](../项目说明.md)，环境见 [构建与复现](../文档/构建与复现.md)。

## 版本目录

| 目录 | 类型 | 版本/说明 |
| --- | --- | --- |
| [v54_150_70_20261104](v54_150_70_20261104/README.md) | 冻结完整工程源码 | 当前基线；Gowin 工具时序通过；本版 FPGA 尚无烧录/实车验收记录。 |
| [candidate-26-live-factor](历史版本/candidate-26-live-factor/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [imu_camera_dual_1080_30fps](历史版本/imu_camera_dual_1080_30fps/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [motor_fmc_direction_sequence_20261022](历史版本/motor_fmc_direction_sequence_20261022/README.md) | 电机诊断源码 | 历史源文件；不表示当前版本的硬件验证状态 |
| [motor_fmc_direction_sequence_v2_20261022](历史版本/motor_fmc_direction_sequence_v2_20261022/README.md) | 电机诊断源码 | 历史源文件；不表示当前版本的硬件验证状态 |
| [motor_fmc_four_load_diag_direction_v2_20261019](历史版本/motor_fmc_four_load_diag_direction_v2_20261019/README.md) | 电机诊断源码 | 历史源文件；不表示当前版本的硬件验证状态 |
| [motor_fmc_four_test](历史版本/motor_fmc_four_test/README.md) | 电机诊断源码 | 历史源文件；不表示当前版本的硬件验证状态 |
| [raw_calibration_standalone](历史版本/raw_calibration_standalone/README.md) | 独立标定源码 | 历史源文件；不表示当前版本的硬件验证状态 |
| [robot_control_v21](历史版本/robot_control_v21/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [robot_control_v39_rgb](历史版本/robot_control_v39_rgb/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [robot_fmc_remote](历史版本/robot_fmc_remote/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [robot_navigation_v18](历史版本/robot_navigation_v18/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [robot_navigation_v20](历史版本/robot_navigation_v20/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [robot_navigation_v42_candidate26_source](历史版本/robot_navigation_v42_candidate26_source/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [robot_navigation_v52_budget_split16](历史版本/robot_navigation_v52_budget_split16/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [robot_navigation_v53_obstacle_reverse](历史版本/robot_navigation_v53_obstacle_reverse/README.md) | 源码快照 | 历史源文件；不表示当前版本的硬件验证状态 |
| [robot_navigation_v54_place2_retry](历史版本/robot_navigation_v54_place2_retry/README.md) | 实验源码 | 布局/布线尝试，不是最终交付版。 |
| [robot_navigation_v54_place52](历史版本/robot_navigation_v54_place52/README.md) | 实验源码 | 布局/布线尝试，不是最终交付版。 |
| [robot_navigation_v54_route1](历史版本/robot_navigation_v54_route1/README.md) | 实验源码 | 布局/布线尝试，不是最终交付版。 |
| [公共驱动模块](历史版本/公共驱动模块/README.md) | 独立模块集合 | 历史源文件；不表示当前版本的硬件验证状态 |

## 归档边界

源码补丁不能独立代表完整版本；仅有输出的旧目录未以其他源码冒充，见 [版本沿革与恢复缺口](../文档/版本沿革.md)。各版本 README 记录来源。源码内原始说明如含旧报告/安装路径，保留在历史说明.md，当前入口以本页为准。
