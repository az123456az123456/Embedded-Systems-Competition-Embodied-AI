# ACG138 v16：RK 建图，FPGA A* 路径规划

以已确认的 `robot_fmc_encoder_sync_v15` 为基线，摄像头、IMU、电机、编码器、超声波的采集与控制代码及全部引脚不变。新工程独立保存，旧工程可回退。

本工程让现有 PicoRV32 软核从定期 A* 自检改为实际服务：接收 RK 上传的 64×64 障碍栅格，校验完整性后原子提交；按起点和目标做四邻域 A*；分段返回完整路径。新增 EtherType `0x88b9`，原 `IMU/MOT1/ACK1/SNS1/NAV1` 通道保留。

FPGA 软核只做规划，没有电机写寄存器。RK 端融合编码器和 IMU、更新三个固定超声波的局部地图、膨胀障碍、跟踪路径、执行避障并通过已有 MOT1 通道给电机下令。遇到未知区域、过期传感器、断链或无法规划时，由 RK 停车，原 FPGA 电机超时保护继续生效。

## 与硬件保持一致

- M1 右前；M2 左后；M3 右后；M4 左前。
- 左轮掩码 10（M2、M4），方向翻转掩码 15（四个全部）。
- 手机旧设置会覆盖默认值，须与上述一致。导航与手机不能同时取得电机控制权。
- 全部引脚沿用 `robot_fmc_remote.cst` 和 `P3_接线说明.md`。
- A02YYUW：前 P2-1/Y18；左 P2-3/N20；右 P2-5/U20；供电与地沿用基线接法。
- 64 KiB 软核 SRAM 无新增外部 SRAM/Flash 分区；固件随位流初始化。
- 配套 RK 程序：`rk3588/robot_navigation_v16`。地图分辨率和物理原点属于 RK 端参数；FPGA 只处理栅格下标。

## 地图与协议

详见 `docs/NAV2_PROTOCOL.txt`。64×64 bitmap 共512B，16个32B块，携带消息 CRC16 和整图 CRC32。每次完整提交才改变规划地图。事务 stop-and-wait，有限重试；重复最新请求返回缓存；旧请求、错误代号、混入的旧路径页拒绝。RK 重启先 INFO 查询，再重新上传和规划。

障碍 bit=1；空闲 bit=0；行优先 `cell=y*64+x`，字节内 LSB first。未知区域及车体宽度膨胀由 RK 端处理。A* 为4邻域 Manhattan 路径，不允许对角穿墙。

## 构建与验证

`make firmware` 编译 RV32IM；`make test-navigation` 跑主机协议和真实软核/GMII/CDC 集成测试；`make test-softcore` 跑启动、栈保护和旧观测通道测试。`make synth` 使用本机 Gowin V1.9.12.03 构建，只有源码一致、全部75引脚属性一致、内部时序无违例才由审计脚本复制 `.fs` 到 `impl/pnr`。

结果见 `reports/release_verification.json` 和 `BUILD_STATUS.txt`。仿真及静态时序不代替实际板上相机、RGMII、导航和冷启动验证。没有新板联调数据时，不能宣称已实车验证。
