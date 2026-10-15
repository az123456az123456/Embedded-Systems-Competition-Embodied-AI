# FPGA 远控工程验证记录

> 日期：2026-10-15
> 工具：Gowin V1.9.12.03；GW5AT-LV60PG484AC1/I0。
> 本记录只反映本机验证，尚未烧录或在实车上验证。

## ✅ 已完成

| 检查 | 结果 |
|---|---|
| MOT1 最小以太帧：正常/坏 FCS/坏 CRC16/错 MAC/非零 padding/RX_ER/截短及恢复 | 通过 |
| 首次 STOP 握手、运行中配置保护、序号重复/旧序号/回卷/主机重启 | 通过 |
| 左右有符号 PWM、60% 限幅、加减速、M3/M4 反相、1ms 方向空挡、PWM=0 立即停车 | 通过 |
| CDC 数据保持一致性、RX 时钟完全停止后独立看门狗停车、时钟恢复重握手 | 通过 |
| ACK1 字节序、CRC16、最小帧 padding、完整 Ethernet FCS | 通过 |
| 摄像头/IMU/ACK 帧边界仲裁及各帧 IFG 不被插入打断 | 通过 |
| 高云官方 IDDR 功能模型：RGMII DDR 半字节及控制顺序、错误帧拒收 | 通过 |
| 实际 50MHz 仿真：20kHz、每周期 2500 时钟、60% 高电平 1500 时钟、相位间隔 625 时钟 | 通过 |
| 完整工程全部 64 个信号、独立电机工程全部 29 个信号的实际布局布线球位对照 CST | 完全一致，无重复球位 |
| MPU9250 / 摄像头数据协议和拼接逻辑 | IMU 不变；相机仅两份包装源修复复位和 Init_Done 跨域 |
| 相机复位同步释放、停止 PCLK、24 帧三路实际报文仲裁及 1080 行拼接回归 | 通过 |

## ⏱️ 布局布线时序

| 工程 / 时钟 | 要求 | 布局布线 Fmax | 同域 setup/hold |
|---|---|---|---|
| 完整工程：50MHz 电机控制 | 50MHz | 73.513MHz | 0 / 0 |
| 完整工程：RGMII 接收 | 125MHz | 139.729MHz | 0 / 0 |
| 完整工程：共享 RGMII 发送 | 125MHz | 126.343MHz | 0 / 0 |
| 独立电机：电机控制 | 50MHz | 89.093MHz | 0 / 0 |
| 独立电机：RGMII 收发 | 125MHz | 130.097MHz | 0 / 0 |

**独立电机工程总体报告为 0 setup、0 hold 违例。** 专门的异步断言、同步释放复位链已处理；握手 mailbox 数据保留 12ns 最大延迟，hold 仅针对其保持到返回 ACK 的异步总线作例外，未全局切断时钟域。

**完整工程最终总体报告也为 0 setup、0 hold 违例。** 原 99 项已定位处理：相机配置50MHz/发送125MHz/两个PCLK域分别同步释放复位；Init_Done 分别同步到相机域；两路 FIFO 复位组合同步释放拼接控制器。仅对厂商异步 FIFO 的指针跨域施加 8ns 最大延迟（仍保留检查），最后 4 个专用 reset_r 同步器 PRESET 端点允许异步复位，其下游仍正常检查。没有全局切断时钟域或屏蔽整块摄像头逻辑。

FIFO IP 为厂商加密实现；官方接口文档确认其两独立时钟域，具体 Gray 编码方程无法独立检查，因此本记录不声称已经独立证明 IP 内部 Gray 性质。当前使用厂商异步 FIFO 边界和精确时序网表端点实施有界约束，详细证据和修复前最后四条恢复路径见 `reports/camera_cdc_audit.json`。参考[高云 FIFO 官方指南](https://www.gowinsemi.com/upload/database_doc/2574/document/68dc39a49d8ad.pdf)。

上述是已声明约束下的内部时序结果，**相机/RGMII外部 I/O 约束尚未完成板级签核，也尚未上板验证**。仍需实测 camera/IMU 帧连续性、ACK、PWM 和断链停车，未执行硬件烧录。

RGMII 输入输出板级延迟沿用资料包的 RK GMAC 配置起点 `tx_delay=0x20/rx_delay=0x48`，仍须在实物链路验证。高云报告中的通用时钟布线提示已保留在原始构建日志中。

## 📦 位流和追溯

| 位流 | 包含功能 | SHA256 |
|---|---|---|
| `impl/pnr/robot_fmc_remote.fs` | 双目＋MPU9250＋四电机远控＋ACK | `f24ad0f88a42b778578bfd95510894d7abf5fab92f2505cbc39eecb90dec7bd3` |
| `impl/pnr/fmc_remote_motor_only.fs` | 独立四电机远控＋ACK，不含摄像头/IMU | `3998da3029e163b15cda305bf0415f7eab3ffe6de0ff6a87ed0081fb9228c351` |

源文件 SHA256、位流 SHA256 和物理管脚验证分别见 `reports/source_bitstream_sha256.json`、`reports/pin_verification.json`；原始高云构建、时序、布局布线、功耗报告保留在 `reports/`。`reports/timing_summary.json` 汇总时钟和违例统计。旧 .fs 未被覆盖。
