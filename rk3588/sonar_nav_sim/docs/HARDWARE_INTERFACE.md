# 仿真与真实硬件的接入边界

> 日期：2026-10-20。当前交付仅运行电脑仿真；未连接实车，未更改 FPGA 位流、冻结的 UART 协议或现有 RGMII 协议。

## 📦 数据边界

导航只接收 `SensorPacket`，只输出 `Command`。它不能读取仿真场景、真实障碍物坐标或小车真值位置。接入实车时保留估计、建图和导航层，只替换 `RobotIO.advance(dt_s, command)` 的实现。

| 字段 | 单位及约定 |
|---|---|
| `Command.linear_m_s` | 米/秒，正值向前；不能直接当成 PWM |
| `Command.angular_rad_s` | 弧度/秒，正值向左逆时针旋转 |
| `Pose.x/y/yaw` | 米、米、弧度；初始位姿为人为定义的局部地图坐标 |
| `left_ticks/right_ticks` | 左/右侧累计有符号编码器计数，向前为正；不是本周期增量 |
| `gyro_z_rad_s` | 去除安装方向差异后的 Z 轴角速度，弧度/秒；不是偏航角 |
| `SensorPacket.stamp_s` | 本帧编码器与 IMU 的采样时间，秒，必须严格递增 |
| `RangeReading.stamp_s` | 超声波测量采样时间，供位姿历史补偿使用 |
| `RangeReading.received_s` | 该测距结果送达接入层的时间，与采样时间使用同一时间域 |
| `range_m/status` | 距离以米表示；`ok` 有有效值，`no_echo/fault` 必须为 `null` |

车体坐标为 X 向前、Y 向左、Z 向上。探头固定为 `front`、`left`、`right`，默认安装角为 0°、+90°、−90°，偏置见 `Config.mounts`。三路 A02YYUW 是宽波束距离观测，不能当成三束精确激光；无回波也不能直接标为空地。当前参数和低速上限是仿真假设，需用实际安装测量替换。

### 编码器与时间戳

`ticks_per_revolution` 必须是**车轮转一圈的最终计数**，包含编码器原始脉冲数、X1/X2/X4 倍频以及减速比。先手动转一整圈核对；还应测轮径和左右有效轮距。计数回绕应在接入层展开，设备重启后应停止并重建适配器，不能把归零误当倒退。

四轮车应先把每个轮子的计数转换成各自的行驶距离，再对左侧两个、右侧两个分别平均，形成左右里程。只有轮径和每圈计数一致时，才可直接平均同侧累计计数；直接相加会把距离放大一倍。打滑、轮径误差和四轮原地转向会使里程计漂移，IMU 不能自动消除位置误差。

现有手机遥控返回的 PWM 是控制指令，**不能用作真实轮速或编码器反馈**。真实电机后端需要用左右目标速度、编码器闭环及标定参数生成 PWM。

各传感器时间须转换到统一的单调时钟域，不能混用 `time.time()`、设备开机时间和 PC 开机时间。应分别保留采样与接收时间，考虑超声波处理延迟；如果只能获得接收时间，要明确这是近似值。绝不能把长时间缓存的数据重新标成“刚采样”。

## 🧾 JSONL 记录与离线回放

`JsonlRecorder(path, metadata=...)` 的可选首行为 `kind="metadata"`，可包含 `config`、`scenario`、`initial_pose`。后续每行是一帧；没有元数据的日志同样可以回放。简化的单帧示例：

```json
{"kind":"frame","version":1,"packet":{"stamp_s":12.3,"left_ticks":1040,"right_ticks":1048,"gyro_z_rad_s":0.012,"ranges":[{"name":"front","stamp_s":12.2,"received_s":12.3,"range_m":1.12,"status":"ok"},{"name":"left","stamp_s":12.2,"received_s":12.3,"range_m":0.84,"status":"ok"},{"name":"right","stamp_s":12.2,"received_s":12.3,"range_m":null,"status":"no_echo"}],"encoder_valid":true,"imu_valid":true},"command":{"linear_m_s":0.1,"angular_rad_s":0.0},"estimated_pose":{"x":1.2,"y":1.1,"yaw":0.02}}
```

```python
from dataclasses import asdict
from navsim.adapters import JsonlRecorder, JsonlReplay
from navsim.model import Command, Config, Pose

config = Config()
recorder = JsonlRecorder("trip.jsonl", config, metadata={
    "config": asdict(config), "scenario": "room", "initial_pose": asdict(Pose())
})
# 在导航主循环已有 packet、command、estimated_pose 时：
# recorder.write(packet, command, estimated_pose)
# 结束时 recorder.close()

# 日志至少有一帧后才可回放；每次 advance 返回下一原始帧。
# replay = JsonlReplay("trip.jsonl")
# while not replay.finished:
#     packet = replay.advance(0.04, Command())
#     # 估计器使用 packet.stamp_s 的差值，而非假定每帧固定 40 ms。
# replay.close()
```

回放会验证有限数值、有效距离、采样/接收时间顺序和严格递增帧时间；拒绝 NaN/Inf、倒序或不符合格式的数据。`advance` 的新指令不影响录制观测，也不会下发到硬件；历史指令可从 `recorded_command` 查看，历史估计位姿可从 `recorded_pose` 查看。最后一帧返回后 `finished=True`，之后继续调用只返回最后一帧，不表示有新观测。

## 🔌 有界队列硬件适配器

`QueueHardwareIO` 不启动串口、网口或电机。未来通信接收线程解析现有协议后调用 `submit()`；导航线程调用 `advance()`。默认队列最多 128 帧，溢出丢弃最旧帧并累计 `dropped_packets`。编码器使用累计计数，最新帧仍保留总位移；丢掉的超声波观测不可恢复。

```python
import time
from navsim.adapters import QueueHardwareIO
from navsim.model import Command, Config, RangeReading, SensorPacket

def send_to_existing_controller(command):
    # 示例仅打印，不控制任何硬件。
    # 后续在这里转换左右轮目标速度，并对接已有控制器。
    print("target", command.linear_m_s, command.angular_rad_s)

io = QueueHardwareIO(Config(), command_sink=send_to_existing_controller)
now = time.monotonic()
# 示例把所有 packet 时间都放在主机单调时钟域；设备时钟需先转换。
packet = SensorPacket(
    stamp_s=now, left_ticks=1200, right_ticks=1200, gyro_z_rad_s=0.0,
    ranges=tuple(RangeReading(name, now - 0.1, now, 1.0)
                 for name in ("front", "left", "right")))
io.submit(packet, sampled_monotonic_s=now)
try:
    observed = io.advance(0.04, Command(0.1, 0.0))
finally:
    io.close()  # 尽力发送零速；不代表底层已经执行停车。
```

`sampled_monotonic_s` 是整帧采样时间映射到主机单调时钟后的值，用于识别“刚收到但早已采样”的数据。省略时按本机接收时间近似，只能判断接收后等待和帧内测距年龄，无法识别接入层之前的未知缓存延迟。接入真实设备时应提供该值。

### 失联与停车条件

| 条件 | 适配器行为 |
|---|---|
| 尚未收到遥测，或本次 `advance()` 没有任何新帧 | 发送零速，返回 `encoder_valid=False`、`imu_valid=False`，不重复提交旧测距 |
| 整帧采样年龄或接收后等待超过 `stale_after_s`，默认 0.65 秒 | 发送零速，编码器/IMU 标为无效 |
| 编码器或 IMU 无效 | 发送零速，保留对应无效标志 |
| 任一方向测距缺失、无回波、故障或采样年龄超过 0.65 秒 | 默认发送零速；导航还应独立检查安全距离和观测新鲜度 |
| 命令非有限值或步长无效 | 尽力发送零速，然后抛出异常 |
| 命令超出配置速度范围 | 限幅到 `max_speed_m_s/max_turn_rad_s` |
| 指令回调报错 | 再尽力发送一次零速，并把异常交给调用者 |
| `close()` | 清空队列，尽力发送零速，拒绝后续提交 |

仅调试编码器/IMU 时可设置 `require_ranges=False`；实际导航应保留默认测距检查。解析失败、时间戳倒序或设备重启会拒绝该帧，通信所有者应立即停止导航并关闭适配器。

**以上软件检查需要主循环继续运行才能执行，不能替代底层电机看门狗。** 真实 FPGA/电机控制器必须具备独立命令超时停车、指令序号/有效期校验、ACK 或状态反馈、急停和上电默认零输出；这些均未由本仿真实现。收到主机指令或回调成功，也不代表电机已执行。实车接入前还需验证地线、电平、编码器接线和最大制动距离。

## ✅ 边界测试

在程序目录执行 `python3 -m unittest discover -s tests -p test_adapters.py -v`。测试涵盖：原始帧可重复回放、元数据兼容、非法数据拒绝、倒序时间戳、队列溢出、线程并发、缺包停车、迟到旧帧停车、旧测距停车、输入限幅、失联/无回波、回调故障和关闭停车。
