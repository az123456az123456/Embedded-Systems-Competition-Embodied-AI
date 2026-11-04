# 虚拟 FPGA 联调说明

> 日期：2026-09-12
> 目的：为独立 Python 原型提供协议 v2 和无板卡闭环测试

---

## 一、当前闭环

```text
摄像头
  ↓
YOLO 检测 + ByteTrack 跟踪 + Deep ReID
  ↓
任务 6 状态机：SEARCH → LOCK → ALIGN → APPROACH → ARRIVE
  ↓
协议 v2：SET_TWIST / STOP / HEARTBEAT
  ↓
虚拟 FPGA（PTY 串口）
  ↓
左右轮速度、超声波距离、航向、STATUS
  ↑
任务 6 使用真实距离决定是否 ARRIVE
```

虚拟 FPGA 没有真实电机，只做速度和距离积分，因此测试过程中不会驱动任何硬件。

## 二、文件说明

| 文件 | 作用 |
|---|---|
| `fpga_protocol.py` | 协议 v2 编解码、CRC16、流式帧解析 |
| `fpga_link.py` | RK3588 串口客户端、心跳、STATUS 缓存、超时停车 |
| `virtual_fpga.py` | PTY 虚拟 FPGA、车体运动与超声波模型 |
| `task6_mission.py` | 任务 6 主程序，已接入协议 v2 |
| `test_fpga_protocol.py` | 协议单元测试 |
| `test_virtual_fpga_link.py` | PTY 真实串口链路测试 |
| `run_virtual_demo.sh` | 一键启动虚拟 FPGA + 摄像头任务 6 |

协议详细定义见：

```text
fpga/通信协议.md
```

## 三、一键运行

在桌面终端执行：

```bash
cd rk3588
bash run_virtual_demo.sh
```

默认行为：

- 摄像头编号 `0`
- 目标类别 `person`
- 置信度 `0.25`
- 推理尺寸 `320`
- 虚拟超声波初始距离 `200 cm`
- 到达距离 `60 cm`
- 自动使用 Deep ReID

退出：

- 摄像头窗口按 `q`
- 或终端按 `Ctrl+C`

退出时会发送 STOP，随后关闭虚拟 FPGA。

## 四、自定义参数

脚本参数直接传给 `task6_mission.py`：

```bash
bash run_virtual_demo.sh \
  --source 0 \
  --target bottle \
  --target-color red \
  --conf 0.25 \
  --arrive-distance-cm 45
```

可用环境变量：

| 变量 | 默认值 | 说明 |
|---|---:|---|
| `PYTHON` | 仓库 `.venv/bin/python` / `python3` | 可显式指定 Python 解释器 |
| `MODEL` | 本机 `yolov8n.pt` | 检测权重 |
| `VIRTUAL_DISTANCE_CM` | 200 | 虚拟超声波初始距离 |
| `WATCHDOG_MS` | 500 | 虚拟 FPGA 通信看门狗 |
| `SIM_DURATION_SECONDS` | 600 | 虚拟 FPGA 最长运行时间 |

## 五、单独运行虚拟 FPGA

终端 A：

```bash
python virtual_fpga.py --pty-file /tmp/virtual_fpga.tty --status-hz 10
```

终端 B：

```bash
python task6_mission.py \
  --serial /tmp/virtual_fpga.tty \
  --distance-source auto
```

虚拟 FPGA 会把 PTY 从设备路径写入 `/tmp/virtual_fpga.tty`。

## 六、自动测试

```bash
python -m unittest -v test_fpga_protocol.py test_virtual_fpga_link.py
```

测试覆盖：

- CRC16 标准向量
- 正常帧编解码
- 串口分片与噪声重同步
- 坏 CRC 后恢复
- 状态帧字段
- PTY 串口连接
- 心跳和 STATUS 接收
- SET_TWIST 后虚拟距离下降
- STOP 后左右轮速度归零

## 七、安全验证

可以用以下方式测试看门狗：

1. 启动虚拟 FPGA 和任务 6。
2. 让任务 6 因 `Ctrl+C` 退出。
3. 观察虚拟 FPGA：超过 `WATCHDOG_MS` 未收到心跳后自动归零。

真实 FPGA 必须实现同样的独立看门狗逻辑。

## 八、当前限制

- 虚拟 FPGA 不是物理仿真器，不包含电机惯量、摩擦和轮胎打滑。
- UART v2 配套顶层为 `fpga/top/top_robot_v2.v`；本原型不使用手机遥控的 RGMII 电机协议。
- 虚拟 PTY 使用 Linux 伪终端；Windows 需改用 COM 串口对或 TCP 桥。
- Deep ReID 权重不进入 Git，需单独放置。
