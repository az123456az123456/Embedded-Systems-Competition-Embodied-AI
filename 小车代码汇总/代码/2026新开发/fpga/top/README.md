# FPGA 顶层集成

> 日期：2026-09-12
> 当前推荐：`top_robot_v2.v`，协议 v2 + 双电机差速

---

## 一、推荐顶层：`top_robot_v2`

已通过仿真：

| 用例 | 结果 |
|---|---|
| SET_TWIST `(300,0)` → 左右轮目标 100 | ✅ |
| SET_TWIST `(300,500)` → 左 86 / 右 113 | ✅ |
| UART 返回 ACK + STATUS，CRC 正确 | ✅ |
| 停止 HEARTBEAT 10ms → 看门狗停车 | ✅ |
| 避障模式 + 10cm 回波 → 双轮停转 | ✅ |

集成内容：

```text
UART RX
  ↓
protocol_rx（AA 55 + CRC16 + 序号 + 长度）
  ↓
SET_TWIST / STOP / SET_MODE / HEARTBEAT
  ↓
Twist → 左右轮目标
  ↓
左右轮独立 PID → 双 PWM

超声波 ──> 避障/看门狗安全逻辑
编码器 ──> 左右轮测速反馈
IMU ────> 数据有效标志

STATUS / ACK ──> protocol_tx ──> UART TX
```

端口：

- UART：`uart_rx` / `uart_tx`，115200 8N1
- 左编码器：`enc_left_a` / `enc_left_b`
- 右编码器：`enc_right_a` / `enc_right_b`
- 左电机：`motor_left_pwm` / `motor_left_dir`
- 右电机：`motor_right_pwm` / `motor_right_dir`
- 超声波：`echo` / `trig`
- IMU：`imu_sda` / `imu_scl`
- 调试：左右目标、左右速度、距离、模式、flags、watchdog

协议见 `../通信协议.md`。

### 单独仿真

使用 `run_all_sim.sh` 会自动完成：

```bash
bash 小车代码汇总/代码/2026新开发/fpga/run_all_sim.sh
```

或在本机 EDA 环境激活后：

```bash
cd 小车代码汇总/代码/2026新开发/fpga/top
iverilog -g2001 -o sim \
  ../protocol/crc16_byte.v ../protocol/protocol_rx.v ../protocol/protocol_tx.v \
  ../protocol/comm_watchdog.v ../protocol/twist_to_wheel.v \
  ../uart/uart_tx.v ../uart/uart_rx.v \
  ../encoder/quadrature_decoder.v \
  ../motor/motor_pwm.v ../motor/encoder_velocity.v ../motor/motor_pid.v \
  ../ultrasonic/ultrasonic.v ../imu/i2c_master.v ../imu/imu_mpu6050.v \
  top_robot_v2.v tb_top_v2.v
vvp sim
```

---

## 二、安全行为

1. STOP 原因 `WATCHDOG` 或 `FAULT` 会锁存停车。
2. 通信看门狗默认 500 ms，超时后双击目标归零并禁用 PID。
3. 超声波第一次测距完成前不会把初始 `0 cm` 当成障碍。
4. 障碍 `<20cm` 时双轮目标归零并禁用 PID。
5. 接近障碍 `<40cm` 时线速度和角速度减半。
6. `CLEAR_FAULT` 或新的有效 `SET_TWIST` 可解除普通停车/看门狗锁存。

---

## 三、旧顶层：`top_robot.v`

`top_robot.v` 是 2026-09-12 上午完成的 v1 版本：

- 5 字节 XOR 协议
- 单编码器、单电机
- 固定速度控制

它保留用于回归旧测试，不再作为 RK3588 联调入口。

---

## 四、E2E 联调顺序

1. 用 `run_all_sim.sh` 确认 protocol_v2 和 top_v2 全部 PASS。
2. 在电脑端运行 `rk3588/run_virtual_demo.sh` 验证 RK 协议栈。
3. 板卡到货后，将 `top_robot_v2` 移植进高云工程并建立引脚约束。
4. 真串口联调时，先只验证 STATUS，再验证 STOP，最后验证 SET_TWIST。
5. 实际驱动电机前，先悬空车轮并限制最大 PWM。

---

## 五、待完善

- IMU 姿态角融合（当前 `yaw_centideg` 固定为 0）
- 编码器里程计
- 加速度限制和更细的转向闭环
- 高云 PDS 综合、时序分析和引脚约束
