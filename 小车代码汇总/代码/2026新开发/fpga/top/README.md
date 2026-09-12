# 顶层集成 top_robot（已仿真通过 ✅）

把五大模块集成为完整的小车控制系统，支持串口指令控制 + 自动避障。

## 集成内容
```
        UART(与RK3588)          ┌─ 编码器解码 ─┐
 上位机 ──命令──> 解析 ──> 安全逻辑 ──> PID ──> PWM ──> 电机
                              ↑                    │
                       超声波测距(避障)      编码器反馈(闭环)
                              ↑
                            IMU(姿态)
```

## 端口
- UART: `uart_rx`, `uart_tx`（115200 8N1）
- 编码器: `enc_a`, `enc_b`
- 电机: `motor_pwm`, `motor_dir`
- 超声波: `echo`, `trig`
- IMU: `imu_sda`(inout), `imu_scl`
- 调试: `o_target`, `o_speed`, `o_distance`, `o_mode`

## 协议
见 `../通信协议.md`（下行 5 字节 / 上行 7 字节）

## 测试结果
| 用例 | 结果 |
|---|---|
| SET_SPEED=500 → 目标生效 + 电机输出 | ✅ |
| 避障模式 + 10cm → 自动停车 | ✅ |
| 退出避障 → 恢复速度 | ✅ |
| STOP → 归零 | ✅ |

## 仿真
```bash
bash 开发/fpga/run_all_sim.sh     # 含顶层
# 或单独:
cd 开发/fpga/top
iverilog -o sim ../uart/uart_tx.v ../uart/uart_rx.v ../encoder/quadrature_decoder.v \
  ../motor/motor_pwm.v ../motor/encoder_velocity.v ../motor/motor_pid.v \
  ../ultrasonic/ultrasonic.v ../imu/i2c_master.v ../imu/imu_mpu6050.v \
  top_robot.v tb_top.v
vvp sim
```

## ⚠️ 关键经验
1. **急停必须 disable PID**：否则积分 windup 会让电机停不下来（本测试就抓到过这个 bug）
2. 安全逻辑（避障停车/减速）放在**目标速度之前**，优先级高于上位机指令
3. 上位机发 `SET_SPEED` 后，FPGA 内部闭环自主控制，不需要上位机持续干预

## 待完善（板卡到货后）
- 双电机差速（左右轮独立 PID）→ 转向
- 里程计（编码器+IMU 融合）
- 摄像头图像链路
- 高云工程综合 + 引脚约束
