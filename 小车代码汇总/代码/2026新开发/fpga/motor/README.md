# 电机控制模块（已仿真通过 ✅）

| 文件 | 说明 |
|---|---|
| `motor_pwm.v` | PWM 输出 + 方向（duty -1000..+1000）|
| `encoder_velocity.v` | 编码器测速（窗口内计脉冲数）|
| `motor_pid.v` | 速度 PID（比例/积分/微分，抗饱和+限幅）|
| `tb_motor.v` | 综合测试（PWM/测速/PID 收敛）|

## 仿真
```bash
cd 开发/fpga/motor
iverilog -o sim motor_pwm.v encoder_velocity.v motor_pid.v tb_motor.v
vvp sim
```

## 测试结果（50MHz）
- PWM duty=500 → 占空比 50%，方向正确 ✅
- 测速：每10时钟1脉冲 → 窗口测速 50/-50 ✅
- PID：target=500 → 实际 500；target=-300 → 实际 -300 ✅

## ⚠️ 调参经验（重要）
- **增益必须配合电机模型/控制周期**：测试中电机模型增益 0.125 时 KP=8 稳定；
  模型增益 0.5 时 KP=8 会振荡（`A*KP ≥ 2` 不稳）
- 实际电机调参建议：先只加 P（KP 从小到大），有稳态误差再加 I，最后加 D 抑制超调
- PID 控制周期 `CONTROL_DIV` 按实际需要设（如 1kHz = 50000 @50MHz）

## 下一步
- 接入 FPGA 顶层：编码器→测速→PID→PWM→电机
- 与 UART 指令联动（上位机下发目标速度）
- 真实电机调参（KP/KI/KD 现场整定）
