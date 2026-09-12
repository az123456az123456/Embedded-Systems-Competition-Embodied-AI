# I2C 读 IMU（MPU6050）（已仿真通过 ✅）

| 文件 | 说明 |
|---|---|
| `i2c_master.v` | 通用 I2C 主控（三相位位引擎，上升沿触发）|
| `imu_mpu6050.v` | MPU6050 封装：初始化唤醒 + 周期读加速度计 6 字节 |
| `tb_imu.v` | 模拟从机测试（PASS）|

## 接口
- `imu_mpu6050` 输出: `accel_x/y/z[15:0]`(有符号), `data_valid`(新数据)
- I2C 地址 0x68，SCL 400kHz

## 测试结果
- ACCEL_X=0x1234 / Y=0x5678 / Z=0x9ABC ✅

## ⚠️ 踩坑记录（重要，都是仿真调出来的）
1. **I2C 位时序**：sda 只能在 scl 为低时改变。用"三相"位引擎（①拉低scl ②低相设数据 ③拉高scl），否则 sda 与 scl 下降沿同拍变化会被从机误判为 START
2. **op 触发**：FSM 的 op 信号保持高电平期间，主控空闲后会自动重发 → 主控必须用**上升沿触发**（op_rise）
3. **握手**：发起 op → 等 busy 变高 → 等 busy 变低（下降沿=完成），否则会重复/漏发
4. **读突发结束**：从机按主控的 ACK/NACK 判断是否继续驱动总线（NACK 后释放，否则和 STOP 冲突导致 sda=X）
5. **16 位负数**：0x9ABC 有符号 = -25924（测试期望值要算对！）

## 仿真
```bash
cd 开发/fpga/imu
iverilog -o sim i2c_master.v imu_mpu6050.v tb_imu.v
vvp sim
```
