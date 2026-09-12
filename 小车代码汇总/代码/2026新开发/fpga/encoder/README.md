# 编码器正交解码模块（已仿真通过 ✅）

| 文件 | 说明 |
|---|---|
| `quadrature_decoder.v` | A/B 相正交解码，4 倍频，位置计数 + 方向 |
| `tb_quadrature.v` | 正转/反转各 2 圈验证（PASS）|

## 接口
- 输入: `enc_a`, `enc_b`（编码器两相）
- 输出: `position`(带符号), `pulse_tick`(每计数脉冲), `direction`

## 仿真
```bash
cd 开发/fpga/encoder
iverilog -o sim quadrature_decoder.v tb_quadrature.v
vvp sim
```

## 说明
- 4 倍频解码：每圈 4 次跳变 = 4 个计数，分辨率 ×4
- A 领先 B = 正转(+1)，B 领先 A = 反转(-1)
- 输入打了 2 拍同步防亚稳态

## 下一步
- 与 `encoder_velocity`（测速）和 `motor_pid`（闭环）配合
