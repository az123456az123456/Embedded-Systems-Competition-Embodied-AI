# ACG138 v17：手机选框、RK 计算目标角、FPGA 执行差速跟随

此工程以 `robot_navigation_v16` 为基准，保留双目 OV5640 30 FPS、IMU、4 路编码器、A02YYUW 超声波、编码器直线均速、PicoRV32/NAV2 A*。相对于 v16，新增 MOT2/ACK2 目标角指令，并将角度译码、接收帧提交与共享发送仲裁流水化，降低时序路径深度；电机、摄像头等引脚及约束不变。

- 工程：`robot_follow_v17.gprj`
- 位流：`impl/pnr/robot_follow_v17.fs`（通过综合、布线及审计后生成）
- 默认轮组：M1 右前，M2 左后，M3 右后，M4 左前。
- 手机第一排左侧电机勾 M2、M4（mask=10），第二排额外翻转四个全勾（mask=15）。
- 手动摇杆、停车、导航继续使用 MOT1；新 RK 跟随模式使用 MOT2。
- 上电、失联及无合法首次停车握手时不会自动运动。

## 目标角的含义

RK 根据选中框的目标视线及摄像头标定参数，反复计算相对目标方位。MOT2 的角度单位是 0.01 度，正数为车头右侧，负数为左侧。这是持续刷新目标偏角的视觉闭环输入，不是“一次发送 30 度，随后 FPGA 保证精确转完 30 度”的定角旋转命令。FPGA 把每次偏角转换成左右轮目标 PWM，目标回到中间后转向减弱；准确角度依赖 RK 端标定，车体响应仍需实车验证。

FPGA 规则：

1. 绝对偏角 ≤ 2 度时，转向分量为 0。
2. 其余角度：转向 PWM 千分比 = 角度厘度 / 25，向零截断，限制到 ±220。
3. 绝对偏角 ≥ 25 度时，前进分量强制为 0，优先转向目标。
4. 左侧目标 = 前进分量 + 转向分量；右侧目标 = 前进分量 − 转向分量。
5. 仍经过原有坡度限制、反转死区、命令序号与 500 ms 失联停车。任意合法零速度立即停止。最大前进指令为 220/1000。

## MOT2/ACK2 线格式

沿用原始以太网 EtherType 0x88B6，目标 MAC 02:ac:13:80:00:01。以太网 payload 前 16 字节是命令，随后是 CCITT CRC16 的高低字节；剩余 Ethernet 最小帧补 0，末尾 Ethernet FCS 校验。CRC16 初值 0xFFFF、多项式 0x1021，覆盖 16 字节。所有多字节数为大端。

| 字节 | MOT2 命令 | ACK2 状态 |
|---|---|---|
| 0–3 | ASCII MOT2 | ASCII ACK2 |
| 4 | 2 | 2 |
| 5 | bit0 软件使能，其余 0 | bit0 已使能，bit1 看门狗，bit2 已配置 |
| 6–9 | 全模式共享递增 uint32 序号 | 最后接受的序号 |
| 10–11 | int16 偏角厘度，−9000…9000，正右负左 | int16 实际已应用左侧 PWM 千分比 |
| 12–13 | uint16 前进 PWM 千分比，0…220 | int16 实际已应用右侧 PWM 千分比 |
| 14 | 左侧电机 mask，高 4 位为 0 | 当前左侧 mask |
| 15 | 电机方向翻转 mask，高 4 位为 0 | 当前方向 mask |

软件未使能包必须角度和前进均为 0。该停车包可配置 mask；已经使能时禁止变更 mask。MOT1 与 MOT2 共用序号，不允许通过模式切换重放旧指令。ACK 的版本跟随最后一次成功接受的指令；非法包不改变 ACK 版本，也不刷新看门狗。ACK2 表示支持角度模式，不表示实车已经转过某一角度。

## 构建与验证

`GOWIN_SH=/path/to/IDE/bin/gw_sh sh tools/build_softcore.sh`

保留并使用原 Gowin V1.9.12.03 和 GW5AT-LV60PG484AC1/I0 目标；构建脚本会在临时目录综合、布局布线，再由 `tools/audit_release.py` 检查全部 75 根引脚及内部 STA。

`python3 tests/run_follow_regression.py` 运行 RTL 回归。`python3 tests/test_angle_crosswire.py` 以同级 RK v17 的真实编码器/解码器验证完整 MOT1/MOT2 → IDDR → parser → CDC → 电机控制 → ACK1/ACK2 链路。`sh tests/test_scheduler_v17.sh` 运行 5 路共享链路压力仿真，检查完整输出帧 FCS、独占授权、帧间隙及 FIFO。详见 `reports/rtl-regression.log`、`reports/release_verification.json`。旧版本说明保存于 `README-v16-baseline.md` 仅供历史参考。

尚未在真实车辆或替换后的开发板上验证；报告中的内部 STA 不等同于摄像头及 RGMII 外部时序签核。

本次最终布局布线：Logic 17,061 / 59,904；BSRAM 56 / 118；内部 setup/hold 违例均为 0。最小 setup 余量仅 0.002 ns，仍很紧；这不代表外部 RGMII/摄像头时序已在真实板卡上验证。完整资源、时序和位流 SHA256 见 `reports/release_verification.json`。
