# ACG138 v24 超声波远距与稳定性更新

本工程从冻结的 robot_control_v21 复制。唯一改变的合成 RTL 是 `sensors/ultrasonic_sensor.v`；电机控制、编码器、摄像头、RGMII、IMU、A* 软核、初始化固件、CST 与 SDC 保持逐字节一致。

## 测距语义

- 校验正确、30～4500 mm：保留有效数值。4001～4500 mm 由配套 RK 显示为 `>4m`。
- 校验正确、4501～65534：参考 DFRobot 官方驱动的超量程钳位行为，进入滤波前钳到 4500 mm。
- 0～29、65535：无效，不能据此声称远方无障碍。
- 单个坏校验帧不刷新旧距离和时间戳，打断远距确认链；旧有效值仍受原有超时约束。断线/连续坏帧超时保持 fault。
- SNS1 字段顺序、valid/fault 位、CRC16、EtherType、全部引脚保持不变。没有新增信号线。

参考：[DFRobot 官方 Python 驱动](https://github.com/DFRobot/DFRobot_RaspberryPi_A02YYUW/blob/master/DFRobot_RaspberryPi_A02YYUW.py)。本工程额外保守拒绝 65535；不能把断线、零值或坏帧转换成有效远距。

## FPGA 数字滤波

更近的有效距离在第一条完整有效 UART 帧结束时立即上报，不加入多帧近障延迟。小幅变远由原 1/2 平滑改为 1/4 平滑；仿真中 980/1020 mm 交替输入的 40 mm 峰峰值降至输出 10 mm。较大的变远仍需两条连续、彼此相近的有效观测，使用两者较小值。

记住最后一次真实距离与在线状态分离：坏值或断线不会抹掉之前的近障参考。恢复后，更近值立即生效；变远仍需确认。候选远值独立在 1 秒过期，不会跨长时间断线拼凑确认；已拒绝的远尖峰不刷新上次真实测距的年龄。该策略偏保守，会快速响应靠近、缓慢确认远离。

## 编译与验证

打开 `robot_control_v24.gprj`，或使用现有 Gowin 1.9.12.03：

```sh
GOWIN_COMPAT_CXX=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 \
GOWIN_SH=/home/tan/qiansai/Gowin_V1.9.12.03/IDE/bin/gw_sh \
sh tools/build_softcore.sh
make test-release
```

现有 `firmware.hex` 已包含于 CPU BSRAM 初始化；不要单独替换导航固件。新位流位于 `impl/pnr/robot_control_v24.fs`。`reports/release_verification.json` 绑定本次源码、位流、75 个管脚、实际内部 STA、64 KiB 固件初始化和 88 条 Gray CDC 检查。

本次真实 Gowin 构建：setup/hold 违例均 0，最差 setup +0.173 ns、hold +0.200 ns；Logic 18261/59904，Register 16009/60780，BSRAM 66/118。未进行实体车验收，外部摄像头/RGMII 板级 IO 不是已完成物理签核。

`reports/sonar-v24-sns.hex` 为真实 UART→采集 RTL 产生的 64 字节 SNS1 回放；`sonar-v24-sns-cases.tsv` 提供输入场景编号。更多原工程背景保留在 `README-v21-baseline.md`。
