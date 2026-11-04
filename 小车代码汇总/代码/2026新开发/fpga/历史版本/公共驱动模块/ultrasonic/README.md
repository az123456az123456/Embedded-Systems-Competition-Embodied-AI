# 超声波模块入口

当前硬件使用三个A02YYUW，安装方向为前、左、右。以 [A02YYUW驱动与接口说明](a02yyuw/README.md) 为准。

电机控制调用 `a02yyuw/a02yyuw_obstacle_if.v`，获取各方向障碍标志、毫米距离、有效状态和故障状态；单独板级调试选择 `a02yyuw/top.v` 的 `top`。

运行当前实现测试：

```bash
bash a02yyuw/run_tests.sh
```

`legacy_hcsr04/`是旧Trig/Echo模块及测试，仅供现有旧机器人顶层回归；它不适用于A02YYUW。旧 `top_robot` 和 `top_robot_v2` 尚未迁移为三路UART输入，集成步骤见当前接口说明。
