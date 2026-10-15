# 分级负载诊断版编译与验证

2026-10-13，器件GW5AT-LV60PG484AC1/I0，Gowin V1.9.12.03。

- 完整编译成功，退出码0，位流为impl/pnr/fmc_motor_load_diag.fs。
- 时钟50MHz，报告Fmax 165.513MHz；Setup/Hold违规均为0，同域TNS均为0。
- 17个已约束信号，12个电机输出球位与已跑通的v2/v3接线一致。
- 仿真通过逐级加入1/2/3/4电机、LED运行指令、M3/M4反向掩码、60%匀速段PWM连续性、最终停车。
- 构建输入与当前源码逐文件一致，散列和位流散列记录在reports/source_bitstream_sha256.json。

本版保持20kHz、最高60%占空比。等待3秒，M1缓升至60%，随后每秒加入一个电机；四个全部加入后保持约12秒，再全部缓降至0、停止，不循环。M2/M3/M4加入时直接进入60%，请保持轮子悬空。D0～D3是对应电机的非零指令灯，LED不测量驱动实际输出。

首要排查项是用户已报告的两套独立电源未接信号共地，以及VCC实际供电未确认。先修正接线、确认VCC约3.3V，再测试。STBY常高不能替代VCC供电。本次没有上板测试，不能确认实物卡顿已经消除；静态时序通过也不代表电源和驱动验收完成。

相关报告：reports/simulation.log、reports/gowin.log、reports/fmc_motor_load_diag.pin.html、reports/fmc_motor_load_diag_tr_content.html。
