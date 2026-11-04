候选26：本轮最终选择。实际整工程setup=0、hold=0，RX129.926MHz，全工程最小setup余量+0.303ns。

位流：robot_navigation_v42_candidate26_timing.fs；工具本次实际输出的复制件。
源码：robot_navigation_v42_candidate26_source.tar.gz；含158个完整输入文件，包括RTL、相机通路、SDC/CST、软核源码及已用固件/初始化映像。解压后用同版Gowin_V1.9.12.03打开.gprj或工程目录内build.tcl；重新构建需重新核对时序。
校验：SHA256SUMS和manifest.json，源码包逐文件解压重算hash验证。

从18到26：新增最终12路PWM/IN1/IN2成组输出寄存器；nav_live内等价消除重复组合条件。原owner撤权事件、优先级和状态更新不改，PWM20kHz、反转死区1ms、CPU协议不改。工具最终Reg较18增加14，CLS减少120。
新FF输入正常50MHz setup/hold检查；FF输出到引脚仍12ns，原CDC12ns不改；没有用falsepath或multicycle豁免新逻辑。

等价证明通过，owner135例/519224周期对照通过。最终完整取消组件链9例通过（184记录/8合格关断），parser首idle→引脚首次全低最大111ns。不是桥输出/电流/机械停稳时间，也不是实板上界。P0/K未正式签收，临时1微秒测试口径不变。
未烧板，未改RK或手机；原线A通过前不烧线B联调，不自动使能。22、18以及旧v40回滚资料保留。最终审查见上级keep-record.md及../../motor-output-optimization-summary.md。
