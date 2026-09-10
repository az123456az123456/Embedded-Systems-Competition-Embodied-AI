usb.arm/ 机械臂控制系统
├── ui.py              → 主界面（完整版，包含摄像头+舵机控制+自动抓取）
├── uixin.py           → 界面升级版（与ui.py功能类似，优化了部分代码）
├── servo(1).py        → 纯舵机控制界面（无摄像头，模拟控制）
├── yanse.py           → 颜色识别+机械臂抓取（命令行版）
├── yansexiugai.py     → 颜色识别修改版（优化了HSV范围和显示）
├── jiepin.py          → 屏幕截取版颜色识别（使用mss库）
├── color.py           → 颜色识别基础类（仅供测试）
├── jixiebitest.py     → 舵机控制测试脚本
├── usbtest.py         → USB串口发送测试
└── chuandi.py         → 参数传递测试
