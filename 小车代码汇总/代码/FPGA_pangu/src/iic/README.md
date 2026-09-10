I2C通信系统
├── XR_iic.v (顶层)
│   ├── XR_iic_ctrl.v (读写控制)
│   └── XR_iic_com.v (I2C底层协议)
├── prj7620_top.v (手势传感器顶层)
│   ├── i2c_ctrl.v (I2C控制器)
│   ├── prj7620_cfg.v (配置数据)
│   └── beep.v (蜂鸣器)
└── hand_ctrl.v (独立手势控制)
