# 2026 嵌入式系统竞赛 · 具身智能机器人

> 选题三：基于 FPGA 的具身智能机器人实时感知与自主控制系统设计
> 平台：小梅哥 ACG138-RK3588-CORE（RK3588 + 高云 GW5AT-60K FPGA）

## 快速开始

```bash
# 1. 先读团队文档导航
#    -> 团队文档/README.md

# 2. FPGA 全量仿真
bash 小车代码汇总/代码/2026新开发/fpga/run_all_sim.sh

# 3. 无板卡完整闭环（摄像头 + Deep ReID + 虚拟 FPGA）
cd 小车代码汇总/代码/2026新开发/rk3588
bash run_virtual_demo.sh

# 4. 任务4 导航模拟（A* + 里程计 + 虚拟 FPGA）
cd 小车代码汇总/代码/2026新开发/rk3588
python nav_demo.py --instruction "走到右上角"
```

## 当前状态（2026-09-12）

- ✅ 通信协议 v2（CRC16、SET_TWIST、STATUS、心跳、看门狗）
- ✅ FPGA 双电机差速 RTL 仿真（`top_robot_v2`）
- ✅ RK 视觉 + 任务 6 状态机 PC 端闭环
- ✅ 虚拟 FPGA 无板卡联调
- ✅ 任务4 PC 导航模拟（A* + 里程计 + 虚拟 FPGA）
- ✅ 中文语义解析（`--instruction`）
- ⏳ 高云综合、实板联调、机械臂

## 目录

```text
团队文档/              团队文档导航、需求、规范、进度、技能库
小车代码汇总/代码/
├── FPGA_pangu/        2025 旧框架（紫光 Pango，只读参考）
├── ultralytics-main/  2025 旧视觉代码（只读参考）
├── usb.arm/           2025 机械臂代码（只读参考）
└── 2026新开发/
    ├── fpga/          FPGA 协议 v2、双电机、传感器模块
    └── rk3588/        RK 视觉、任务 6、虚拟 FPGA、协议通信层
```

## 注意事项

- 比赛源码私有，请勿公开
- 模型权重 `.pt/.pth/.onnx/.rknn` 不提交 Git
- 详细协作规范见 [`团队文档/团队规范/团队Git协作规范.md`](团队文档/团队规范/团队Git协作规范.md)
