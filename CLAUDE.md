# CLAUDE.md

> 面向 AI 协作者（Claude / Codex 等）与新增成员的**项目导航 + 协作约定**。
> 先读这个，再动代码。日期：2026-09-12

## 项目是什么

2026 深技大嵌入式竞赛 **选题三**：`基于 FPGA 的具身智能机器人实时感知与自主控制系统设计`（高云半导体赛道，小梅哥 ACG138-RK3588 板）。

- **任务 5（视觉目标识别）**：人员/物体识别、指定目标跟随。
- **任务 6（智能任务执行）**：指定目标搜索、语义导航、自主执行（例：找到红色水瓶并移动到其附近）。

视觉在 **RK3588** 上跑，运动/测距在 **FPGA** 上跑，两者串口通信。

## 目录地图

```
├── 团队文档/          团队文档导航（README.md）+ 需求/规范/环境/进度/技能库
├── 小车代码汇总/代码/  全部代码：旧框架（FPGA_pangu / ultralytics-main / usb.arm）+ 新开发 fpga/rk3588
├── 开发/              新开发目录（本机目前只有 rk3588/）
├── yolov8n.pt         根目录遗留的重复权重（与 小车代码汇总/代码/2026新开发/rk3588/yolov8n.pt 重复，可清理）
└── .venv/             Python 虚拟环境（本机实际使用，已 gitignore，不提交）
```

## 协作约定（务必遵守）

1. **Python 环境**：用仓库根目录下的 `.venv`（Python 3.10.12，torch 2.14.0+cpu / ultralytics 8.4.147）。
   ```bash
   cd <仓库根目录> && source .venv/bin/activate
   ```
   > 原开发机（woonlf）用的是 `开发/venv`（Python 3.14，另装了 pyautogui / mss）。**本机没有这个目录**，
   > 也没有 `开发/工具/`、`开发/测试输出/`。文档里的路径一律写成相对仓库根的形式，别写死绝对路径。
2. **权重不进版本库**：`*.pt / *.pth / *.onnx / *.rknn` 已 gitignore，从官方链接单独下载（见 `团队文档/进度/2026-09-12_视觉模块新增功能说明.md`）。
3. **代码放 `小车代码汇总/代码/`，说明放 `团队文档/`**：新文档沿用现有格式（`# 标题` + `> 日期` + emoji + 表格）。
4. **协议 v2 已冻结**：`AA 55 + CMD + SEQ + LEN + PAYLOAD + CRC16`。RK 端和 FPGA `top_robot_v2` 已实现，待高云综合/实板联调。任务 6 优先使用 STATUS 中的超声波距离，失效时回退框高。
5. **旧框架代码只读**：`小车代码汇总/代码/` 下的 `FPGA_pangu` / `ultralytics-main` / `usb.arm` 是历史参照，不要改。

## 视觉模块关键文件（均在 `小车代码汇总/代码/2026新开发/rk3588/`）

| 文件 | 干什么 | 改它时注意 |
|---|---|---|
| `vision_demo.py` | 任务5 感知（检测+跟踪+颜色，开环） | 颜色判定 `COLOR_RANGES` |
| `task6_mission.py` | 任务6 状态机闭环 + 运动指令（**主入口**） | 状态机 / `MissionReID` / `MotionCommand` |
| `deep_reid.py` | 深度 ReID 特征（ResNet50-IBN-a，2048 维） | 已做次正规数清零提速，勿删 |
| `fpga_protocol.py` | 协议 v2 编解码、CRC16、流式帧解析 | 帧格式变更必须同步通信协议文档 |
| `fpga_link.py` | 串口、心跳、STATUS、看门狗停车 | 不能在 RX/心跳线程中做重计算 |
| `virtual_fpga.py` | PTY 虚拟 FPGA，无板卡闭环（支持障碍物/位置） | 只模拟运动学，不代替实板验证 |
| `navigation.py` | 任务4：A* 规划、里程计、路径点控制 | 纯逻辑模块，不启动串口 |
| `nav_demo.py` | 任务4 无板卡导航演示（虚拟 FPGA） | 依赖 `virtual_fpga.py` |
| `semantic_parser.py` | 中文语义指令解析 | 与 `task6_mission.py` 的 `--instruction` 配合 |

两层 ID：**ByteTrack 过程 ID**（遮挡会变）→ **ReID 逻辑 ID**（跨遮挡稳定），跟随只看逻辑 ID。

## 当前进度 / 下一步（截至 2026-09-12）

- ✅ 检测 + 跟踪 + 颜色；✅ 任务6 状态机闭环；✅ 深度 ReID（外观+空间双特征+空间连续性兜底）
- ✅ RK 端协议 v2 + CRC16 + 心跳 + 看门狗 + 虚拟 FPGA 闭环
- ✅ 任务 6 优先使用 FPGA 超声波距离，STATUS 失效时回退框高
- ✅ FPGA `top_robot_v2` 已实现协议 v2、CRC16、看门狗和双电机差速，并通过 Icarus 仿真
- ✅ 任务4 PC 导航模拟（A* + 里程计 + 差速 + 虚拟 FPGA）
- ⏳ `top_robot_v2` 高云综合、引脚约束和实板联调
- ✅ 中文语义解析（`semantic_parser.py`，已接入 `--instruction`）
- ✅ 摄像头普通水瓶识别实测（YOLO + Deep ReID + 状态机）
- ✅ 动作日志去抖（`--action-log-interval/delta-v/delta-w`）
- ⏳ `.pt → .rknn` 部署链路

## 常见任务的入口

- **跑通任务6 演示**：`cd 小车代码汇总/代码/2026新开发/rk3588 && python task6_mission.py --target bottle --target-color red`
- **无板卡完整闭环**：`cd 小车代码汇总/代码/2026新开发/rk3588 && bash run_virtual_demo.sh --target bottle --target-color red`
- **调跟随逻辑**：改 `task6_mission.py` 的状态机 / `pick_follow_target`
- **接真实测距/串口**：协议实现改 `fpga_protocol.py` / `fpga_link.py`，动作映射改 `MotionCommand`
- **新增文档**：团队级说明写 `团队文档/` 对应分类；协作记录写 `团队文档/进度/`；代码说明写对应模块 `README.md`
