# EDA 与仿真工具说明

> 更新：2026-09-18 ｜ 本机工具链装在 `~/tools/oss-cad-suite/`（**仓库外**，不随 git 走）

## 一、Verilog 仿真工具（本机已装好 ✅）

| 工具 | 版本 | 位置 |
|---|---|---|
| iverilog | 14.0 | `~/tools/oss-cad-suite/bin/iverilog` |
| vvp（仿真运行时）| 14.0 | 同目录 |
| GTKWave（波形查看）| 已装 | 同目录 |
| Verilator | 5.051 | 同目录 |
| Yosys | 0.67 | 同目录 |

**用法**：`~/.bashrc` 里已经把这套工具的 `bin` 加进 PATH（第 128 行），新开终端直接调用即可：

```bash
which iverilog     # 应输出 /home/phr/tools/oss-cad-suite/bin/iverilog
```

如果换了终端类型（比如非交互式 shell）找不到命令，手动激活：
```bash
source ~/tools/oss-cad-suite/environment
```

**仿真流程示例**：
```bash
cd <有 testbench 的目录>
iverilog -o sim led_counter.v tb_led.v   # 编译
vvp sim                                   # 仿真，输出 PASS
gtkwave led_wave.vcd                      # 查看波形（需图形界面）
```

> 原开发机把这套工具装在**仓库内**的 `开发/工具/eda-env/`，用 `开发/工具/setup_env.sh` 激活。
> 本机**没有** `开发/工具/` 这个目录，用的是仓库外的 `~/tools/oss-cad-suite`。

## 二、高云 EDA（云源软件）— 本机已装 ✅

位置 `~/gowin/`（含 `IDE/` 和 `Programmer/`），器件库里已经有 **GW5AT-138B / GW5AT-138C**。

```bash
~/gowin/IDE/bin/gw_ide
```

如果报 freetype 符号错误 / Qt 版本混用 / Wayland 下起不来，用技能库 `gowin-eda-setup` 里那套环境变量
（`LD_LIBRARY_PATH` 指向 `IDE/lib` + `LD_PRELOAD` 系统 freetype + `QT_QPA_PLATFORM=xcb`）——那是原开发机验证过的组合，本机还没试过。

> ⚠️ **两点没验证，别当成已就绪**：
> - 比赛板那颗 **GW5AT-LV138PG484AC1/I0** 的封装能不能直接选中，要建工程时才看得出。器件库里只有 `GW5AT-138B` / `GW5AT-138C` 这类条目，**没找到带 `PG484` 的封装文件**。
> - License：`~/gowin/IDE/bin/gwlicense.ini` 指的是 license 服务器 `jinan3016:10559`，本机**没有**本地 `license.dat`。能不能正常综合得实跑一次才知道。

**队友要新装**：见《高云EDA下载指引.md》，装在仓库**外面**（几个 GB，别进 git）。

> 日常学习 Verilog 和 RTL 仿真用 iverilog 就够了；高云 EDA 主要管「综合 + 布线 + 生成比特流」。

## 三、当前学习/开发工具链

| 用途 | 工具 | 状态 |
|---|---|---|
| Verilog RTL 仿真 | iverilog + GTKWave（oss-cad-suite）| ✅ 已装 |
| Verilog 静态检查 / 综合探索 | Verilator + Yosys（oss-cad-suite）| ✅ 已装 |
| Python 视觉 / YOLO | 仓库根 `.venv` + ultralytics | ✅ 已装 |
| FPGA 综合/烧录 | 高云 EDA | ⏳ 待官网注册下载 |
