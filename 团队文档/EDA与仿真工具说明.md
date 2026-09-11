# EDA 与仿真工具说明（已配置）

> 配置日期：2026-09-11 ｜ 位置：`开发/工具/`

## 一、Verilog 仿真工具（已装好 ✅）

| 工具 | 版本 | 位置 |
|---|---|---|
| iverilog | 13.0 | `开发/工具/eda-env/bin/iverilog` |
| vvp（仿真运行时）| 13.0 | 同目录 |
| GTKWave（波形查看）| 已装 | 同目录 |

**用法**（一键激活）：
```bash
source /home/woonlf/Documents/Codex/Embedded-Systems-Competition/开发/工具/setup_env.sh
```

**仿真流程示例**（LED 闪烁已验证 ✅）：
```bash
cd 开发/工具/demo
iverilog -o sim led_counter.v tb_led.v   # 编译
vvp sim                                   # 仿真，输出 PASS
gtkwave led_wave.vcd                      # 查看波形（需图形界面）
```

## 二、高云 EDA（云源软件）— 需要官网注册后下载 ⏳

**为什么还没装**：下载必须登录高云官网账号（带验证码），无法自动化。

**请队员按以下步骤操作**（约 10 分钟）：
1. 打开 <https://www.gowinsemi.com.cn/software_share>（软件下载中心）
2. 注册/登录高云账号（建议用学校邮箱）
3. 下载 **Gowin EDA（云源软件）For Linux**（选择支持 GW5AT 系列的最新版）
4. 同时申请 **License**（免费，官网 profile/license）
5. 下载完成后告诉 Codex，帮你安装到项目里

> 💡 现在没装不影响：**学习 Verilog 和 RTL 仿真用 iverilog 就够了**；
> 高云 EDA 主要用于「综合 + 布线 + 生成比特流」，等板卡（9/20）到了再装完全来得及。

## 三、当前学习/开发工具链

| 用途 | 工具 | 状态 |
|---|---|---|
| Verilog RTL 仿真 | iverilog + GTKWave | ✅ 已装 |
| Python 视觉 / YOLO | venv + ultralytics | ✅ 已装 |
| FPGA 综合/烧录 | 高云 EDA | ⏳ 待官网注册下载 |
