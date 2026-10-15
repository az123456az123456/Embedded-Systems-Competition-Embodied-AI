# 高云 EDA（云源软件）安装说明

> 更新：2026-09-18

## 本机（phr）已经装好了 ✅

位置 `~/gowin/`（含 `IDE/` 和 `Programmer/`），器件库里已经有 **GW5AT-138B / GW5AT-138C**，138 系列在列表里。

启动：
```bash
~/gowin/IDE/bin/gw_ide
```

如果报 freetype 符号错误 / Qt 版本混用 / Wayland 下起不来，用技能库 `gowin-eda-setup` 里那套环境变量
（`LD_LIBRARY_PATH` 指向 `IDE/lib` + `LD_PRELOAD` 系统 freetype + `QT_QPA_PLATFORM=xcb`）——那是原开发机验证过的组合，本机还没试过。

> ⚠️ **两点没验证，别当成已就绪**：
> - 比赛板那颗 **GW5AT-LV138PG484AC1/I0** 的封装能不能直接选中，要建工程时才看得出。器件库里只有 `GW5AT-138B` / `GW5AT-138C` 这类条目，**没找到带 `PG484` 的封装文件**。
> - License：`~/gowin/IDE/bin/gwlicense.ini` 指的是 license 服务器 `jinan3016:10559`，本机**没有**本地 `license.dat`。能不能正常综合得实跑一次才知道。

## 队友要新装的话（约 10 分钟）

下载必须**登录高云官网账号**（带验证码），无法自动化，需手动完成。

1. 打开高云软件下载中心：<https://www.gowinsemi.com.cn/software_share>
2. **注册/登录**账号（建议用学校邮箱，可申请免费 License）
3. 找到 **Gowin EDA（云源软件）For Linux**，选择**支持 GW5AT 系列**的版本下载
4. 同时到 **profile/license** 申请免费 License
5. 下载完成后告诉 Codex，帮忙安装

> 建议装在仓库**外面**（比如 `~/gowin/`）——安装包有几个 GB，别进 git。
> 原开发机装在仓库内的 `开发/工具/gowin-eda/`，本机没有这个目录。

## 注意

- 文件较大（可能数 GB），请预留磁盘空间
- Linux 版通常是自解压安装包或 tar 包
- 装好后主要用于：**综合 + 布局布线 + 生成比特流**（等板卡到了使用）
- 学习 Verilog 和 RTL 仿真目前用 iverilog 即可（已装好，见《EDA与仿真工具说明.md》）
