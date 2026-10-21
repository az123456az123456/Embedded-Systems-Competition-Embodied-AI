# RTL8821CU 驱动：RK3588 Linux 6.1.75

> 日期：2026-10-17

## ✅ 交付范围

为 ACG138 当前 Buildroot 系统单独安装 USB Wi-Fi 驱动，设备 ID `0bda:c820`。不构建系统镜像。
板端说明见 [安装说明](board/安装说明.txt)。

| 项目 | 匹配基线 |
|---|---|
| BSP | ACG138-Offline-BSP-20260923-r4 |
| 内核 / 架构 | 6.1.75 / aarch64 |
| GCC | BSP 自带 10.3.1 20210621 |
| 配置 SHA256 | `7114825305fb4b5f032a523d009ae4eaa321f1c816ff60f44eeaca3b9a0ec20f` |
| 模块标识 | `6.1.75 SMP mod_unload aarch64` |
| 驱动源版本 | `morrownr/8821cu-20210916`，`5b39398e2de146edeb76716420f3288f508bea61` |
| 许可 | GPL-2.0 |

板端配置与编译器已由用户终端输出核实；实板模块加载和无线联网仍待验收。
选用的是 BSP 的 `sdk/yy3588_buildroot/package/rtl8821cu/rtl8821cu.mk` 已指定的提交，
下载归档与 `.hash` 的 SHA256 一致。没有移植新内核，也没有使用 PPT 中不同芯片的二进制驱动。

## 🛠️ 重新编译

电脑需有 GNU make、GCC 主机编译器、flex、bison、bc、OpenSSL 开发头文件、Python 3 和 modinfo。
交叉工具链使用 BSP 自带版本。内核源树保持原样，生成文件放在本目录 `build/kernel/`。

在本目录执行，参数填写本机已有 BSP 和配套符号表路径：

```sh
bash build-driver.sh /path/to/ACG138-Offline-BSP-20260923-r4 /path/to/Module.symvers > reports/build.log 2>&1
python3 verify-build.py /path/to/ACG138-Offline-BSP-20260923-r4 /path/to/Module.symvers
python3 test-installer.py /path/to/ACG138-Offline-BSP-20260923-r4/configs/kernel.config
```

源码归档放在 `downloads/rtl8821cu-5b39398e2de146edeb76716420f3288f508bea61.tar.gz`。
交付源码包已带此归档、配套 `inputs/Module.symvers`、runtime.config、prepared.config。
本内核未启用 CONFIG_MODVERSIONS，使用配套符号表做 modpost 导出和依赖检查。
`modules_prepare` 本身不会生成符号表，不能用空符号表或压制未解析符号错误替代。

模块源码未修改；交叉编译参数沿用 BSP Buildroot 配方，显式指定 arm64 和小端，
关闭平台自动探测，启用 cfg80211 接口。不执行上游面向桌面发行版的安装脚本。

打包命令：

```sh
python3 package.py /path/to/ACG138-Offline-BSP-20260923-r4 /path/to/Module.symvers /path/to/output-directory
```

`package.py` 从实际编译结果更新板端脚本中的模块指纹。重新编译后先打包，再运行脚本测试；
如果有源码、参数或内核基线变化，应重新检查整个适配，而非只更新指纹。

## 📋 检查记录

- `reports/verification.json`：实际 ELF、USB alias、vermagic、源码完整性、191 个外部符号与配置检查。
- `reports/build.log`：完整编译日志。
- `reports/installer-tests.json`：临时目录中的模拟板端测试，不代表硬件加载测试。
- 只安装 `.ko`、该驱动的 modprobe 参数文件和 Buildroot 开机脚本；存在不同版本时停止。
- 包含逐文件 SHA256 检查；安装与开机加载都检查当前内核配置，不强制加载。

## 来源

- [固定版本驱动源码](https://github.com/morrownr/8821cu-20210916/tree/5b39398e2de146edeb76716420f3288f508bea61)
- [USB ID 表](https://github.com/morrownr/8821cu-20210916/blob/5b39398e2de146edeb76716420f3288f508bea61/os_dep/linux/usb_intf.c)
- [Linux 外部模块构建文档](https://docs.kernel.org/kbuild/modules.html)
