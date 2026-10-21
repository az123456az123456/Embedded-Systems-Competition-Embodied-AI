# ACG138 RTL8821CU USB 无线网卡连接手机

> 日期：2026-10-15。当前硬件按用户确认：USB 网卡连接 RK3588。用户确认芯片为 RTL8821CU；网卡尚未实测，USB ID、运行内核及驱动状态待确认。

## 📡 当前连接方式

USB 无线网卡插入 RK3588 USB 主机口，板子与 Android 手机连接同一个路由器。手机 APK 使用板子的无线 IPv4 和配对码；APK、RK3588 服务协议、FPGA 远控位流与网卡接口类型无关，可以继续使用原交付版本。

先用 2.4GHz 网络联调。也可让板子连接手机热点；是否允许手机访问热点客户端需实测。当前没有实现或要求板子自身开热点。

本次明确使用 **Realtek RTL8821CU USB 无线网卡**。先前 AX210/PCIe 的硬件接入说明不适用于这张网卡。网络应用仍采用板子与手机同一局域网的方式。

## 🧩 本地 BSP 核对结果

| 项目 | 资料包中的结果 | 对当前接入的含义 |
|---|---|---|
| 内核源码版本 | Linux 6.1.75 | 用户实机版本尚需 `uname -r` 确认 |
| 资料 boot 内嵌配置和发布配置 | Linux 6.1.75、`CONFIG_RTW88` 未启用，二者配置一致 | 资料内核不提供该 rtw88 路径的 RTL8821CU 支持 |
| 本地 6.1 rtw88 Kconfig | 包含 PCIe 8821CE，未包含 USB 8821CU 配置项 | 不能直接把 CE 当作 CU，或只开启现有 RTW88 就认为支持 USB |
| 资料 rootfs 模块目录 | 核对 `/lib/modules/6.1.75`，未找到 8821cu/rtl8821cu/rtw88_8821cu 模块，modules.builtin 也未列出 | 不能按资料镜像已预装相应驱动来部署 |
| Buildroot 源码 | 有独立 `package/rtl8821cu` 配方 | 说明有构建途径，不能证明发布镜像已装好模块 |
| 内核／rootfs 构建 | 该 rootfs 配置 `BR2_LINUX_KERNEL` 未启用，内核由 BSP 单独构建 | 不能只在这份 Buildroot rootfs 配置里勾驱动包，就假定它会自动匹配并生成 |

Linux Wireless 文档列出主线 rtw88 对这组 USB 设备的支持从 6.2 开始。对于本资料的 6.1 内核，需要检查厂商 `8821cu` 驱动或适配的回移驱动是否已部署；不为增加此网卡而直接替换整套内核，以免影响已跑通的相机与板内 RGMII。

如果最终需要补 `.ko`，应基于**板子实际运行内核**的源码、配置、构建输出和符号版本表交叉编译。Buildroot 没有必要安装 Ubuntu/Debian 的 `apt`/DKMS 驱动包，其他机器的 `.ko` 也不能直接视为兼容。当前没有生成或宣称提供可直接加载的匹配驱动模块。

## 🔍 在板子上确认设备

插入网卡后，在 RK3588 终端执行：

```sh
uname -r
lsusb
ip link
iw dev
```

把输出用于确认设备、接口和驱动；命令缺失不等于设备不存在。更新版脚本不要求安装 lsusb，仍可查询 USB sysfs：

```sh
cd /root/robot-phone
sh scripts/diagnose-network.sh
```

脚本只查询 USB VID:PID、产品名称、设备接口驱动、8821cu/rtw88 模块与可读内核配置、实际网络接口、IPv4 和管理进程；不改网络、不启动服务、不读取 Wi-Fi 密码、不打印进程完整参数。

| 实际现象 | 下一步 |
|---|---|
| USB sysfs 和 lsusb 都没有对应设备 | 检查 USB 主机口、接触、转接与供电；先解决枚举问题 |
| 有 USB 设备，但没有无线接口 | 按 VID:PID、接口驱动及内核信息核对驱动/固件；不能仅添加 Wi-Fi 配置 |
| 只有 Bluetooth/蓝牙接口 | 这不能证明 Wi-Fi 已连接；需确认真实硬件及其 Wi-Fi 通路 |
| 已有无线接口但没有 IPv4 | 检查关联状态、已有网络管理器和 DHCP |
| 无线关联成功且已有 IPv4 | 启动服务，用此地址连接 APK，排除路由器隔离和端口限制 |

无线接口可能叫 wlan0、wlan1 或其他名字，按实际结果填写。板内 FPGA RGMII 按设备 `fe1c0000.ethernet` 识别；不要为了 USB 联网停用、改地址或桥接它。外部 RJ45 是 `fe1b0000.ethernet`，eth0/eth1 编号可能变化。

## 🔑 手动连接模板：仅在接口无人管理时使用

若诊断显示已有 `wpa_supplicant`、`dhcpcd`、`udhcpc`、NetworkManager、connman 或 iwd，先沿用现有管理方式，查它管理的是哪个接口。**进程存在不等于一定管理这张网卡；也不能因此随意停止全部网络服务。**资料镜像包含 `S40network` 和 `S41dhcpcd`，当前实际镜像应以诊断为准。

只有确认目标无线接口没有管理器，才使用以下临时方式。先创建仅自己可读的配置文件，再用板上编辑器填写自己的网络名和密码；不要把密码放在终端命令参数、聊天或日志里：

```sh
umask 077
touch /root/acg138-wifi.conf
chmod 600 /root/acg138-wifi.conf
```

配置模板如下：

```ini
ctrl_interface=/run/wpa_supplicant
update_config=0

network={
    ssid="你的2.4GHz网络名"
    psk="你的无线密码"
    key_mgmt=WPA-PSK
}
```

此模板用于 WPA2-Personal 网络。若路由器使用其他安全模式，配置应按该网络及当前 WPA 版本调整。完成后：

```sh
chmod 600 /root/acg138-wifi.conf
WIFI_IF=实际无线接口名
wpa_supplicant -i "$WIFI_IF" -c /root/acg138-wifi.conf
```

保持该前台进程，在另一个终端只读查询：

```sh
WIFI_IF=实际无线接口名
wpa_cli -i "$WIFI_IF" status
ip -4 addr show dev "$WIFI_IF"
```

`wpa_state=COMPLETED` 表示关联成功。若已有 DHCP 管理器，等待/使用该管理器获取地址；不要再运行下面命令。**仅当该接口既没有 IPv4 地址，也没有 DHCP 客户端在管理，且网络提供 DHCP 时**，在第二终端执行：

```sh
udhcpc -i "$WIFI_IF" -n -q
ip -4 addr show dev "$WIFI_IF"
```

这是临时获取地址，`-q` 成功后退出，不负责长期续租；长期使用应纳入已有网络管理配置。手动前台 WPA 按 Ctrl+C 会断开此无线关联。这里不自动创建启动服务，也不修改现有有线或 FPGA 内部网络。

## 📱 填写 APK 地址

板子连通后，例如无线接口地址为 `192.168.1.120`，在 APK 设置填写 `http://192.168.1.120:8080` 和服务端配对码。服务端要监听该可达接口或 `0.0.0.0`；仅监听 `127.0.0.1` 时手机无法访问。先看实时视频和 IMU，再使能电机。

APP 连接失败不必先换模型或重烧 FPGA：先区分没有关联、没有 IP、访客/热点隔离、服务端未启动、端口监听地址及配对码错误。手机和板子处于同一 SSID 并不能保证互相可达。

## 📚 依据与验证范围

- 本地 `bsp_r4/ACG138-Offline-BSP-20260923-r4/sdk/yy3588_linux/Makefile`、`configs/kernel.config` 和 `drivers/net/wireless/realtek/rtw88/Kconfig`：版本、配置及本地驱动范围。
- 本地 `sdk/yy3588_buildroot/package/rtl8821cu/rtl8821cu.mk` 和 `configs/rockchip_rk3588.config`：驱动配方及内核与 rootfs 分离构建现状。
- [Linux Wireless 官方 Realtek 驱动文档](https://wireless.docs.kernel.org/en/latest/en/users/drivers/rtl819x.html)：rtw88 的 USB 设备支持范围。
- 用户确认型号为 RTL8821CU，但尚未提供实机输出；当前不能提前保证这张网卡插入后即可联网。网卡到手后先跑上面的检查，若已识别无线接口就沿用后续 WPA/DHCP 步骤。
