# ACG138 USB 无线网卡连接手机

> 日期：2026-10-15。当前硬件按用户确认：USB 网卡连接 RK3588。芯片型号、USB ID、实机驱动状态尚待确认。

## 📡 当前连接方式

USB 无线网卡插入 RK3588 USB 主机口，板子与 Android 手机连接同一个路由器。手机 APK 使用板子的无线 IPv4 和配对码；APK、RK3588 服务协议、FPGA 远控位流与网卡接口类型无关，可以继续使用原交付版本。

先用 2.4GHz 网络联调。也可让板子连接手机热点；是否允许手机访问热点客户端需实测。当前没有实现或要求板子自身开热点。

之前提到的 AX210 需要核对：Intel 官方规格明确 **AX210 Wi-Fi 使用 PCIe，蓝牙使用 USB**。若是原生 AX210 加普通 USB 转接，枚举到蓝牙并不等于有 Wi-Fi；若是其他 USB 无线芯片，要按其 USB VID:PID 和当前内核驱动判断。不能由外壳名称或“AX”速率标签推断芯片，也不能在确认前安装 iwlwifi 或任意桌面 Linux 驱动。

## 🔍 在板子上确认设备

插入网卡后，在 RK3588 终端执行：

```sh
lsusb
ip link
iw dev
```

把输出用于确认设备、接口和驱动；命令缺失不等于设备不存在。更新版脚本不要求安装 lsusb，仍可查询 USB sysfs：

```sh
cd /root/robot-phone
sh scripts/diagnose-network.sh
```

脚本只查询 USB VID:PID、产品名称、设备接口驱动、实际网络接口、IPv4 和管理进程；不改网络、不启动服务、不读取 Wi-Fi 密码、不打印进程完整参数。

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

- [Intel AX210 官方规格](https://www.intel.com/content/www/us/en/products/sku/204836/intel-wifi-6e-ax210-gig/specifications.html)：System Interface Type 为 Wi-Fi(PCIe)、BT(USB)。
- 本地 ACG138 手册与设备树只用于识别板内 GMAC/原 PCIe 参考方案；不能作为当前未知 USB 芯片驱动已支持的证明。
- 尚未收到实机 USB ID/驱动输出，也未连接实物；当前只能确认软件不依赖特定外部网卡，不能提前保证此 USB 设备已能联网。
