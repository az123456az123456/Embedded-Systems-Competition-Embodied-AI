#!/bin/sh
# Read-only Buildroot/BusyBox network inventory. Never changes an interface,
# starts a daemon, reads Wi-Fi configuration, or prints process command lines.
set -u

has() { command -v "$1" >/dev/null 2>&1; }
heading() { printf '\n%s\n' "$1"; }
target_of() {
    [ -e "$1" ] || [ -L "$1" ] || return 0
    if has readlink; then readlink -f "$1" 2>/dev/null || readlink "$1" 2>/dev/null || true; fi
}
usb_text() {
    # USB descriptor strings are untrusted display data: bound their length,
    # strip terminal control characters, and never evaluate them as shell code.
    [ -r "$1" ] || return 0
    dd if="$1" bs=256 count=1 2>/dev/null | tr -d '\000-\037\177'
}

printf '%s\n' 'ACG138 / RK3588 网络只读诊断'
printf '%s\n' '不会联网、改配置、重启服务或读取 Wi-Fi 密码。'
printf '内核: '; uname -r 2>/dev/null || true
if [ -r /proc/device-tree/model ]; then
    printf '设备: '; tr -d '\000' < /proc/device-tree/model; printf '\n'
fi

heading '1. 工具是否存在（存在不表示当前网卡可用）'
for acg_tool in ip ifconfig route iw wpa_supplicant wpa_cli wpa_passphrase hostapd udhcpc dhcpcd lsusb lspci rfkill modinfo zcat; do
    if has "$acg_tool"; then printf '  %-18s %s\n' "$acg_tool" "$(command -v "$acg_tool")"
    else printf '  %-18s %s\n' "$acg_tool" '未找到'; fi
done

heading '2. 实际网络接口、设备身份与 IPv4'
acg_wifi_count=0
acg_fpga_count=0
acg_external_count=0
acg_wifi_names=''
for acg_net in /sys/class/net/*; do
    [ -e "$acg_net" ] || continue
    acg_iface=${acg_net##*/}
    acg_device=$(target_of "$acg_net/device")
    acg_driver=$(target_of "$acg_net/device/driver")
    acg_subsystem=$(target_of "$acg_net/device/subsystem")
    acg_transport='未知（以实际 device 路径为准）'
    case "$acg_device" in
        */usb[0-9]*/*) acg_transport='USB' ;;
        */pci[0-9a-fA-F]*/*) acg_transport='PCI/PCIe' ;;
        */mmc_host/*) acg_transport='SDIO/MMC' ;;
    esac
    case "${acg_subsystem##*/}" in
        usb) acg_transport='USB' ;;
        pci) acg_transport='PCI/PCIe' ;;
        sdio) acg_transport='SDIO' ;;
    esac
    acg_kind='其他接口（需按实际用途核对）'
    case "$acg_device" in
        */fe1c0000.ethernet|*/fe1c0000.ethernet/*)
            acg_kind='FPGA 内部 RGMII / IMU 专用，不用于手机联网'
            acg_fpga_count=$((acg_fpga_count + 1)) ;;
        */fe1b0000.ethernet|*/fe1b0000.ethernet/*)
            acg_kind='外部 RJ45 / GMAC0，可连接路由器'
            acg_external_count=$((acg_external_count + 1)) ;;
    esac
    if [ -d "$acg_net/wireless" ] || [ -e "$acg_net/phy80211" ]; then
        acg_kind="无线网络接口（$acg_transport）"
        acg_wifi_count=$((acg_wifi_count + 1))
        acg_wifi_names="$acg_wifi_names $acg_iface"
    fi
    printf '\n  %s: %s\n' "$acg_iface" "$acg_kind"
    printf '    device: %s\n' "${acg_device:-无法读取或虚拟接口}"
    [ -z "$acg_driver" ] || printf '    driver: %s\n' "${acg_driver##*/}"
    if [ -r "$acg_net/operstate" ]; then printf '    状态: %s\n' "$(cat "$acg_net/operstate" 2>/dev/null)"; fi
    acg_carrier=$(cat "$acg_net/carrier" 2>/dev/null || true)
    [ -z "$acg_carrier" ] || printf '    carrier: %s\n' "$acg_carrier"
    acg_query_ok=0
    acg_addr=''
    if has ip; then
        if acg_raw=$(ip -4 addr show dev "$acg_iface" 2>/dev/null); then
            acg_query_ok=1; acg_addr=$(printf '%s\n' "$acg_raw" | sed -n '/inet /p')
        fi
    fi
    if [ "$acg_query_ok" -eq 0 ] && has ifconfig; then
        if acg_raw=$(ifconfig "$acg_iface" 2>/dev/null); then
            acg_query_ok=1; acg_addr=$(printf '%s\n' "$acg_raw" | sed -n '/inet /p')
        fi
    fi
    if [ "$acg_query_ok" -eq 0 ]; then acg_addr='查询不可用（检查工具、参数支持或权限）'; fi
    if [ -n "$acg_addr" ]; then printf '    IPv4: %s\n' "$acg_addr"
    else printf '    IPv4: 暂未发现地址\n'; fi
done

heading '3. 实际 USB 设备及各接口驱动（不依赖 lsusb，不读取序列号）'
acg_usb_count=0
for acg_usb in /sys/bus/usb/devices/*; do
    [ -r "$acg_usb/idVendor" ] && [ -r "$acg_usb/idProduct" ] || continue
    acg_usb_count=$((acg_usb_count + 1))
    acg_usb_vendor=$(usb_text "$acg_usb/idVendor")
    acg_usb_product=$(usb_text "$acg_usb/idProduct")
    acg_usb_binding=$(target_of "$acg_usb/driver")
    printf '\n  USB %s VID:PID=%s:%s device-driver=%s\n' "${acg_usb##*/}" "$acg_usb_vendor" "$acg_usb_product" "${acg_usb_binding##*/}"
    acg_usb_label=$(usb_text "$acg_usb/product")
    acg_usb_maker=$(usb_text "$acg_usb/manufacturer")
    [ -z "$acg_usb_label" ] || printf '    product（仅显示）: %s\n' "$acg_usb_label"
    [ -z "$acg_usb_maker" ] || printf '    manufacturer（仅显示）: %s\n' "$acg_usb_maker"
    acg_usb_interface_count=0
    for acg_usb_interface in "$acg_usb"/*:*; do
        [ -d "$acg_usb_interface" ] || continue
        acg_usb_interface_count=$((acg_usb_interface_count + 1))
        acg_usb_interface_binding=$(target_of "$acg_usb_interface/driver")
        acg_usb_interface_class=$(usb_text "$acg_usb_interface/bInterfaceClass")
        printf '    接口 %s class=%s driver=%s\n' "${acg_usb_interface##*/}" "$acg_usb_interface_class" "${acg_usb_interface_binding##*/}"
        [ -n "$acg_usb_interface_binding" ] || printf '%s\n' '      未绑定驱动，需按实际 VID:PID/芯片核对当前内核支持。'
        for acg_usb_net in "$acg_usb_interface"/net/*; do
            [ -e "$acg_usb_net" ] || continue
            printf '      创建的网络接口: %s\n' "${acg_usb_net##*/}"
        done
    done
    [ "$acg_usb_interface_count" -gt 0 ] || printf '%s\n' '    未发现可查询的 USB 接口节点。'
done
[ "$acg_usb_count" -gt 0 ] || printf '%s\n' '  未发现已枚举 USB 设备，先核对 USB 主机口、连接线、转接器和供电。'
printf '%s\n' '  USB 已枚举不等于 Wi-Fi 可用；以无线接口、其实际驱动、关联和 IPv4 为准。'
printf '%s\n' '  btusb 等蓝牙接口不等于已提供 Wi-Fi；USB 无线网卡应按实际芯片/VID:PID 核对。'

heading '4. RTL8821CU 驱动与内核配置（只查询，不加载模块）'
acg_rtl_loaded=0
for acg_module in 8821cu rtl8821cu rtw88_8821cu rtw88_usb rtw88_8821c; do
    if [ -d "/sys/module/$acg_module" ]; then
        printf '  已存在内核模块: %s（还需核对 USB 接口绑定）\n' "$acg_module"
        acg_rtl_loaded=$((acg_rtl_loaded + 1))
    fi
done
[ "$acg_rtl_loaded" -gt 0 ] || printf '%s\n' '  未看到上述驱动模块；网卡未插入、模块未加载、缺驱动均可能，不直接判定故障。'
if has find && [ -d /lib/modules ]; then
    printf '%s\n' '  /lib/modules 中相关模块文件（存在不表示与当前内核匹配）:'
    find /lib/modules -type f -name '*8821*.ko*' -print 2>/dev/null | head -n 20
fi
if has modinfo; then
    for acg_module in 8821cu rtl8821cu rtw88_8821cu; do
        acg_module_info=$(modinfo "$acg_module" 2>/dev/null | sed -n '/^filename:/p; /^vermagic:/p')
        [ -z "$acg_module_info" ] || printf '  %s:\n%s\n' "$acg_module" "$acg_module_info"
    done
fi
if [ -r /proc/config.gz ] && has zcat; then
    printf '%s\n' '  运行内核的相关配置（若开放 /proc/config.gz）:'
    zcat /proc/config.gz 2>/dev/null | sed -n \
        '/^CONFIG_RTW88=/p; /^# CONFIG_RTW88 is not set/p; /^CONFIG_RTW88_USB=/p; /^CONFIG_RTW88_8821CU=/p; /^CONFIG_RTL8821CU=/p; /^CONFIG_CFG80211=/p; /^CONFIG_MAC80211=/p; /^CONFIG_MODULES=/p; /^CONFIG_MODVERSIONS=/p'
else
    printf '%s\n' '  /proc/config.gz 不可读或无 zcat，不能据此判断配置；请保留 uname -r 输出。'
fi

heading '5. RTL8821CU 固件参考与加载错误'
if [ -f /lib/firmware/rtw88/rtw8821c_fw.bin ]; then
    printf '%s\n' '  找到 /lib/firmware/rtw88/rtw8821c_fw.bin（rtw88 驱动所用固件）。'
else
    printf '%s\n' '  未找到 rtw88/rtw8821c_fw.bin；厂商 8821cu 驱动可能采用不同方式，需按实际驱动判断。'
fi
printf '%s\n' '  rtl_bt 下的 8821c 文件是蓝牙固件，不能作为 Wi-Fi 驱动已可用的证明。'
if has dmesg; then
    dmesg 2>/dev/null | sed -n \
        '/8821.*Unknown symbol/p; /8821.*disagrees about version/p; /8821.*version magic/p; /8821.*Direct firmware load/p; /8821.*failed to load/p; /8821.*Failed to load/p; /8821.*probe.*fail/p' | tail -n 12
fi

heading '6. 网络管理/WPA/DHCP 进程（只显示名称和 PID）'
acg_manager_count=0
for acg_proc in /proc/[0-9]*/comm; do
    [ -r "$acg_proc" ] || continue
    acg_name=$(cat "$acg_proc" 2>/dev/null || true)
    case "$acg_name" in
        wpa_supplicant|hostapd|dhcpcd|udhcpc|connmand|NetworkManager|iwd)
            acg_pid=${acg_proc%/comm}; acg_pid=${acg_pid##*/}
            printf '  %-18s PID=%s\n' "$acg_name" "$acg_pid"
            acg_manager_count=$((acg_manager_count + 1)) ;;
    esac
done
[ "$acg_manager_count" -gt 0 ] || printf '%s\n' '  未发现上述进程；这不排除静态地址或其他网络管理方式。'
printf '%s\n' '  若已有管理进程，不要直接再启动 wpa_supplicant 或 DHCP 客户端。'
if has wpa_cli; then
    for acg_iface in $acg_wifi_names; do
        printf '  %s 的只读 WPA 状态:\n' "$acg_iface"
        acg_wpa=$(wpa_cli -i "$acg_iface" status 2>/dev/null | sed -n '/^wpa_state=/p; /^ip_address=/p; /^key_mgmt=/p')
        if [ -n "$acg_wpa" ]; then printf '%s\n' "$acg_wpa"
        else printf '%s\n' '    状态不可读，可能未运行 WPA 或由其他管理器接管。'; fi
    done
fi
if has iw; then
    for acg_iface in $acg_wifi_names; do
        printf '  %s 的只读 iw 模式/信道:\n' "$acg_iface"
        iw dev "$acg_iface" info 2>/dev/null | sed -n '/^[[:space:]]*type /p; /^[[:space:]]*channel [0-9]/p'
    done
fi
if has rfkill; then
    printf '%s\n' '  无线软/硬阻止状态（只读）:'
    rfkill list 2>/dev/null || true
fi

heading '7. 当前 IPv4 路由（仅查询）'
if has ip; then ip -4 route 2>/dev/null || true
elif has route; then route -n 2>/dev/null || true
else printf '%s\n' '  无查询工具。'; fi

heading '8. 接下来怎么连接手机'
if [ "$acg_wifi_count" -gt 0 ]; then
    printf '%s\n' '  已看到无线接口。优先让板子与手机接入同一 2.4GHz 路由器，确认板子有 IPv4 地址。'
    printf '%s\n' '  也可让板子连接手机 2.4GHz 热点，但热点/路由器客户端隔离可能阻断访问，必须实际连接测试。'
else
    printf '%s\n' '  暂未看到无线接口。你的网卡通过 USB 连接：先核对上面的 USB VID:PID、接口驱动及实际芯片，再核对对应驱动/固件。'
    printf '%s\n' '  若连 USB 枚举都没有，先核对 USB 主机口、连接线、转接器和供电；RTL8821CU 使用 USB，按其实际 USB ID 和驱动绑定排查。'
fi
if [ "$acg_external_count" -gt 0 ]; then
    printf '%s\n' '  可先用外部 RJ45 接路由器，手机接该路由器 Wi-Fi，验证 APK 和服务端，无需等待无线扩展。'
fi
if [ "$acg_fpga_count" -gt 0 ]; then
    printf '%s\n' '  fe1c0000.ethernet 是 FPGA/IMU 内部口：不要为手机联网而给它改地址、桥接或停用。'
fi
printf '%s\n' '  在 APK 填写可达的 RK3588 局域网 IPv4:8080 及配对码；服务端须另行由你启动。'
printf '%s\n' '  本脚本没有启动任何服务，没有修改网络。'
