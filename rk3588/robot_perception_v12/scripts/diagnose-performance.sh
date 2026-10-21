#!/bin/sh
# Read-only RK3588/Buildroot performance snapshot. No root or Python required.
# SYSFS_ROOT and PROCFS_ROOT permit tests against a temporary filesystem.
set -u
export LC_ALL=C

if [ "$#" -ne 0 ]; then
    case "$1" in
        -h|--help)
            printf '%s\n' 'Usage: sh diagnose-performance.sh' \
                'Read-only: NPU/CPU frequencies, thermal data, memory and RKNN paths.' \
                'Optional test roots: SYSFS_ROOT=/sys PROCFS_ROOT=/proc'
            exit 0 ;;
        *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
    esac
fi

sysroot=${SYSFS_ROOT:-/sys}
procroot=${PROCFS_ROOT:-/proc}
sysroot=${sysroot%/}
procroot=${procroot%/}

read_value() {
    [ -r "$1" ] || return 1
    tr '\n\t' '  ' < "$1" 2>/dev/null
}

show_value() {
    diag_value=$(read_value "$2") || return 1
    [ -n "$diag_value" ] || return 1
    printf '  %s=%s\n' "$1" "$diag_value"
}

printf '%s\n' 'RK3588 performance snapshot (read-only; missing entries are not errors)'
show_value kernel "$procroot/version" || :
show_value uptime_seconds "$procroot/uptime" || :

printf '%s\n' 'NPU devfreq (frequency unit: Hz)'
found_npu=0
for device in "$sysroot"/class/devfreq/*; do
    [ -d "$device" ] || continue
    diag_name=$(read_value "$device/name") || diag_name=${device##*/}
    case "$diag_name ${device##*/}" in
        *[Nn][Pp][Uu]*) ;;
        *) continue ;;
    esac
    found_npu=1
    printf '  device=%s name=%s\n' "${device##*/}" "$diag_name"
    show_value current_freq "$device/cur_freq" || :
    show_value available_frequencies "$device/available_frequencies" || :
    show_value governor "$device/governor" || :
    show_value min_freq "$device/min_freq" || :
    show_value max_freq "$device/max_freq" || :
done
[ "$found_npu" -ne 0 ] || printf '%s\n' '  unavailable (devfreq entry not exposed/readable)'

printf '%s\n' 'RKNPU driver diagnostics'
printf '%s\n' '  load_scope=whole NPU device (all processes); driver sampling interval, not this application alone'
show_value load "$sysroot/kernel/debug/rknpu/load" || \
    show_value load "$procroot/rknpu/load" || \
    printf '%s\n' '  load=unavailable (debugfs may be absent or restricted)'
show_value driver_version "$sysroot/kernel/debug/rknpu/version" || \
    show_value driver_version "$sysroot/kernel/debug/rknpu/driver_version" || \
    show_value driver_version "$procroot/rknpu/version" || :

printf '%s\n' 'Thermal zones (temperature unit: millidegrees Celsius)'
found_thermal=0
for zone in "$sysroot"/class/thermal/thermal_zone*; do
    [ -d "$zone" ] || continue
    found_thermal=1
    diag_type=$(read_value "$zone/type") || diag_type=unknown
    diag_temp=$(read_value "$zone/temp") || diag_temp=unavailable
    printf '  %s type=%s temp_mC=%s\n' "${zone##*/}" "$diag_type" "$diag_temp"
done
[ "$found_thermal" -ne 0 ] || printf '%s\n' '  unavailable'

printf '%s\n' 'CPU frequency policies (frequency unit: kHz)'
found_cpu=0
for policy in "$sysroot"/devices/system/cpu/cpufreq/policy*; do
    [ -d "$policy" ] || continue
    found_cpu=1
    printf '  %s\n' "${policy##*/}"
    show_value related_cpus "$policy/related_cpus" || :
    show_value governor "$policy/scaling_governor" || :
    show_value current_freq "$policy/scaling_cur_freq" || :
    show_value available_frequencies "$policy/scaling_available_frequencies" || :
    show_value min_freq "$policy/scaling_min_freq" || :
    show_value max_freq "$policy/scaling_max_freq" || :
done
if [ "$found_cpu" -eq 0 ]; then
    for cpu in "$sysroot"/devices/system/cpu/cpu[0-9]*/cpufreq; do
        [ -d "$cpu" ] || continue
        found_cpu=1
        diag_cpu=${cpu%/cpufreq}
        printf '  %s\n' "${diag_cpu##*/}"
        show_value governor "$cpu/scaling_governor" || :
        show_value current_freq "$cpu/scaling_cur_freq" || :
    done
fi
[ "$found_cpu" -ne 0 ] || printf '%s\n' '  unavailable'

printf '%s\n' 'Memory'
if [ -r "$procroot/meminfo" ]; then
    while IFS= read -r diag_line; do
        case "$diag_line" in
            MemTotal:*|MemFree:*|MemAvailable:*|Buffers:*|Cached:*|SwapTotal:*|SwapFree:*|CmaTotal:*|CmaFree:*)
                printf '  %s\n' "$diag_line" ;;
        esac
    done < "$procroot/meminfo"
else
    printf '%s\n' '  unavailable'
fi

printf '%s\n' 'RKNN runtime paths (mapped libraries, if process permissions permit)'
diag_libraries=$(
    for maps in "$procroot"/[0-9]*/maps; do
        [ -r "$maps" ] || continue
        # /proc/PID/maps has five fixed columns before the pathname.
        while read -r diag_addr diag_perms diag_offset diag_dev diag_inode diag_path; do
            case "$diag_path" in
                */librknnrt.so|*/librknnrt.so.*|*/librknnrt.so\ \(deleted\))
                    printf '%s\n' "$diag_path" ;;
            esac
        done 2>/dev/null < "$maps"
    done | sort -u
)
if [ -n "$diag_libraries" ]; then
    printf '%s\n' "$diag_libraries"
else
    printf '%s\n' '  no readable process mapping; the application may not be running'
fi
# Looking for a version-like byte sequence inside an ELF library is unreliable.
printf '%s\n' '  runtime_version=not inferred; use the application RKNN SDK/runtime startup log'
printf '%s\n' 'No settings were changed. Run while detection is active to capture operating frequencies/load.'
