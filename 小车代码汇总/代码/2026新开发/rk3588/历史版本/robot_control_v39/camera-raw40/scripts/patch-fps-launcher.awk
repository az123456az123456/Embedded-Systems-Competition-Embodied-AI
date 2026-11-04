# Transform only recognized camera-FPS statements. Everything else is retained.
# kind=app or kind=isp. A recognized v20 patch is idempotent.
function config_block(   line) {
    print "# CAMERA_HIGHFPS_CONFIG_V20"
    print "if [ \"${CAMERA_TARGET_FPS+x}\" != x ]; then"
    if (kind == "app")
        print "    FPS_CONFIG=${CAMERA_FPS_CONFIG:-$(dirname -- \"$ISP_CAMERA_SCRIPT\")/camera-target-fps}"
    else
        print "    FPS_CONFIG=${CAMERA_FPS_CONFIG:-$BASE/camera-target-fps}"
    print "    if [ -r \"$FPS_CONFIG\" ]; then CAMERA_TARGET_FPS=$(cat \"$FPS_CONFIG\")"
    print "    else CAMERA_TARGET_FPS=40; fi"
    print "fi"
}
BEGIN {
    old_app="CAMERA_TARGET_FPS=${CAMERA_TARGET_FPS:-30}"
    old_isp="if [ \"${CAMERA_TARGET_FPS+x}\" = x ]; then"
    old_validation_app="case \"$CAMERA_TARGET_FPS\" in 15|20|30) ;; *) printf '%s\\n' 'CAMERA_TARGET_FPS must be 15, 20 or 30' >&2; exit 2 ;; esac"
    old_validation_isp=" case \"$CAMERA_TARGET_FPS\" in 15|20|30) ;; *) echo 'CAMERA_TARGET_FPS must be 15, 20 or 30' >&2; exit 2 ;; esac"
    new_validation_app="case \"$CAMERA_TARGET_FPS\" in 15|20|30|35|40) ;; *) printf '%s\\n' 'CAMERA_TARGET_FPS must be 15, 20, 30, 35 or 40' >&2; exit 2 ;; esac"
    new_validation_isp=" case \"$CAMERA_TARGET_FPS\" in 15|20|30|35|40) ;; *) echo 'CAMERA_TARGET_FPS must be 15, 20, 30, 35 or 40' >&2; exit 2 ;; esac"
}
$0 == "# CAMERA_HIGHFPS_CONFIG_V20" { markers++ }
kind == "app" && $0 == old_app { defaults++; config_block(); next }
kind == "isp" && $0 == old_isp {
    starts++
    if (!markers) config_block()
    print; next
}
kind == "app" && $0 == old_validation_app { validations++; print new_validation_app; next }
kind == "isp" && $0 == old_validation_isp { validations++; print new_validation_isp; next }
kind == "app" && $0 == new_validation_app { newvalid++ }
kind == "isp" && $0 == new_validation_isp { newvalid++ }
{ print }
END {
    good = (kind == "app" ? defaults == 1 : starts == 1)
    fresh = markers == 0 && good && validations == 1 && newvalid == 0
    patched = markers == 1 && validations == 0 && newvalid == 1 && defaults == 0
    if (!(fresh || patched)) {
        print "Unrecognized or duplicated FPS configuration; original launcher was not changed." > "/dev/stderr"
        exit 2
    }
}
