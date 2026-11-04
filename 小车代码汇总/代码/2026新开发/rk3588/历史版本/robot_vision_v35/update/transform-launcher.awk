# Preserve installed lock, Weston, controls and logging. Only recognized model
# defaults and camera preparation change. Reject ambiguous launchers.
{
 line=$0
 if(index(line,"CAMERA_TARGET_FPS=")) sub(/CAMERA_TARGET_FPS:-30/,"CAMERA_TARGET_FPS:-40",line)
 if(index(line,"case \"$CAMERA_TARGET_FPS\" in")) {
  sub(/15\|20\|30\)/,"15|20|30|40)",line)
  sub(/15, 20 or 30/,"15, 20, 30 or 40",line)
 }
 if (index(line,"MODEL_PATH=")) {
  n=sub(/\/model\/(yolo11s-int8-rk3588|yolov8s-int8-rk3588|industrial-yolov8s-int8-rk3588|acg138-best-e7f26253-int8-rk3588)\.rknn/,"/model/acg138-best-e7f26253-int8-rk3588.rknn",line)
  models+=n
 }
 if (index(line,"LABELS_PATH=") && index(line,"$APP_DIR/model/")) {
  n=sub(/\/model\/(coco_80_labels_list|industrial_labels_list|acg138-best-e7f26253-labels)\.txt/,"/model/acg138-best-e7f26253-labels.txt",line)
  labels+=n
 }
 if(line=="# V35 FPGA-RGB camera preparation v1") patched=1
 if(!patched && line=="if [ \"$NO_CAPTURE\" = 0 ]; then") {
  starts++
  print "# V35 FPGA-RGB camera preparation v1"
  print "V35_CAMERA_SOURCE=${PERCEPTION_CAMERA_SOURCE:-fpga-raw}"
  print "case \"$CLI_CAMERA\" in fpga-rgb:*) V35_CAMERA_SOURCE=fpga-rgb ;; esac"
  print "case \"$CLI_CAMERA\" in fpga-raw|fpga-raw:*) V35_CAMERA_SOURCE=fpga-raw ;; esac"
  print "case \"$V35_CAMERA_SOURCE\" in fpga-raw|v4l2|fpga-rgb) ;; *) echo 'Invalid PERCEPTION_CAMERA_SOURCE' >&2; exit 2 ;; esac"
  print "PERCEPTION_CAMERA_SOURCE=$V35_CAMERA_SOURCE"
  print "export PERCEPTION_CAMERA_SOURCE"
 }
 if(!patched && index(line,"sh \"$APP_DIR/scripts/check-board.sh\" --run")) {
  sub(/sh \"\$APP_DIR\/scripts\/check-board.sh\" --run/,"if [ \"$V35_CAMERA_SOURCE\" = v4l2 ]; then sh \"$APP_DIR/scripts/check-board.sh\" --run; fi",line)
 }
 if(!patched && index(line,"case \"$WAIT_ATTEMPTS\" in")) {
  preps++
  print "    if [ \"$V35_CAMERA_SOURCE\" = v4l2 ] || [ \"$V35_CAMERA_SOURCE\" = fpga-raw ]; then"
 }
 if(!patched && line=="    prepare_wayland") {
  print "    fi"
  print line
  print "    if [ \"$V35_CAMERA_SOURCE\" = v4l2 ] || [ \"$V35_CAMERA_SOURCE\" = fpga-raw ]; then"
  next
 }
 if(!patched && index(line,"printf 'Using camera: %s")) {
  ends++
  print "    else"
  print "        VIDEO=${CLI_CAMERA:-fpga-rgb:${PERCEPTION_RGB_INTERFACE:-eth0}}"
  print "    fi"
 }
 print line
}
END {
 if (models!=1 || labels!=1 || (!patched && (starts!=1 || preps!=1 || ends!=1))) {
  print "Unrecognized or ambiguous model/camera launcher; nothing installed." > "/dev/stderr"
  exit 2
 }
}
