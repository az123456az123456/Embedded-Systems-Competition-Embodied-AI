# Replace only recognized model/label default assignments. No code is executed.
{
 line=$0
 if (index(line,"MODEL_PATH=")) {
  n=sub(/\/model\/(yolo11s-int8-rk3588|yolov8s-int8-rk3588|industrial-yolov8s-int8-rk3588|acg138-best-e7f26253-int8-rk3588)\.rknn/,"/model/acg138-best-e7f26253-int8-rk3588.rknn",line)
  models+=n
 }
 if (index(line,"LABELS_PATH=") && index(line,"$APP_DIR/model/")) {
  n=sub(/\/model\/(coco_80_labels_list|industrial_labels_list|acg138-best-e7f26253-labels)\.txt/,"/model/acg138-best-e7f26253-labels.txt",line)
  labels+=n
 }
 print line
}
END {
 if (models!=1 || labels!=1) {
  print "Unrecognized or ambiguous model/label defaults; nothing installed." > "/dev/stderr"
  exit 2
 }
}
