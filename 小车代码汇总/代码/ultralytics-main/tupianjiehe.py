import cv2
import numpy as np
from ultralytics import YOLO

model = YOLO('yolov8n-pose.pt')  # 使用更大的模型
img = cv2.imread('data/OIP-C.webp')
results = model(img)

# COCO 关键点连接规则（17点）
skeleton = [
    [0, 1], [0, 2], [1, 3], [2, 4],  # 脸
    [5, 6], [5, 7], [7, 9], [6, 8], [8, 10],  # 身体
    [5, 11], [6, 12], [11, 12],  # 躯干
    [11, 13], [13, 15], [12, 14], [14, 16]  # 腿
]

# 绘制关键点和连线
for kpts in results[0].keypoints.data:
    for i, (x, y, conf) in enumerate(kpts):
        if conf > 0.1:  # 即使置信度低也绘制
            cv2.circle(img, (int(x), int(y)), 3, (0, 255, 0), -1)
    
    for (a, b) in skeleton:
        if kpts[a, 2] > 0.1 and kpts[b, 2] > 0.1:  # 只连接可见点
            x1, y1 = int(kpts[a, 0]), int(kpts[a, 1])
            x2, y2 = int(kpts[b, 0]), int(kpts[b, 1])
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
model_1 = YOLO('yolov8n.pt')

# 图片路径（替换为你的图片）
results_1 = model_1.predict(source='data/OIP-C.webp', imgsz=640, conf=0.6, save=False)
for r in results_1:
    boxes = r.boxes  # 边界框信息
    for box in boxes:
            # 获取边界框坐标
        xyxy = box.xyxy[0].cpu().numpy().astype(int)
        x1, y1, x2, y2 = xyxy
        cls = int(box.cls)
        conf = float(box.conf)
        label = f"{model_1.names[cls]} {conf:.2f}"
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(img, label, (x1, y1 - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
cv2.imshow('pi',img)
cv2.waitKey(0)