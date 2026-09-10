from ultralytics import YOLO
import cv2
# 加载预训练模型
model_1 = YOLO('yolov8n.pt')

# 图片路径（替换为你的图片）
img=cv2.imread('data/OIP-C.webp')
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
cv2.imshow('video',img)
cv2.waitKey(0)