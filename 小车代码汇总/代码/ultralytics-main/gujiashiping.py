import cv2
import numpy as np
from mss import mss
from datetime import datetime
from ultralytics import YOLO
# 配置
output_file = f"screen_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
fps = 30.0
# 获取屏幕尺寸
with mss() as sct:
    monitor = {
    "left": 650,   # 可根据需求调整，比如从左数 100 像素开始
    "top": 350,     # 从上数 50 像素开始
    "width": 750,  # 区域宽度
    "height": 500 # 区域高度
    }

width, height = monitor['width'], monitor['height']
# 加载模型（假设使用YOLOv8，因为官方没有YOLOv10）
model = YOLO("yolov8n-pose.pt")  # 请使用正确的模型路径

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_file, fourcc, fps, (width, height))

print(f"开始录制: {output_file}")
print("按 'q' 键停止录制")
try:
    with mss() as sct:
        while True:
            # 捕获屏幕
            frame= sct.grab(monitor)
            frame = np.array(frame)
            img = cv2.resize(frame, (640, 640))
            results = model.predict(source=cv2.cvtColor(img,cv2.COLOR_BGR2RGB), imgsz=640, conf=0.6, save=False)
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
            img = cv2.resize(img, (1150, 750))
            out.write(img)
            
            # 实时显示画面（关键修改：在这里显示当前帧，而不是读取已保存的文件）
            cv2.imshow('正在录制 (按q停止)', img)
            
            # 按 'q' 键停止录制
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
except KeyboardInterrupt:
    print("录制已停止")
finally:
    # 清理资源
    out.release()
    cv2.destroyAllWindows()
    print(f"视频已保存: {output_file}")