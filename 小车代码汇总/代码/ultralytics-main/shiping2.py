import cv2
import numpy as np
from mss import mss
from datetime import datetime
from ultralytics import YOLO
from yanse import ColorDetector
from tanchuangxiugaipro import ColorDetector_tanchuang
import tkinter as tk
from tkinter import Label
import threading
import pyautogui
import time 
def click_based_on_flag(flag, pos1=(500, 500), pos2=(800, 500)):
    """
    根据布尔值 flag 点击不同位置
    
    :param flag: 布尔值（1 或 0），1 点击 pos1，0 点击 pos2
    :param pos1: flag=1 时的点击坐标，默认 (500, 500)
    :param pos2: flag=0 时的点击坐标，默认 (800, 500)
    """
    # 确保输入是有效的布尔值（1 或 0）
    if flag not in (0, 1):
        raise ValueError("输入必须是 0 或 1")
    
    # 根据 flag 选择点击位置
    target_pos = pos1 if flag == 1 else pos2
    
    # 执行点击操作
    pyautogui.click(target_pos[0], target_pos[1])
    
    # 打印日志（可选）
    print(f"flag={flag}，已点击坐标：{target_pos}")
    
    # 短暂延迟防止连续点击冲突
    time.sleep(0.1)
# 配置
output_file = f"screen_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
fps = 30.0
# 获取屏幕尺寸
with mss() as sct:
    monitor = {
    "left": 1330,   # 可根据需求调整，比如从左数 100 像素开始
    "top": 690,     # 从上数 50 像素开始
    "width": 675,  # 区域宽度
    "height": 450 # 区域高度
    }

width, height = monitor['width'], monitor['height']
# 加载模型（假设使用YOLOv8，因为官方没有YOLOv10）
model = YOLO("yolov8n-pose.pt")  # 请使用正确的模型路径
model_1 = YOLO("yolov8n.pt")
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
            img_1=cv2.resize(frame, (640, 640))
            detector = ColorDetector()
            print("开始检测颜色物块...")
            print("按ESC或q键退出")
            pre="none"
            now=detector1.state
            mmk=0
            # 处理帧
            processed_frame = detector1.detect_colors(img)
            if(pre!=now):
                sta=1
            else:
                sta=0
            if(detector1.state== "detected" and sta==1):
                print(1)
                if(mmk==0):
                    mmk=mmk+1
                    detector1 = ColorDetector_tanchuang(area_threshold=25000)
                elif(mmk==1):
                    continue
                else:
                    continue
            if(detector1.state== "lost" and sta==1):
                print(0)
            pre=now
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
                        cv2.circle(processed_frame, (int(x), int(y)), 3, (0, 255, 0), -1)
                
                for (a, b) in skeleton:
                    if kpts[a, 2] > 0.1 and kpts[b, 2] > 0.1:  # 只连接可见点
                        x1, y1 = int(kpts[a, 0]), int(kpts[a, 1])
                        x2, y2 = int(kpts[b, 0]), int(kpts[b, 1])
                        cv2.line(processed_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            results_1 = model_1.predict(source=cv2.cvtColor(img_1,cv2.COLOR_BGR2RGB), imgsz=640, conf=0.6, save=False)
            for r in results_1:
                boxes = r.boxes  # 边界框信息
                for box in boxes:
                        # 获取边界框坐标
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    x1, y1, x2, y2 = xyxy
                    cls = int(box.cls)
                    conf = float(box.conf)
                    label = f"{model_1.names[cls]} {conf:.2f}"
                    cv2.rectangle(processed_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(processed_frame, label, (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            processed_frame = cv2.resize(processed_frame, (1265, 825))
            out.write(processed_frame)
            
            # 实时显示画面（关键修改：在这里显示当前帧，而不是读取已保存的文件）
            cv2.imshow(detector1.cv_window_name, processed_frame)
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