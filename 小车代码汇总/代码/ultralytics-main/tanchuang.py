import cv2
import numpy as np
import time
import tkinter as tk
from tkinter import Label

class ColorDetector:
    def __init__(self, area_threshold=10000):
        self.color_ranges = {
            'red': [
                (np.array([0, 150, 120]), np.array([10, 255, 255])),
                (np.array([170, 150, 120]), np.array([180, 255, 255]))
            ],
            'green': [
                (np.array([40, 60, 50]), np.array([80, 255, 255]))
            ],
            'blue': [
                (np.array([100, 120, 80]), np.array([135, 255, 255]))
            ]
        }
        self.detection_results = {}
        self.area_threshold = area_threshold
        self.previous_largest_area = 0
        self.current_largest_area = 0
        self.state = "none"
        self.popup_displayed = False

    def detect_colors(self, frame):
        if frame is None:
            raise Exception("输入的帧为空")
            
        self.previous_largest_area = self.current_largest_area
        self.current_largest_area = 0
        self.detection_results = {}
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        for color_name, ranges in self.color_ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask += cv2.inRange(hsv, lower, upper)
            
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest_contour = max(contours, key=lambda x: cv2.contourArea(x))
                area = cv2.contourArea(largest_contour)
                
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    
                    self.detection_results[color_name] = {
                        'center': (cX, cY),
                        'area': area,
                        'contour': largest_contour
                    }
                    
                    if area > self.current_largest_area:
                        self.current_largest_area = area
        
        # 判断状态变化并触发弹窗
        if self.previous_largest_area < self.area_threshold <= self.current_largest_area and not self.popup_displayed:
            self.state = "detected"
            self.show_auto_close_popup(f"颜色物块出现! ")
            self.popup_displayed = True
        elif self.previous_largest_area >= self.area_threshold > self.current_largest_area and not self.popup_displayed:
            self.state = "lost"
            self.show_auto_close_popup("颜色物块消失!")
            self.popup_displayed = True
        elif self.current_largest_area == 0:
            self.state = "none"
            self.popup_displayed = False
            
        frame = self._draw_detection_results(frame)
        return frame

    def _draw_detection_results(self, frame):
        color_display = {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0)
        }
        
        for color_name, result in self.detection_results.items():
            cv2.drawContours(frame, [result['contour']], -1, color_display[color_name], 2)
            cv2.circle(frame, result['center'], 5, color_display[color_name], -1)
            label = f"{color_name}: ({result['center'][0]}, {result['center'][1]}), Area: {int(result['area'])}"
            cv2.putText(frame, label, (result['center'][0] - 70, result['center'][1] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_display[color_name], 2)
        return frame

    def show_auto_close_popup(self, message):
        # 使用线程避免阻塞OpenCV主循环
        import threading
        thread = threading.Thread(target=self._show_popup, args=(message,))
        thread.daemon = True
        thread.start()
        print('j')

    def _show_popup(self, message):
        root = tk.Tk()
        root.title("提示")
        root.geometry("300x100")
        root.overrideredirect(True)  # 去掉窗口边框
        
        # 计算屏幕中心位置
        screen_width = root.winfo_screenwidth()
        screen_height = root.winfo_screenheight()
        x = (screen_width - 300) // 2
        y = (screen_height - 100) // 2
        root.geometry(f"+{x}+{y}")
        
        # 设置窗口半透明效果
        root.attributes("-alpha", 0.9)
        
        # 创建标签显示消息
        label = Label(root, text=message, font=("SimHei", 14), bg="#333333", fg="white")
        label.pack(fill="both", expand=True)
        
        # 2秒后自动关闭
        root.after(2000, root.destroy)
        root.mainloop()

# 使用示例（调用摄像头）
if __name__ == "__main__":
    # 创建检测器实例，设置面积阈值为2000像素
    detector = ColorDetector(area_threshold=20000)
    
    # 打开摄像头
    cap = cv2.VideoCapture(0)
    
    # 检查摄像头是否成功打开
    if not cap.isOpened():
        print("无法打开摄像头")
        exit()
    
    print("开始检测颜色物块...")
    print("按ESC或q键退出")
    
    while True:
        # 读取一帧
        ret, frame = cap.read()
        
        # 检查是否成功读取帧
        if not ret:
            print("无法获取帧，退出")
            break
        
        # 处理帧
        processed_frame = detector.detect_colors(frame)
        
        # 显示处理后的帧
        cv2.imshow("颜色检测", processed_frame)
        
        # 按ESC或q键退出
        key = cv2.waitKey(30)
        if key == 27 or key == ord('q'):
            break
    
    # 释放摄像头并关闭所有窗口
    cap.release()
    cv2.destroyAllWindows()