import cv2
import numpy as np
import time
import tkinter as tk
from tkinter import Label
import threading
import pyautogui
class ColorDetector_tanchuang:
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
        self.popup_thread = None
        self.popup_lock = threading.Lock()
        self.cv_window_name = "颜色检测"
        self.previous_dominant_color = None  # 跟踪上一次检测到的主导颜色
        self.color_chinese = {
            'red': '红色',
            'green': '绿色',
            'blue': '蓝色'
        }

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
        
        # 状态变化检测逻辑
        state_changed = False
        if self.previous_largest_area < self.area_threshold <= self.current_largest_area:
            self.state = "detected"
            state_changed = True
        elif self.previous_largest_area >= self.area_threshold > self.current_largest_area:
            self.state = "lost"
            state_changed = True
        elif self.current_largest_area == 0:
            self.state = "none"
        
        # 只有状态变化时才触发弹窗
        if state_changed:
            self._trigger_popup()
            
        # 保存当前主导颜色，用于下次检测
        dominant_color = None
        max_area = 0
        for color_name, result in self.detection_results.items():
            if result['area'] > max_area:
                max_area = result['area']
                dominant_color = color_name
        self.previous_dominant_color = dominant_color
        
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

    def _trigger_popup(self):
        """安全地触发弹窗，避免线程冲突"""
        with self.popup_lock:
            # 如果有正在运行的弹窗线程，等待它结束
            if self.popup_thread and self.popup_thread.is_alive():
                return
                
            if self.state == "detected":
                if self.detection_results:
                    # 找到面积最大的颜色
                    dominant_color = max(self.detection_results, 
                                        key=lambda x: self.detection_results[x]['area'])
                    chinese_color = self.color_chinese.get(dominant_color, "颜色")
                    area = self.detection_results[dominant_color]['area']
                    message = f"{chinese_color}物体出现! 面积: {int(area)}"
                else:
                    message = "颜色物块出现!"
            elif self.state == "lost":
                if self.previous_dominant_color:
                    chinese_color = self.color_chinese.get(self.previous_dominant_color, "颜色")
                    message = f"{chinese_color}物体消失!"
                else:
                    message = "颜色物块消失!"
            else:
                return 
                
            self.popup_thread = threading.Thread(target=self._show_popup, args=(message,))
            self.popup_thread.daemon = True
            self.popup_thread.start()
            self.popup_displayed = True

    def _show_popup(self, message):
        """在线程中显示弹窗"""
        # 获取 OpenCV 显示窗口的位置和尺寸
        try:
            window_rect = cv2.getWindowImageRect(self.cv_window_name)
            if window_rect[2] > 0 and window_rect[3] > 0:
                # 窗口已创建且有尺寸
                win_x, win_y, win_w, win_h = window_rect
                x = win_x + (win_w - 300) // 2
                y = win_y + (win_h - 100) // 2
            else:
                # 窗口未正确获取，使用屏幕居中
                screen_width = tk.Tk().winfo_screenwidth()
                screen_height = tk.Tk().winfo_screenheight()
                x = (screen_width - 300) // 2
                y = (screen_height - 100) // 2
        except:
            # 异常处理，使用屏幕居中
            screen_width = tk.Tk().winfo_screenwidth()
            screen_height = tk.Tk().winfo_screenheight()
            x = (screen_width - 300) // 2
            y = (screen_height - 100) // 2

        root = tk.Tk()
        root.title("提示")
        root.geometry("300x100")
        root.overrideredirect(True)
        root.geometry(f"+{x}+{y}")
        
        root.configure(bg="#333333")
        root.attributes("-alpha", 0.9)
        
        label = Label(root, text=message, font=("SimHei", 14), bg="#333333", fg="white")
        label.pack(fill="both", expand=True)
        
        root.after(2000, root.destroy)
        root.mainloop()
        
        with self.popup_lock:
            self.popup_displayed = False
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

# 使用示例（调用摄像头）
if __name__ == "__main__":
    # 创建检测器实例，设置面积阈值
    detector1 = ColorDetector_tanchuang(area_threshold=25000)
    
    # 打开摄像头
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("无法打开摄像头")
        exit()
    
    print("开始检测颜色物块...")
    print("按ESC或q键退出")
    pre="none"
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("无法获取帧，退出")
            break
        now=detector1.state
        # 处理帧
        processed_frame = detector1.detect_colors(frame)
        if(pre!=now):
            sta=1
        else:
            sta=0
        if(detector1.state== "detected" and sta==1):
            print(1)
        if(detector1.state== "lost" and sta==1):
            print(0)
        pre=now
        # 显示处理后的帧
        cv2.imshow(detector1.cv_window_name, processed_frame)
        
        # 按ESC或q键退出
        key = cv2.waitKey(30)
        if key == 27 or key == ord('q'):
            break
    
    # 释放摄像头并关闭所有窗口
    cap.release()
    cv2.destroyAllWindows()