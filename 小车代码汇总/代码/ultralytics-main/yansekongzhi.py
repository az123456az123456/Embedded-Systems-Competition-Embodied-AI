import cv2
import numpy as np
import pyautogui
import time # 导入你的颜色识别模块
class ColorDetector:
    def __init__(self):
        # 定义三原色的HSV范围
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
    
    def detect_colors(self, frame):
        """检测颜色并返回处理后的帧"""
        if frame is None:
            raise Exception("输入的帧为空")
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.detection_results = {}
        
        for color_name, ranges in self.color_ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask += cv2.inRange(hsv, lower, upper)
            
            # 形态学处理降噪
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # 查找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest_contour = max(contours, key=lambda x: cv2.contourArea(x))
                area = cv2.contourArea(largest_contour)
                
                if area > 500:  # 过滤小噪声
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        
                        self.detection_results[color_name] = {
                            'center': (cX, cY),
                            'area': area,
                            'contour': largest_contour
                        }
        
        # 绘制检测结果
    

    
    def get_detection_results(self):
        return self.detection_results

class ColorTriggerClick:
    def __init__(self, target_color='red', click_pos=(100, 100), detection_area=None):
        """
        初始化
        :param target_color: 要检测的目标颜色，默认 'red'
        :param click_pos: 要点击的屏幕坐标，默认 (100, 100)
        :param detection_area: 检测区域坐标 (x, y, width, height)，默认 None 表示全屏
        """
        self.detector = ColorDetector()
        self.target_color = target_color
        self.click_pos = click_pos
        self.detection_area = detection_area  # (x, y, width, height)
        self.click_executed = False

    def capture_screen(self):
        """捕获指定区域的屏幕画面，转换为 OpenCV 适用格式"""
        if self.detection_area:
            x, y, width, height = self.detection_area
            screenshot = pyautogui.screenshot(region=(x, y, width, height))  # 截取指定区域
        else:
            screenshot = pyautogui.screenshot()  # 截取全屏
        
        return cv2.cvtColor(np.array(screenshot), cv2.COLOR_RGB2BGR)

    def detect_and_click(self):
        """
        持续检测屏幕，识别到目标颜色后执行一次点击
        识别到颜色、完成点击后自动停止
        """
        while not self.click_executed:
            frame = self.capture_screen()  # 获取屏幕帧
            self.detector.detect_colors(frame)  # 执行颜色识别
            results = self.detector.get_detection_results()  # 获取识别结果

            # 检查是否识别到目标颜色
            if self.target_color in results:
                # 执行静默点击（无鼠标移动可视化，直接触发点击）
                pyautogui.click(self.click_pos[0], self.click_pos[1])
                print(f"识别到 {self.target_color}，已在 ({self.click_pos[0]}, {self.click_pos[1]}) 执行点击")
                self.click_executed = True  # 标记为已点击，退出循环
            time.sleep(0.1)  # 控制检测频率，避免高占用

if __name__ == "__main__":
    # 仅检测屏幕区域 (200, 200, 400, 300) 内的红色，识别到后点击 (500, 300)
    click_handler = ColorTriggerClick(
        target_color='red',
        click_pos=(500, 300),
        detection_area=(200, 200, 400, 300)  # (x, y, width, height)
    )
    click_handler.detect_and_click()