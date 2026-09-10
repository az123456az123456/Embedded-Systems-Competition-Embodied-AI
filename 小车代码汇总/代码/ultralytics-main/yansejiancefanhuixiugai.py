import cv2
import numpy as np
import time

class ColorDetector:
    def __init__(self, area_threshold=1000):
        """初始化颜色检测器，添加面积阈值参数"""
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
        self.area_threshold = area_threshold  # 面积阈值，用于判断
        self.previous_largest_area = 0        # 上一帧的最大面积
        self.current_largest_area = 0         # 当前帧的最大面积
        self.state = "none"                   # 当前状态：none, detected, lost
        self.popup_start_time = 0             # 弹窗开始时间
        self.popup_duration = 1               # 弹窗持续时间（秒）

    def detect_colors(self, frame):
        """检测颜色并返回处理后的帧"""
        if frame is None:
            raise Exception("输入的帧为空")
            
        # 保存上一帧的最大面积
        self.previous_largest_area = self.current_largest_area
        self.current_largest_area = 0
        self.detection_results = {}
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        for color_name, ranges in self.color_ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask += cv2.inRange(hsv, lower, upper)
            
            # 形态学处理降噪
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # 查找轮廓
            contours, _  = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
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
                    
                    # 更新当前帧的最大面积
                    if area > self.current_largest_area:
                        self.current_largest_area = area
        
        # 判断状态变化
        if self.previous_largest_area < self.area_threshold <= self.current_largest_area:
            # 从小于阈值变为大于阈值
            self.state = "detected"
            self.popup_start_time = time.time()
        elif self.previous_largest_area >= self.area_threshold > self.current_largest_area:
            # 从大于阈值变为小于阈值
            self.state = "lost"
            self.popup_start_time = time.time()
        elif self.current_largest_area == 0:
            # 没有检测到任何颜色物块
            self.state = "none"
        
        # 绘制检测结果
        frame = self._draw_detection_results(frame)
        
        # 显示弹窗（如果需要）
        if time.time() - self.popup_start_time < self.popup_duration:
            frame = self._draw_popup(frame)
            
        return frame

    def _draw_detection_results(self, frame):
        """在图像上绘制轮廓、坐标和面积"""
        color_display = {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0)
        }
        
        for color_name, result in self.detection_results.items():
            # 绘制轮廓
            cv2.drawContours(frame, [result['contour']], -1, color_display[color_name], 2)
            # 绘制中心点
            cv2.circle(frame, result['center'], 5, color_display[color_name], -1)
            # 显示坐标和面积
            label = f"{color_name}: ({result['center'][0]}, {result['center'][1]}), Area: {int(result['area'])}"
            cv2.putText(frame, label, (result['center'][0]-70, result['center'][1]-20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_display[color_name], 2)
        return frame

    def _draw_popup(self, frame):
        """绘制更清晰的弹窗提示"""
        height, width = frame.shape[:2]
        
        # 创建半透明覆盖层
        overlay = frame.copy()
        
        if self.state == "detected":
            message = f"颜色物块出现! 面积: {int(self.current_largest_area)}"
            color = (0, 200, 0)  # 绿色
            border_color = (0, 255, 0)
        elif self.state == "lost":
            message = "颜色物块消失!"
            color = (0, 0, 200)  # 红色
            border_color = (0, 0, 255)
        else:
            return frame
        
        # 弹窗尺寸
        popup_width = width * 2 // 3
        popup_height = height // 3
        popup_x = (width - popup_width) // 2
        popup_y = (height - popup_height) // 2
        
        # 创建模糊背景
        blur = cv2.GaussianBlur(frame, (99, 99), 30)
        frame[popup_y:popup_y+popup_height, popup_x:popup_x+popup_width] = blur[popup_y:popup_y+popup_height, popup_x:popup_x+popup_width]
        
        # 绘制带圆角的弹窗
        radius = 20
        cv2.rectangle(overlay, (popup_x+radius, popup_y), (popup_x+popup_width-radius, popup_y+popup_height), color, -1)
        cv2.rectangle(overlay, (popup_x, popup_y+radius), (popup_x+popup_width, popup_y+popup_height-radius), color, -1)
        cv2.circle(overlay, (popup_x+radius, popup_y+radius), radius, color, -1)
        cv2.circle(overlay, (popup_x+popup_width-radius, popup_y+radius), radius, color, -1)
        cv2.circle(overlay, (popup_x+radius, popup_y+popup_height-radius), radius, color, -1)
        cv2.circle(overlay, (popup_x+popup_width-radius, popup_y+popup_height-radius), radius, color, -1)
        
        # 添加边框
        cv2.rectangle(overlay, (popup_x+radius, popup_y), (popup_x+popup_width-radius, popup_y+popup_height), border_color, 2)
        cv2.rectangle(overlay, (popup_x, popup_y+radius), (popup_x+popup_width, popup_y+popup_height-radius), border_color, 2)
        cv2.circle(overlay, (popup_x+radius, popup_y+radius), radius, border_color, 2)
        cv2.circle(overlay, (popup_x+popup_width-radius, popup_y+radius), radius, border_color, 2)
        cv2.circle(overlay, (popup_x+radius, popup_y+popup_height-radius), radius, border_color, 2)
        cv2.circle(overlay, (popup_x+popup_width-radius, popup_y+popup_height-radius), radius, border_color, 2)
        
        # 将覆盖层与原图融合
        alpha = 0.8  # 提高透明度
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
        
        # 添加标题
        title = "颜色检测状态"
        cv2.putText(frame, title, (popup_x + 30, popup_y + 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3, cv2.LINE_AA)
        
        # 添加分隔线
        cv2.line(frame, (popup_x + 20, popup_y + 80), (popup_x + popup_width - 20, popup_y + 80),
                (255, 255, 255), 2)
        
        # 添加主要消息
        cv2.putText(frame, message, (popup_x + 30, popup_y + 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 4, cv2.LINE_AA)
        
        # 添加面积阈值信息
        threshold_info = f"阈值: {self.area_threshold} 像素"
        cv2.putText(frame, threshold_info, (popup_x + 30, popup_y + 220),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
        
        return frame

    def get_detection_results(self):
        """获取检测结果"""
        return self.detection_results

    def get_current_state(self):
        """获取当前状态：'none', 'detected', 'lost'"""
        return self.state

    def get_largest_area(self):
        """获取当前检测到的最大面积"""
        return self.current_largest_area

# 使用示例（实时视频流）
if __name__ == "__main__":
    # 创建检测器实例，设置面积阈值为2000像素
    detector = ColorDetector(area_threshold=2000)
    
    # 打开摄像头
    cap = cv2.VideoCapture(0)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # 处理帧
        processed_frame = detector.detect_colors(frame)
        
        # 获取当前状态
        state = detector.get_current_state()
        
        if state == "detected":
            print(f"检测到颜色物块! 面积: {detector.get_largest_area()}")
        elif state == "lost":
            print("颜色物块消失!")
            
        # 显示结果
        cv2.imshow("Color Detection", processed_frame)
        
        # 按ESC或q退出
        key = cv2.waitKey(30)
        if key == 27 or key == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()