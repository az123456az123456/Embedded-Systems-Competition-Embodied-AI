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
        self.area_threshold = area_threshold  # 面积阈值，默认为1000像素
        self.previous_state = False  # 上一帧的检测状态
        self.current_state = False   # 当前帧的检测状态
        self.popup_start_time = 0    # 弹窗开始时间
        self.popup_duration = 1      # 弹窗持续时间（秒）
        self.popup_message = ""      # 弹窗显示的消息

    def detect_colors(self, frame):
        """检测颜色并返回处理后的帧"""
        if frame is None:
            raise Exception("输入的帧为空")
            
        # 保存上一帧的检测状态
        self.previous_state = self.current_state
        self.current_state = False  # 默认当前状态为未检测到
        
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
                
                if area > self.area_threshold:  # 使用面积阈值过滤
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        
                        self.detection_results[color_name] = {
                            'center': (cX, cY),
                            'area': area,  # 存储面积值
                            'contour': largest_contour
                        }
                        
                        # 更新当前状态为已检测到
                        self.current_state = True
        
        # 状态变化检测
        if self.current_state and not self.previous_state:
            # 物块出现
            self.popup_message = "已识别到颜色物块!"
            self.popup_start_time = time.time()
        elif not self.current_state and self.previous_state:
            # 物块消失
            self.popup_message = "颜色物块已消失!"
            self.popup_start_time = time.time()
        
        # 绘制检测结果
        frame = self._draw_detection_results(frame)
        
        # 显示弹窗（如果需要）
        if time.time() - self.popup_start_time < self.popup_duration:
            frame = self._draw_popup(frame, self.popup_message)
            
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

    def _draw_popup(self, frame, message):
        """绘制弹窗提示"""
        height, width = frame.shape[:2]
        
        # 创建半透明覆盖层
        overlay = frame.copy()
        
        # 根据消息类型设置不同颜色
        if "已识别到" in message:
            color = (0, 200, 0)  # 绿色 - 物块出现
        else:
            color = (0, 0, 200)  # 红色 - 物块消失
            
        cv2.rectangle(overlay, (width//4, height//4), (width*3//4, height*3//4), color, -1)
        
        # 将覆盖层与原图融合
        alpha = 0.6  # 透明度
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
        
        # 添加文字
        cv2.putText(frame, message, (width//2 - 120, height//2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)
        return frame

    def get_detection_results(self):
        """获取检测结果"""
        return self.detection_results

    def get_detection_state(self):
        """获取当前检测状态（True=检测到，False=未检测到）"""
        return self.current_state

    def get_state_change(self):
        """获取状态变化信息：
           1 = 物块出现
          -1 = 物块消失
           0 = 无变化
        """
        if self.current_state and not self.previous_state:
            return 1
        elif not self.current_state and self.previous_state:
            return -1
        else:
            return 0

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
        
        # 获取状态变化
        state_change = detector.get_state_change()
        
        if state_change == 1:
            print("检测到符合面积要求的颜色物块!")
        elif state_change == -1:
            print("颜色物块已消失!")
            
        # 显示结果
        cv2.imshow("Color Detection", processed_frame)
        
        # 按ESC或q退出
        key = cv2.waitKey(30)
        if key == 27 or key == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()