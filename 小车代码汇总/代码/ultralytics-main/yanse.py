import cv2
import numpy as np

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
        frame = self._draw_detection_results(frame)
        return frame
    
    def _draw_detection_results(self, frame):
        """在图像上绘制轮廓和坐标"""
        color_display = {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0)
        }
        
        for color_name, result in self.detection_results.items():
            cv2.drawContours(frame, [result['contour']], -1, color_display[color_name], 2)
            cv2.circle(frame, result['center'], 5, color_display[color_name], -1)
            label = f"{color_name}: ({result['center'][0]}, {result['center'][1]})"
            cv2.putText(frame, label, (result['center'][0]-50, result['center'][1]-20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_display[color_name], 2)
        return frame
    
    def get_detection_results(self):
        return self.detection_results

# 使用示例
if __name__ == "__main__":
    # 假设你已经有了一个帧
    frame = cv2.imread("data/bule.jpg")
    
    # 创建检测器实例
    detector = ColorDetector()
    
    # 处理单帧
    processed_frame = detector.detect_colors(frame)
    
    # 显示结果
    cv2.imshow("Color Detection", processed_frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()