import cv2
import time
import serial
import numpy as np
class PrimaryColorDetector:
    def __init__(self, camera_index=0):
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise ValueError(f"无法打开摄像头 {camera_index}")
        
        self.color_ranges = {
            'red': [
                (np.array([0, 120, 70]), np.array([10, 255, 255])),
                (np.array([170, 120, 70]), np.array([180, 255, 255]))
            ],
            'green': [
                (np.array([40, 100, 100]), np.array([80, 255, 255]))
            ],
            'blue': [
                (np.array([100, 150, 0]), np.array([140, 255, 255]))
            ]
        }
        
        self.detection_results = {}
    
    def detect_colors(self, frame=None, display=False):
        """检测颜色并可选择显示结果"""
        if frame is None:
            ret, frame = self.cap.read()
            if not ret:
                raise Exception("无法获取图像帧")
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.detection_results = {}
        
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
        
        # 在图像上绘制检测结果
        frame = self._draw_detection_results(frame)
        
        # 如果需要显示，创建窗口并显示
        if display:
            cv2.imshow("Color Detection", frame)
            cv2.waitKey(1)  # 必须调用waitKey，否则窗口不会更新
        
        return frame
    
    def _draw_detection_results(self, frame):
        """在图像上绘制检测结果"""
        color_display = {
            'red': (0, 0, 255),    # 红色用BGR中的红色表示
            'green': (0, 255, 0),  # 绿色用BGR中的绿色表示
            'blue': (255, 0, 0)    # 蓝色用BGR中的蓝色表示
        }
        
        for color_name, result in self.detection_results.items():
            center = result['center']
            area = result['area']
            
            # 绘制轮廓和中心点
            cv2.drawContours(frame, [result['contour']], -1, color_display[color_name], 2)
            cv2.circle(frame, center, 5, color_display[color_name], -1)
            
            # 添加标签
            label = f"{color_name}: ({center[0]}, {center[1]})"
            cv2.putText(frame, label, (center[0]-30, center[1]-20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_display[color_name], 2)
        
        return frame
    
    def get_detection_results(self):
        """返回所有检测结果"""
        return self.detection_results
    
    def run(self):
        """运行颜色识别主循环"""
        cv2.namedWindow("Color Detection")
        
        while True:
            frame = self.detect_colors(display=True)
            
            # 显示检测状态
            status = "Detected: " + ", ".join(self.detection_results.keys()) if self.detection_results else "No targets detected"
            cv2.putText(frame, status, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            cv2.imshow("Color Detection", frame)
            
            # 按 'q' 键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.cap.release()
        cv2.destroyAllWindows()
def grab_color_block(target_color='red', camera_index=0, serial_port='COM5', baudrate=115200):
    print(f"准备抓取{target_color}木块...")
    detector = PrimaryColorDetector(camera_index)
    ser = serial.Serial(serial_port, baudrate, timeout=1)
    
    # 创建显示窗口
    cv2.namedWindow("Color Detection")    # 舵机ID定义
    frame = detector.detect_colors()
    cv2.imshow("Color Detection", frame)
    cv2.waitKey(1)
    SERVO_BASE = 0       # 000：底座旋转（控制X坐标居中）
    SERVO_ARM1 = 1       # 001：大臂（控制够到木块）
    SERVO_ARM2 = 2       # 002：小臂（固定角度，确保摄像头视野）
    SERVO_WRIST = 3      # 003：腕部（控制Y坐标到夹取位置）
    SERVO_ROTATE = 4     # 004：旋转关节（固定不动）
    SERVO_GRIPPER = 5    # 005：夹爪（开合控制）
        # 固定参数（根据实际调试填写）
    FIXED_ARM2 = 2100    # 002固定PWM值（确保摄像头视野）
    FIXED_ROTATE = 1500  # 004固定PWM值
    GRIPPER_OPEN = 500  # 夹爪张开PWM
    GRIPPER_CLOSE = 1700 # 夹爪闭合PWM
    TARGET_X_CENTER = 325  # 目标X坐标中心（摄像头宽度的一半）
    TARGET_Y_GRAB = 425    # 目标Y坐标（可夹取位置）
    ERROR_TOLERANCE = 30   # 坐标误差容忍范围（像素）
    def send_cmd(servo_id, pwm, time_ms=1000):
        cmd = f"#{servo_id:03d}P{pwm:04d}T{time_ms:04d}!"
        ser.write(cmd.encode())
        print(f"发送指令: {cmd}")  
        time.sleep(time_ms/1000+0.2)# 等待动作完成
    try:
        # 1. 基本启动（按顺序移动舵机）
        print("初始化机械臂...")
        send_cmd(SERVO_BASE, 1500, 1000)    # 000：底座初始位置
        send_cmd(SERVO_ARM1, 1500, 1000)    # 001：大臂初始位置
        send_cmd(SERVO_ROTATE, FIXED_ROTATE, 1000)  # 004：固定旋转关节
        send_cmd(SERVO_ARM2, FIXED_ARM2, 1000)      # 002：固定小臂位置
        send_cmd(SERVO_WRIST, 1000, 1000)   # 003：腕部初始位置
        send_cmd(SERVO_GRIPPER, GRIPPER_OPEN, 1000)  # 005：夹爪张开
        print("初始化完成：摄像头已就绪，夹爪张开")
        time.sleep(2)
        print(f"调整X坐标居中（目标颜色：{target_color}）...")
        x_centered = False
        current_pwm = 1500  # 假设初始值
        while not x_centered:
            frame = detector.detect_colors()  # 获取处理后的帧
            cv2.imshow("Color Detection", frame)  # 显示帧
            cv2.waitKey(1)  # 更新窗口
            
            results = detector.get_detection_results()
            if target_color not in results:
                print(f"未检测到{target_color}木块，重试...")
                time.sleep(0.2)
                continue
            
            current_x = results[target_color]['center'][0]
            print(f"当前X坐标: {current_x}，目标: {TARGET_X_CENTER}")
            
            if abs(current_x - TARGET_X_CENTER) <= ERROR_TOLERANCE:
                print("X坐标已居中")
                x_centered = True
                break
            
            # 调整000（底座）
            if current_x < TARGET_X_CENTER:
                current_pwm = current_pwm + 50  # 右转
            else:
                current_pwm = current_pwm - 50  # 左转
            
            new_pwm = max(500, min(2500, current_pwm))
            send_cmd(SERVO_BASE, new_pwm, 500)
        
        # 3. 调整001（大臂）确保能够到木块
        print(f"调整大臂以够到{target_color}木块...")
        current_pwm = 1500
        
        '''for new_pwm in range(current_pwm, 1000, -100):
            send_cmd(SERVO_ARM1, new_pwm, 1000)'''
        frame = detector.detect_colors()  # 获取处理后的帧
        cv2.imshow("Color Detection", frame)  # 显示帧
        cv2.waitKey(1)  # 更新窗口
        send_cmd(SERVO_ARM1, 1300, 1000)   
            # 后续逻辑保持不变...
        
        # 4. 调整003（腕部）使木块Y坐标到夹取位置
        print(f"调整Y坐标至夹取位置（目标颜色：{target_color}）...")
        y_positioned = False
        current_pwm = 1100  # 假设初始值
        while not y_positioned:
            frame = detector.detect_colors()  # 获取处理后的帧
            cv2.imshow("Color Detection", frame)  # 显示帧
            cv2.waitKey(1)  # 更新窗口
            results = detector.get_detection_results()
            
            if target_color not in results:
                print(f"未检测到{target_color}木块，重试...")
                time.sleep(0.2)
                continue
            
            current_y = results[target_color]['center'][1]
            print(f"当前Y坐标: {current_y}，目标: {TARGET_Y_GRAB}")
            
            if abs(current_y - TARGET_Y_GRAB) <= ERROR_TOLERANCE:
                print("Y坐标已到位")
                y_positioned = True
                break
            
            # 调整003（腕部）
            if current_y > TARGET_Y_GRAB:
                current_pwm = current_pwm - 20  # 降低腕部
            else:
                current_pwm = current_pwm + 20  # 抬高腕部
            
            new_pwm = max(500, min(2500, current_pwm))
            send_cmd(SERVO_WRIST, new_pwm, 500)
            # 后续逻辑保持不变...
        
        # 5. 控制005（夹爪）闭合抓取木块...（保持不变）
        print("执行抓取...")
        send_cmd(SERVO_GRIPPER, GRIPPER_CLOSE, 1000)
        time.sleep(0.2)
        print(f"{target_color}木块抓取完成！")       
    except Exception as e:
        print(f"抓取失败：{e}")
    finally:
        # 释放资源
        ser.close()
        detector.cap.release()
        cv2.destroyAllWindows()
if __name__ == "__main__":
    grab_color_block(target_color='red')  # 可改为'green'或'blue'