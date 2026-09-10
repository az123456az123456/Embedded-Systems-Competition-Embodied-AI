import cv2
import time
import serial
import numpy as np
# 颜色识别类（假设已定义，保持原代码不变）
class PrimaryColorDetector:
    def __init__(self, camera_index=1):
        """初始化三原色识别器"""
        # 打开摄像头
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise ValueError(f"无法打开摄像头 {camera_index}")
        
        # 定义三原色的HSV范围（已优化）
        self.color_ranges = {
            'red': [
                # 红色在HSV中分为两个区域
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
        
        # 结果存储
        self.detection_results = {}
    
    def detect_colors(self, frame):
        """检测三原色物体"""
        # 转换为HSV颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 重置检测结果
        self.detection_results = {}
        
        for color_name, ranges in self.color_ranges.items():
            # 创建掩码
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            
            # 处理多区域颜色（如红色）
            for lower, upper in ranges:
                mask += cv2.inRange(hsv, lower, upper)
            
            # 应用形态学操作减少噪声
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # 查找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 找到最大轮廓并记录信息
            if contours:
                largest_contour = max(contours, key=lambda x: cv2.contourArea(x))
                area = cv2.contourArea(largest_contour)
                
                # 设置最小面积阈值，过滤小噪声
                if area > 500:  # 可根据实际情况调整
                    # 计算轮廓的矩
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        
                        # 存储检测结果
                        self.detection_results[color_name] = {
                            'center': (cX, cY),
                            'area': area,
                            'contour': largest_contour
                        }
        
        # 在原图上标记检测结果
        self._draw_detection_results(frame)
        
        return frame
    
    def _draw_detection_results(self, frame):
        """在图像上绘制检测结果"""
        # 定义每种颜色的显示颜色和标签
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
    
    def get_detection_results(self):
        """返回所有检测结果"""
        return self.detection_results
    
    def run(self):
        """运行颜色识别主循环"""
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("无法获取图像帧")
                break
            
            # 水平翻转图像（镜像效果，可选）
            frame = cv2.flip(frame, 1)
            
            # 检测颜色
            processed_frame = self.detect_colors(frame)
            
            # 显示检测状态
            status = "Detected: " + ", ".join(self.detection_results.keys()) if self.detection_results else "No targets detected"
            cv2.putText(processed_frame, status, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            # 显示结果
            cv2.imshow("Primary Color Detection", processed_frame)
            
            # 按 'q' 键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        # 释放资源
        self.cap.release()
        cv2.destroyAllWindows()

# 主控制函数
def grab_color_block(target_color='red', camera_index=1, serial_port='COM5', baudrate=115200):
    # 初始化
    print(f"准备抓取{target_color}木块...")
    detector = PrimaryColorDetector(camera_index)
    detector.run()
    ser = serial.Serial(serial_port, baudrate, timeout=1)
    
    # 舵机ID定义
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
    ERROR_TOLERANCE = 15   # 坐标误差容忍范围（像素）
    
    # 发送舵机指令函数（简化版）
    def send_cmd(servo_id, pwm, time_ms=1000):
        cmd = f"#{servo_id:03d}P{pwm:04d}T{time_ms:04d}!"
        ser.write(cmd.encode())
        print(f"发送指令: {cmd}")
        time.sleep(time_ms / 1000 + 0.5)  # 等待动作完成
    
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
        
        # 2. 调整000（底座）使木块X坐标居中
        print(f"调整X坐标居中（目标颜色：{target_color}）...")
        x_centered = False
        while not x_centered:
            ret, frame = detector.cap.read()
            if not ret:
                raise Exception("无法获取摄像头画面")
            detector.detect_colors(frame)
            results = detector.get_detection_results()
            
            if target_color not in results:
                print(f"未检测到{target_color}木块，重试...")
                time.sleep(1)
                continue
            
            current_x = results[target_color]['center'][0]
            print(f"当前X坐标: {current_x}，目标: {TARGET_X_CENTER}")
            
            if abs(current_x - TARGET_X_CENTER) <= ERROR_TOLERANCE:
                print("X坐标已居中")
                x_centered = True
                break
            
            # 调整000（底座）
            current_pwm = 1500  # 假设初始值
            if current_x < TARGET_X_CENTER:
                new_pwm = current_pwm + 20  # 右转
            else:
                new_pwm = current_pwm - 20  # 左转
            
            new_pwm = max(500, min(2500, new_pwm))
            send_cmd(SERVO_BASE, new_pwm, 500)
        
        # 3. 调整001（大臂）确保能够到木块
        print(f"调整大臂以够到{target_color}木块...")
        current_pwm = 1500  # 假设初始值
        '''for new_pwm in range(current_pwm, 1000, -100):  # 逐步降低大臂
            send_cmd(SERVO_ARM1, new_pwm, 1000)
            ret, frame = detector.cap.read()
            detector.detect_colors(frame)
            if target_color in detector.get_detection_results():
                print(f"大臂调整到位，PWM: {new_pwm}")
                break'''
        send_cmd(SERVO_ARM1, 1500, 1000)
        print("大臂调整完成")
        time.sleep(1)
        
        # 4. 调整003（腕部）使木块Y坐标到夹取位置
        print(f"调整Y坐标至夹取位置（目标颜色：{target_color}）...")
        y_positioned = False
        while not y_positioned:
            ret, frame = detector.cap.read()
            detector.detect_colors(frame)
            results = detector.get_detection_results()
            
            if target_color not in results:
                print(f"未检测到{target_color}木块，重试...")
                time.sleep(1)
                continue
            
            current_y = results[target_color]['center'][1]
            print(f"当前Y坐标: {current_y}，目标: {TARGET_Y_GRAB}")
            
            if abs(current_y - TARGET_Y_GRAB) <= ERROR_TOLERANCE:
                print("Y坐标已到位")
                y_positioned = True
                break
            
            # 调整003（腕部）
            current_pwm = 1500  # 假设初始值
            if current_y > TARGET_Y_GRAB:
                new_pwm = current_pwm - 20  # 降低腕部
            else:
                new_pwm = current_pwm + 20  # 抬高腕部
            
            new_pwm = max(500, min(2500, new_pwm))
            send_cmd(SERVO_WRIST, new_pwm, 500)
        
        # 5. 控制005（夹爪）闭合抓取木块
        print("执行抓取...")
        send_cmd(SERVO_GRIPPER, GRIPPER_CLOSE, 1000)
        time.sleep(1)
        print(f"{target_color}木块抓取完成！")
        
    except Exception as e:
        print(f"抓取失败：{e}")
    finally:
        # 释放资源
        ser.close()
        detector.cap.release()
        cv2.destroyAllWindows()

# 直接运行主函数
if __name__ == "__main__":
    grab_color_block(target_color='red')  # 可改为'greendetector = PrimaryColorDetector(camera_index=1)'或'blue'