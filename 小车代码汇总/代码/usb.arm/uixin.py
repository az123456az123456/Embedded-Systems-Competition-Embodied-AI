import sys
import cv2
import time
import serial
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QGroupBox, 
                             QVBoxLayout, QHBoxLayout, QLabel, QSlider, 
                             QGridLayout, QFrame, QPushButton, QMessageBox,
                             QComboBox)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QIcon, QImage, QPixmap

# --------------------------
# 颜色识别类（机械臂视觉部分）
# --------------------------
class PrimaryColorDetector:
    def __init__(self, camera_index=0):
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise ValueError(f"无法打开摄像头 {camera_index}")
        
        # 定义三原色的HSV范围
        self.color_ranges = {
            'red': [
                # 低范围（0°附近）：收紧橙色边界，提高饱和度
                (np.array([0, 150, 120]), np.array([10, 255, 255])),
                # 高范围（180°附近）：保持不变
                (np.array([170, 150, 120]), np.array([180, 255, 255]))
            ],
            'green': [
                (np.array([40, 60, 50]), np.array([80, 255, 255]))  # 绿色范围（降低亮度和饱和度要求）
            ],
            'blue': [
                (np.array([100, 120, 80]), np.array([135, 255, 255]))  # 蓝色范围（收紧H范围）
            ]
        }
        
        self.detection_results = {}
    
    def detect_colors(self, frame=None):
        """检测颜色并返回处理后的帧"""
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
        
        # 绘制检测结果（框和坐标）
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
            # 绘制轮廓
            cv2.drawContours(frame, [result['contour']], -1, color_display[color_name], 2)
            # 绘制中心点
            cv2.circle(frame, result['center'], 5, color_display[color_name], -1)
            # 显示坐标
            label = f"{color_name}: ({result['center'][0]}, {result['center'][1]})"
            cv2.putText(frame, label, (result['center'][0]-50, result['center'][1]-20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_display[color_name], 2)
        return frame
    
    def get_detection_results(self):
        return self.detection_results
    
    def release(self):
        self.cap.release()


# --------------------------
# 自动夹取线程（避免UI卡顿）
# --------------------------
class GrabThread(QThread):
    # 信号用于更新UI状态
    status_signal = pyqtSignal(str)
    error_signal = pyqtSignal(str)
    
    def __init__(self, serial_port, target_color, detector):
        super().__init__()
        self.serial_port = serial_port
        self.target_color = target_color
        self.detector = detector
        self.running = True
    
    def run(self):
        try:
            self.status_signal.emit(f"开始抓取{self.target_color}木块...")
            
            # 舵机ID定义
            SERVO_BASE = 0       # 底座旋转
            SERVO_ARM1 = 1       # 大臂
            SERVO_ARM2 = 2       # 小臂（固定）
            SERVO_WRIST = 3      # 腕部
            SERVO_ROTATE = 4     # 旋转关节（固定）
            SERVO_GRIPPER = 5    # 夹爪
            
            # 固定参数
            FIXED_ARM2 = 2100    # 小臂固定值
            FIXED_ROTATE = 1500  # 旋转关节固定值
            GRIPPER_OPEN = 500   # 夹爪张开
            GRIPPER_CLOSE = 1700 # 夹爪闭合
            TARGET_X_CENTER = 325  # X坐标目标
            TARGET_Y_GRAB = 425    # Y坐标目标
            ERROR_TOLERANCE = 15   # 误差容忍范围
            
            # 发送指令函数
            def send_cmd(servo_id, pwm, time_ms=1000):
                cmd = f"#{servo_id:03d}P{pwm:04d}T{time_ms:04d}!"
                self.serial_port.write(cmd.encode())
                self.status_signal.emit(f"发送指令: {cmd}")
                time.sleep(time_ms / 1000 + 0.2)
            
            # 1. 初始化机械臂
            self.status_signal.emit("初始化机械臂...")
            send_cmd(SERVO_BASE, 1500, 500)
            time.sleep(0.1)
            send_cmd(SERVO_ARM1, 1500, 500)
            time.sleep(0.1)
            send_cmd(SERVO_ROTATE, FIXED_ROTATE, 500)
            time.sleep(0.1)
            send_cmd(SERVO_ARM2, FIXED_ARM2, 500)
            time.sleep(0.1)
            send_cmd(SERVO_WRIST, 1000, 500)
            time.sleep(0.1)
            send_cmd(SERVO_GRIPPER, GRIPPER_OPEN, 500)
            self.status_signal.emit("初始化完成")
            time.sleep(0.5)
            
            # 2. 调整底座X坐标居中
            self.status_signal.emit(f"调整X坐标居中（目标：{self.target_color}）")
            x_centered = False
            current_pwm = 1500
            while not x_centered and self.running:
                frame = self.detector.detect_colors()
                results = self.detector.get_detection_results()
                
                if self.target_color not in results:
                    self.status_signal.emit(f"未检测到{self.target_color}，重试...")
                    time.sleep(0.2)
                    continue
                
                current_x = results[self.target_color]['center'][0]
                self.status_signal.emit(f"当前X: {current_x}，目标: {TARGET_X_CENTER}")
                
                if abs(current_x - TARGET_X_CENTER) <= ERROR_TOLERANCE:
                    self.status_signal.emit("X坐标已居中")
                    x_centered = True
                    break
                
                # 调整底座
                current_pwm = current_pwm + 20 if current_x < TARGET_X_CENTER else current_pwm - 20
                new_pwm = max(500, min(2500, current_pwm))
                send_cmd(SERVO_BASE, new_pwm, 500)
            
            # 3. 调整大臂
            self.status_signal.emit("调整大臂位置...")
            send_cmd(SERVO_ARM1, 1300, 1000)
            
            # 4. 调整腕部Y坐标
            self.status_signal.emit(f"调整Y坐标至夹取位置（目标：{self.target_color}）")
            y_positioned = False
            current_pwm = 1100
            while not y_positioned and self.running:
                frame = self.detector.detect_colors()
                results = self.detector.get_detection_results()
                
                if self.target_color not in results:
                    self.status_signal.emit(f"未检测到{self.target_color}，重试...")
                    time.sleep(0.2)
                    continue
                
                current_y = results[self.target_color]['center'][1]
                self.status_signal.emit(f"当前Y: {current_y}，目标: {TARGET_Y_GRAB}")
                
                if abs(current_y - TARGET_Y_GRAB) <= ERROR_TOLERANCE:
                    self.status_signal.emit("Y坐标已到位")
                    y_positioned = True
                    break
                
                # 调整腕部
                current_pwm = current_pwm - 20 if current_y > TARGET_Y_GRAB else current_pwm + 20
                new_pwm = max(500, min(2500, current_pwm))
                send_cmd(SERVO_WRIST, new_pwm, 500)
            
            # 5. 闭合夹爪抓取
            self.status_signal.emit("执行抓取...")
            send_cmd(SERVO_GRIPPER, GRIPPER_CLOSE, 1000)
            time.sleep(1)
            self.status_signal.emit(f"{self.target_color}木块抓取完成！")
        #放下物块 
            if (self.target_color=='red') :
                send_cmd(SERVO_ARM1, 1500, 1000)
                time.sleep(0.1)
                send_cmd(SERVO_BASE, 2200, 1000)
                time.sleep(0.1)   
                send_cmd(SERVO_ARM2, 2140, 1000)
                time.sleep(0.1)
                send_cmd(SERVO_WRIST, 970,1000)
                time.sleep(0.1)
                send_cmd(SERVO_ARM1, 1340, 1000)
                time.sleep(0.1) 
                send_cmd(SERVO_GRIPPER, 500,1000)
                time.sleep(0.5)
            if(self.target_color=='blue'):
                send_cmd(SERVO_ARM1, 1500, 1000)
                time.sleep(0.1)
                send_cmd(SERVO_BASE, 900, 1000)
                time.sleep(0.1)   
                send_cmd(SERVO_ARM2, 2140, 1000)
                time.sleep(0.1)
                send_cmd(SERVO_WRIST, 970,1000)
                time.sleep(0.1)
                send_cmd(SERVO_ARM1, 1340, 1000)
                time.sleep(0.1) 
                send_cmd(SERVO_GRIPPER, 500,1000)
                time.sleep(0.5)
        #复位
            send_cmd(SERVO_ARM1, 1500, 500)
            time.sleep(0.1)
            send_cmd(SERVO_ROTATE, 1500, 500)
            time.sleep(0.1)
            send_cmd(SERVO_ARM2, 1500, 500)
            time.sleep(0.1)
            send_cmd(SERVO_WRIST, 1500, 500)
            time.sleep(0.1)
            send_cmd(SERVO_BASE, 1500, 800)
            time.sleep(0.1) 
            send_cmd(SERVO_GRIPPER, 1500, 500)
            time.sleep(0.5)

        except Exception as e:
            self.error_signal.emit(f"抓取失败：{str(e)}")
        finally:
            self.running = False


# --------------------------
# 主UI界面类
# --------------------------
class ServoControlApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial_port = None  # 串口对象（共享）
        self.grab_thread = None  # 自动抓取线程
        self.detector = PrimaryColorDetector()  # 颜色识别器
        self.init_ui()
    
    def init_ui(self):
        self.setWindowTitle("机械臂控制系统")
        self.setGeometry(100, 100, 1000, 600)
        self.setup_style()
        self.setup_layout()
        self.setup_connections()
        self.setup_camera_timer()
    
    def setup_style(self):
        """设置UI样式表"""
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1e1e2d;
                color: #e0e0ff;
                font-family: 'Segoe UI', sans-serif;
            }
            QGroupBox {
                background-color: #252536;
                border: 2px solid #3a3a4a;
                border-radius: 10px;
                padding: 15px;
                margin: 10px;
                font-size: 14px;
                font-weight: bold;
            }
            QGroupBox::title {
                color: #4cc9f0;
                left: 10px;
                padding: 0 5px;
            }
            QSlider::groove:horizontal {
                border: 1px solid #4a4a5a;
                height: 10px;
                background: #2d2d3d;
                border-radius: 5px;
            }
            QSlider::handle:horizontal {
                background: #4cc9f0;
                width: 20px;
                margin: -6px 0;
                border-radius: 10px;
            }
            QPushButton {
                background-color: #4361ee;
                color: white;
                border: none;
                border-radius: 5px;
                padding: 8px 16px;
            }
            QPushButton:hover {
                background-color: #4895ef;
            }
            QPushButton:pressed {
                background-color: #3f37c9;
            }
            QPushButton:disabled {
                background-color: #6c757d;
            }
            QComboBox {
                background-color: #2d2d3d;
                color: white;
                border-radius: 5px;
                padding: 5px;
            }
        """)
    
    def setup_layout(self):
        """构建UI布局"""
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        
        # 标题
        title = QLabel("机械臂控制系统")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Arial", 24, QFont.Bold))
        title.setStyleSheet("color: #4cc9f0; margin: 10px;")
        main_layout.addWidget(title)
        
        # 上半部分：摄像头画面 + 舵机控制
        top_layout = QHBoxLayout()
        
        # 摄像头显示区域
        self.camera_label = QLabel("摄像头画面")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setStyleSheet("background-color: #2d2d3d; border-radius: 8px;")
        self.camera_label.setMinimumSize(400, 300)
        top_layout.addWidget(self.camera_label, 1)
        
        # 舵机控制网格
        servo_grid = QGridLayout()
        servos = [
            ("S00", 1500, "#4cc9f0"),  # 底座
            ("S01", 1500, "#f72585"),   # 大臂
            ("S02", 1500, "#4caf50"),  # 小臂
            ("S03", 1500, "#ff9800"),   # 腕部
            ("S04", 1500, "#9c27b0"),  # 旋转关节
            ("S05", 1500, "#00bcd4")   # 夹爪
        ]
        
        self.servo_sliders = {}  # 存储舵机滑块
        self.servo_labels = {}   # 存储值显示标签
        
        for i, (name, init_val, color) in enumerate(servos):
            group = self.create_servo_group(name, init_val, color)
            servo_grid.addWidget(group, i//2, i%2)
        
        top_layout.addLayout(servo_grid, 1)
        main_layout.addLayout(top_layout)
        
        # 下半部分：控制按钮
        btn_layout = QHBoxLayout()
        
        # 颜色选择下拉框
        self.color_combo = QComboBox()
        self.color_combo.addItems(["red", "green", "blue"])
        btn_layout.addWidget(QLabel("目标颜色："))
        btn_layout.addWidget(self.color_combo)
        
        # 自动抓取按钮
        self.auto_grab_btn = QPushButton("启动自动抓取")
        btn_layout.addWidget(self.auto_grab_btn)
        
        # 手动控制按钮
        self.reset_btn = QPushButton("重置舵机")
        self.apply_btn = QPushButton("应用设置")
        btn_layout.addWidget(self.reset_btn)
        btn_layout.addWidget(self.apply_btn)
        
        # 串口连接按钮
        self.serial_connect_btn = QPushButton("连接串口")
        btn_layout.addWidget(self.serial_connect_btn)
        
        main_layout.addLayout(btn_layout)
        
        # 状态栏
        self.status_bar = self.statusBar()
        self.status_bar.showMessage("就绪 | 请连接串口")
    
    def create_servo_group(self, name, init_val, color):
        """创建单个舵机控制组件"""
        group = QGroupBox(f"{name} 舵机")
        group.setStyleSheet(f"border: 2px solid {color};")
        layout = QVBoxLayout(group)
        
        # 舵机值显示
        self.servo_labels[name] = QLabel(str(init_val))
        self.servo_labels[name].setAlignment(Qt.AlignCenter)
        self.servo_labels[name].setStyleSheet(f"color: {color}; font-size: 20px;")
        layout.addWidget(self.servo_labels[name])
        
        # 滑块控制
        slider = QSlider(Qt.Horizontal)
        slider.setRange(500, 2500)
        slider.setValue(init_val)
        slider.setStyleSheet(f"QSlider::sub-page:horizontal {{ background: {color}; }}")
        self.servo_sliders[name] = slider
        layout.addWidget(slider)
        
        # 连接滑块值改变事件
        slider.valueChanged.connect(lambda value, lbl=self.servo_labels[name]: lbl.setText(str(value)))
        slider.valueChanged.connect(lambda value, n=name: self.send_servo_command(n, value))
        
        return group
    
    def setup_connections(self):
        """连接信号与槽"""
        self.auto_grab_btn.clicked.connect(self.start_auto_grab)
        self.reset_btn.clicked.connect(self.reset_all_servos)
        self.apply_btn.clicked.connect(self.apply_settings)
        self.serial_connect_btn.clicked.connect(self.toggle_serial_connection)
    
    def setup_camera_timer(self):
        """设置摄像头定时更新"""
        self.camera_timer = QTimer(self)
        self.camera_timer.timeout.connect(self.update_camera_display)
        self.camera_timer.start(30)  # 30ms更新一次
    
    def update_camera_display(self):
        """更新摄像头画面显示"""
        try:
            frame = self.detector.detect_colors()
            if frame is not None:
                # 转换为Qt可显示的格式
                rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w
                q_img = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
                self.camera_label.setPixmap(QPixmap.fromImage(q_img).scaled(
                    self.camera_label.width(), 
                    self.camera_label.height(), 
                    Qt.KeepAspectRatio
                ))
        except Exception as e:
            self.status_bar.showMessage(f"摄像头错误: {str(e)}")
    
    def toggle_serial_connection(self):
        """切换串口连接状态"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.serial_connect_btn.setText("连接串口")
            self.status_bar.showMessage("串口已断开")
            self.auto_grab_btn.setEnabled(False)
            self.apply_btn.setEnabled(False)
            self.reset_btn.setEnabled(False)
        else:
            try:
                # 尝试连接串口（根据实际情况修改端口和波特率）
                self.serial_port = serial.Serial('COM5', 115200, timeout=1)
                self.serial_connect_btn.setText("断开串口")
                self.status_bar.showMessage("串口已连接")
                self.auto_grab_btn.setEnabled(True)
                self.apply_btn.setEnabled(True)
                self.reset_btn.setEnabled(True)
            except Exception as e:
                QMessageBox.critical(self, "连接失败", f"无法连接串口: {str(e)}")
                self.status_bar.showMessage("串口连接失败")
    
    def start_auto_grab(self):
        """启动自动抓取线程"""
        if not self.serial_port or not self.serial_port.is_open:
            QMessageBox.warning(self, "操作失败", "请先连接串口")
            return
        
        if self.grab_thread and self.grab_thread.isRunning():
            QMessageBox.warning(self, "操作失败", "已有自动抓取任务正在运行")
            return
        
        target_color = self.color_combo.currentText()
        self.auto_grab_btn.setEnabled(False)
        
        self.grab_thread = GrabThread(self.serial_port, target_color, self.detector)
        self.grab_thread.status_signal.connect(self.status_bar.showMessage)
        self.grab_thread.error_signal.connect(self.handle_grab_error)
        self.grab_thread.finished.connect(self.on_grab_finished)
        self.grab_thread.start()
    
    def handle_grab_error(self, error_msg):
        """处理自动抓取错误"""
        QMessageBox.critical(self, "抓取失败", error_msg)
        self.status_bar.showMessage("自动抓取失败")
    
    def on_grab_finished(self):
        """自动抓取完成后回调"""
        self.auto_grab_btn.setEnabled(True)
        self.status_bar.showMessage("自动抓取已完成")
    
    def reset_all_servos(self):
        """重置所有舵机到初始位置"""
        if not self.serial_port or not self.serial_port.is_open:
            QMessageBox.warning(self, "操作失败", "请先连接串口")
            return
        
        initial_values = {
            "S00": 1500,  # 底座
            "S01": 1500,   # 大臂
            "S02": 1500,  # 小臂
            "S03": 1500,   # 腕部
            "S04": 1500,  # 旋转关节
            "S05": 1500   # 夹爪
        }
        
        for name, value in initial_values.items():
            self.servo_sliders[name].setValue(value)
            self.send_servo_command(name, value)
        '''home_cmd = self.generate_servo_commands({i: 1500 for i in range(6)}, 2000)
        print(f"生成归位指令: {home_cmd}")
        self.send_generated_command(home_cmd)
        time.sleep(2.5)
        self.status_bar.showMessage("所有舵机已重置到初始位置")'''
    
    def apply_settings(self):
        """应用当前所有舵机设置"""
        if not self.serial_port or not self.serial_port.is_open:
            QMessageBox.warning(self, "操作失败", "请先连接串口")
            return
        
        for name, slider in self.servo_sliders.items():
            value = slider.value()
            self.send_servo_command(name, value)
        
        self.status_bar.showMessage("所有舵机设置已应用")
    
    def send_servo_command(self, servo_name, value):
        """发送舵机控制指令"""
        if not self.serial_port or not self.serial_port.is_open:
            return
        # 从名称中提取舵机ID（例如 S00 → 0）
        servo_id = int(servo_name[1:])
        
        # 构建指令（格式：#000P1500T1000!）
        cmd = f"#{servo_id:03d}P{value:04d}T800!"
        
        try:
            self.serial_port.write(cmd.encode())
            time.sleep(0.1)
            print(f"发送指令: {cmd}")
            self.status_bar.showMessage(f"已发送: {cmd}")
        except Exception as e:
            self.status_bar.showMessage(f"发送失败: {str(e)}")
    '''def generate_servo_commands(self,servo_info, time_ms):
        if not isinstance(servo_info, dict) or not servo_info:
            raise ValueError("请提供格式为 {舵机ID: PWM值} 的非空字典")
        
        commands = []
        for servo_id, pwm in servo_info.items():
            # 验证参数范围
            if not (0 <= servo_id <= 5):
                raise ValueError(f"舵机ID {servo_id} 超出范围 (0-5)")
            if not (500 <= pwm <= 2500):
                raise ValueError(f"PWM值 {pwm} 超出范围 (500-2500)")
            
            # 生成单条指令
            formatted_id = f"{servo_id:03d}"
            formatted_pwm = f"P{pwm:04d}"
            formatted_time = f"T{time_ms:04d}"
            commands.append(f"#{formatted_id}{formatted_pwm}{formatted_time}!")
        
        # 单指令直接返回，多指令用花括号包裹
        return commands[0] if len(commands) == 1 else "{" + "".join(commands) + "}"
    def send_generated_command(self,command, port='COM5', baudrate=115200):
        """发送已生成的舵机控制指令"""
        try:
            with serial.Serial(port, baudrate, timeout=1) as ser:
                time.sleep(0.1)  # 等待串口初始化
                ser.write(command.encode())
                time.sleep(0.1)  # 等待命令发送完成
                response = ser.read_all().decode().strip()
                print(f"发送命令: {command}")
                if response:
                    print(f"收到响应: {response}")
                return True
        except Exception as e:
            print(f"发送命令时出错: {e}")
            return False   '''   
    def closeEvent(self, event):
        """窗口关闭时清理资源"""
        # 停止摄像头
        if self.detector:
            self.detector.release()
        
        # 停止自动抓取线程
        if self.grab_thread and self.grab_thread.isRunning():
            self.grab_thread.running = False
            self.grab_thread.wait()
        
        # 关闭串口连接
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ServoControlApp()
    window.show()
    sys.exit(app.exec_())