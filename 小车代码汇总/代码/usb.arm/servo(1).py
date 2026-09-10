import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QGroupBox, 
                             QVBoxLayout, QHBoxLayout, QLabel, QSlider, 
                             QGridLayout, QFrame, QPushButton, QMessageBox)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QIcon

# 特殊函数 test
def test():
    print("启动智能识别夹取")

class ServoControlApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("舵机控制系统")
        self.setGeometry(100, 100, 800, 450)
        self.setWindowIcon(QIcon("servo_icon.png"))
        self.setup_ui()
        self.setup_connections()

    def setup_ui(self):
        # 主窗口样式
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1e1e2d;
                color: #e0e0ff;
                font-family: 'Segoe UI', 'Arial', sans-serif;
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
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QLabel {
                color: #e0e0ff;
                font-size: 14px;
            }
            QSlider::groove:horizontal {
                border: 1px solid #4a4a5a;
                height: 10px;
                background: #2d2d3d;
                margin: 3px 0;
                border-radius: 5px;
            }
            QSlider::handle:horizontal {
                background: #4cc9f0;
                border: 1px solid #3a9bc0;
                width: 20px;
                margin: -6px 0;
                border-radius: 10px;
            }
            QSlider::sub-page:horizontal {
                background: #4361ee;
                border-radius: 5px;
            }
            QPushButton {
                background-color: #4361ee;
                color: white;
                border: none;
                border-radius: 5px;
                padding: 8px 16px;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #4895ef;
            }
            QPushButton:pressed {
                background-color: #3f37c9;
            }
        """)

        # 创建主布局
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(20, 20, 20, 10)
        main_layout.setSpacing(15)

        # 标题
        title = QLabel("舵机控制系统")
        title.setAlignment(Qt.AlignCenter)
        title_font = QFont("Arial", 24, QFont.Bold)
        title.setFont(title_font)
        title.setStyleSheet("color: #4cc9f0; margin-bottom: 10px;")
        main_layout.addWidget(title)

        # 创建舵机控制面板
        grid_layout = QGridLayout()
        grid_layout.setSpacing(15)

        # 舵机配置数据 (名称, 初始值, 颜色)
        servos = [
            ("S00", 1875, "#4cc9f0"),  # 青色
            ("S01", 500, "#f72585"),   # 粉色
            ("S02", 1300, "#4caf50"),  # 绿色
            ("S03", 968, "#ff9800"),    # 橙色
            ("S04", 1100, "#9c27b0"),  # 紫色
            ("S05", 1500, "#00bcd4")   # 蓝绿色
        ]

        self.servo_sliders = {}
        self.servo_labels = {}

        # 创建舵机控制面板
        for i, (name, init_value, color) in enumerate(servos):
            row = i // 2
            col = i % 2
            servo_group = self.create_servo_group(name, init_value, color)
            grid_layout.addWidget(servo_group, row, col)

        main_layout.addLayout(grid_layout)

        # 添加控制按钮
        button_layout = QHBoxLayout()
        button_layout.setContentsMargins(20, 10, 20, 10)

        self.reset_btn = QPushButton("重置所有舵机")
        self.reset_btn.setFixedHeight(40)
        self.reset_btn.setStyleSheet("background-color: #f72585;")
        button_layout.addWidget(self.reset_btn)

        self.apply_btn = QPushButton("应用设置")
        self.apply_btn.setFixedHeight(40)
        self.apply_btn.setStyleSheet("background-color: #4caf50;")
        button_layout.addWidget(self.apply_btn)

        # 新增调用特殊函数 test 的按钮
        self.test_btn = QPushButton("调用特殊函数 test")
        self.test_btn.setFixedHeight(40)
        self.test_btn.setStyleSheet("background-color: #ff9800;")
        button_layout.addWidget(self.test_btn)

        main_layout.addLayout(button_layout)

        # 状态栏
        self.status_bar = self.statusBar()
        self.status_bar.showMessage("就绪 | 系统运行中...")

        # 添加底部信息
        footer = QLabel("舵机值范围: 500-2500 | 系统版本: v1.2.3 | © 2023 舵机控制系统")
        footer.setAlignment(Qt.AlignCenter)
        footer.setStyleSheet("font-size: 12px; color: #7f8c8d; margin-top: 10px;")
        main_layout.addWidget(footer)

    def create_servo_group(self, name, init_value, color):
        group = QGroupBox(f"{name} 舵机窗口")
        group.setStyleSheet(f"border: 2px solid {color};")
        layout = QVBoxLayout(group)
        layout.setSpacing(12)

        # ID 显示
        id_label = QLabel("ID 00")
        id_label.setAlignment(Qt.AlignCenter)
        id_label.setStyleSheet(f"color: {color}; font-size: 16px;")
        layout.addWidget(id_label)

        # 值显示区域
        value_frame = QFrame()
        value_frame.setStyleSheet("background-color: #2d2d3d; border-radius: 8px; padding: 8px;")
        value_layout = QHBoxLayout(value_frame)

        value_text = QLabel("舵机值: ")
        value_text.setStyleSheet("font-weight: bold;")
        value_layout.addWidget(value_text)

        value_label = QLabel(str(init_value))
        value_label.setStyleSheet(f"""
            font-size: 20px;
            font-weight: bold;
            color: {color};
            min-width: 70px;
        """)
        value_layout.addWidget(value_label)

        value_symbol = QLabel("")
        value_layout.addWidget(value_symbol)

        layout.addWidget(value_frame)

        # Bias 显示
        bias_frame = QFrame()
        bias_frame.setStyleSheet("background-color: #2d2d3d; border-radius: 8px; padding: 8px;")
        bias_layout = QHBoxLayout(bias_frame)

        bias_text = QLabel("Bias: ")
        bias_text.setStyleSheet("font-weight: bold;")
        bias_layout.addWidget(bias_text)

        bias_value = QLabel("1500")
        bias_value.setStyleSheet("font-weight: bold; color: #ff9800; font-size: 16px;")
        bias_layout.addWidget(bias_value)

        bias_symbol = QLabel("")
        bias_layout.addWidget(bias_symbol)

        layout.addWidget(bias_frame)

        # 滑块控制
        slider = QSlider(Qt.Horizontal)
        slider.setRange(500, 2500)
        slider.setValue(init_value)
        slider.setSingleStep(10)
        slider.setPageStep(50)
        slider.setStyleSheet(f"""
            QSlider::handle:horizontal {{
                background: {color};
            }}
            QSlider::sub-page:horizontal {{
                background: {color};
            }}
        """)

        # 保存引用
        self.servo_sliders[name] = slider
        self.servo_labels[name] = value_label

        # 连接滑块值改变事件
        slider.valueChanged.connect(lambda value, lbl=value_label: lbl.setText(str(value)))
        slider.valueChanged.connect(lambda value, n=name: self.send_servo_command(n, value))

        layout.addWidget(slider)

        return group

    def setup_connections(self):
        # 连接按钮事件
        self.reset_btn.clicked.connect(self.reset_all_servos)
        self.apply_btn.clicked.connect(self.apply_settings)
        self.test_btn.clicked.connect(test)

        # 连接滑块事件
        for name, slider in self.servo_sliders.items():
            slider.valueChanged.connect(lambda value, n=name: self.update_status(f"{n} 值已更新: {value}"))

    def reset_all_servos(self):
        # 重置所有舵机到初始值
        initial_values = {
            "S00": 1875,
            "S01": 500,
            "S02": 1300,
            "S03": 968,
            "S04": 1100,
            "S05": 1500
        }

        for name, value in initial_values.items():
            self.servo_sliders[name].setValue(value)

        self.status_bar.showMessage("所有舵机已重置")
        QMessageBox.information(self, "重置成功", "所有舵机值已重置到初始状态")

    def apply_settings(self):
        # 获取所有舵机当前值
        values = {name: slider.value() for name, slider in self.servo_sliders.items()}

        # 在实际应用中，这里会发送数据到舵机控制器
        self.status_bar.showMessage("设置已应用到舵机")

        # 显示应用成功的消息
        msg = "舵机值已更新:\n"
        for name, value in values.items():
            msg += f"{name}: {value}\n"

        QMessageBox.information(self, "设置已应用", msg)

    def update_status(self, message):
        self.status_bar.showMessage(message)

    def send_servo_command(self, servo_name, value):
        # 模拟发送指令到舵机，实际应用中需要替换为真实的通信代码
        print(f"向 {servo_name} 舵机发送指令: {value}")

    def closeEvent(self, event):
        # 关闭前的确认
        reply = QMessageBox.question(
            self, '退出确认',
            "确定要退出舵机控制系统吗？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            event.accept()
        else:
            event.ignore()

if __name__ == "__main__":
    app = QApplication(sys.argv)

    # 设置应用样式
    app.setStyle("Fusion")

    window = ServoControlApp()
    window.show()
    sys.exit(app.exec_())