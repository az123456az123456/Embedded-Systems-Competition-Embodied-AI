"""Desktop viewer for the sensor-only navigation simulation.

The viewer reads ground truth only when the comparison overlay is enabled.
Navigation is owned by Session and does not receive canvas/UI world geometry.
"""
from __future__ import annotations

import math
import sys
import time
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import QPointF, QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QImage, QKeySequence, QPainter, QPainterPath, QPen, QShortcut
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QFileDialog, QFrame, QHBoxLayout,
    QLabel, QMainWindow, QMessageBox, QPushButton, QScrollArea, QSizePolicy, QVBoxLayout, QWidget,
)


TEAL = QColor("#39d9b1")
TEXT = QColor("#e7edf6")
MUTED = QColor("#91a4bc")
BG = QColor("#0b1422")
FREE = QColor("#d3dedf")
OCCUPIED = QColor("#32485c")
UNKNOWN = QColor("#182638")


def _yaw(pose):
    return float(pose.yaw)


def _xy(point):
    if hasattr(point, "x"):
        return float(point.x), float(point.y)
    return float(point[0]), float(point[1])


def _range_value(reading):
    return float(reading.range_m) if reading.range_m is not None else float("nan")


def _range_valid(reading):
    return reading.status == "ok" and math.isfinite(_range_value(reading))


def _rect_xywh(rect):
    return tuple(float(v) for v in rect[:4])


def _state_text(state):
    states = {
        "idle": "等待开始", "paused": "已暂停", "exploring": "自主探索",
        "explore": "自主探索", "navigating": "前往目标", "navigate": "前往目标",
        "planning": "规划路径", "turning": "转向探测", "scanning": "原地扫描",
        "scan": "原地扫描", "avoiding": "避障调整", "recovering": "避障恢复",
        "blocked": "前方受阻", "stopped": "已停车", "estop": "急停已锁定",
        "complete": "探索完成", "finished": "探索完成", "goal_reached": "已到达目标",
        "no_frontier": "暂无可达边界", "waiting": "等待测距", "sensor_timeout": "测距超时停车",
        "calibrating": "静止校准", "sensor_wait": "等待传感器",
        "replanning": "重新规划", "arrived": "已到达目标",
    }
    text = str(state)
    return states.get(text.lower(), text.replace("_", " "))


class MapCanvas(QWidget):
    goal_requested = Signal(float, float)

    def __init__(self, session, parent=None):
        super().__init__(parent)
        self.session = session
        self.show_truth = False
        self._scale = 1.0
        self._origin = QPointF()
        self._area = QRectF()
        self._hover = None
        self._map_image = None
        self.setMinimumSize(540, 440)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setMouseTracking(True)
        self.setCursor(Qt.CursorShape.CrossCursor)

    def refresh(self):
        grid = self.session.navigator.grid
        image = QImage(grid.cols, grid.rows, QImage.Format.Format_RGB32)
        palette = { -1: UNKNOWN.rgb(), 0: FREE.rgb(), 1: OCCUPIED.rgb() }
        for row in range(grid.rows):
            for col in range(grid.cols):
                image.setPixel(col, grid.rows - row - 1, palette[grid.cell_state(col, row)])
        self._map_image = image
        self.update()

    def _screen(self, x, y):
        return QPointF(self._origin.x() + x * self._scale,
                       self._origin.y() - y * self._scale)

    def _world(self, point):
        return ((point.x() - self._origin.x()) / self._scale,
                (self._origin.y() - point.y()) / self._scale)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), BG)
        grid = self.session.navigator.grid
        margin = 43.0
        map_w = float(grid.width)
        map_h = float(grid.height)
        self._scale = min((self.width() - 2 * margin) / map_w,
                          (self.height() - 2 * margin) / map_h)
        pixel_w, pixel_h = map_w * self._scale, map_h * self._scale
        left, top = (self.width() - pixel_w) / 2, (self.height() - pixel_h) / 2
        self._origin = QPointF(left, top + pixel_h)
        self._area = QRectF(left, top, pixel_w, pixel_h)
        if self._map_image is not None:
            p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform, False)
            p.drawImage(self._area, self._map_image)
        else:
            p.fillRect(self._area, UNKNOWN)

        p.setPen(QPen(QColor("#24374b"), 1))
        for x in range(int(map_w) + 1):
            p.drawLine(self._screen(x, 0), self._screen(x, map_h))
        for y in range(int(map_h) + 1):
            p.drawLine(self._screen(0, y), self._screen(map_w, y))
        p.setPen(QPen(QColor("#53657b"), 1))
        p.drawRect(self._area)
        p.setFont(QFont("Noto Sans CJK SC", 8))
        p.setPen(MUTED)
        for x in range(0, int(map_w) + 1, 2):
            point = self._screen(x, 0)
            p.drawText(QRectF(point.x() - 14, point.y() + 7, 36, 20), Qt.AlignmentFlag.AlignCenter, f"{x} m")
        for y in range(0, int(map_h) + 1, 2):
            point = self._screen(0, y)
            p.drawText(QRectF(point.x() - 38, point.y() - 9, 31, 20), Qt.AlignmentFlag.AlignRight, str(y))

        p.save()
        p.setClipRect(self._area)
        if self.show_truth:
            self._draw_truth(p)
        self._draw_trajectory(p)
        self._draw_plan(p)
        self._draw_sensors(p)
        pose = self.session.navigator.pose
        self._draw_robot(p, pose.x, pose.y, _yaw(pose), TEAL, solid=True)
        p.restore()

        if self._hover and self._area.contains(self._hover):
            x, y = self._world(self._hover)
            label = f"{x:.2f}, {y:.2f} m"
            p.setPen(TEXT)
            p.fillRect(QRectF(self._hover.x() + 12, self._hover.y() - 27, 118, 23), QColor("#122438"))
            p.drawText(QRectF(self._hover.x() + 18, self._hover.y() - 27, 110, 23), Qt.AlignmentFlag.AlignVCenter, label)

        p.setPen(MUTED)
        p.setFont(QFont("Noto Sans CJK SC", 9))
        if self.show_truth:
            p.setPen(QColor("#ffbd70"))
            p.drawText(QRectF(left + 12, top + 9, pixel_w - 24, 25),
                       Qt.AlignmentFlag.AlignLeft, "真实环境对照 · 不参与导航算法")
        else:
            p.drawText(QRectF(left + 12, top + 9, pixel_w - 24, 25),
                       Qt.AlignmentFlag.AlignLeft, "机器人正在从未知区域建立地图")
        p.end()

    def _draw_truth(self, p):
        p.save()
        p.setPen(QPen(QColor("#ffb86b"), 1.5, Qt.PenStyle.DashLine))
        p.setBrush(QColor(255, 184, 107, 30))
        for rect in self.session.world.rectangles:
            x, y, w, h = _rect_xywh(rect)
            p.drawRect(QRectF(self._screen(x, y + h), self._screen(x + w, y)))
        truth = self.session.simulator.truth_pose
        self._draw_robot(p, truth.x, truth.y, _yaw(truth), QColor("#ffb86b"), solid=False)
        p.restore()

    def _draw_trajectory(self, p):
        trajectory = self.session.navigator.trajectory
        if len(trajectory) < 2:
            return
        path = QPainterPath(self._screen(*_xy(trajectory[0])))
        for point in trajectory[1:]:
            path.lineTo(self._screen(*_xy(point)))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(QColor("#33b89c"), 2, Qt.PenStyle.SolidLine,
                      Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin))
        p.drawPath(path)

    def _draw_plan(self, p):
        navigator = self.session.navigator
        if navigator.path:
            path = QPainterPath(self._screen(*_xy(navigator.path[0])))
            for point in navigator.path[1:]:
                path.lineTo(self._screen(*_xy(point)))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.setPen(QPen(QColor("#9ea7ff"), 2, Qt.PenStyle.DashLine))
            p.drawPath(path)
        if navigator.goal is not None:
            center = self._screen(*_xy(navigator.goal))
            p.setPen(QPen(QColor("#b3adff"), 2))
            p.setBrush(QColor(179, 173, 255, 30))
            p.drawEllipse(center, 9, 9)
            p.drawLine(QPointF(center.x() - 14, center.y()), QPointF(center.x() + 14, center.y()))
            p.drawLine(QPointF(center.x(), center.y() - 14), QPointF(center.x(), center.y() + 14))

    def _draw_sensors(self, p):
        pose = self.session.navigator.pose
        yaw = _yaw(pose)
        mounts = {mount.name: mount for mount in self.session.config.mounts}
        colors = {"front": QColor("#69dcff"), "left": QColor("#b1a4ff"), "right": QColor("#ffc781")}
        for name, reading in self.session.navigator.ranges.items():
            if (name not in mounts or not _range_valid(reading)
                    or self.session.simulator.time_s - reading.stamp_s > self.session.config.stale_after_s):
                continue
            mount = mounts[name]
            distance = max(0, min(self.session.config.sonar_max_m, _range_value(reading)))
            angle = yaw + mount.yaw
            sx = pose.x + math.cos(yaw) * mount.x - math.sin(yaw) * mount.y
            sy = pose.y + math.sin(yaw) * mount.x + math.cos(yaw) * mount.y
            # Broad cone denotes angular uncertainty; it is not a laser ray.
            half_cone = self.session.config.sonar_fov_rad / 2
            origin = self._screen(sx, sy)
            cone = QPainterPath(origin)
            for i in range(13):
                a = angle - half_cone + (2 * half_cone) * i / 12
                cone.lineTo(self._screen(sx + math.cos(a) * distance,
                                         sy + math.sin(a) * distance))
            cone.closeSubpath()
            color = colors[name]
            p.setPen(QPen(QColor(color.red(), color.green(), color.blue(), 115), 1))
            p.setBrush(QColor(color.red(), color.green(), color.blue(), 23))
            p.drawPath(cone)

    def _draw_robot(self, p, x, y, yaw, color, solid):
        p.save()
        center = self._screen(x, y)
        p.translate(center)
        p.rotate(-math.degrees(yaw))
        size = max(12, self._scale * 0.25)
        length = size * 1.25
        width = size * .85
        p.setPen(QPen(color, 2 if solid else 1.5,
                      Qt.PenStyle.SolidLine if solid else Qt.PenStyle.DashLine))
        p.setBrush(QColor("#0d2831") if solid else Qt.BrushStyle.NoBrush)
        p.drawRoundedRect(QRectF(-length / 2, -width / 2, length, width), 3, 3)
        p.setBrush(QColor("#11212e") if solid else Qt.BrushStyle.NoBrush)
        for wheel_x in (-length * .35, length * .35):
            for wheel_y in (-width * .65, width * .65):
                p.drawRoundedRect(QRectF(wheel_x - size * .17, wheel_y - size * .11,
                                        size * .34, size * .22), 1, 1)
        arrow = QPainterPath(QPointF(length * .12, -width * .24))
        arrow.lineTo(length * .38, 0)
        arrow.lineTo(length * .12, width * .24)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawPath(arrow)
        p.restore()

    def mouseMoveEvent(self, event):
        self._hover = event.position()
        self.update()

    def leaveEvent(self, event):
        self._hover = None
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton and self._area.contains(event.position()):
            self.goal_requested.emit(*self._world(event.position()))


class Dashboard(QMainWindow):
    def __init__(self, session):
        super().__init__()
        self.session = session
        self.setWindowTitle("小车导航实验室 · 三向超声波建图仿真")
        self.resize(1360, 960)
        self.setMinimumSize(1000, 700)
        self.speed = 1
        self._last_tick = time.monotonic()
        self._accumulator = 0.0
        self._last_draw = 0.0
        self._last_nav_message = ""
        self._build_ui()
        self._timer = QTimer(self)
        self._timer.setInterval(16)
        self._timer.timeout.connect(self._tick)
        self._timer.start()
        QShortcut(QKeySequence(Qt.Key.Key_Space), self, activated=self.toggle_running)
        self.refresh()

    def _build_ui(self):
        self.setStyleSheet("""
            QMainWindow, QWidget { background:#0b1422; color:#e7edf6;
                font-family:'Noto Sans CJK SC','Microsoft YaHei','PingFang SC',sans-serif; font-size:13px; }
            QFrame#panel { background:#111f30; border:1px solid #22344a; border-radius:12px; }
            QLabel { background:transparent; border:none; }
            QLabel#muted { color:#91a4bc; font-size:12px; }
            QLabel#title { font-size:23px; font-weight:700; }
            QLabel#section { font-size:14px; font-weight:600; color:#e3edf5; }
            QLabel#big { font-size:26px; font-weight:600; }
            QPushButton { background:#1b3047; border:1px solid #30465f; border-radius:7px;
                padding:9px 14px; font-weight:500; }
            QPushButton:hover { background:#27435e; border-color:#4b718a; }
            QPushButton#primary { background:#26b897; color:#06251e; border:none; font-weight:700; }
            QPushButton#primary:hover { background:#53dcb9; }
            QPushButton#danger { background:#3b2230; color:#ffb2ba; border-color:#6c3447; }
            QPushButton#danger:hover { background:#622a3c; }
            QComboBox { background:#182b40; border:1px solid #30465f; border-radius:6px;
                padding:7px 10px; min-width:85px; }
            QComboBox QAbstractItemView { background:#182b40; color:#e7edf6; selection-background-color:#287c75; }
            QCheckBox { background:transparent; spacing:8px; color:#abbcd0; }
            QCheckBox::indicator { width:16px; height:16px; border:1px solid #476078; border-radius:3px; background:#142338; }
            QCheckBox::indicator:checked { background:#36c6a6; border-color:#36c6a6; }
            QStatusBar { color:#a6b8cb; background:#101d2c; }
            QScrollArea { border:none; background:transparent; }
            QScrollBar:vertical { width:7px; background:#101d2c; }
            QScrollBar::handle:vertical { background:#35516a; border-radius:3px; min-height:28px; }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height:0; }
        """)
        central = QWidget()
        self.setCentralWidget(central)
        outer = QVBoxLayout(central)
        outer.setContentsMargins(24, 18, 24, 16)
        outer.setSpacing(16)

        header = QHBoxLayout()
        title_col = QVBoxLayout()
        title = QLabel("小车导航实验室")
        title.setObjectName("title")
        title_col.addWidget(title)
        sub = QLabel("前 / 左 / 右 A02YYUW  ·  编码器  ·  IMU  ·  从未知地图自主探索")
        sub.setObjectName("muted")
        title_col.addWidget(sub)
        header.addLayout(title_col)
        header.addStretch()
        badge = QLabel("  SIMULATION · 仅仿真  ")
        badge.setStyleSheet("background:#17372f;color:#77e6c4;border:1px solid #28624f;border-radius:6px;padding:6px;")
        badge.setFixedHeight(34)
        header.addWidget(badge)
        outer.addLayout(header)

        toolbar = QHBoxLayout()
        self.start_button = QPushButton("开始探索")
        self.start_button.setObjectName("primary")
        self.start_button.clicked.connect(self.toggle_running)
        toolbar.addWidget(self.start_button)
        reset = QPushButton("重置")
        reset.clicked.connect(self.reset)
        toolbar.addWidget(reset)
        estop = QPushButton("急停")
        estop.setObjectName("danger")
        estop.clicked.connect(self.estop)
        toolbar.addWidget(estop)
        toolbar.addSpacing(12)
        toolbar.addWidget(QLabel("场景"))
        self.scenario_combo = QComboBox()
        for text, value in (("室内房间", "apartment"), ("开阔空间", "open"), ("转角走廊", "corridor")):
            self.scenario_combo.addItem(text, value)
        index = self.scenario_combo.findData(self.session.world.name)
        self.scenario_combo.setCurrentIndex(max(0, index))
        self.scenario_combo.currentIndexChanged.connect(self.reset)
        toolbar.addWidget(self.scenario_combo)
        toolbar.addWidget(QLabel("仿真速度"))
        self.speed_combo = QComboBox()
        for multiplier in (1, 2, 4):
            self.speed_combo.addItem(f"{multiplier} ×", multiplier)
        self.speed_combo.currentIndexChanged.connect(self._change_speed)
        toolbar.addWidget(self.speed_combo)
        toolbar.addStretch()
        self.truth_checkbox = QCheckBox("显示真实环境对照")
        self.truth_checkbox.toggled.connect(self._toggle_truth)
        toolbar.addWidget(self.truth_checkbox)
        outer.addLayout(toolbar)

        body = QHBoxLayout()
        body.setSpacing(18)
        map_panel = QFrame()
        map_panel.setObjectName("panel")
        map_layout = QVBoxLayout(map_panel)
        map_layout.setContentsMargins(8, 8, 8, 8)
        self.canvas = MapCanvas(self.session)
        self.canvas.goal_requested.connect(self._set_goal)
        map_layout.addWidget(self.canvas, 1)
        legend = QLabel('<span style="color:#526a83">■</span> 未知区域　'
                        '<span style="color:#d3dedf">■</span> 已探测空地　'
                        '<span style="color:#32485c">■</span> 障碍物　　'
                        '<span style="color:#39d9b1">━</span> 估计轨迹　　'
                        '<span style="color:#b3adff">┄</span> 规划路径')
        legend.setStyleSheet("color:#9aadc2;font-size:11px;padding:3px 8px;")
        map_layout.addWidget(legend)
        body.addWidget(map_panel, 1)
        sidebar = QVBoxLayout()
        sidebar.setSpacing(12)
        self.mode_label = QLabel()
        self.mode_label.setStyleSheet("color:#6be3c0;font-size:20px;font-weight:600;")
        self.time_label = QLabel()
        self.map_label = QLabel()
        self.pose_label = QLabel()
        self.motion_label = QLabel()
        self.travel_label = QLabel()
        self.collision_label = QLabel()
        self._card(sidebar, "导航状态", [self.mode_label, self.time_label, self.map_label])
        self._card(sidebar, "编码器 + IMU 位置估计", [self.pose_label, self.motion_label, self.travel_label])
        self.sensor_labels = {}
        sensor_rows = []
        for name, label, color in (("front", "前方", "#69dcff"), ("left", "左侧", "#b1a4ff"), ("right", "右侧", "#ffc781")):
            text = QLabel()
            text.setStyleSheet(f"font-size:16px;color:{color};padding:3px 0;")
            self.sensor_labels[name] = text
            sensor_rows.append(text)
        note = QLabel("固定朝向 · 有盲区与延迟\n超声波扇区仅表示测距方向范围")
        note.setObjectName("muted")
        sensor_rows.append(note)
        self._card(sidebar, "三向超声波", sensor_rows)
        target_card = QFrame()
        target_card.setObjectName("panel")
        target_layout = QVBoxLayout(target_card)
        target_layout.setContentsMargins(16, 14, 16, 14)
        target_title = QLabel("探索与目标")
        target_title.setObjectName("section")
        target_layout.addWidget(target_title)
        target_note = QLabel("点击地图中已知的空闲位置，\n让小车规划路径前往该处。")
        target_note.setObjectName("muted")
        target_layout.addWidget(target_note)
        explore = QPushButton("恢复自主探索")
        explore.setMinimumHeight(38)
        explore.clicked.connect(self._explore)
        target_layout.addWidget(explore)
        export = QPushButton("导出地图与记录")
        export.setMinimumHeight(38)
        export.clicked.connect(self._export)
        target_layout.addWidget(export)
        sidebar.addWidget(target_card)
        sidebar.addStretch()
        self.collision_label.setObjectName("muted")
        sidebar_widget = QWidget()
        sidebar_widget.setLayout(sidebar)
        sidebar.setContentsMargins(0, 0, 8, 0)
        for label in sidebar_widget.findChildren(QLabel):
            label.setMinimumHeight(label.sizeHint().height())
        sidebar_scroll = QScrollArea()
        sidebar_scroll.setFixedWidth(292)
        sidebar_scroll.setWidgetResizable(True)
        sidebar_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        sidebar_scroll.setWidget(sidebar_widget)
        body.addWidget(sidebar_scroll)
        outer.addLayout(body, 1)
        footer = QLabel("空格：开始 / 暂停    ·    地图与轨迹来自传感器估计    ·    未连接真实硬件")
        footer.setObjectName("muted")
        footer_row = QHBoxLayout()
        footer_row.addWidget(footer)
        footer_row.addStretch()
        footer_row.addWidget(self.collision_label)
        outer.addLayout(footer_row)
        self.statusBar().showMessage("准备就绪。点击“开始探索”，机器人将仅使用模拟传感器进行建图。")

    @staticmethod
    def _card(layout, title, labels):
        panel = QFrame()
        panel.setObjectName("panel")
        panel_layout = QVBoxLayout(panel)
        panel_layout.setContentsMargins(14, 12, 14, 12)
        panel_layout.setSpacing(6)
        heading = QLabel(title)
        heading.setObjectName("section")
        panel_layout.addWidget(heading)
        for label in labels:
            panel_layout.addWidget(label)
        layout.addWidget(panel)

    def _tick(self):
        now = time.monotonic()
        elapsed = min(0.20, now - self._last_tick)
        self._last_tick = now
        if self.session.running:
            self._accumulator += elapsed * self.speed
            # Fixed steps preserve sensor cadence even when rendering stalls.
            for _ in range(16):
                if self._accumulator < 0.05 or not self.session.running:
                    break
                self.session.step(0.05)
                self._accumulator -= 0.05
        else:
            self._accumulator = 0.0
        if now - self._last_draw >= 0.08:
            self.refresh()
            message = self.session.navigator.message
            if message and message != self._last_nav_message:
                self.statusBar().showMessage(message)
                self._last_nav_message = message
            self._last_draw = now

    def refresh(self):
        simulator = self.session.simulator
        navigator = self.session.navigator
        pose = navigator.pose
        self.start_button.setText("暂停运动" if self.session.running else "开始 / 继续")
        self.mode_label.setText(_state_text(navigator.state))
        elapsed_seconds = int(simulator.time_s + 1e-7)
        self.time_label.setText(f"仿真时间    {elapsed_seconds // 60:02d}:{elapsed_seconds % 60:02d}")
        fraction = navigator.grid.known_fraction()
        area = fraction * navigator.grid.width * navigator.grid.height
        self.map_label.setText(f"已知地图    {fraction * 100:.1f}%  ·  {area:.1f} m²")
        self.pose_label.setText(f"X  {pose.x:.2f} m       Y  {pose.y:.2f} m\n航向  {math.degrees(_yaw(pose)) % 360:.1f}°")
        command = navigator.command
        linear = command.linear_m_s
        angular = command.angular_rad_s
        self.motion_label.setText(f"指令速度  {linear:.2f} m/s\n指令转速  {math.degrees(angular):.1f} °/s")
        self.travel_label.setText(f"估计行程  {navigator.localizer.distance_m:.2f} m")
        self.collision_label.setText(f"仿真碰撞计数  {simulator.collision_count}  ·  用于离线评估")
        for name, chinese in (("front", "前方"), ("left", "左侧"), ("right", "右侧")):
            reading = navigator.ranges.get(name)
            if reading is None:
                self.sensor_labels[name].setText(f"{chinese}       — —  / 等待数据")
            elif simulator.time_s - reading.stamp_s > self.session.config.stale_after_s:
                self.sensor_labels[name].setText(f"{chinese}       — —  / 数据超时")
            elif not _range_valid(reading):
                state = "无回波" if reading.status == "no_echo" else "传感器故障"
                self.sensor_labels[name].setText(f"{chinese}       — —  / {state}")
            else:
                self.sensor_labels[name].setText(f"{chinese}       {_range_value(reading):.2f} m")
        self.canvas.refresh()
        for label in self.findChildren(QLabel):
            label.setMinimumHeight(label.sizeHint().height())

    def toggle_running(self):
        if self.session.running:
            self.session.pause()
            self.statusBar().showMessage("仿真已暂停；空格继续。")
        else:
            self.session.start()
            self.statusBar().showMessage("自主探索运行中。点击已知空闲区域可以指定目标；急停会立即停止运动。")
        self._last_tick = time.monotonic()
        self.refresh()

    def reset(self, *_):
        self.session.reset(scenario=self.scenario_combo.currentData())
        self._accumulator = 0.0
        self._last_tick = time.monotonic()
        self.statusBar().showMessage("场景已重置，等待开始。真实地图默认不提供给导航算法。")
        self.refresh()

    def estop(self):
        self.session.estop()
        self._accumulator = 0.0
        self.statusBar().showMessage("急停：小车已停止。点击开始可重新运行仿真。")
        self.refresh()

    def _change_speed(self, *_):
        self.speed = self.speed_combo.currentData()

    def _toggle_truth(self, checked):
        self.canvas.show_truth = checked
        self.canvas.update()

    def _set_goal(self, x, y):
        result = self.session.set_goal(x, y)
        if result is False:
            self.statusBar().showMessage("目标无效：请选择已探测且留有车身通行空间的空地。", 6000)
        else:
            self.statusBar().showMessage(f"目标已设置：({x:.2f}, {y:.2f}) m。点击“恢复自主探索”可取消手动目标。")
        self.refresh()

    def _explore(self):
        self.session.explore()
        self.statusBar().showMessage("已恢复自主探索模式。")
        self.refresh()

    def _export(self):
        root = QFileDialog.getExistingDirectory(self, "选择导出目录", str(Path.home()))
        if not root:
            return
        directory = Path(root) / f"navsim-{datetime.now():%Y%m%d-%H%M%S}"
        try:
            directory.mkdir(parents=True, exist_ok=True)
            self.session.export(directory)
            self.canvas.grab().save(str(directory / "map-view.png"))
        except Exception as exc:
            QMessageBox.warning(self, "导出失败", str(exc))
            return
        self.statusBar().showMessage(f"地图、轨迹和传感器记录已导出至 {directory}", 15000)

    def closeEvent(self, event):
        self._timer.stop()
        self.session.close()
        event.accept()


def run_gui(config=None, scenario="apartment", screenshot_path=None, demo_seconds=0):
    """Start the desktop app, or render a deterministic offscreen smoke frame.

    screenshot_path is useful for GUI checks on a host without a display; set
    QT_QPA_PLATFORM=offscreen externally before starting Python.
    """
    from .session import Session

    app = QApplication.instance() or QApplication(sys.argv[:1])
    app.setApplicationName("小车导航实验室")
    app.setFont(QFont("Noto Sans CJK SC", 10))
    session = Session(config=config, scenario=scenario)
    window = Dashboard(session)
    window.show()
    if screenshot_path:
        window._timer.stop()
        def capture():
            if demo_seconds > 0:
                session.start()
                for _ in range(round(demo_seconds / .05)):
                    session.step(.05)
            window.refresh()
            window.statusBar().showMessage(session.navigator.message)
            app.processEvents()
            target = Path(screenshot_path)
            target.parent.mkdir(parents=True, exist_ok=True)
            if not window.grab().save(str(target)):
                raise RuntimeError(f"Could not save screenshot: {target}")
            window.close()
            app.quit()
        QTimer.singleShot(150, capture)
    return app.exec()
