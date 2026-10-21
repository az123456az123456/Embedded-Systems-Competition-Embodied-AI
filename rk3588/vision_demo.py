#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
选题三 任务5/6 —— 视觉目标识别 + 智能任务执行  最小可用 Demo
================================================================
对应赛题：
  任务5 视觉目标识别  → YOLOv8n 目标检测 + ByteTrack 目标跟踪
  任务6 智能任务执行  → "指定目标搜索"（如：找到红色水瓶并移动到其附近）

本脚本做三件事：
  1. 用 YOLOv8n 检测目标（人/瓶子/杯子...），并用 ByteTrack 给每个目标稳定 ID
  2. 对每个检测框做 HSV 颜色判定 —— YOLO 只看形状，颜色要靠传统 CV 补
     （这段逻辑复用旧框架 代码/ultralytics-main/yanse.py 的思路）
  3. 锁定"目标类别 + 目标颜色"的目标，输出它的画面坐标 / 方位角，
     并可通过串口下发给 FPGA（任务6 的"视觉→决策→运动"链路）

⚠️ 本脚本在 PC 上用 .pt 跑，验证的是【算法逻辑】。
   部署到 RK3588 时改成 .rknn + NPU 推理，算法部分基本不动。

用法：
  # 默认：找"人"，不筛颜色
  python vision_demo.py

  # 找红色瓶子（赛题任务6 的原例子）
  python vision_demo.py --target bottle --target-color red

  # 画面翻转：默认 h（左右镜像）；跑起来后按 f 键可循环切换
  python vision_demo.py --flip none
  python vision_demo.py --flip hv     # 画面上下颠倒时用

  # 不弹窗，跑 100 帧后自动退出
  python vision_demo.py --no-show --frames 100

  # 通过串口把目标坐标发给 FPGA
  python vision_demo.py --serial /dev/ttyUSB0 --baud 115200
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    sys.exit("缺少 ultralytics，请先激活虚拟环境：source .venv/bin/activate")


# ============================================================================
# 一、HSV 颜色判定（YOLO 认形状，颜色得自己判）
# ============================================================================

# 三原色 HSV 范围（OpenCV 的 H: 0~180, S/V: 0~255）
# 红色在 HSV 色环上横跨 0°，所以要两段
COLOR_RANGES = {
    'red': [
        (np.array([0, 120, 70]),   np.array([10, 255, 255])),
        (np.array([170, 120, 70]), np.array([180, 255, 255])),
    ],
    'green': [
        (np.array([40, 100, 100]), np.array([80, 255, 255])),
    ],
    'blue': [
        (np.array([100, 150, 0]),  np.array([140, 255, 255])),
    ],
}

# 画面上显示用的 BGR 颜色
COLOR_DISPLAY = {
    'red':   (0, 0, 255),
    'green': (0, 255, 0),
    'blue':  (255, 0, 0),
}

# 画面翻转方式。有些摄像头模组输出的是镜像画面，需要在这里纠正。
# 运行中按 f 键可以循环切换，不用改参数重启。
FLIP_CODES = {
    'none': None,   # 不翻转
    'h':    1,      # 左右镜像（"画面是镜像的"一般用这个纠正）
    'v':    0,      # 上下翻转
    'hv':   -1,     # 旋转 180°
}
FLIP_ORDER = ['none', 'h', 'v', 'hv']


# ============================================================================
# 一·五、中文文字绘制（cv2.putText 画不出中文）
# ============================================================================
# ⚠️ OpenCV 的 putText 只认 ASCII，中文会渲染成一串 "???"。
#    所以画面上任何含中文的地方都走 PIL 渲染，字体优先用 Noto Sans CJK SC。

CJK_FONTS = [
    ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 2),  # 2 = SC 简体
    ('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc', 2),
    ('/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc', 0),
    ('/usr/share/fonts/truetype/arphic/uming.ttc', 0),
]

_font_cache = {}


def _get_font(size):
    """找一个能显示中文的字体；都找不到返回 None（调用方退回 putText）"""
    if size in _font_cache:
        return _font_cache[size]

    from PIL import ImageFont
    font = None
    for path, index in CJK_FONTS:
        if not os.path.exists(path):
            continue
        try:
            font = ImageFont.truetype(path, size, index=index)
            break
        except Exception:
            continue

    _font_cache[size] = font
    return font


def draw_text_cn(frame, lines, origin=(8, 8), font_size=22, line_gap=6):
    """
    在 frame 上就地画多行文字（可含中文），带半透明黑板底。

    lines: [(文本, BGR颜色), ...]

    整帧只做一次 BGR↔PIL 转换 —— 逐行转换在 40FPS 下开销很明显。
    """
    if not lines:
        return

    font = _get_font(font_size)
    if font is None:
        # 没装中文字体：退回 putText，中文会变 ???，但至少不崩
        y = origin[1] + font_size
        for text, color in lines:
            cv2.putText(frame, text, (origin[0], y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            y += font_size + line_gap
        return

    from PIL import Image, ImageDraw

    img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(img)

    # 量出最宽/最高的一行，用来定底板尺寸
    bboxes = [draw.textbbox((0, 0), t, font=font) for t, _ in lines]
    text_w = max(b[2] - b[0] for b in bboxes)
    text_h = max(b[3] - b[1] for b in bboxes)

    pad = 8
    panel_w = min(frame.shape[1] - origin[0], text_w + pad * 2)
    panel_h = (text_h + line_gap) * len(lines) + pad * 2

    # 半透明黑板底 —— 不然浅色背景下白字看不见
    overlay = Image.new('RGBA', img.size, (0, 0, 0, 0))
    ImageDraw.Draw(overlay).rectangle(
        [origin[0], origin[1], origin[0] + panel_w, origin[1] + panel_h],
        fill=(0, 0, 0, 165))
    img = Image.alpha_composite(img.convert('RGBA'), overlay).convert('RGB')

    draw = ImageDraw.Draw(img)
    y = origin[1] + pad
    for text, bgr in lines:
        draw.text((origin[0] + pad, y), text, font=font,
                  fill=(bgr[2], bgr[1], bgr[0]))   # BGR -> RGB
        y += text_h + line_gap

    frame[:] = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)


def dominant_color(bgr_roi, min_ratio=0.12):
    """
    判断一块图像区域的主色。

    返回: (颜色名 或 None, 该色占比 0.0~1.0)
    占比低于 min_ratio 视为"没有明显颜色"（比如灰色/白色物体）
    """
    if bgr_roi is None or bgr_roi.size == 0:
        return None, 0.0

    hsv = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2HSV)
    total = hsv.shape[0] * hsv.shape[1]
    if total == 0:
        return None, 0.0

    best_name, best_ratio = None, 0.0
    for name, ranges in COLOR_RANGES.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, lo, hi)   # 多段范围取并集
        ratio = cv2.countNonZero(mask) / total
        if ratio > best_ratio:
            best_name, best_ratio = name, ratio

    return (best_name, best_ratio) if best_ratio >= min_ratio else (None, best_ratio)


# ============================================================================
# 一·六、目标重识别（ReID）
# ============================================================================
# ⚠️ 为什么需要：ByteTrack 在目标被遮挡后重新出现时，会分配一个【新 ID】，
#    导致"跟随指定的人"跟丢。ReID 用外观特征（HSV 颜色直方图）在目标短暂
#    丢失后把它认回来，保持同一个逻辑 ID。
#
# 方案：轻量外观特征（HSV 分块直方图），纯 numpy/cv2，CPU 实时无压力，
#       且和上面的 HSV 颜色判定天然契合。架构可插拔 —— 以后要换深度特征
#       （OSNet 等）只改 extract() 一个函数。

class AppearanceReID:
    """
    基于 HSV 颜色直方图的目标重识别，负责给检测分配稳定的【逻辑 ID】。

    分层关系：
      ByteTrack ID —— 连续跟踪时的"过程 ID"，遮挡后可能变
      逻辑 ID     —— 本类输出的"身份 ID"，遮挡重出后保持不变
    """

    def __init__(self, lost_max=30, sim_thresh=0.62, hbins=8, sbins=8):
        self.lost_max = lost_max      # 丢失后保留多少帧用于重识别
        self.sim_thresh = sim_thresh  # 相似度阈值 0~1（越高越保守，越难认领）
        self.hbins, self.sbins = hbins, sbins

        self.track2logic = {}   # ByteTrack tid -> 逻辑 id
        self.features = {}      # 逻辑 id -> 外观特征（EMA 平滑后）
        self.lost = {}          # 逻辑 id -> 丢失记录（等待重识别）
        self.next_id = 1
        self.last_event = None  # 最近一次重识别事件 (逻辑id, 相似度)，供面板显示

    # ---- 特征提取（可插拔：换深度特征只改这里）----
    def extract(self, bgr_roi):
        """HSV 颜色直方图特征（H+S 两通道，上下两半拼接，保留一点空间信息）"""
        if bgr_roi is None or bgr_roi.size == 0:
            return None
        hsv = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2HSV)
        h, w = hsv.shape[:2]
        if h < 4 or w < 4:
            return None
        top = hsv[0:h // 2, :]
        bot = hsv[h // 2:, :]
        feat = None
        for half in (top, bot):
            hist = cv2.calcHist([half], [0, 1], None,
                                [self.hbins, self.sbins], [0, 180, 0, 256])
            cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
            flat = hist.flatten()
            feat = flat if feat is None else np.concatenate([feat, flat])
        return feat.astype(np.float32)

    @staticmethod
    def similarity(f1, f2):
        """相似度 0~1（1 = 完全相同），用巴氏系数。"""
        if f1 is None or f2 is None:
            return 0.0
        d = cv2.compareHist(f1, f2, cv2.HISTCMP_BHATTACHARYYA)
        return max(0.0, 1.0 - float(d))

    @staticmethod
    def _ema(old, new, alpha=0.35):
        """指数滑动平均，让特征随光照/姿态缓慢适应"""
        if old is None:
            return new
        return alpha * new + (1.0 - alpha) * old

    def assign(self, tid, roi, cx, cy, w, h, frame_idx):
        """
        给一个检测分配逻辑 id。返回 (逻辑id, 是否发生重识别认领)。
        """
        feat = self.extract(roi)
        self.last_event = None

        # 1) ByteTrack tid 连续跟踪正常 —— 直接沿用
        if tid is not None and tid in self.track2logic:
            logic = self.track2logic[tid]
            self.features[logic] = self._ema(self.features.get(logic), feat)
            self.lost.pop(logic, None)
            return logic, False

        # 2) 在丢失池里做 外观 + 位置 + 时间 三重匹配
        best_logic, best_sim = None, self.sim_thresh
        for logic, rec in list(self.lost.items()):
            if frame_idx - rec['frame'] > self.lost_max:
                self.lost.pop(logic, None)   # 太久没回来，放弃
                continue
            gate = rec['pos_gate']
            if abs(cx - rec['cx']) > gate or abs(cy - rec['cy']) > gate:
                continue
            sim = self.similarity(rec['feature'], feat)
            if sim > best_sim:
                best_logic, best_sim = logic, sim

        if best_logic is not None:
            # 认领回来 —— 这就是"重识别"
            logic = best_logic
            self.features[logic] = self._ema(self.features[logic], feat)
            self.lost.pop(logic, None)
            if tid is not None:
                self.track2logic[tid] = logic
            self.last_event = (logic, best_sim)
            return logic, True

        # 3) 全新身份
        logic = self.next_id
        self.next_id += 1
        self.features[logic] = feat
        if tid is not None:
            self.track2logic[tid] = logic
        return logic, False

    def register_lost(self, logic, cx, cy, w, h, frame_idx):
        """目标丢失，进缓冲池等待重识别。位置门控按目标大小自适应。"""
        if logic not in self.features:
            return
        self.lost[logic] = {
            'feature': self.features[logic],
            'cx': cx, 'cy': cy,
            'pos_gate': max(150.0, 2.0 * max(w, h)),
            'frame': frame_idx,
        }


# ============================================================================
# 二、主程序
# ============================================================================

def parse_args():
    p = argparse.ArgumentParser(description="任务5/6 视觉 demo（检测+跟踪+颜色）")
    p.add_argument('--source', default='0',
                   help='视频源：摄像头编号(0) 或 视频文件路径')
    p.add_argument('--model', default='yolov8n.pt', help='模型权重')
    p.add_argument('--target', default='person',
                   help='目标类别名（COCO 类名，如 person/bottle/cup）；空字符串=不筛选')
    p.add_argument('--target-color', default='',
                   help='要求的目标颜色 red/green/blue；默认空=不筛颜色')
    p.add_argument('--conf', type=float, default=0.4, help='检测置信度阈值')
    p.add_argument('--imgsz', type=int, default=320,
                   help='推理输入尺寸。CPU 上建议 320；RK3588 NPU 上可用 640')
    p.add_argument('--flip', default='h', choices=FLIP_ORDER,
                   help='画面翻转：none=不翻 / h=左右镜像 / v=上下 / hv=180°。'
                        '默认 h（纠正镜像画面）；运行中按 f 键可循环切换')
    p.add_argument('--serial', default=None,
                   help='串口设备，如 /dev/ttyUSB0（不给则只在屏幕显示）')
    p.add_argument('--baud', type=int, default=115200, help='串口波特率')
    p.add_argument('--no-reid', action='store_true',
                   help='关闭重识别（回退到 ByteTrack 原始 ID）')
    p.add_argument('--reid-sim', type=float, default=0.62,
                   help='重识别外观相似度阈值 0~1（越高越保守）')
    p.add_argument('--reid-lost', type=int, default=30,
                   help='目标丢失后保留多少帧用于重识别')
    p.add_argument('--no-show', action='store_true', help='不弹窗（无显示器环境）')
    p.add_argument('--frames', type=int, default=0, help='最多处理多少帧后退出（0=不限）')
    return p.parse_args()


def open_serial(port, baud):
    """打开串口；失败不影响主流程（只是不下发）"""
    if not port:
        return None
    try:
        import serial
        ser = serial.Serial(port, baud, timeout=0.1)
        print(f"[串口] 已连接 {port} @ {baud}")
        return ser
    except Exception as e:
        print(f"[串口] 打开失败（继续运行，不下发）：{e}")
        return None


def send_target(ser, tid, cls_name, color_name, cx, cy, w, h, frame_w):
    """
    把目标信息下发给 FPGA（任务6 的视觉→运动链路）。

    ⚠️ 本 demo 保留早期占位输出；协议 v2 闭环请使用 task6_mission.py。
       当前格式（ASCII，一行）：  $T,<id>,<cls>,<color>,<cx>,<cy>,<w>,<h>,<bearing>!
       其中 bearing = 目标相对画面中心的偏角，-1.0(最左) ~ +1.0(最右)
    """
    if ser is None:
        return
    bearing = (cx - frame_w / 2.0) / (frame_w / 2.0)
    line = f"$T,{tid},{cls_name},{color_name or 'none'},{cx},{cy},{w},{h},{bearing:.3f}!\n"
    try:
        ser.write(line.encode('ascii'))
    except Exception as e:
        print(f"[串口] 发送失败：{e}")


def main():
    args = parse_args()

    # ---- 加载模型 ----
    print(f"[模型] 加载 {args.model} ...")
    model = YOLO(args.model)
    print(f"[模型] 就绪，共 {len(model.names)} 个类别")

    # ---- 视频源 ----
    src = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        sys.exit(f"无法打开视频源：{args.source}")

    # ---- 串口 ----
    ser = open_serial(args.serial, args.baud)

    target_cls = args.target.strip()
    target_color = args.target_color.strip()

    # ---- 重识别（ReID）----
    reid = None
    if not args.no_reid:
        reid = AppearanceReID(lost_max=args.reid_lost, sim_thresh=args.reid_sim)
        print(f"[ReID] 已启用（外观特征，相似度阈值={args.reid_sim}，"
              f"丢失保留={args.reid_lost} 帧）")
    active_state = None   # 当前锁定目标的信息，供丢失后重识别用

    print(f"[配置] 目标类别='{target_cls or '任意'}'  目标颜色='{target_color or '任意'}'")
    print(f"[配置] imgsz={args.imgsz}  conf={args.conf}  画面翻转={args.flip}")
    print("[提示] 按 q 退出，按 f 切换画面翻转\n")

    # 翻转方式运行中可以按 f 循环切换，所以用可变变量而不是直接读 args
    flip_name = args.flip
    flip_code = FLIP_CODES[flip_name]
    flip_idx = FLIP_ORDER.index(flip_name)

    n = 0
    t_prev = time.time()
    fps = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("读取帧失败，结束")
                break

            # 先翻转再检测 —— 保证检测坐标、画框、串口下发的偏角
            # 全都和屏幕上看到的画面一致
            if flip_code is not None:
                frame = cv2.flip(frame, flip_code)

            n += 1
            frame_h, frame_w = frame.shape[:2]

            all_dets = []   # 本帧全部检出（不管是否匹配目标），用于诊断显示

            # ---- 检测 + 跟踪（persist=True 让 ID 跨帧稳定）----
            results = model.track(
                source=frame,
                persist=True,
                imgsz=args.imgsz,
                conf=args.conf,
                verbose=False,
            )[0]

            found_targets = []
            if reid is not None:
                reid.last_event = None   # 每帧开头清空重识别事件

            if results.boxes is not None and len(results.boxes) > 0:
                boxes = results.boxes
                # 有跟踪 ID 就用，没有（没跟到）就填 None
                ids = boxes.id.int().cpu().tolist() if boxes.id is not None else [None] * len(boxes)

                for box, tid in zip(boxes, ids):
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    cls_id = int(box.cls)
                    cls_name = model.names[cls_id]
                    conf = float(box.conf)

                    # 边界裁剪，防止 ROI 越界
                    x1c, y1c = max(0, x1), max(0, y1)
                    x2c, y2c = min(frame_w, x2), min(frame_h, y2)
                    roi = frame[y1c:y2c, x1c:x2c]

                    # ---- 关键一步：在检测框里判颜色 ----
                    color_name, color_ratio = dominant_color(roi)

                    # ---- 判断是不是我们要找的目标 ----
                    cls_ok = (not target_cls) or (cls_name == target_cls)
                    color_ok = (not target_color) or (color_name == target_color)

                    is_target = cls_ok and color_ok
                    all_dets.append({
                        'cls': cls_name, 'color': color_name, 'cls_ok': cls_ok,
                    })
                    if is_target:
                        cx = (x1 + x2) // 2
                        cy = (y1 + y2) // 2
                        w, h = x2 - x1, y2 - y1
                        # ReID：分配稳定逻辑 id（遮挡重出后不变）
                        if reid is not None:
                            logic_id, reacquired = reid.assign(
                                tid, roi, cx, cy, w, h, n)
                        else:
                            logic_id, reacquired = tid, False
                        found_targets.append({
                            'id': logic_id, 'cls': cls_name, 'color': color_name,
                            'cx': cx, 'cy': cy, 'w': w, 'h': h,
                            'reacquired': reacquired,
                        })

                    # ---- 画框 ----
                    if is_target:
                        box_color = (0, 255, 255)      # 目标：黄色高亮
                        thickness = 3
                    else:
                        box_color = COLOR_DISPLAY.get(color_name, (160, 160, 160))
                        thickness = 2

                    cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, thickness)

                    label = f"#{tid} {cls_name} {conf:.2f}"
                    if color_name:
                        label += f" [{color_name} {color_ratio:.0%}]"
                    if is_target:
                        label = ">>> " + label
                    cv2.putText(frame, label, (x1, max(20, y1 - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, box_color, 2)

            # ---- 状态面板：一眼看出是"没检出"还是"检出了但不匹配" ----
            panel = []

            # 第1行：锁定状态（没锁定时不占位，让第2、3行直接说明原因）
            if found_targets:
                t = max(found_targets, key=lambda x: x['w'] * x['h'])
                active_state = {'id': t['id'], 'cx': t['cx'], 'cy': t['cy'],
                                'w': t['w'], 'h': t['h']}
                bearing = (t['cx'] - frame_w / 2.0) / (frame_w / 2.0)
                panel.append((f"锁定 #{t['id']} {t['cls']}({t['color']}) "
                              f"@({t['cx']},{t['cy']})  偏角 {bearing:+.2f}",
                              (0, 255, 255)))
                send_target(ser, t['id'], t['cls'], t['color'],
                            t['cx'], t['cy'], t['w'], t['h'], frame_w)
                print(f"[锁定] {panel[0][0]}")
            else:
                # 目标丢失 → 交给 ReID 记进缓冲池，等它重新出现时认领
                if reid is not None and active_state is not None:
                    reid.register_lost(active_state['id'], active_state['cx'],
                                       active_state['cy'], active_state['w'],
                                       active_state['h'], n)
                    active_state = None

            # 第2行：本帧到底检出了什么
            if all_dets:
                parts = []
                for d in all_dets:
                    s = d['cls']
                    if target_color:
                        s += f"({d['color'] or '无色'})"
                    parts.append(s)
                shown = ", ".join(parts[:6])
                if len(parts) > 6:
                    shown += f" …另{len(parts) - 6}个"
                panel.append((f"本帧检出 {len(all_dets)} 个: {shown}", (0, 255, 0)))
            else:
                panel.append(("本帧检出 0 个 —— YOLO 什么都没看到（查光线/距离/镜头）",
                              (0, 165, 255)))

            # 第3行：没锁定时，直接指出卡在哪个条件上
            if not found_targets and all_dets:
                if target_cls and not any(d['cls_ok'] for d in all_dets):
                    panel.append((f"→ 有检出，但没有 '{target_cls}' 这一类",
                                  (0, 165, 255)))
                elif target_color:
                    panel.append((f"→ 类别对了，但颜色不是 '{target_color}'",
                                  (0, 165, 255)))

            # 第4行：重识别事件（目标遮挡重出后认领回来）
            if reid is not None and reid.last_event is not None:
                logic, sim = reid.last_event
                panel.append((f"重识别找回 #{logic}（相似度 {sim:.0%}）",
                              (0, 200, 255)))

            draw_text_cn(frame, panel, origin=(8, 8), font_size=22)

            # ---- FPS ----
            dt = time.time() - t_prev
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps else 1.0 / dt
            t_prev = time.time()
            cv2.putText(frame, f"FPS {fps:.1f}  imgsz={args.imgsz}  flip={flip_name}",
                        (10, frame_h - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            if not args.no_show:
                cv2.imshow("Task5/6 Vision Demo", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                if key == ord('f'):
                    # 循环切换翻转方式，不用改参数重启
                    flip_idx = (flip_idx + 1) % len(FLIP_ORDER)
                    flip_name = FLIP_ORDER[flip_idx]
                    flip_code = FLIP_CODES[flip_name]
                    print(f"[翻转] 切换为 {flip_name}")

            if args.frames and n >= args.frames:
                print(f"\n[完成] 已处理 {n} 帧，平均 {fps:.1f} FPS")
                break

    except KeyboardInterrupt:
        print("\n[中断] 用户停止")
    finally:
        cap.release()
        if ser:
            ser.close()
        if not args.no_show:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
