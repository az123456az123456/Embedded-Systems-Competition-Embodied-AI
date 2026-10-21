#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
选题三 任务6 —— 智能任务执行 状态机 Demo
================================================================
对应赛题任务6：机器人自主执行"指定目标搜索 → 靠近 → 到达"的完整闭环。

和 vision_demo.py 的区别：
  vision_demo.py 只做【感知】：检测 + 跟踪 + 颜色，是开环（看到 → 打印）。
  本脚本做【执行】：把感知结果喂给一个状态机，产出运动指令（转向/前进/停车），
  把"散装能力"串成"自主任务"。

架构借鉴 open-source 项目 "Security Smart Body"（/tmp/security_repo/，评审时参考）：
  1. 状态机 —— 借鉴其 RulesEngine._evaluate_duration_rule 的【时序确认】模式：
     每个状态转换必须"连续 N 帧满足"才触发，单帧抖动不切换、短暂消失不误报。
  2. 重识别 —— 借鉴其 TrackRecovery 的【空间距离恢复】思路，和现有 HSV 外观特征
     融合成"外观 + 空间"双特征，比 vision_demo 里"外观阈值 + 位置硬门控"更平滑。

状态机（SEARCH → LOCK → ALIGN → APPROACH → ARRIVE）：
  SEARCH   原地旋转扫描，找目标
  LOCK     连续 N 帧确认目标（消抖）
  ALIGN    转向使偏角 |bearing| 归零
  APPROACH 前进靠近，直到"足够近"
  ARRIVE   停车 + 上报，任务完成
  任意状态：目标连续 M 帧丢失 → 回到 SEARCH

测距：默认优先使用 FPGA 串口返回的超声波 STATUS；数据失效时自动回退到框高估算。
      可用 --distance-source fpga 强制要求真实测距。

运动接口：协议 v2 SET_TWIST/STOP，定义见 ../fpga/通信协议.md。

用法：
  python task6_mission.py                     # 找人（默认）
  python task6_mission.py --target bottle --target-color red   # 找红色瓶子
  python task6_mission.py --no-show --frames 200               # 无窗口跑 200 帧
  python task6_mission.py --serial /dev/ttyUSB0 --baud 115200  # 真实/虚拟串口闭环
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

# 复用 vision_demo 的公共部分：颜色判定、中文渲染、画面翻转
from vision_demo import (
    COLOR_DISPLAY, FLIP_CODES, FLIP_ORDER,
    dominant_color, draw_text_cn,
)
from fpga_link import FpgaLink
from fpga_protocol import Mode
from semantic_parser import parse_instruction


# ============================================================================
# 一、时序门控 —— 借鉴 RulesEngine._evaluate_duration_rule 的"连续确认"思想
# ============================================================================
# 原版逻辑（Security 仓库 rules_engine.py）：
#   条件为真则累计时长，达到阈值才触发一次；条件变假则清零，下次从头累计。
# 这里把"按时间"抽象成"按帧"，用在状态机转换上，避免单帧抖动导致状态乱跳。

class TemporalGate:
    """一个必须【连续满足 required 帧】才"确认"的条件门。

    每帧调用 update(active)：
      active=True  计数器 +1，达到 required 返回 True（只在这一帧触发）
      active=False 计数器清零，返回 False
    """

    def __init__(self, required_frames):
        if required_frames < 1:
            raise ValueError("required_frames 至少为 1")
        self.required = required_frames
        self.count = 0

    def update(self, active):
        if not active:
            self.count = 0
            return False
        self.count += 1
        return self.count >= self.required

    def reset(self):
        self.count = 0

    @property
    def progress(self):
        """当前累计 / 要求，供面板显示进度条"""
        return min(1.0, self.count / self.required)


# ============================================================================
# 二、状态机 —— 任务6 的"自主执行"骨架
# ============================================================================

class MissionState:
    SEARCH = "SEARCH"
    LOCK = "LOCK"
    ALIGN = "ALIGN"
    APPROACH = "APPROACH"
    ARRIVE = "ARRIVE"


class MissionStateMachine:
    """SEARCH → LOCK → ALIGN → APPROACH → ARRIVE 状态机（响应式）。

    SEARCH / LOCK 负责"发现并确认目标"，之后进入【跟踪簇】(ALIGN/APPROACH/ARRIVE)。
    跟踪簇里每一帧都按当前 (偏角, 是否够近) 重新决定下一步，而不是一次走完就停：

      SEARCH     连续 conf_frames 帧有目标 → LOCK（消抖）
      LOCK       瞬时态，立即并入跟踪簇
      跟踪簇内    对准 + 够近 同时满足(连续 arrive_frames 帧) → ARRIVE 停车
                 没对准                      → ALIGN 转向
                 对准但未够近                → APPROACH 前进
      ARRIVE     对准/够近 任一打破(连续 arrive_frames 帧) → 回到 ALIGN/APPROACH
      任意态     连续 lost_frames 帧丢目标 → SEARCH

    关键：ARRIVE 不是终点，且到达必须"对准 + 够近"同时成立——目标偏了会转向、
    跑了会追，绝不会在没对准的情况下停车。
    """

    def __init__(self, conf_frames=5, arrive_frames=5,
                 lost_frames=15, align_thresh=0.2):
        self.state = MissionState.SEARCH
        self.align_thresh = align_thresh

        self._found = TemporalGate(conf_frames)   # 搜索→锁定 确认
        self._near = TemporalGate(arrive_frames)  # 靠近→到达 确认
        self._far = TemporalGate(arrive_frames)   # 到达→重新靠近 确认（目标跑远）
        self._lost = TemporalGate(lost_frames)    # 丢失→回搜索 确认

    def step(self, obs):
        """输入一帧观测，返回 (新状态, 动作名, 说明文字)。

        obs: {
            'has_target': bool,       # 本帧是否锁定到目标
            'target': dict | None,    # 目标信息（id/cls/color/cx/cy/w/h）
            'bearing': float,         # 偏角 -1(最左) ~ +1(最右)
            'near': bool,             # 是否"足够近"（FPGA 距离优先，框高回退）
        }
        """
        lost_confirmed = self._lost.update(not obs["has_target"])

        # ---- SEARCH：找目标，连续 N 帧确认才锁定 ----
        if self.state == MissionState.SEARCH:
            if self._found.update(obs["has_target"]):
                self.state = MissionState.LOCK
                return self.state, "LOCK", f"确认锁定目标 #{obs['target']['id']}"
            return self.state, "ROTATE", ""

        # ---- 目标连续丢失 → 回搜索 ----
        if lost_confirmed:
            self.state = MissionState.SEARCH
            self._reset_transitions()
            return self.state, "ROTATE", "目标丢失，回到搜索"

        # LOCK 是瞬时态，立即并入跟踪簇
        if self.state == MissionState.LOCK:
            self.state = MissionState.ALIGN

        has = obs["has_target"]
        near = obs["near"]
        aligned = has and abs(obs["bearing"]) <= self.align_thresh
        arrive_cond = aligned and near   # 对准 + 够近 同时满足才算"到达"

        # ---- ARRIVE：已到达，条件打破（没对准 或 跑远）才离开 ----
        if self.state == MissionState.ARRIVE:
            if arrive_cond:
                self._far.update(False)
                return self.state, "STOP", "已到达，保持停车"
            if self._far.update(True):
                self._near.reset()   # 重新进入 ARRIVE 要重新确认
                if has and not aligned:
                    self.state = MissionState.ALIGN
                    return self.state, "TURN", "目标偏了（未对准），重新对齐"
                self.state = MissionState.APPROACH
                return self.state, "FORWARD", "目标离开，重新靠近"
            return self.state, "STOP", ""   # 离开确认累计中，先停

        # ---- 未到达（ALIGN / APPROACH）：对准+够近才停，否则没对准先转 ----
        if arrive_cond:
            self._far.update(False)
            if self._near.update(True):
                self.state = MissionState.ARRIVE
                return self.state, "STOP", "对准且够近，停车，任务完成"
            return self.state, "STOP", ""   # 到达确认累计中，先停

        self._near.update(False)
        if has and not aligned:
            self.state = MissionState.ALIGN
            return self.state, "TURN", ""
        self.state = MissionState.APPROACH
        return self.state, "FORWARD", ""

    def _reset_transitions(self):
        for gate in (self._found, self._near, self._far):
            gate.reset()

    def gate_progress(self):
        """当前状态相关的门控进度，供面板画进度条"""
        if self.state == MissionState.SEARCH:
            return ("搜索确认", self._found.progress)
        if self.state == MissionState.ARRIVE:
            return ("离开确认", self._far.progress)
        return ("到达确认", self._near.progress)


# ============================================================================
# 三、重识别（ReID）升级版 —— 外观 + 空间 双特征
# ============================================================================
# 外观特征默认用深度特征（ResNet50-IBN-a 2048 维，deep_reid.py），
# 加载失败时回退 HSV 颜色直方图兜底。
# 空间特征借鉴 Security 仓库 TrackRecovery：目标丢失后缓存 N 秒，用【空间距离】
# （欧氏距离最近者优先）辅助把新检测认领回旧 ID。
# 加权匹配：score = w_app * 外观相似度(余弦) + w_spa * 空间相似度(归一化距离)

class MissionReID:
    """外观(深度特征/HSV 兜底) + 空间(欧氏距离) 双特征重识别。

    分层关系（同 vision_demo）：
      ByteTrack ID —— 连续跟踪的"过程 ID"，遮挡后可能变
      逻辑 ID     —— 本类输出的"身份 ID"，遮挡重出后保持不变
    """

    def __init__(self, lost_max=300, sim_thresh=0.5,
                 w_app=0.75, w_spa=0.25, hbins=12, sbins=8,
                 deep_weight=None):
        self.lost_max = lost_max
        self.sim_thresh = sim_thresh
        self.w_app, self.w_spa = w_app, w_spa
        self.hbins, self.sbins = hbins, sbins

        # 深度特征（ResNet50-IBN-a 2048 维）优先；加载失败回退 HSV 直方图
        self.deep = None
        self.use_deep = False
        if deep_weight and os.path.exists(deep_weight):
            try:
                from deep_reid import DeepReID
                self.deep = DeepReID(deep_weight)
                self.use_deep = True
            except Exception as e:
                print(f"[ReID] 深度特征加载失败，回退 HSV：{e}")
        elif deep_weight:
            print(f"[ReID] 未找到权重 {deep_weight}，回退 HSV 直方图")

        self.track2logic = {}   # ByteTrack tid -> 逻辑 id
        self.features = {}      # 逻辑 id -> 外观特征（最近一版"完整全身适中"特征）
        self.lost = {}          # 逻辑 id -> 丢失记录（外观+位置，等认领）
        self.recent = {}        # 逻辑 id -> (cx, cy, frame_idx) 最近出现位置，无 tid 时兜底
        self.spa_buf = 5        # 空间连续性缓冲：最近 N 帧内出现过的 id 才算"还在"
        self._frame = -1        # 上一帧号，用于每帧清空"已认领"集合
        self._claimed = set()   # 本帧已被认领的逻辑 id（一帧内一个 id 只能给一个检测）
        self.next_id = 1
        self.last_event = None   # 最近一次认领事件 (逻辑id, 综合得分)
        self.last_debug = None   # 最近一次"没认领"的原因（现场排查用）

    # ---- 外观特征（可插拔：换特征只改这里）----
    def extract(self, bgr_roi):
        """外观特征：深度特征（ResNet50-IBN-a 2048 维）优先，HSV 直方图兜底。"""
        if self.use_deep:
            return self.deep.extract(bgr_roi)
        return self._extract_hsv(bgr_roi)

    def _extract_hsv(self, bgr_roi):
        """HSV 颜色直方图特征（深度特征不可用时的兜底）：中心裁剪 + 上/中/下三分带。"""
        if bgr_roi is None or bgr_roi.size == 0:
            return None
        h, w = bgr_roi.shape[:2]
        if h < 12 or w < 12:
            return None
        x0, x1 = int(w * 0.12), int(w * 0.88)
        y0, y1 = int(h * 0.08), int(h * 0.92)
        crop = bgr_roi[y0:y1, x0:x1]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        hh = hsv.shape[0]
        bands = [hsv[0:hh // 3, :], hsv[hh // 3:2 * hh // 3, :], hsv[2 * hh // 3:, :]]
        feat = None
        for band in bands:
            hist = cv2.calcHist([band], [0, 1], None,
                                [self.hbins, self.sbins], [0, 180, 0, 256])
            cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
            flat = hist.flatten()
            feat = flat if feat is None else np.concatenate([feat, flat])
        return feat.astype(np.float32)

    def _app_similarity(self, f1, f2):
        """外观相似度：深度特征用余弦 [-1,1]，HSV 兜底用巴氏系数 [0,1]。"""
        if f1 is None or f2 is None:
            return -1.0 if self.use_deep else 0.0
        if self.use_deep:
            return self.deep.cosine(f1, f2)
        d = cv2.compareHist(f1, f2, cv2.HISTCMP_BHATTACHARYYA)
        return max(0.0, 1.0 - float(d))

    @staticmethod
    def _spa_similarity(cx, cy, ox, oy, gate):
        """空间相似度 0~1：欧氏距离落在 gate 内线性衰减，gate 外为 0（借鉴 TrackRecovery）"""
        if gate <= 0:
            return 0.0
        dist = float(np.hypot(cx - ox, cy - oy))
        return max(0.0, 1.0 - dist / gate)

    @staticmethod
    def _good_size(h, frame_h):
        """框高占画面 25%~75% 视为"完整全身、大小适中"——ReID 模型最稳区间。

        特写（头+躯干，超出 Market1501 训练分布）和远处极小框（糊）都会让特征
        跑偏，把基准特征污染，导致"离开再回来"的余弦掉到 0.1~0.3。
        """
        return frame_h > 0 and 0.25 <= h / frame_h <= 0.75

    def _update_feature(self, logic, feat, h, frame_h):
        """只在"完整全身适中"时把特征覆盖为最近一版。

        不用 EMA、不用"最大框"：离开画面前的尺度最接近回来时的尺度，所以直接
        存最近一版合适特征，比存最清晰的特写更匹配回来时的全身视图。
        """
        if feat is None:
            return
        if self._good_size(h, frame_h):
            self.features[logic] = feat

    def _touch(self, logic, cx, cy, frame_idx):
        """记录该逻辑 id 最近出现的位置/帧号，供无 tid 时的空间连续性兜底。"""
        self.recent[logic] = (cx, cy, frame_idx)

    def _spa_recent_match(self, cx, cy, frame_idx, w, h):
        """无 ByteTrack tid 时，按空间连续性找"最近刚出现过"的 id。

        同一人在相邻帧里位置几乎不变，这一步是 ByteTrack 失效时的轻量兜底跟踪：
        避免每次都去跑脆弱的外观匹配，从而对同一个人反复分配新 id。
        门控取一个身位（max(w,h)）：一帧内同一人移动不会超过一个身位。
        """
        best, best_d = None, float('inf')
        gate = max(w, h)
        for logic, (px, py, pf) in list(self.recent.items()):
            if frame_idx - pf > self.spa_buf:
                self.recent.pop(logic, None)   # 太久没出现，不算"还在"
                continue
            if logic in self._claimed:
                continue   # 本帧这个 id 已经分配给别的检测了，不能撞 id
            d = float(np.hypot(cx - px, cy - py))
            if d <= gate and d < best_d:
                best, best_d = logic, d
        return best

    def assign(self, tid, roi, cx, cy, w, h, frame_idx, frame_h=0):
        """给一个检测分配逻辑 id。返回 (逻辑id, 是否认领)。"""
        self.last_event = None
        self.last_debug = None

        # 每帧清空"已认领"集合：同一个逻辑 id 一帧内只能分配给一个检测，防止多人撞 id
        if frame_idx != self._frame:
            self._frame = frame_idx
            self._claimed.clear()

        # 1) ByteTrack tid 连续跟踪正常 —— 直接沿用。
        #    只在目标处于"完整全身、大小适中"时提取并更新特征：既省算力，
        #    又保证基准特征是完整全身（特写/小框会污染基准，拉低回来时的余弦）。
        if tid is not None and tid in self.track2logic:
            logic = self.track2logic[tid]
            if self._good_size(h, frame_h):
                self._update_feature(logic, self.extract(roi), h, frame_h)
            self.lost.pop(logic, None)
            self._touch(logic, cx, cy, frame_idx)
            return logic, False

        # 1.5) ByteTrack 没跟上（tid 为 None）—— 先按空间连续性认领。
        #      连续可见的人即使没有 tid 也能稳住同一个 id，不再每帧换新 id。
        logic = self._spa_recent_match(cx, cy, frame_idx, w, h)
        if logic is not None:
            self._claimed.add(logic)
            if self._good_size(h, frame_h):
                self._update_feature(logic, self.extract(roi), h, frame_h)
            self.lost.pop(logic, None)
            if tid is not None:
                self.track2logic[tid] = logic
            self._touch(logic, cx, cy, frame_idx)
            return logic, False

        # 只有 tid 是新的（新身份 or 遮挡重出）才跑外观特征
        feat = self.extract(roi)

        # 2) 丢失池里做 外观 + 空间 加权匹配
        best_logic, best_score = None, self.sim_thresh
        best_app, best_spa = 0.0, 0.0
        top_score, top_app, top_spa = -1e9, 0.0, 0.0   # 实际最高分候选（诊断用）
        n_expired = 0
        for logic, rec in list(self.lost.items()):
            if logic in self._claimed:
                continue   # 本帧已被认领，跳过
            if frame_idx - rec['frame'] > self.lost_max:
                self.lost.pop(logic, None)   # 太久没回来，放弃
                n_expired += 1
                continue
            app = self._app_similarity(rec['feature'], feat)
            spa = self._spa_similarity(cx, cy, rec['cx'], rec['cy'], rec['gate'])
            score = self.w_app * app + self.w_spa * spa
            if score > top_score:
                top_score, top_app, top_spa = score, app, spa
            if score > best_score:
                best_logic, best_score = logic, score
                best_app, best_spa = app, spa

        if best_logic is not None:
            # 认领回来 —— 这就是"重识别"
            logic = best_logic
            self._claimed.add(logic)
            self._update_feature(logic, feat, h, frame_h)
            self.lost.pop(logic, None)
            if tid is not None:
                self.track2logic[tid] = logic
            self._touch(logic, cx, cy, frame_idx)
            self.last_event = (logic, best_score)
            print(f"[ReID] 找回 #{logic}：外观{best_app:.2f} 空间{best_spa:.2f} "
                  f"综合{best_score:.2f} ≥ 阈值{self.sim_thresh:.2f}")
            return logic, True

        # 3) 全新身份 —— 若刚才有候选却没能认领，记下原因方便现场排查。
        #    注意用 top_*（实际最高分候选），而不是 best_*（没命中时是阈值初值）。
        if n_expired and not self.lost:
            self.last_debug = (f"重识别未认领: 记录已过期（离开超 {self.lost_max} 帧，"
                               f"可调大 --reid-lost）")
            print(f"[ReID] 匹配失败：记录已过期（> {self.lost_max} 帧），可调大 --reid-lost")
        elif self.lost:
            self.last_debug = (f"重识别未认领: 外观{top_app:.2f} 空间{top_spa:.2f} "
                               f"综合{top_score:.2f} < {self.sim_thresh:.2f}（可调低 --reid-sim）")
            print(f"[ReID] 匹配失败：外观{top_app:.2f} 空间{top_spa:.2f} "
                  f"综合{top_score:.2f} < 阈值{self.sim_thresh:.2f}（可调低 --reid-sim）")

        # 认领失败 → 新身份。多目标场景下绝不能"沿用最近活跃 id"，否则新出现的人
        # 会和仍在画面里的人撞 id，导致跟随对象串号。真正的新人就是新 id。
        logic = self.next_id
        self.next_id += 1
        self._claimed.add(logic)
        if feat is not None:
            self.features[logic] = feat   # 首见必存（哪怕框不合适），先有一个基准
        if tid is not None:
            self.track2logic[tid] = logic
        self._touch(logic, cx, cy, frame_idx)
        print(f"[ReID] 新身份 #{logic}（tid {tid}）")
        return logic, False

    def register_lost(self, logic, cx, cy, w, h, frame_idx):
        """目标丢失，进缓冲池等待重识别。空间门控按目标大小自适应。"""
        if logic not in self.features:
            return
        self.lost[logic] = {
            'feature': self.features[logic],
            'cx': cx, 'cy': cy,
            'gate': max(150.0, 2.0 * max(w, h)),   # 空间门控（借鉴 TrackRecovery 的尺寸自适应）
            'frame': frame_idx,
        }


# ============================================================================
# 四、运动指令下发 —— FPGA 协议 v2
# ============================================================================

class MotionCommand:
    """把状态机产出的【语义动作】翻译成串口字节下发 FPGA。

    状态机只关心“做什么”，本类负责映射成协议 v2：
      ROTATE   -> SET_TWIST(v=0, w=rotate_rate)
      TURN     -> SET_TWIST(v=0, w=-bearing*turn_rate)
      FORWARD  -> SET_TWIST(v=forward_speed, w=0)
      STOP     -> STOP

    协议定义见 ../fpga/通信协议.md。
    """

    def __init__(self, link, forward_mm_s=300, turn_max_mrad_s=500,
                 rotate_mrad_s=350, log_interval_s=0.5,
                 log_delta_v=60, log_delta_w=60):
        self.link = link
        self.forward_mm_s = int(forward_mm_s)
        self.turn_max_mrad_s = int(turn_max_mrad_s)
        self.rotate_mrad_s = int(rotate_mrad_s)
        # 日志去抖：指令仍每帧发送，只有动作变化足够明显或超过间隔才打印。
        self.log_interval_s = max(0.05, float(log_interval_s))
        self.log_delta_v = int(log_delta_v)
        self.log_delta_w = int(log_delta_w)
        self._last_log_label = None
        self._last_log_v = 0
        self._last_log_w = 0
        self._last_log_time = 0.0

    def _maybe_log(self, label, v_mm_s, w_mrad_s):
        now = time.monotonic()
        same = (
            label == self._last_log_label
            and abs(v_mm_s - self._last_log_v) <= self.log_delta_v
            and abs(w_mrad_s - self._last_log_w) <= self.log_delta_w
        )
        if same and now - self._last_log_time < self.log_interval_s:
            return
        print(f"[动作] {label}: v={v_mm_s} mm/s, w={w_mrad_s} mrad/s")
        self._last_log_label = label
        self._last_log_v = v_mm_s
        self._last_log_w = w_mrad_s
        self._last_log_time = now

    def send_twist(self, v_mm_s, w_mrad_s, label):
        command = (int(v_mm_s), int(w_mrad_s))
        self._maybe_log(label, command[0], command[1])
        self.link.set_twist(command[0], command[1])

    def stop(self, label="STOP"):
        self._maybe_log(label, 0, 0)
        self.link.stop()

    def execute(self, action, obs):
        """把动作名映射成具体指令。bearing 来自 obs。"""
        if action == "ROTATE":
            self.send_twist(0, self.rotate_mrad_s, "ROTATE")
        elif action == "TURN":
            bearing = max(-1.0, min(1.0, float(obs.get("bearing", 0.0))))
            # bearing>0 表示目标在画面右侧；w<0 表示顺时针转向右侧。
            w = int(round(-bearing * self.turn_max_mrad_s))
            self.send_twist(0, w, "TURN")
        elif action == "FORWARD":
            self.send_twist(self.forward_mm_s, 0, "FORWARD")
        elif action == "STOP" or action == "LOCK":
            self.stop()


# ============================================================================
# 五、测距与目标选择
# ============================================================================

def is_near_bbox(box_h, frame_h, arrive_ratio=0.5):
    """占位测距：目标框高度占画面高度比例 >= arrive_ratio 视为"足够近"。

    仅在 FPGA 超声波数据不可用时作为回退，不能代替真实测距。
    """
    return frame_h > 0 and (box_h / frame_h) >= arrive_ratio


def is_near_distance(distance_cm, threshold_cm):
    """使用 FPGA 超声波距离判断目标是否足够近。"""

    return distance_cm is not None and distance_cm <= threshold_cm


def pick_follow_target(candidates, locked, frame_w):
    """从候选目标里挑一个"要跟随"的。

    多目标场景的关键：先跟随已锁定的身份（ReID 保证身份连续，离开再回来仍是
    同一个 id），锁定目标消失后才允许换人，避免画面里人多时跟随对象乱跳。
    换人时优先选最显眼的目标：面积大（通常最近）；面积接近时取更靠近画面中心的。
    """
    if locked is not None:
        for c in candidates:
            if c['id'] == locked['id']:
                return c
    # 锁定目标不在了：换一个新目标。面积最大优先，越居中越优先。
    return max(candidates, key=lambda c: (c['w'] * c['h'],
                                          -abs(c['cx'] - frame_w / 2.0)))


# ============================================================================
# 六、主程序
# ============================================================================

def parse_args():
    p = argparse.ArgumentParser(description="任务6 智能任务执行 状态机 demo")
    p.add_argument('--source', default='0', help='视频源：摄像头编号(0) 或 视频文件路径')
    p.add_argument('--model', default='yolov8n.pt', help='模型权重')
    p.add_argument('--target', default='person', help='目标类别（COCO 类名）；空=不筛')
    p.add_argument('--target-color', default='', help='目标颜色 red/green/blue；空=不筛')
    p.add_argument('--instruction', default=None,
                   help='中文指令，如 "找到红色水瓶"；会自动覆盖 --target/--target-color')
    p.add_argument('--conf', type=float, default=0.4, help='检测置信度阈值')
    p.add_argument('--imgsz', type=int, default=320, help='推理输入尺寸')
    p.add_argument('--flip', default='h', choices=FLIP_ORDER, help='画面翻转方式')
    p.add_argument('--serial', default=None, help='串口设备，如 /dev/ttyUSB0')
    p.add_argument('--baud', type=int, default=115200, help='串口波特率')
    p.add_argument('--heartbeat-ms', type=int, default=100,
                   help='协议 v2 心跳周期')
    p.add_argument('--watchdog-ms', type=int, default=500,
                   help='超过此时间没收到 STATUS，FPGA 自动停车')
    p.add_argument('--distance-source', choices=('auto', 'fpga', 'bbox'),
                   default='auto',
                   help='到达判定距离来源：自动回退/仅FPGA/仅框高')
    p.add_argument('--arrive-distance-cm', type=int, default=60,
                   help='FPGA 测距下的到达距离阈值')
    p.add_argument('--forward-speed', type=int, default=300,
                   help='前进线速度 mm/s')
    p.add_argument('--turn-rate', type=int, default=500,
                   help='最大转向角速度 mrad/s')
    p.add_argument('--rotate-rate', type=int, default=350,
                   help='搜索旋转角速度 mrad/s')
    p.add_argument('--action-log-interval', type=float, default=0.5,
                   help='动作日志去抖间隔（秒）')
    p.add_argument('--action-log-delta-v', type=int, default=60,
                   help='线速度变化超过该值才重新打印日志')
    p.add_argument('--action-log-delta-w', type=int, default=60,
                   help='角速度变化超过该值才重新打印日志')
    # 状态机参数
    p.add_argument('--conf-frames', type=int, default=5, help='锁定确认帧数')
    p.add_argument('--align-thresh', type=float, default=0.2,
                   help='偏角对齐阈值 0~1：|偏角| 小于此值才算"对准"（默认 0.2 ≈ 偏离画面中心 10%% 宽度）')
    p.add_argument('--arrive-frames', type=int, default=5, help='靠近确认帧数')
    p.add_argument('--arrive-ratio', type=float, default=0.5,
                   help='回退测距：框高/画面高 >= 此值视为"足够近"')
    p.add_argument('--lost-frames', type=int, default=15, help='连续丢失多少帧回搜索')
    # ReID 参数
    p.add_argument('--no-reid', action='store_true', help='关闭重识别')
    p.add_argument('--reid-sim', type=float, default=0.5,
                   help='ReID 综合匹配阈值（深度特征时为余弦阈值，越大越严）')
    p.add_argument('--reid-lost', type=int, default=300,
                   help='丢失后保留多少帧用于重识别（默认 300 ≈ 10~20 秒）')
    p.add_argument('--reid-weight', default=None,
                   help='ReID 深度特征权重路径（默认自动找脚本目录下 yolov8_reid/ReID_resnet50_ibn_a.pth）')
    p.add_argument('--no-show', action='store_true', help='不弹窗')
    p.add_argument('--frames', type=int, default=0, help='最多处理多少帧后退出（0=不限）')
    return p.parse_args()


def _resolve_model_path(model: str) -> str:
    """自动定位本机 YOLO 权重，避免误触发联网下载。"""

    if not model:
        return model
    if os.path.exists(model):
        return model
    if model != "yolov8n.pt":
        return model

    candidates = [
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "yolov8n.pt"),
        os.path.join(os.getcwd(), "yolov8n.pt"),
    ]
    current = os.path.dirname(os.path.abspath(__file__))
    while True:
        candidates.append(os.path.join(current, ".ultralytics", "yolov8n.pt"))
        parent = os.path.dirname(current)
        if parent == current:
            break
        current = parent
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return model


def main():
    args = parse_args()
    args.model = _resolve_model_path(args.model)

    # ---- 中文语义指令 -> 结构化意图 ----
    instruction_intent = None
    if args.instruction:
        try:
            instruction_intent = parse_instruction(args.instruction)
        except ValueError as exc:
            print(f"[语义] 解析失败：{exc}")
            return
        print(f"[语义] {args.instruction} -> "
              f"action={instruction_intent.action} "
              f"target={instruction_intent.target or '任意'} "
              f"color={instruction_intent.color or '任意'} "
              f"zone={instruction_intent.zone or '-'}")
        if instruction_intent.action == "STOP":
            print("[语义] 指令为停止，不启动视觉任务")
            return

    print(f"[模型] 加载 {args.model} ...")
    model = YOLO(args.model)
    print(f"[模型] 就绪，共 {len(model.names)} 个类别")

    src = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        sys.exit(f"无法打开视频源：{args.source}")

    link = FpgaLink(
        args.serial,
        baud=args.baud,
        heartbeat_ms=args.heartbeat_ms,
        watchdog_ms=args.watchdog_ms,
    )
    link.start()
    if link.connected:
        link.set_mode(Mode.VISION)
    motion = MotionCommand(
        link,
        forward_mm_s=args.forward_speed,
        turn_max_mrad_s=args.turn_rate,
        rotate_mrad_s=args.rotate_rate,
        log_interval_s=args.action_log_interval,
        log_delta_v=args.action_log_delta_v,
        log_delta_w=args.action_log_delta_w,
    )

    target_cls = args.target.strip()
    target_color = args.target_color.strip()
    if instruction_intent is not None:
        if instruction_intent.target:
            target_cls = instruction_intent.target
        if instruction_intent.color:
            target_color = instruction_intent.color
        if instruction_intent.action == "GOTO" and instruction_intent.zone:
            print(f"[语义] GOTO 区域 {instruction_intent.zone} 由任务4导航模块执行，任务6只负责目标识别")

    reid = None
    if not args.no_reid:
        reid_weight = args.reid_weight
        if reid_weight is None:
            reid_weight = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                'yolov8_reid', 'ReID_resnet50_ibn_a.pth')
        reid = MissionReID(lost_max=args.reid_lost, sim_thresh=args.reid_sim,
                           deep_weight=reid_weight)
        feat_name = '深度特征(ResNet50-IBN-a 2048维)' if reid.use_deep else 'HSV直方图兜底'
        print(f"[ReID] 已启用（{feat_name} + 空间双特征，阈值={args.reid_sim}）")

    sm = MissionStateMachine(
        conf_frames=args.conf_frames,
        arrive_frames=args.arrive_frames,
        lost_frames=args.lost_frames,
        align_thresh=args.align_thresh,
    )

    print(f"[配置] 目标='{target_cls or '任意'}' 颜色='{target_color or '任意'}'")
    print(f"[配置] imgsz={args.imgsz} conf={args.conf} flip={args.flip} "
          f"对齐阈值={args.align_thresh}")
    print(f"[控制] 距离源={args.distance_source} "
          f"到达阈值={args.arrive_distance_cm}cm")
    print(f"[状态机] SEARCH→LOCK→ALIGN→APPROACH→ARRIVE  "
          f"(锁{args.conf_frames}帧/靠近{args.arrive_frames}帧/丢{args.lost_frames}帧)")
    print("[提示] 按 q 退出，按 f 切换画面翻转\n")

    flip_name = args.flip
    flip_code = FLIP_CODES[flip_name]
    flip_idx = FLIP_ORDER.index(flip_name)

    locked = None      # 跨帧保存当前锁定目标 {'id','cx','cy','w','h'}
    n = 0
    n_notrack = 0      # 统计"有检测但 ByteTrack 没给 id"的帧数（诊断 tid=None）
    t_prev = time.time()
    fps = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("读取帧失败，结束")
                break

            if flip_code is not None:
                frame = cv2.flip(frame, flip_code)

            n += 1
            frame_h, frame_w = frame.shape[:2]

            # ---- 检测 + 跟踪 ----
            results = model.track(source=frame, persist=True,
                                  imgsz=args.imgsz, conf=args.conf,
                                  verbose=False)[0]

            if reid is not None:
                reid.last_event = None

            candidates = []   # 满足"类别+颜色"的候选目标
            all_dets = []     # 本帧全部检出（诊断用）

            if results.boxes is not None and len(results.boxes) > 0:
                boxes = results.boxes
                if boxes.id is None:
                    # ByteTrack 有检测却没吐出 id —— 这是 tid=None 的直接来源，
                    # 打印一次便于定位（此后靠 ReID 空间连续性兜底，id 仍能稳定）。
                    if n_notrack == 0:
                        print(f"[跟踪] 警告：第 {n} 帧起，检测到 {len(boxes)} 个框但 "
                              f"ByteTrack 没给 id（tid=None），可能原因：目标置信度"
                              f"偏低/间歇性检出。已由 ReID 空间连续性兜底。")
                    n_notrack += 1
                ids = boxes.id.int().cpu().tolist() if boxes.id is not None \
                    else [None] * len(boxes)

                for box, tid in zip(boxes, ids):
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    cls_id = int(box.cls)
                    cls_name = model.names[cls_id]
                    conf = float(box.conf)

                    x1c, y1c = max(0, x1), max(0, y1)
                    x2c, y2c = min(frame_w, x2), min(frame_h, y2)
                    roi = frame[y1c:y2c, x1c:x2c]

                    color_name, color_ratio = dominant_color(roi)

                    cls_ok = (not target_cls) or (cls_name == target_cls)
                    color_ok = (not target_color) or (color_name == target_color)
                    is_target = cls_ok and color_ok

                    all_dets.append({'cls': cls_name, 'color': color_name,
                                     'cls_ok': cls_ok})

                    logic_id = tid   # 默认：非目标就用 ByteTrack tid
                    if is_target:
                        cx = (x1 + x2) // 2
                        cy = (y1 + y2) // 2
                        w, h = x2 - x1, y2 - y1
                        if reid is not None:
                            logic_id, reacquired = reid.assign(
                                tid, roi, cx, cy, w, h, n, frame_h)
                        else:
                            logic_id, reacquired = tid, False
                        candidates.append({
                            'id': logic_id, 'cls': cls_name, 'color': color_name,
                            'cx': cx, 'cy': cy, 'w': w, 'h': h,
                            'reacquired': reacquired,
                        })

                    # ---- 画框 ----
                    box_color = (0, 255, 255) if is_target \
                        else COLOR_DISPLAY.get(color_name, (160, 160, 160))
                    cv2.rectangle(frame, (x1, y1), (x2, y2), box_color,
                                  3 if is_target else 2)
                    # 目标显示稳定的"逻辑 ID"（离开再回来不变），非目标显示 ByteTrack tid
                    show_id = logic_id if is_target else tid
                    label = f"#{show_id} {cls_name} {conf:.2f}"
                    if color_name:
                        label += f" [{color_name} {color_ratio:.0%}]"
                    if is_target:
                        label = ">>> " + label
                    cv2.putText(frame, label, (x1, max(20, y1 - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, box_color, 2)

            # ---- 选择要跟随的目标（多目标：先稳住已锁定的，丢了再换新）----
            if candidates:
                # 之前锁定的目标这一帧不在了 → 记进丢失池（可能是被遮挡，
                # 等它重新出现时由 ReID 认领回原 id，保持跟随对象不跳）。
                if (reid is not None and locked is not None
                        and all(c['id'] != locked['id'] for c in candidates)):
                    reid.register_lost(locked['id'], locked['cx'], locked['cy'],
                                       locked['w'], locked['h'], n)
                target = pick_follow_target(candidates, locked, frame_w)
                locked = {'id': target['id'], 'cx': target['cx'],
                          'cy': target['cy'], 'w': target['w'], 'h': target['h']}
                has_target = True
            else:
                target = None
                has_target = False
                # 画面里没人了：把锁定的目标记进丢失池等认领
                if reid is not None and locked is not None:
                    reid.register_lost(locked['id'], locked['cx'], locked['cy'],
                                       locked['w'], locked['h'], n)
                    locked = None

            # ---- 构造观测，驱动状态机 ----
            bearing = (target['cx'] - frame_w / 2.0) / (frame_w / 2.0) \
                if target else 0.0
            fpga_distance = link.distance_cm()
            distance_text = "无"
            if target and args.distance_source != 'bbox' and fpga_distance is not None:
                near = is_near_distance(fpga_distance, args.arrive_distance_cm)
                distance_text = f"FPGA {fpga_distance}cm"
            elif target and args.distance_source == 'bbox':
                near = is_near_bbox(target['h'], frame_h, args.arrive_ratio)
                distance_text = f"框高 {target['h'] / frame_h:.0%}"
            elif target and args.distance_source == 'fpga':
                near = False
                distance_text = "FPGA 无有效数据"
            elif target:
                near = is_near_bbox(target['h'], frame_h, args.arrive_ratio)
                distance_text = f"框高回退 {target['h'] / frame_h:.0%}"
            else:
                near = False

            obs = {'has_target': has_target, 'target': target,
                   'bearing': bearing, 'near': near}
            state, action, note = sm.step(obs)
            distance_required_missing = (
                target is not None
                and args.distance_source == 'fpga'
                and fpga_distance is None
            )
            if link.connected and link.watchdog_tripped:
                action = "STOP"
                note = "FPGA STATUS 超时，安全停车"
            elif distance_required_missing:
                action = "STOP"
                note = "FPGA 距离无效，安全停车"
            motion.execute(action, obs)

            # ---- 面板 ----
            panel = []

            # 第1行：状态机当前状态
            state_color = {
                MissionState.SEARCH: (200, 200, 200),
                MissionState.LOCK: (0, 200, 255),
                MissionState.ALIGN: (0, 255, 255),
                MissionState.APPROACH: (0, 255, 0),
                MissionState.ARRIVE: (0, 255, 128),
            }[state]
            flow = " → ".join([MissionState.SEARCH, MissionState.LOCK,
                               MissionState.ALIGN, MissionState.APPROACH,
                               MissionState.ARRIVE])
            panel.append((f"状态 [{state}]  动作 {action}", state_color))
            panel.append((flow, (120, 120, 120)))

            # 第2行：门控进度 + 说明
            gate_name, gate_prog = sm.gate_progress()
            if gate_name:
                bar = "█" * int(gate_prog * 10) + "░" * (10 - int(gate_prog * 10))
                panel.append((f"{gate_name} [{bar}] {gate_prog:.0%}", (180, 180, 180)))
            if note:
                panel.append((note, state_color))

            # 第3行：目标信息
            if target:
                aligned = abs(bearing) <= args.align_thresh
                panel.append((f"目标 #{target['id']} {target['cls']}"
                              f"({target['color'] or '无色'}) "
                              f"偏角{bearing:+.2f}({'已对准' if aligned else '未对准'}) "
                              f"{'已近' if near else '未近'} "
                              f"距离={distance_text}",
                              (0, 255, 255)))
            else:
                panel.append(("无目标，旋转搜索中…", (0, 165, 255)))

            # 第4行：ReID 认领事件
            if reid is not None and reid.last_event is not None:
                logic, score = reid.last_event
                panel.append((f"重识别找回 #{logic}（综合得分 {score:.0%}）",
                              (0, 200, 255)))
            if reid is not None and reid.last_debug is not None:
                panel.append((reid.last_debug, (0, 120, 255)))

            draw_text_cn(frame, panel, origin=(8, 8), font_size=20)

            # ---- FPS ----
            dt = time.time() - t_prev
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps else 1.0 / dt
            t_prev = time.time()
            cv2.putText(frame, f"FPS {fps:.1f}  imgsz={args.imgsz}  flip={flip_name}",
                        (10, frame_h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 2)

            if not args.no_show:
                cv2.imshow("Task6 Mission", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                if key == ord('f'):
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
        # 退出前停车并关闭心跳线程。
        motion.stop("EXIT STOP")
        link.close()
        cap.release()
        if not args.no_show:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
