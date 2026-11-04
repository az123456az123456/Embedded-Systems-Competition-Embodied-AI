"""Bounded target-search lifecycle. No ROS, motor output or speech here."""
import math
import unicodedata
from .vision import Tracker, iou, number, TARGET_CONFIRM_SECONDS
from .ppe_presence import VestPresence

COMMANDS = {
    '找到没戴口罩的人': 'NO-Mask', '找到未戴口罩的人': 'NO-Mask',
    '找到没有戴口罩的人': 'NO-Mask', '寻找未戴口罩的人': 'NO-Mask',
    '找到未戴口罩的人并提醒': 'NO-Mask',
    '找到没戴安全帽的人': 'NO-Hardhat', '找到未戴安全帽的人': 'NO-Hardhat',
    '找到没穿反光衣的人': 'NO-Safety Vest', '找到未穿反光衣的人': 'NO-Safety Vest',
    '找到人': 'Person', '寻找人员': 'Person', '找到人员': 'Person',
    '找到安全帽': 'Hardhat', '找到口罩': 'Mask', '找到反光衣': 'Safety Vest',
    '找到机械': 'machinery', '找到车辆': 'vehicle',
}
LABELS = ('Hardhat', 'Mask', 'NO-Hardhat', 'NO-Mask', 'NO-Safety Vest',
          'Person', 'Safety Vest', 'machinery', 'vehicle')
STOP_COMMANDS = {'停止', '取消', '取消任务', '停止搜索'}


def parse_command(text):
    if not isinstance(text, str) or len(text) > 80:
        raise ValueError('unsupported_semantic_command')
    text = unicodedata.normalize('NFKC', text).strip().rstrip('。.!！')
    if text in STOP_COMMANDS:
        return 'STOP'
    if text in COMMANDS:
        return COMMANDS[text]
    if text in LABELS:
        return text
    raise ValueError('unsupported_semantic_command')


class TargetEvidence:
    """Reuse the supplied same-person PPE association independently per eye."""
    def __init__(self, target, started):
        if target not in LABELS:
            raise ValueError('unsupported_target')
        self.target, self.started = target, started
        self.trackers = [Tracker(0, .5), Tracker(1, .5)]
        self.last_valid = 0.
        self.objects = {}
        self.vest_presence = VestPresence(started) if target=='NO-Safety Vest' else None

    def observe(self, meta, now):
        if self.vest_presence is not None:
            hits=self.vest_presence.observe(meta,now)
            self.last_valid=self.vest_presence.last_valid
            return hits
        matched = []
        for eye, tracker in enumerate(self.trackers):
            try:
                people = tracker.update(meta, now)
            except (ValueError, TypeError, KeyError):
                continue
            if tracker.last_stamp < self.started:
                continue
            self.last_valid = max(self.last_valid, tracker.last_stamp)
            if self.target.startswith('NO-'):
                matched.extend({'eye': eye, 'track': p.track, 'frame': tracker.last_frame,
                                'stamp': p.stamp, 'box': list(p.box)}
                               for p in tracker.confirmed(people, self.target))
                continue
            # Generic labels require the same box for a full second, just as
            # person/PPE evidence does. A missing frame or >250ms gap resets it.
            key = (eye, tracker.epoch)
            previous = self.objects.get(key, [])
            current = []
            for d in meta['detections']:
                if not isinstance(d, dict) or d.get('eye') != eye or d.get('label') != self.target:
                    continue
                b, score = d.get('box'), d.get('score')
                if not isinstance(b, list) or len(b) != 4 or not all(number(x) for x in b) or not number(score) or not .5 <= score <= 1:
                    continue
                width, height = meta['width'], meta['height']
                if not eye*width/2 <= b[0] < b[2] <= (eye+1)*width/2 or not 0 <= b[1] < b[3] <= height:
                    continue
                candidates = [p for p in previous if 0 < tracker.last_stamp-p['last'] <= .25 and iou(b, p['box']) >= .3]
                old = candidates[0] if len(candidates) == 1 else None
                record = dict(box=b, first=old['first'] if old else tracker.last_stamp,
                              last=tracker.last_stamp, count=old['count']+1 if old else 1)
                current.append(record)
                if record['count'] >= 3 and record['last']-record['first']+1e-9 >= TARGET_CONFIRM_SECONDS:
                    matched.append(dict(eye=eye, frame=tracker.last_frame, stamp=record['last'], box=b))
            self.objects = {k: v for k, v in self.objects.items() if k[0] != eye}
            self.objects[key] = current
        return matched


class SearchSession:
    TERMINAL = ('IDLE', 'FOUND', 'NOT_FOUND', 'ERROR', 'CANCELLED')

    def __init__(self):
        self.state, self.reason = 'IDLE', 'enter_target_then_start'
        self.task = ''; self.generation = 0; self.target = ''; self.started = 0.
        self.evidence = None; self.match = None; self.stop_since = None
        self.pending_outcome = ''; self.stopping_at = 0.

    @property
    def active(self):
        return self.state not in self.TERMINAL

    def start(self, target, task, generation, produced_ns, ttl_ms, now):
        if self.active:
            raise ValueError('semantic_task_already_active')
        if target not in LABELS or not isinstance(task, str) or not 1 <= len(task) <= 128:
            raise ValueError('invalid_semantic_task')
        if type(generation) is not int or generation <= 0 or type(ttl_ms) is not int or not 1 <= ttl_ms <= 500:
            raise ValueError('invalid_semantic_lease')
        if type(produced_ns) is not int or not 0 <= now-produced_ns/1e9 < ttl_ms/1000:
            raise ValueError('expired_semantic_request')
        self.state, self.reason = 'WAIT_EXECUTOR', 'waiting_camera_and_gateway_executor'
        self.task, self.generation, self.target, self.started = task, generation, target, now
        self.evidence = TargetEvidence(target, now)
        self.match = None; self.stop_since = None; self.pending_outcome = ''

    def stop(self, outcome, reason, now):
        if not self.active or self.state == 'STOPPING':
            return
        self.state, self.reason = 'STOPPING', reason
        self.pending_outcome, self.stopping_at = outcome, now
        self.stop_since = None

    def observe(self, meta, now):
        if not self.active or self.state == 'STOPPING':
            return
        hits = self.evidence.observe(meta, now)
        if hits:
            self.match = hits[0]
            self.stop('FOUND', 'target_confirmed_waiting_stop', now)

    def tick(self, now, executor_ready=False, stopped=False, mapping_ready=None):
        # mapping_ready is retained as a compatibility keyword for older callers;
        # semantic admission is deliberately independent of maps and SLAM.
        if mapping_ready is not None:
            executor_ready = executor_ready or mapping_ready
        if not self.active:
            return
        if self.state == 'STOPPING':
            if stopped:
                if self.stop_since is None:
                    self.stop_since = now
                if now-self.stop_since >= .5:
                    self.state = self.pending_outcome
                    self.reason = 'target_found_stopped' if self.state == 'FOUND' else self.reason
            else:
                self.stop_since = None
            if now-self.stopping_at > 5 and self.state == 'STOPPING':
                self.state, self.reason = 'ERROR', 'stop_not_confirmed:'+self.pending_outcome
            return
        if now-self.started > 600:
            self.stop('NOT_FOUND', 'search_timeout_600s', now)
            return
        camera_fresh = self.evidence.last_valid > 0 and 0 <= now-self.evidence.last_valid <= 1
        if self.state == 'WAIT_EXECUTOR':
            if now-self.started > 30:
                self.stop('ERROR', 'semantic_start_timeout', now)
            elif executor_ready and camera_fresh:
                self.state, self.reason = 'SEARCHING', 'semantic_random_searching'
        elif self.state == 'SEARCHING' and not camera_fresh:
            self.stop('ERROR', 'yolo_observation_stale', now)

    def gate(self, now):
        # Fresh heartbeat alone is insufficient: original YOLO evidence ages too.
        allowed = (self.state == 'SEARCHING' and self.evidence is not None and
                   0 <= now-self.evidence.last_valid <= 1)
        return dict(task=self.task, gateway_generation=str(self.generation),
                    semantic=True, allow_motion=allowed, produced_mono_ns=int(now*1e9), valid_for_ms=200)

    def status(self, now, ready):
        return dict(state=self.state, reason=self.reason, task=self.task,
                    gateway_generation=str(self.generation), target=self.target,
                    active=self.active, ready=bool(ready), found=self.state == 'FOUND',
                    match=self.match, published_mono_ns=int(now*1e9),
                    planner='sonar_150mm_turn_300mm_forward', speech_added=False)
