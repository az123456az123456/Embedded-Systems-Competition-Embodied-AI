"""Conservative, single-eye person association. Detector IDs are NOT track IDs.

Box coordinates follow the captured dashboard X-Frame-Meta contract (preview
canvas XYXY). Every new source epoch invalidates all identity/evidence state.
No 'absence of Mask means NO-Mask' inference is made here.
"""
from dataclasses import dataclass, field
import math

TARGET_CONFIRM_SECONDS = 1.0


def number(x):
    return type(x) in (int, float) and math.isfinite(x)


def iou(a, b):
    x = max(0., min(a[2], b[2])-max(a[0], b[0]))
    y = max(0., min(a[3], b[3])-max(a[1], b[1]))
    intersection = x*y
    return intersection / max(1e-9, (a[2]-a[0])*(a[3]-a[1]) +
                              (b[2]-b[0])*(b[3]-b[1])-intersection)


@dataclass
class Person:
    track: int
    box: tuple
    stamp: float
    negative: set = field(default_factory=set)
    positive: set = field(default_factory=set)


class Tracker:
    def __init__(self, eye=0, confidence=.5):
        self.eye = eye
        self.confidence = confidence
        self.epoch = None
        self.last_stamp = -1.
        self.last_frame = -1
        self.next_id = 0
        self.tracks = []
        self.evidence = {}

    def update(self, meta, now):
        """Return fresh persons, or raise; invalid frames never extend freshness."""
        if not isinstance(meta, dict) or meta.get("demo") is not False or \
                meta.get("control_eligible") is not True or \
                meta.get("detections_pair_status") != "PAIR_COMPLETE":
            raise ValueError("invalid_detection_pair")
        stamp = meta.get("detection_timestamp_ms")
        fid = meta.get("detections_frame_id")
        epoch = meta.get("source_epoch")
        if type(epoch) is not int or epoch < 0:
            raise ValueError("missing_source_epoch")
        if type(stamp) is not int or type(fid) is not int or stamp <= 0 or fid <= 0:
            raise ValueError("invalid_detection_stamp")
        stamp /= 1000.
        if not 0 <= now-stamp <= .35:
            raise ValueError("stale_detections")
        if self.epoch != epoch:
            self.tracks = []
            self.evidence.clear()
            self.last_frame = -1
            self.last_stamp = -1.
            self.epoch = epoch
        if stamp <= self.last_stamp or fid <= self.last_frame:
            raise ValueError("nonprogressing_detections")
        width, height = meta.get("width"), meta.get("height")
        ds = meta.get("detections")
        if type(width) is not int or type(height) is not int or width <= 0 or height <= 0 \
                or not isinstance(ds, list) or len(ds) > 256:
            raise ValueError("invalid_detection_layout")
        boxes = []
        for d in ds:
            if not isinstance(d, dict) or d.get("eye") != self.eye:
                continue
            b, score = d.get("box"), d.get("score")
            if not isinstance(b, list) or len(b) != 4 or not all(number(x) for x in b) \
                    or not number(score) or not self.confidence <= score <= 1:
                continue
            if not (0 <= b[0] < b[2] <= width and 0 <= b[1] < b[3] <= height):
                continue
            # Crop only by configured eye; never fuse left/right duplicate people.
            left, right = self.eye*width/2, (self.eye+1)*width/2
            if not left <= b[0] < b[2] <= right:
                continue
            boxes.append((d.get("label"), (b[0]-left, b[1], b[2]-left, b[3])))
        people = [b for label, b in boxes if label == "Person"]
        old = [p for p in self.tracks if 0 <= stamp-p.stamp <= .6]
        # A match must be unique in BOTH directions. Crowded/overlapping people
        # deliberately lose identity rather than inheriting another person's PPE.
        edges = [[j for j, p in enumerate(old) if iou(b, p.box) >= .25] for b in people]
        result = []
        for i, b in enumerate(people):
            candidates = edges[i]
            matched = len(candidates) == 1 and sum(candidates[0] in e for e in edges) == 1
            if matched:
                track = old[candidates[0]].track
            else:
                self.next_id += 1
                track = self.next_id
            result.append(Person(track, b, stamp))
        for label, b in boxes:
            if label not in ("NO-Mask", "NO-Hardhat", "NO-Safety Vest", "Mask", "Hardhat", "Safety Vest"):
                continue
            cx, cy = (b[0]+b[2])/2, (b[1]+b[3])/2
            associates = []
            for p in result:
                x1, y1, x2, y2 = p.box
                fraction = (cy-y1)/(y2-y1)
                upper = label in ("NO-Mask", "Mask", "NO-Hardhat", "Hardhat")
                if x1 <= cx <= x2 and y1 <= cy <= y2 and \
                        ((upper and fraction <= .5) or (not upper and .15 <= fraction <= .85)):
                    associates.append(p)
            if len(associates) == 1:
                p = associates[0]
                if label.startswith("NO-"):
                    p.negative.add(label)
                else:
                    p.positive.add(label)
        self.last_stamp, self.last_frame, self.tracks = stamp, fid, result
        alive = {p.track for p in result}
        self.evidence = {k: v for k, v in self.evidence.items() if k[0] in alive}
        return result

    def confirmed(self, persons, label):
        found = []
        positive = label.removeprefix("NO-")
        for p in persons:
            key = (p.track, label)
            if label in p.negative and positive not in p.positive:
                first, last, count = self.evidence.get(key, (p.stamp, p.stamp, 0))
                if p.stamp-last > .25:
                    first, last, count = p.stamp, p.stamp, 0
                if p.stamp != last or count == 0:
                    count += 1
                self.evidence[key] = (first, p.stamp, count)
                if count >= 3 and p.stamp-first+1e-9 >= TARGET_CONFIRM_SECONDS:
                    found.append(p)
            else:
                self.evidence.pop(key, None)
        return found
