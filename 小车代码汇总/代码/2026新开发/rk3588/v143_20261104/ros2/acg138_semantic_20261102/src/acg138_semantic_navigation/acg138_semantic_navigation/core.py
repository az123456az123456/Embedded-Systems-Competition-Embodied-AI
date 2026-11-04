"""Metric coordinates come from surveyed landmarks, never from VLM prose."""
import hashlib
import math
import re
from dataclasses import dataclass
from pathlib import Path
import yaml


class Rejected(ValueError):
    pass


def target_from_instruction(text):
    if not isinstance(text, str) or not 1 <= len(text) <= 256:
        raise Rejected('INVALID_INSTRUCTION')
    nouns = {'瓶子': 'bottle', '水瓶': 'bottle', '杯子': 'cup', '椅子': 'chair',
             '门': 'door', '灭火器': 'fire extinguisher', '人': 'person',
             '桌子': 'table', '安全帽': 'hard hat', '反光衣': 'safety vest'}
    colors = {'红': 'red', '蓝': 'blue', '绿': 'green', '黄': 'yellow',
              '白': 'white', '黑': 'black'}
    if re.search('[\u3400-\u9fff]', text):
        found = {v for k, v in nouns.items() if k in text}
        color = {v for k, v in colors.items() if k in text}
        if len(found) != 1 or len(color) > 1:
            raise Rejected('UNSUPPORTED_OR_AMBIGUOUS_CHINESE_TARGET')
        return ' '.join([*sorted(color), *found])
    text = text.lower().strip().rstrip('.!?')
    text = re.sub(r'^(?:find|locate|approach|go to|move near)\s+(?:(?:a|the)\s+)?', '', text)
    if not re.fullmatch('[a-z][a-z -]{0,63}', text):
        raise Rejected('INVALID_ENGLISH_TARGET')
    return text


def presence_answer(text):
    answer = text.strip().upper().rstrip('.!')
    if answer == 'NO':
        raise Rejected('TARGET_NOT_FOUND')
    if answer != 'YES':
        raise Rejected('INVALID_PRESENCE_OUTPUT')


def direction_answer(text):
    answer = text.strip().upper().rstrip('.!')
    if answer == 'NOT_FOUND':
        raise Rejected('TARGET_NOT_FOUND')
    if answer not in ('LEFT', 'CENTER', 'RIGHT'):
        raise Rejected('INVALID_DIRECTION_OUTPUT')
    return answer


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float


class Grid:
    def __init__(self, yaml_path):
        path = Path(yaml_path)
        cfg = yaml.safe_load(path.read_text())
        image_path = path.parent / cfg['image']
        raw = image_path.read_bytes()
        self.sha256 = hashlib.sha256(raw).hexdigest()
        # ROS map_saver's P5/P2 PGM, including comments in the header.
        pos = 0
        def token():
            nonlocal pos
            while True:
                while pos < len(raw) and raw[pos:pos+1].isspace():
                    pos += 1
                if raw[pos:pos+1] != b'#':
                    break
                pos = raw.index(b'\n', pos) + 1
            start = pos
            while pos < len(raw) and not raw[pos:pos+1].isspace():
                pos += 1
            return raw[start:pos]
        magic = token()
        self.width, self.height, maximum = int(token()), int(token()), int(token())
        if maximum != 255 or magic not in (b'P5', b'P2'):
            raise Rejected('UNSUPPORTED_PGM')
        if magic == b'P5':
            if raw[pos:pos+2] == b'\r\n':
                pos += 2
            else:
                pos += 1
            pixels = raw[pos:]
        else:
            pixels = bytes(int(token()) for _ in range(self.width*self.height))
        if len(pixels) != self.width*self.height:
            raise Rejected('BAD_PGM_SIZE')
        self.resolution = float(cfg['resolution'])
        self.origin = tuple(float(v) for v in cfg['origin'])
        if self.resolution <= 0 or len(self.origin) != 3:
            raise Rejected('INVALID_MAP_GEOMETRY')
        self.cells = []
        for row in range(self.height-1, -1, -1):
            for value in pixels[row*self.width:(row+1)*self.width]:
                occupied = value/255 if cfg.get('negate', 0) else (255-value)/255
                self.cells.append(100 if occupied > cfg['occupied_thresh'] else
                                  0 if occupied < cfg['free_thresh'] else -1)

    def world(self, col, row):
        dx, dy = (col+.5)*self.resolution, (row+.5)*self.resolution
        x, y, yaw = self.origin
        return x+math.cos(yaw)*dx-math.sin(yaw)*dy, y+math.sin(yaw)*dx+math.cos(yaw)*dy

    def is_free(self, x, y, radius=.22):
        if not all(math.isfinite(v) for v in (x, y, radius)) or radius < 0:
            return False
        ox, oy, yaw = self.origin
        dx, dy = x-ox, y-oy
        col = math.floor((math.cos(yaw)*dx+math.sin(yaw)*dy)/self.resolution)
        row = math.floor((-math.sin(yaw)*dx+math.cos(yaw)*dy)/self.resolution)
        n = math.ceil(radius/self.resolution)
        # Conservative square footprint also covers the circular inflation.
        return all(0 <= c < self.width and 0 <= r < self.height and
                   self.cells[r*self.width+c] == 0
                   for r in range(row-n, row+n+1) for c in range(col-n, col+n+1))


def short_goal(target, direction, pose, cfg, grid):
    camera = cfg.get('camera', {})
    if (camera.get('calibrated') is not True or camera.get('fx_px', 0) <= 0
            or camera.get('width_px', 0) <= 0 or not camera.get('calibration_source')):
        raise Rejected('NEEDS_CAMERA_CALIBRATION')
    if cfg.get('map_sha256') != grid.sha256:
        raise Rejected('LANDMARK_MAP_MISMATCH')
    if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw)):
        raise Rejected('INVALID_POSE')
    fx, cx, width = camera['fx_px'], camera['cx_px'], camera['width_px']
    if not 0 <= cx < width:
        raise Rejected('INVALID_PRINCIPAL_POINT')
    ex, ey, eyaw = camera['base_x_m'], camera['base_y_m'], camera['base_yaw_rad']
    cam_x = pose.x+math.cos(pose.yaw)*ex-math.sin(pose.yaw)*ey
    cam_y = pose.y+math.sin(pose.yaw)*ex+math.cos(pose.yaw)*ey
    candidates = []
    for landmark in cfg.get('landmarks', []):
        if target not in landmark.get('targets', []) or landmark.get('surveyed') is not True:
            continue
        x, y = float(landmark['x_m']), float(landmark['y_m'])
        if not all(math.isfinite(v) for v in (x, y)):
            continue
        bearing = wrap(math.atan2(y-cam_y, x-cam_x)-pose.yaw-eyaw)
        if abs(bearing) >= math.pi/2:
            continue
        pixel = cx-fx*math.tan(bearing)  # ROS positive y is left; image positive u is right.
        projected = 'LEFT' if pixel < width/3 else 'RIGHT' if pixel >= 2*width/3 else 'CENTER'
        if 0 <= pixel < width and projected == direction:
            candidates.append((landmark['id'], x, y, pixel))
    if len(candidates) != 1:
        raise Rejected('NO_UNIQUE_SURVEYED_LANDMARK')
    name, x, y, pixel = candidates[0]
    distance = math.hypot(x-pose.x, y-pose.y)
    step = min(.75, max(0., distance-max(.6, float(cfg.get('standoff_m', .6)))))
    if step < .1:
        raise Rejected('ALREADY_NEAR_TARGET')
    gx, gy = pose.x+step*(x-pose.x)/distance, pose.y+step*(y-pose.y)/distance
    if not grid.is_free(gx, gy):
        raise Rejected('GOAL_OCCUPIED_UNKNOWN_OR_OUTSIDE_MAP')
    return {'x': gx, 'y': gy, 'yaw': math.atan2(y-pose.y, x-pose.x),
            'step_m': step, 'landmark_id': name, 'projected_u_px': pixel,
            'coordinate_source': 'surveyed_map_landmark'}
