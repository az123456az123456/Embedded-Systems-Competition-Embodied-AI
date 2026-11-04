"""Authenticated MJPEG reader; require frame progress with YOLO disabled."""
import base64
import json
from pathlib import Path
import time
import urllib.request
from .core import Rejected


def backend_image(cfg):
    token = Path(cfg.get('token_file', '/run/acg138-ros-bridge/semantic/token')).read_text().strip()
    if not token or '\n' in token or '\r' in token:
        raise Rejected('INVALID_CAMERA_TOKEN')
    request = urllib.request.Request('http://127.0.0.1:8082/api/video.mjpg',
                                     headers={'Authorization': 'Bearer '+token})
    deadline, first_id = time.monotonic()+3., None
    with urllib.request.urlopen(request, timeout=3) as response:
        for _ in range(8):
            total, headers = 0, {}
            for _ in range(64):
                line = response.readline(65537)
                total += len(line)
                if total > 65536 or not line or time.monotonic() > deadline:
                    raise Rejected('INVALID_OR_STALLED_CAMERA_HEADERS')
                if line in (b'\r\n', b'\n') and headers:
                    break
                if b':' in line:
                    key, value = line.split(b':', 1)
                    headers[key.strip().lower()] = value.strip()
            else:
                raise Rejected('INVALID_CAMERA_HEADERS')
            length = int(headers.get(b'content-length', b'0'))
            if not 0 < length <= 8*1024*1024:
                raise Rejected('INVALID_CAMERA_FRAME_LENGTH')
            meta = json.loads(base64.b64decode(headers[b'x-frame-meta'], validate=True))
            if meta.get('demo') is not False:
                raise Rejected('DEMO_CAMERA_FRAME')
            frame_id = meta.get('frame_id')
            if type(frame_id) is not int or frame_id <= 0:
                raise Rejected('INVALID_CAMERA_FRAME_ID')
            data = response.read(length)
            if len(data) != length or time.monotonic() > deadline:
                raise Rejected('TRUNCATED_OR_STALLED_CAMERA_FRAME')
            # This backend supplies frame_id, not a capture timestamp in the
            # multipart header. A new frame after opening the stream proves
            # progress even when detection_timestamp_ms is zero in no-detect mode.
            if first_id is None:
                first_id = frame_id
            elif frame_id > first_id:
                return data
            elif frame_id < first_id:
                raise Rejected('CAMERA_SOURCE_RESTARTED')
    raise Rejected('CAMERA_FRAME_NOT_PROGRESSING')
