"""Bounded MJPEG metadata reader for the existing YOLO dashboard."""
import base64
import json
from pathlib import Path
import urllib.request


def frames(cfg, stop):
    # URL is fixed to the authenticated local perception endpoint in deployment.
    if cfg["url"] != "http://127.0.0.1:8082/api/video.mjpg":
        raise ValueError("camera_must_be_local_dashboard")
    token = Path(cfg["token_file"]).read_text().strip()
    if not token or "\n" in token or "\r" in token:
        raise ValueError("invalid_camera_token")
    req = urllib.request.Request(cfg["url"], headers={"Authorization": "Bearer "+token})
    with urllib.request.urlopen(req, timeout=1.) as response:
        while not stop.is_set():
            headers, size = {}, 0
            for _ in range(64):
                line = response.readline(65537)
                size += len(line)
                if not line or size > 65536:
                    raise ValueError("camera_headers_invalid")
                if line in (b"\r\n", b"\n") and headers:
                    break
                if b":" in line:
                    key, value = line.split(b":", 1)
                    headers[key.strip().lower()] = value.strip()
            else:
                raise ValueError("camera_headers_overflow")
            length = int(headers.get(b"content-length", b"0"))
            if not 0 < length <= 8*1024*1024:
                raise ValueError("camera_body_invalid")
            meta = json.loads(base64.b64decode(headers[b"x-frame-meta"], validate=True))
            # Don't allocate JPEGs or decode images: YOLO already ran on NPU.
            remaining = length
            while remaining:
                chunk = response.read(min(remaining, 65536))
                if not chunk:
                    raise ValueError("camera_body_truncated")
                remaining -= len(chunk)
            yield meta
