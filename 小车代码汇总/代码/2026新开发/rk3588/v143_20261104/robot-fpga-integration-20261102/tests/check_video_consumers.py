"""Real phone service with synthetic frames; no camera/NPU/motor hardware."""
import http.client
import json
import socket
import subprocess
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='semantic-video-clients-') as directory:
    token = Path(directory) / 'token'
    token.write_text('012345\n')
    with socket.socket() as free:
        free.bind(('127.0.0.1', 0))
        port = free.getsockname()[1]
    process = subprocess.Popen([str(ROOT / 'perception/build/phone-publish-probe'), str(port), str(token)],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    connections = []
    try:
        deadline = time.monotonic() + 2
        while True:
            try:
                with socket.create_connection(('127.0.0.1', port), timeout=.1):
                    break
            except OSError:
                assert time.monotonic() < deadline, 'fixture startup failed'
                time.sleep(.02)

        def connect():
            connection = http.client.HTTPConnection('127.0.0.1', port, timeout=2)
            connections.append(connection)
            connection.request('GET', '/api/video.mjpg', headers={'Authorization': 'Bearer 012345'})
            return connection, connection.getresponse()

        def frame(response):
            headers = {}
            for _ in range(20):
                line = response.readline(65536)
                assert line, 'stream closed'
                if line in (b'\r\n', b'\n') and headers:
                    break
                if b':' in line:
                    key, value = line.split(b':', 1)
                    headers[key.lower().strip()] = value.strip()
            body = response.read(int(headers[b'content-length']))
            assert body.startswith(b'\xff\xd8') and body.endswith(b'\xff\xd9')
            return int(headers[b'x-frame-id'])

        clients = [connect() for _ in range(3)]
        assert all(response.status == 200 for _, response in clients), 'phone/semantic/voice must coexist'
        limited, response = connect()
        assert response.status == 429
        assert json.loads(response.read())['error'] == 'video_client_limit'
        response.close(); limited.close()
        first = [frame(response) for _, response in clients]
        latest = first
        for _ in range(12):
            latest = [frame(response) for _, response in clients]
        assert all(last > start for start, last in zip(first, latest)), 'all consumers need fresh frames'
        for connection, response in clients:
            response.close(); connection.close()
        time.sleep(.15)
        connection, response = connect()
        assert response.status == 200 and frame(response) > 0, 'disconnect must release its slot'
        response.close(); connection.close()
        print('PASS: three concurrent fresh JPEG consumers; fourth rejected; slot reusable; no hardware')
    finally:
        for connection in connections:
            connection.close()
        process.terminate()
        process.wait(timeout=3)
