---
name: camera-device-in-sandbox
description: Handle the case where a vision script cannot see the user's webcam even though the host has one. Codex sandboxes often do not forward /dev/video* devices, so OpenCV VideoCapture fails inside the sandbox while working on the user's own desktop. Use when a camera test fails inside a sandboxed/agent environment.
metadata:
  short-description: Why the camera is invisible inside a sandbox and how to test it correctly
---

# Camera invisible in a sandbox

## Symptom
- `ls /dev/video*` is empty in the agent shell
- `cv2.VideoCapture(0)` fails or returns no frames
- The user insists their machine has a working webcam

## Diagnosis
1. Check the sandbox device nodes: `ls -la /dev/video*` (often absent in bwrap-style sandboxes).
2. Check the host driver is actually loaded (shares the kernel): `lsmod | grep uvcvideo`. If `uvcvideo` is present, the host camera exists and is driver-enabled — the sandbox simply isn't forwarding the device.

## What to do
- Do NOT keep trying to open the camera from inside the sandbox.
- Give the user a small self-run test script and have them execute it **in their own desktop terminal** with the project venv (e.g. `python camera_test.py`, press q to quit, s to snapshot).
- If detection is expected, provide a combined camera + YOLO script (`model.predict(frame)`) with drawn boxes so they can see results.
- If the window opens but no boxes appear: raise/lower the confidence threshold (`conf=0.4` → `0.2`) and point at people/phones/bottles; also allow camera index override (`python script.py 1`) for multiple cameras.
