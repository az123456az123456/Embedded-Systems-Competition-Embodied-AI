---
name: python-venv-nosudo
description: Set up a Python virtual environment and install pip/PyTorch/ultralytics/OpenCV on a restricted Linux host that has no sudo, no ensurepip, and read-only HOME directories (e.g., a sandbox or minimal Ubuntu). Use when pip is missing, `python3 -m venv` fails on ensurepip, or installs must stay inside a writable project folder.
metadata:
  short-description: Bootstrap pip and install PyTorch/ultralytics in a venv without sudo
---

# Python venv without sudo

On Ubuntu 26.04+ (and other minimal images) `pip` and `ensurepip` are often absent and `apt` needs sudo. These steps install everything into a project-local venv.

## When this applies
- `python3 -m pip` reports "No module named pip"
- `python3 -m venv` fails with "ensurepip is not available"
- The writable area is a project folder only (HOME may be read-only)

## Workflow
1. Create a venv **without pip**: `python3 -m venv --without-pip <venv>`
2. Bootstrap pip from the PyPI JSON API (the `/simple/` index can be slow or blocked, but `https://pypi.org/pypi/pip/json` usually works):
   - Get the wheel URL, download it, unzip, then run `python /tmp/pipx/pip/__main__.py install --force-reinstall --no-deps pip`.
   - **Gotcha:** running `__main__.py` with the unzipped dir on sys.path makes pip see itself as "already installed" in that dir; pass `--force-reinstall` so it lands in the venv.
3. Point the pip cache at a writable dir to silence warnings: `export PIP_CACHE_DIR=<venv>/../.pip-cache`.
4. Install CPU PyTorch from the official CPU index (avoids multi-GB CUDA wheels):
   `pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu`
5. Install vision libs: `pip install ultralytics opencv-python pyserial numpy pillow`.
6. Set `YOLO_CONFIG_DIR` to a writable dir before importing ultralytics (avoids "user config directory not writable").
7. Validate: `python -c "import torch, cv2, ultralytics; print(torch.__version__, cv2.__version__, ultralytics.__version__)"`.

The full flow is in [scripts/bootstrap_venv.sh](scripts/bootstrap_venv.sh); run it instead of hand-typing when possible.
