#!/usr/bin/env bash
# Bootstrap a pip-enabled venv without sudo, then install a CPU ML/vision stack.
# Usage: bootstrap_venv.sh <target-venv-dir> [pip-cache-dir]
set -euo pipefail

VENV="${1:?usage: bootstrap_venv.sh <target-venv-dir> [pip-cache-dir]}"
CACHE="${2:-$(dirname "$VENV")/.pip-cache}"

# 1. venv without pip
python3 -c "import shutil; shutil.rmtree('$VENV', ignore_errors=True)"
python3 -m venv --without-pip "$VENV"

# 1b. Writable pip cache (before pip runs, avoids read-only HOME warning)
export PIP_CACHE_DIR="$CACHE"
mkdir -p "$CACHE"

# 2. Bootstrap pip via PyPI JSON API
WHL_URL=$(curl -fsSL --max-time 30 https://pypi.org/pypi/pip/json | python3 -c "
import json, sys
d = json.load(sys.stdin); v = d['info']['version']
print([f['url'] for f in d['releases'][v] if f['filename'].endswith('py3-none-any.whl')][0])")
rm -rf /tmp/pipx_$$; mkdir -p /tmp/pipx_$$
curl -fsSL --max-time 120 -o /tmp/pip_$$.whl "$WHL_URL"
python3 -c "import zipfile; zipfile.ZipFile('/tmp/pip_$$.whl').extractall('/tmp/pipx_$$')"
"$VENV/bin/python" /tmp/pipx_$$/pip/__main__.py install --force-reinstall --no-deps --quiet pip
rm -rf /tmp/pipx_$$ /tmp/pip_$$.whl


# 4. CPU torch + vision stack
"$VENV/bin/pip" install --quiet torch torchvision --index-url https://download.pytorch.org/whl/cpu
"$VENV/bin/pip" install --quiet ultralytics opencv-python pyserial numpy pillow

echo "Done. Activate with: source $VENV/bin/activate"
"$VENV/bin/python" -c "import torch, cv2, ultralytics; print('torch', torch.__version__, '| cv2', cv2.__version__, '| ultralytics', ultralytics.__version__)"
