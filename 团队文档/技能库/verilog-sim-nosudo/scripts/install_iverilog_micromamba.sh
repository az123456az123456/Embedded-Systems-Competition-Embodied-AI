#!/usr/bin/env bash
# Install iverilog + gtkwave into a project-local prefix via micromamba (no sudo).
# Usage: install_iverilog_micromamba.sh <tools-dir>
set -euo pipefail

TOOLS="${1:?usage: install_iverilog_micromamba.sh <tools-dir>}"
mkdir -p "$TOOLS"

# micromamba binary from GitHub releases
if [ ! -x "$TOOLS/micromamba" ]; then
  curl -fsSL --max-time 120 -o "$TOOLS/micromamba" \
    "https://github.com/mamba-org/micromamba-releases/releases/latest/download/micromamba-linux-64"
  chmod +x "$TOOLS/micromamba"
fi

# Keep caches inside the writable tools dir (HOME may be read-only)
export HOME="$TOOLS/.mamba-home"
mkdir -p "$HOME"
export MAMBA_ROOT_PREFIX="$TOOLS/.mamba"
mkdir -p "$MAMBA_ROOT_PREFIX"

"$TOOLS/micromamba" create -y -p "$TOOLS/eda-env" -c conda-forge iverilog gtkwave

echo "Done. Add to PATH: export PATH=\"$TOOLS/eda-env/bin:\$PATH\""
"$TOOLS/eda-env/bin/iverilog" -V | head -1
