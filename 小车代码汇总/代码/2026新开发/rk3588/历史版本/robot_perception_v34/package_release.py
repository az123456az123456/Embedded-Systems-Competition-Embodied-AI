#!/usr/bin/env python3
"""Use the v34 source-bound integration packager."""
from pathlib import Path
import runpy
runpy.run_path(str(Path(__file__).resolve().parents[2]/"integration/robot_perception_v34/package_release.py"),run_name="__main__")
