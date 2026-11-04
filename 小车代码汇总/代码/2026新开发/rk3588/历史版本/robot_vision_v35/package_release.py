#!/usr/bin/env python3
"""The v35 release gate; never package a legacy copied integration."""
from pathlib import Path
import runpy
runpy.run_path(str(Path(__file__).resolve().parents[2]/'integration/robot_vision_v35/package_release.py'),run_name='__main__')
