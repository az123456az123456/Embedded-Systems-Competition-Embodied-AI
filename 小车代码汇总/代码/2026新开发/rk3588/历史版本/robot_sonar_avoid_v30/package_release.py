#!/usr/bin/env python3
"""Package the verified v30 RK and APK release."""
from pathlib import Path
import runpy
runpy.run_path(str(Path(__file__).resolve().parents[2] / 'integration/robot_sonar_avoid_v30/package_release.py'), run_name='__main__')
