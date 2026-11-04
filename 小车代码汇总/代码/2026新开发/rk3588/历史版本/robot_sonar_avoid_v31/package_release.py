#!/usr/bin/env python3
"""Package the verified v31 RK release."""
from pathlib import Path
import runpy
runpy.run_path(str(Path(__file__).resolve().parents[2] / 'integration/robot_sonar_avoid_v31/package_release.py'), run_name='__main__')
