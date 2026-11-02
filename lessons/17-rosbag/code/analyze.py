#!/usr/bin/env python3
from pathlib import Path
import runpy
runpy.run_path(str(Path(__file__).resolve().parents[3]/'ros2/analyze_motor_bag.py'),run_name='__main__')
