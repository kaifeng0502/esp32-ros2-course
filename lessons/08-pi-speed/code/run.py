#!/usr/bin/env python3
"""Lesson entry point; forwards arguments to the shared, tested implementation."""
from pathlib import Path
import runpy
import sys
root=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(root/'host'))
sys.argv=[str(root/'host/pi_control.py'),*[],*sys.argv[1:]]
runpy.run_path(str(root/'host/pi_control.py'),run_name='__main__')
