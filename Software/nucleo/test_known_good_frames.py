#!/usr/bin/env python3
"""Compatibility launcher for moved test script."""
from pathlib import Path
import runpy

runpy.run_path(str(Path(__file__).parent / 'tests' / 'manual' / 'test_known_good_frames.py'), run_name='__main__')