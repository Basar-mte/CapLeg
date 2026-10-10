"""Shared paths for the repository-level tests (run from the repo root:  pytest)."""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DESC = os.path.join(ROOT, 'ros2_ws', 'src', 'capleg_description')
GAZEBO = os.path.join(ROOT, 'ros2_ws', 'src', 'capleg_gazebo')
MUJOCO_DIR = os.path.join(ROOT, 'mujoco')

for p in (os.path.join(ROOT, 'ros2_ws', 'src', 'capleg_control'), MUJOCO_DIR):
    if p not in sys.path:
        sys.path.insert(0, p)
