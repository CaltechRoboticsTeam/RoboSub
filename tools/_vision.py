"""Allow repository tools to run without installing ROS."""

from pathlib import Path
import sys

SOURCE = Path(__file__).resolve().parents[1] / "robosub_ws/src/robosub_vision"
sys.path.insert(0, str(SOURCE))
