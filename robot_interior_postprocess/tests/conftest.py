import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
config = json.loads((HERE / "configs/cactus.json").read_text())
sys.path[:0] = [str(HERE), config["voxel_tool_source"]]
