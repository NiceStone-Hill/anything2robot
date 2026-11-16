import json
from pathlib import Path

p = Path(__file__).parent / "outputs/cactus/manifest.json"
if not p.exists():
    print("尚未运行")
else:
    s = json.loads(p.read_text())
    print("状态:", s["status"], "\n阶段:", s["stage"])
    if s.get("error"):
        print("错误:", s["error"])
    if "routes" in s:
        print(
            "布线分支:", s["routes"], "未放置:", s["unplaced"], "未布通:", s["unrouted"]
        )
