import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def dump(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    def encode(v):
        if isinstance(v, np.ndarray):
            return v.tolist()
        if isinstance(v, np.generic):
            return v.item()
        if isinstance(v, Path):
            return str(v)
        raise TypeError(type(v).__name__)

    temp = path.with_name(path.name + ".tmp")
    temp.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, default=encode) + "\n"
    )
    temp.replace(path)


def read(path):
    return json.loads(Path(path).read_text())


def points_for(mask, origin, pitch):
    return np.asarray(origin) + np.argwhere(mask) * pitch


def cylinder_mask(shape, origin, pitch, start, end, radius, extension=0):
    start = np.asarray(start)
    end = np.asarray(end)
    axis = end - start
    length = np.linalg.norm(axis)
    if length <= 0:
        raise ValueError("Degenerate motor axis")
    axis = axis / length
    xyz = [origin[i] + np.arange(shape[i]) * pitch for i in range(3)]
    d = [
        xyz[0][:, None, None] - start[0],
        xyz[1][None, :, None] - start[1],
        xyz[2][None, None, :] - start[2],
    ]
    along = sum(d[i] * axis[i] for i in range(3))
    radial = sum(v * v for v in d) - along * along
    return (
        (along >= -extension)
        & (along <= length + extension)
        & (radial <= radius * radius)
    )


def geometry_signature(config):
    import hashlib

    signature = {
        k: config[k]
        for k in [
            "design_result",
            "manufacturing_result",
            "pitch_mm",
            "wall_mm",
            "mount_support_mm",
            "motor_clearance_mm",
        ]
    }
    signature["algorithm"] = "solid_offset_conservative_v2"
    files = [ROOT / config["design_result"] / "robot_result.pkl"]
    for folder in sorted(
        (ROOT / config["manufacturing_result"] / "solid_tenon").iterdir()
    ):
        if folder.is_dir() and (folder / (folder.name + "_solid_tenon.stl")).is_file():
            files.extend(
                [
                    folder / (folder.name + "_solid_tenon.stl"),
                    folder / (folder.name + "_replaced.csv"),
                ]
            )

    def digest(path):
        h = hashlib.sha256()
        with path.open("rb") as f:
            for chunk in iter(lambda: f.read(1024 * 1024), b""):
                h.update(chunk)
        return h.hexdigest()

    signature["input_sha256"] = {str(p): digest(p) for p in files}
    return signature
