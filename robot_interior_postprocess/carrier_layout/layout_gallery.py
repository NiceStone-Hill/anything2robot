"""Offline images of real coarse-feasible seeds; never pose or mesh synthesis."""

import base64
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


def render_seed(instances, assets, bounds, normal, path, title, subtitle):
    axis = int(np.argmax(abs(np.asarray(normal))))
    lateral = [k for k in range(3) if k != axis]
    # Keep the world-up axis vertical when it is visible; otherwise use the
    # larger source span horizontally. All seeds use the same source bounds.
    if 2 in lateral:
        lateral = [next(k for k in lateral if k != 2), 2]
    u, v = lateral
    bounds = np.asarray(bounds)
    scale = min(
        730 / max(bounds[1, u] - bounds[0, u], 1),
        420 / max(bounds[1, v] - bounds[0, v], 1),
    )
    center = bounds.mean(0)
    image = Image.new("RGB", (800, 600), "#f0f4f8")
    draw = ImageDraw.Draw(image)
    font_path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    font = (
        ImageFont.truetype(str(font_path), 19)
        if font_path.exists()
        else ImageFont.load_default()
    )
    small = ImageFont.truetype(str(font_path), 15) if font_path.exists() else font
    draw.text((22, 16), title, fill="#17364a", font=font)
    draw.text((22, 45), subtitle, fill="#536b7d", font=small)
    triangles = []
    for item in instances:
        asset = assets[item["asset"]]
        points = (
            np.asarray(asset["vertices"]) @ np.asarray(item["transform"])[:3, :3].T
            + np.asarray(item["transform"])[:3, 3]
        )
        xy = (points[:, lateral] - center[lateral]) * [scale, -scale] + [400, 305]
        if item["role"] == "shell":
            for f in asset["faces"]:
                draw.polygon([tuple(p) for p in xy[f]], fill="#dbe4ec")
            continue
        if item["role"] not in {"component", "carrier"}:
            continue
        faces = np.asarray(asset["faces"])
        depths = points[:, axis] * np.asarray(normal)[axis]
        colors = asset.get("face_colors")
        for j, face in enumerate(faces):
            color = (
                tuple(colors[j])
                if colors
                else "#d6a14c"
                if item["role"] == "carrier"
                else "#637cc6"
            )
            triangles.append((float(depths[face].mean()), xy[face], color))
        lo, hi = xy.min(0), xy.max(0)
        if item["role"] == "component":
            # Labels are placed after all solid faces below.
            item_label = (
                item["id"].split(":component")[0],
                (float(lo[0]), float(hi[1]) + 2),
            )
            triangles.append((float("inf"), item_label, None))
    for _, points, color in sorted(triangles, key=lambda x: x[0]):
        if color is None:
            draw.text(points[1], points[0], fill="#17364a", font=small)
        else:
            draw.polygon([tuple(p) for p in points], fill=color)
    draw.text(
        (22, 535),
        f"Face view along {'XYZ'[axis]} | shared scale | gold: carriers | CAD: components",
        fill="#536b7d",
        font=small,
    )
    draw.text(
        (22, 564),
        "Coarse-feasible seed only. Exact supports, insertion and tool paths follow.",
        fill="#9b6031",
        font=small,
    )
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)
    return "data:image/png;base64," + base64.b64encode(path.read_bytes()).decode(
        "ascii"
    )
