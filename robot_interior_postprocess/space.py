"""Construct a real hollow solid first; placement never treats solid fill as air."""

import numpy as np
import trimesh
from common import cylinder_mask, dump
from scipy.ndimage import (
    binary_erosion,
    distance_transform_edt,
    gaussian_filter,
    label,
)
from skimage.measure import marching_cubes


def mask_mesh(mask, origin, pitch):
    if not np.any(mask):
        raise ValueError("Empty cavity mask")
    padded = np.pad(mask, 1)
    # Shrink/smooth the subtraction tool to remove voxel point contacts; do not
    # enlarge the proposed cavity into protected structure.
    field = gaussian_filter(padded.astype(np.float32), sigma=0.65)
    field[~padded] = 0
    vertices, faces, _, _ = marching_cubes(
        field, level=0.8, spacing=(pitch,) * 3, allow_degenerate=False
    )
    vertices += np.asarray(origin) - pitch
    mesh = trimesh.Trimesh(vertices, faces, process=True)
    trimesh.repair.fix_normals(mesh, multibody=True)
    if mesh.volume < 0:
        mesh.invert()
    if not mesh.is_volume:
        raise ValueError("Reconstructed cavity is not an oriented closed volume")
    return mesh


def region_slices(local_origin, local_shape, origin, pitch):
    start = np.rint((local_origin - origin) / pitch).astype(int)
    return tuple(slice(int(a), int(a + b)) for a, b in zip(start, local_shape))


def build_space(meshes, scene, config, out):
    pitch = config["pitch_mm"]
    wall = config["wall_mm"]
    bounds = np.array([m.bounds for m in meshes.values()])
    origin = np.floor(bounds[:, 0].min(axis=0) / pitch) * pitch - 8 * pitch
    upper = np.ceil(bounds[:, 1].max(axis=0) / pitch) * pitch + 8 * pitch
    shape = tuple((np.rint((upper - origin) / pitch).astype(int) + 1).tolist())
    material = np.zeros(shape, bool)
    cavities = np.zeros(shape, bool)
    supports = np.zeros(shape, bool)
    part_ids = np.zeros(shape, np.uint8)
    hollows = {}
    records = []
    for part_index, (name, mesh) in enumerate(meshes.items(), 1):
        print("Hollowing", name, flush=True)
        # Input adapter above only accepts solid single-boundary parts. Fill is valid here,
        # and is deliberately NOT used as a generic existing-cavity detector.
        vox = mesh.voxelized(pitch).fill()
        filled = np.pad(vox.matrix, 2)
        loc = np.asarray(vox.translation) - 2 * pitch
        protected = np.zeros(filled.shape, bool)
        for motor in scene["motors"]:
            if name in (motor["link"], motor["parent"]):
                protected |= cylinder_mask(
                    filled.shape,
                    loc,
                    pitch,
                    motor["axial_start_mm"],
                    motor["axial_end_mm"],
                    motor["radius_mm"] + config["mount_support_mm"],
                    config["mount_support_mm"],
                )
        # Half voxel diagonal is added to the requested wall as a discretization guard.
        interior = (
            distance_transform_edt(filled, sampling=pitch)
            >= wall + np.sqrt(3) * pitch / 2
        ) & ~protected
        inner = mask_mesh(interior, loc, pitch)
        hollow = trimesh.boolean.difference(
            [mesh, inner], engine="manifold", check_volume=True
        )
        if not hollow.is_volume:
            raise ValueError(f"{name}: invalid hollow Boolean output")
        inner.export(out / "meshes" / f"{name}_cavity_tool_mm.stl")
        hollow.export(out / "meshes" / f"{name}_hollow_assembly_mm.stl")
        hollows[name] = hollow
        slices = region_slices(loc, filled.shape, origin, pitch)
        safe_interior = binary_erosion(interior, iterations=2)
        material[slices] |= filled & ~safe_interior
        cavities[slices] |= safe_interior
        supports[slices] |= filled & protected
        part_ids[slices] = np.where(filled, part_index, part_ids[slices])
        records.append(
            {
                "part": name,
                "source_volume_mm3": mesh.volume,
                "hollow_volume_mm3": hollow.volume,
                "cavity_cells": int(interior.sum()),
                "watertight": bool(hollow.is_watertight),
                "wall_parameter_mm": wall,
                "voxel_guard_mm": np.sqrt(3) * pitch / 2,
                "wall_certification": "Sampling guard only; not an exact minimum-wall/strength certificate",
            }
        )
    # Union material dominates overlap between different parts.
    cavities &= ~material
    motor_block = np.zeros(shape, bool)
    for motor in scene["motors"]:
        motor_block |= cylinder_mask(
            shape,
            origin,
            pitch,
            motor["axial_start_mm"],
            motor["axial_end_mm"],
            motor["radius_mm"] + config["motor_clearance_mm"],
            config["motor_clearance_mm"],
        )
    usable = cavities & ~motor_block
    counts, n = label(usable)
    sizes = np.bincount(counts.ravel())[1:]
    dump(
        out / "space_report.json",
        {
            "parts": records,
            "grid_shape": shape,
            "pitch_mm": pitch,
            "existing_input": "Solid manufacturing parts; hollowing intentionally creates new physical cavities",
            "cavity_cells": int(cavities.sum()),
            "usable_cells": int(usable.sum()),
            "cavity_regions": int(n),
            "largest_region_volumes_mm3": sorted(
                (sizes * pitch**3).tolist(), reverse=True
            )[:12],
            "motion_validation": "Static assembly only; full motion sweep is not yet certified",
        },
    )
    np.savez_compressed(
        out / "space_masks.npz",
        material=material,
        cavities=cavities,
        motor_block=motor_block,
        protected_material=supports,
        usable=usable,
        part_ids=part_ids,
        origin=origin,
        pitch=pitch,
    )
    return {
        "material": material,
        "cavities": cavities,
        "motor_block": motor_block,
        "protected_material": supports,
        "usable": usable,
        "part_ids": part_ids,
        "origin": origin,
        "pitch": pitch,
    }, hollows
