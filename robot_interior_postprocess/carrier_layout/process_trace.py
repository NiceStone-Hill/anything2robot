"""Read-only snapshots of actual generation states, independent of the solver."""

import hashlib
import json
import time
from pathlib import Path

import numpy as np
import trimesh
from common import dump
from hardware import as_mesh


def json_value(value):
    def encode(item):
        if isinstance(item, np.ndarray):
            return item.tolist()
        if isinstance(item, np.generic):
            return item.item()
        raise TypeError(type(item).__name__)

    return json.loads(json.dumps(value, default=encode, allow_nan=False))


class ProcessRecorder:
    """Each event has its own immutable instances; geometry assets are deduplicated.

    Only display copies are simplified. Source bounds, volume, face count and
    hashes describe the actual stage geometry, not its simplified preview.
    """

    def __init__(self, output):
        self.output = Path(output)
        self.started = time.monotonic()
        self.data = {
            "schema_version": 1,
            "units": "mm",
            "status": "running",
            "scope": "Recorded postprocessing stages from Anything2robot output; upstream generation is not rerun or reconstructed",
            "preview_note": "CAD appearance references are separate from planning geometry. Shell previews may be simplified; use meshes/*.stl for fabrication. Playback switches recorded states without interpolating solver states.",
            "assets": {},
            "events": [],
            "candidates": {},
            "model_stages": {},
            "model_provenance": {"source": "not_reached", "live_model_called": False},
        }

    def set_model_provenance(self, provenance, stage="architecture_ranking"):
        self.data.setdefault("model_stages", {})[stage] = json_value(provenance)
        if stage == "architecture_ranking":
            self.data["model_provenance"] = json_value(provenance)

    def set_opening_catalog(self, catalog):
        self.data["opening_catalog"] = json_value(catalog)

    def diagnostic_candidate(self, shells, link, plane, gap_mm):
        from .candidate_preview import cut_diagnostic

        sides, report = cut_diagnostic(shells[link], plane, gap_mm)
        instances = self.shells({k: v for k, v in shells.items() if k != link})
        for side, regions in enumerate(sides):
            for region, mesh in enumerate(regions):
                role = (
                    "fragment" if region else "cut_owner" if side == 0 else "cut_cover"
                )
                instances.append(
                    self.item(
                        f"{link}:diagnostic:{side}:{region}",
                        mesh,
                        role,
                        link,
                        side=side,
                        normal=plane["normal"],
                    )
                )
        return instances, report

    def item(self, key, mesh, role, owner, transform=None, side=None, normal=None):
        mesh = mesh.copy() if isinstance(mesh, trimesh.Trimesh) else as_mesh(mesh)
        vertices = np.asarray(mesh.vertices, dtype="<f8")
        faces = np.asarray(mesh.faces, dtype="<i8")
        digest = hashlib.sha256(vertices.tobytes() + faces.tobytes()).hexdigest()
        if digest not in self.data["assets"]:
            shown = (
                mesh
                if role == "component"
                else (
                    mesh.simplify_quadric_decimation(face_count=1200)
                    if len(faces) > 1200
                    else mesh
                )
            )
            self.data["assets"][digest] = {
                "vertices": np.round(
                    shown.vertices, 8 if role == "component" else 5
                ).tolist(),
                "faces": shown.faces.tolist(),
                "source_geometry_sha256": digest,
                "source_faces": len(faces),
                "source_volume_mm3": float(mesh.volume),
                "source_bounds_mm": mesh.bounds.tolist(),
                "preview_simplified": len(shown.faces) != len(faces),
                "face_colors": shown.visual.face_colors[:, :3].tolist()
                if role == "component"
                else None,
            }
        return json_value(
            {
                "id": key,
                "asset": digest,
                "role": role,
                "owner": owner,
                "transform": np.eye(4) if transform is None else transform,
                "side": side,
                "normal": normal,
            }
        )

    def shells(self, meshes, planes=None):
        result = []
        for name, value in meshes.items():
            pair = value if isinstance(value, list) else [value]
            for side, mesh in enumerate(pair):
                plane = (planes or {}).get(name, {})
                result.append(
                    self.item(
                        f"{name}:shell:{side}",
                        mesh,
                        "shell",
                        name,
                        side=side if len(pair) > 1 else None,
                        normal=plane.get("normal"),
                    )
                )
        return result

    def motors(self, scene):
        return [
            self.item(
                m["id"],
                trimesh.creation.cylinder(
                    radius=m["radius_mm"],
                    segment=np.array([m["axial_start_mm"], m["axial_end_mm"]]),
                    sections=24,
                ),
                "motor",
                m["link"],
            )
            for m in scene.get("motors", [])
        ]

    def bundles(self, profiles, poses=None):
        result, x = [], 0.0
        lookup = {p["id"]: p for p in poses or []}
        for profile in profiles:
            key, owner = profile["spec"]["id"], profile["spec"]["owner_link"]
            if poses is not None and key not in lookup:
                continue
            if poses is None:
                transform = np.eye(4)
                transform[:3, 3] = [x - profile["bounds"][0, 0], 0, 0]
                x += float(np.ptp(profile["bounds"], axis=0)[0]) + 25
            else:
                transform = lookup[key]["transform"]
            result.extend(
                [
                    self.item(
                        f"{key}:component",
                        profile["display"],
                        "component",
                        owner,
                        transform,
                    ),
                    self.item(
                        f"{key}:carrier",
                        profile["carrier"],
                        "carrier",
                        owner,
                        transform,
                    ),
                ]
            )
        return result

    def supports(self, designs):
        return [
            self.item(f"{key}:dock", mesh, "support", name)
            for name, d in designs.items()
            for key, mesh in d["docks"].items()
        ]

    def planes(self, shells, planes):
        result = []
        for name, plane in planes.items():
            normal = np.asarray(plane["normal"])
            transform = trimesh.geometry.align_vectors([0, 0, 1], normal)
            bounds = shells[name].bounds
            center = bounds.mean(0)
            center += normal * (plane["offset_mm"] - center @ normal)
            transform[:3, 3] = center
            extent = float(np.linalg.norm(np.ptp(bounds, axis=0)))
            mesh = trimesh.creation.box([extent, extent, 0.15])
            result.append(
                self.item(f"{name}:cut_plane", mesh, "plane", name, transform)
            )
        return result

    def capture(
        self,
        stage,
        title,
        changes,
        instances=None,
        metrics=None,
        state="recorded",
        frame="world_assembly_mm",
        inspection=None,
        gallery=None,
    ):
        previous = self.data["events"][-1] if self.data["events"] else None
        if instances is None:
            instances = previous["instances"] if previous else []
            if previous:
                changes = [
                    *changes,
                    "This event records a decision; the view retains the previous complete geometric snapshot.",
                ]
        items = {i["id"]: i for i in instances}
        if len(items) != len(instances):
            raise ValueError("Duplicate instance in generation snapshot")
        old = {i["id"]: i for i in previous["instances"]} if previous else {}
        common = items.keys() & old.keys()
        same_frame = previous is not None and previous["coordinate_frame"] == frame
        moved = [
            key
            for key in sorted(common)
            if same_frame
            and not np.allclose(
                items[key]["transform"], old[key]["transform"], atol=1e-7
            )
        ]
        geometry = [
            key for key in sorted(common) if items[key]["asset"] != old[key]["asset"]
        ]
        geometry_details = []
        for key in geometry:
            before = self.data["assets"][old[key]["asset"]]["source_volume_mm3"]
            after = self.data["assets"][items[key]["asset"]]["source_volume_mm3"]
            geometry_details.append(
                {
                    "id": key,
                    "before_volume_mm3": before,
                    "after_volume_mm3": after,
                    "volume_delta_mm3": after - before,
                }
            )
        provenance = self.data.get("model_stages", {}).get(
            stage, self.data["model_provenance"]
        )
        source = provenance["source"]
        model_stage = stage in {
            "architecture_ranking",
            "layout_constraints",
        } and source in {
            "live_llm",
            "saved_schema_replay",
        }
        compute = {
            "kind": "vlm_live"
            if model_stage and provenance["live_model_called"]
            else "vlm_replay"
            if model_stage
            else "local",
            "label": "Live VLM ranking"
            if model_stage and provenance["live_model_called"]
            else "VLM ranking replay"
            if model_stage
            else "Local algorithm",
            "live_model_call_at_this_step": bool(
                model_stage and provenance["live_model_called"]
            ),
            "model_scope": "Ranks supplied opening candidates only. Placement, supports, routing and fasteners are computed locally.",
        }
        if stage == "layout_constraints" and model_stage:
            compute["label"] = (
                "Live VLM restrict schema"
                if provenance["live_model_called"]
                else "VLM restrict schema replay"
            )
            compute["model_scope"] = (
                "Generates bounded bundle-center domains, allowed mounting poses and soft preferences. Local seed generation and EPS enforce the validated schema."
            )
        compute["description"] = (
            "The VLM generates the restricted carrier-layout schema here; it does not generate numeric poses."
            if stage == "layout_constraints" and compute["kind"] == "vlm_live"
            else "Saved VLM restrict schema is read and validated here; no inference request is sent."
            if stage == "layout_constraints" and compute["kind"] == "vlm_replay"
            else "The VLM ranks the supplied opening candidates here."
            if compute["kind"] == "vlm_live"
            else "Saved VLM opening ranking is read here; no inference request is sent."
            if compute["kind"] == "vlm_replay"
            else "This step runs locally. It does not call a language model."
        )
        if inspection:
            inspection = json_value(
                {**self.data["candidates"].get(inspection["key"], {}), **inspection}
            )
            inspection.setdefault("attempt_event_index", len(self.data["events"]))
            if state in {"rejected", "geometry_candidate_passed"}:
                inspection["result_event_index"] = len(self.data["events"])
            self.data["candidates"][inspection["key"]] = inspection
        event = json_value(
            {
                "index": len(self.data["events"]),
                "stage": stage,
                "title": title,
                "elapsed_seconds": round(time.monotonic() - self.started, 3),
                "state": state,
                "coordinate_frame": frame,
                "changes": changes,
                "metrics": metrics or {},
                "instances": instances,
                "inspection": inspection,
                "gallery": gallery or [],
                "compute": compute,
                "delta": {
                    "added": sorted(items.keys() - old.keys()),
                    "removed": sorted(old.keys() - items.keys()),
                    "moved": moved,
                    "geometry_changed": geometry,
                    "geometry_details": geometry_details,
                    "coordinate_frame_changed": previous is not None and not same_frame,
                },
            }
        )
        self.data["events"].append(event)
        self.write()

    def finish(self, status, metrics=None):
        self.data["status"] = status
        self.data["result"] = json_value(metrics or {})
        self.write()

    def write(self):
        from .process_evidence import attach_evidence
        from .process_presentation import pipeline_presentation

        attach_evidence(self.data, self.output)
        self.data["presentation"] = pipeline_presentation(self.data)
        dump(self.output / "process_trace.json", self.data)
        lines = [
            "# Recorded generation process",
            "",
            self.data["scope"],
            "",
            self.data["preview_note"],
            "",
        ]
        for event in self.data["events"]:
            lines.extend(
                [
                    f"## {event['index'] + 1}. {event['title']}",
                    "",
                    f"Stage: {event['stage']}; state: {event['state']}; recorded at {event['elapsed_seconds']} seconds after start.",
                    f"Computation: {event['compute']['label']}; live model call at this step: {event['compute']['live_model_call_at_this_step']}.",
                    "",
                ]
            )
            lines.extend(f"- {change}" for change in event["changes"])
            lines.append(
                f"- Snapshot changes: added {len(event['delta']['added'])}, removed {len(event['delta']['removed'])}, moved {len(event['delta']['moved'])}, geometry changed {len(event['delta']['geometry_changed'])}."
            )
            lines.extend(
                f"- {g['id']}: material volume change {g['volume_delta_mm3']:+.3f} mm³."
                for g in event["delta"]["geometry_details"]
            )
            lines.extend(
                [
                    "",
                    "```json",
                    json.dumps(event["metrics"], ensure_ascii=False, indent=2),
                    "```",
                    "",
                ]
            )
        (self.output / "process_changes.md").write_text("\n".join(lines))
        template = Path(__file__).with_name("process_viewer.html").read_text()
        payload = json.dumps(
            self.data, ensure_ascii=False, separators=(",", ":"), allow_nan=False
        ).replace("<", "\\u003c")
        temp = self.output / "process.html.tmp"
        renderer = Path(__file__).with_name("process_renderer.js").read_text()
        evidence = (
            Path(__file__).with_name("process_evidence.js").read_text()
            + "\n"
            + Path(__file__).with_name("process_schema_guides.js").read_text()
        )
        template = template.replace("__PROCESS_EVIDENCE__", evidence)
        temp.write_text(
            template.replace("__PROCESS_RENDERER__", renderer).replace(
                "__PROCESS_DATA__", payload
            )
        )
        temp.replace(self.output / "process.html")
