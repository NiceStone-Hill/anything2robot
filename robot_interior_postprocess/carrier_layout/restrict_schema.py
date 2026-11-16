"""Bounded VLM carrier policies compiled into initial placement and every EPS move."""

import itertools
import json
from copy import deepcopy
from pathlib import Path

import numpy as np
from common import dump, read
from shell_partition.io import fingerprint


def _number(value, lower, upper):
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and np.isfinite(value)
        and lower <= value <= upper
    )


def _strings(value):
    return isinstance(value, list) and all(isinstance(s, str) for s in value)


def restriction_context(profiles, space, scene, catalog, gravity, floor):
    domains = {}
    for i, link in enumerate(scene["parts"], 1):
        indices = np.argwhere(space["usable"] & (space["part_ids"] == i))
        if len(indices):
            points = np.asarray(space["origin"]) + indices * float(space["pitch"])
            domains[link] = [points.min(0).tolist(), points.max(0).tolist()]
    components = []
    for p in profiles:
        spec = p["spec"]
        link = spec["owner_link"]
        if link not in domains:
            raise ValueError(f"{link}: no usable cavity for carrier restrictions")
        normals = sorted(
            {
                tuple(c["normal"])
                for c in catalog[link]
                if np.isclose(np.max(np.abs(c["normal"])), 1)
            }
        )
        components.append(
            {
                "instance_id": spec["id"],
                "component_id": spec["component_id"],
                "owner_link": link,
                "bounds_local_mm": p["bounds"].tolist(),
                "allowed_rolls_deg": list(p["rolls"]),
                "allowed_opening_normals": [list(n) for n in normals],
                "mass_kg": spec.get("mass_kg"),
                "mounting_pending": p["pending"],
            }
        )
    return {
        "schema_version": 1,
        "units": "mm",
        "components": components,
        "owner_cavity_bounds_mm": domains,
        "gravity": gravity,
        "clearance_floor_mm": floor,
        "fixed_motors": scene.get("motors", []),
        "opening_candidates": catalog,
        "contract": "Fixed motors, component ownership and mounting geometry. Only bundle-center bounds, enumerated poses and bounded soft preferences.",
    }


def local_schema(context):
    return {
        "schema_version": 1,
        "units": "mm",
        "components": [
            {
                **{
                    k: c[k]
                    for k in (
                        "instance_id",
                        "owner_link",
                        "allowed_opening_normals",
                        "allowed_rolls_deg",
                    )
                },
                "allowed_center_normalized": [[0, 1]] * 3,
                "min_clearance_mm": context["clearance_floor_mm"],
                "vertical_preference": "neutral",
                "preference_weight": 0,
                "evidence": "Explicit local rules; not VLM-generated.",
                "unknowns": [],
            }
            for c in context["components"]
        ],
        "relations": [],
        "unknowns": ["Local rules only; no model inference."],
    }


def validate_schema(raw, context):
    if (
        not isinstance(raw, dict)
        or set(raw)
        != {"schema_version", "units", "components", "relations", "unknowns"}
        or type(raw["schema_version"]) is not int
        or raw["schema_version"] != 1
        or raw["units"] != "mm"
        or not isinstance(raw["components"], list)
        or not isinstance(raw["relations"], list)
        or not _strings(raw["unknowns"])
    ):
        raise ValueError(
            "Restricted carrier schema requires version 1, mm, components, relations and unknowns only"
        )
    expected = {c["instance_id"]: c for c in context["components"]}
    received = set()
    guards = []
    normalized = deepcopy(raw)
    for p in normalized["components"]:
        fields = {
            "instance_id",
            "owner_link",
            "allowed_center_normalized",
            "allowed_opening_normals",
            "allowed_rolls_deg",
            "min_clearance_mm",
            "vertical_preference",
            "preference_weight",
            "evidence",
            "unknowns",
        }
        if not isinstance(p, dict) or set(p) != fields:
            raise ValueError("Invalid restricted carrier component fields")
        instance = p["instance_id"]
        if (
            not isinstance(instance, str)
            or instance not in expected
            or instance in received
        ):
            raise ValueError(
                "Unknown/repeated carrier instance; fixed motors are not movable components"
            )
        received.add(instance)
        original = expected[instance]
        if p["owner_link"] != original["owner_link"]:
            raise ValueError("VLM cannot change component ownership")
        region = p["allowed_center_normalized"]
        if (
            not isinstance(region, list)
            or len(region) != 3
            or any(
                not isinstance(r, list)
                or len(r) != 2
                or not all(_number(v, 0, 1) for v in r)
                or r[0] > r[1]
                for r in region
            )
        ):
            raise ValueError(
                "Normalized bundle-center bounds must be three intervals in [0,1]"
            )
        rolls = p["allowed_rolls_deg"]
        if (
            not isinstance(rolls, list)
            or not rolls
            or any(
                not _number(r, 0, 270) or r not in original["allowed_rolls_deg"]
                for r in rolls
            )
            or len(set(rolls)) != len(rolls)
        ):
            raise ValueError(
                "VLM rolls must be a nonempty subset of configured atomic rolls"
            )
        normals = p["allowed_opening_normals"]
        if (
            not isinstance(normals, list)
            or not normals
            or any(
                not isinstance(n, list)
                or len(n) != 3
                or not all(_number(v, -1, 1) for v in n)
                or n not in original["allowed_opening_normals"]
                for n in normals
            )
            or len({tuple(n) for n in normals}) != len(normals)
        ):
            raise ValueError(
                "VLM opening normals must be unique supplied cardinal choices"
            )
        if (
            not _number(
                p["min_clearance_mm"], 0, max(10, context["clearance_floor_mm"])
            )
            or p["vertical_preference"] not in ("low", "neutral", "high")
            or not _number(p["preference_weight"], 0, 5)
            or not isinstance(p["evidence"], str)
            or not p["evidence"].strip()
            or not _strings(p["unknowns"])
        ):
            raise ValueError("Invalid clearance, bounded preference or evidence")
        requested = p["min_clearance_mm"]
        p["min_clearance_mm"] = max(requested, context["clearance_floor_mm"])
        guards.append(
            {
                "instance_id": instance,
                "requested_clearance_mm": requested,
                "effective_clearance_mm": p["min_clearance_mm"],
            }
        )
    if received != set(expected):
        raise ValueError(
            "Restricted schema must cover every movable carrier instance exactly once"
        )
    relations = set()
    for r in normalized["relations"]:
        if (
            not isinstance(r, dict)
            or set(r) != {"type", "a", "b", "weight", "evidence"}
            or r["type"] not in {"near", "coplanar", "separate_projection"}
            or not isinstance(r["a"], str)
            or not isinstance(r["b"], str)
            or r["a"] not in expected
            or r["b"] not in expected
            or r["a"] == r["b"]
            or expected[r["a"]]["owner_link"] != expected[r["b"]]["owner_link"]
            or not _number(r["weight"], 0, 5)
            or not isinstance(r["evidence"], str)
            or not r["evidence"].strip()
        ):
            raise ValueError("Invalid bounded same-link soft relation")
        token = (r["type"], *sorted((r["a"], r["b"])))
        if token in relations:
            raise ValueError("Duplicate soft relation")
        relations.add(token)
    return normalized, guards


class CompiledRestrictions:
    def __init__(self, raw, context):
        self.schema, self.guards = validate_schema(raw, context)
        self.context = context
        self.policies = {p["instance_id"]: p for p in self.schema["components"]}
        self.bounds = {
            k: np.asarray(v, float)
            for k, v in context["owner_cavity_bounds_mm"].items()
        }
        up = -np.asarray(context["gravity"]["world_m_s2"], float)
        if up.shape != (3,) or not np.isfinite(up).all() or np.linalg.norm(up) < 1e-9:
            raise ValueError("Restrict schema requires explicit nonzero finite gravity")
        self.up = up / np.linalg.norm(up)

    def violations(self, pose):
        p = self.policies[pose["id"]]
        b = self.bounds[p["owner_link"]]
        region = np.asarray(p["allowed_center_normalized"])
        lower = b[0] + region[:, 0] * (b[1] - b[0])
        upper = b[0] + region[:, 1] * (b[1] - b[0])
        issues = []
        if np.any(pose["center"] < lower - 1e-6) or np.any(
            pose["center"] > upper + 1e-6
        ):
            issues.append("bundle_center_outside_schema_region")
        if pose["roll"] not in p["allowed_rolls_deg"]:
            issues.append("roll_not_allowed_by_schema")
        if not any(
            np.allclose(pose["transform"][:3, 2], n)
            for n in p["allowed_opening_normals"]
        ):
            issues.append("opening_normal_not_allowed_by_schema")
        return issues

    def clearance(self, pose, others, floor):
        return max(
            [
                floor,
                self.policies[pose["id"]]["min_clearance_mm"],
                *[self.policies[p["id"]]["min_clearance_mm"] for p in others],
            ]
        )

    def soft_cost(self, poses, normal):
        lookup = {p["id"]: p for p in poses}
        value = 0.0
        for instance, pose in lookup.items():
            p = self.policies[instance]
            b = self.bounds[p["owner_link"]]
            corners = np.array(list(itertools.product(*zip(b[0], b[1], strict=True))))
            heights = corners @ self.up
            h = np.clip(
                (pose["center"] @ self.up - heights.min())
                / max(float(np.ptp(heights)), 1),
                0,
                1,
            )
            value += p["preference_weight"] * (
                h
                if p["vertical_preference"] == "low"
                else 1 - h
                if p["vertical_preference"] == "high"
                else 0
            )
        lateral = np.flatnonzero(np.abs(normal) < 0.5)
        for r in self.schema["relations"]:
            if r["a"] not in lookup or r["b"] not in lookup:
                continue
            a, b = lookup[r["a"]], lookup[r["b"]]
            domain = self.bounds[self.policies[r["a"]]["owner_link"]]
            scale = max(float(np.linalg.norm(domain[1] - domain[0])), 1)
            if r["type"] == "near":
                cost = np.linalg.norm(a["center"] - b["center"]) / scale
            elif r["type"] == "coplanar":
                cost = (
                    abs((a["transform"][:3, 3] - b["transform"][:3, 3]) @ normal)
                    / scale
                )
            else:
                overlap = np.maximum(
                    0,
                    np.minimum(a["bounds"][1, lateral], b["bounds"][1, lateral])
                    - np.maximum(a["bounds"][0, lateral], b["bounds"][0, lateral]),
                )
                area = min(
                    np.prod(np.diff(a["bounds"], axis=0)[0, lateral]),
                    np.prod(np.diff(b["bounds"], axis=0)[0, lateral]),
                )
                cost = np.prod(overlap) / max(float(area), 1)
            value += r["weight"] * float(cost)
        return float(value)

    def report(self, poses, require_all=False):
        rows = [
            {"instance_id": p["id"], "violations": self.violations(p)} for p in poses
        ]
        ids = [p["id"] for p in poses]
        missing = sorted(self.policies.keys() - set(ids)) if require_all else []
        return {
            "all_hard_constraints_pass": not missing
            and len(ids) == len(set(ids))
            and all(not p["violations"] for p in rows),
            "missing_instances": missing,
            "components": rows,
            "scope": "Schema bounds, normal and roll; cavity, spacing, exact supports and tool paths are checked separately.",
        }


def prepare_restrictions(
    context, images, out, mode="rules", replay=None, env_file=None, sources=None
):
    dump(out / "layout_constraints_context.json", context)
    # Mechanical adapters may supply NumPy motor vectors; the saved JSON is
    # the exact plain-data context used by both the model and compiler.
    context = read(out / "layout_constraints_context.json")
    prompt = (
        Path(__file__).with_name("restrict_schema_prompt.txt").read_text()
        + "\nCONTEXT_JSON="
        + json.dumps(context)
    )
    (out / "layout_constraints_prompt.txt").write_text(prompt)
    if replay:
        path = Path(replay).resolve()
        raw = read(path)
        if sources is not None:
            sources[str(path)] = fingerprint(path)
        provenance = {
            "source": "saved_schema_replay",
            "path": str(path),
            "live_model_called": False,
        }
        origin = path.parent / "schema_provenance.json"
        if origin.is_file():
            data = read(origin)
            if sources is not None:
                sources[str(origin)] = fingerprint(origin)
            provenance["original_run"] = {
                k: data.get(k) for k in ("model", "source", "live_model_called")
            }
            provenance["original_run"]["evidence_sha256"] = fingerprint(origin)
    elif mode == "llm":
        from voxel_tool.application.llm_agent import (
            FunctionalAnnotationClient,
            load_agent_config,
        )

        config = load_agent_config(env_file)
        client = FunctionalAnnotationClient(
            config, progress=lambda s: print(s, flush=True)
        )
        provenance = {
            "source": "live_llm",
            "model": config.model,
            "live_model_called": True,
        }
        dump(out / "schema_provenance.json", provenance)
        for attempt in range(3):
            print(f"VLM carrier restrict schema, attempt {attempt + 1}", flush=True)
            (out / f"layout_constraints_prompt_attempt_{attempt + 1}.txt").write_text(
                prompt
            )
            raw = client._request_json(
                prompt=prompt,
                image_paths=tuple(images),
                max_tokens=7000,
                system_prompt="Return only the requested restricted carrier-layout JSON. Geometry and physical assembly remain unverified.",
            )
            dump(out / f"layout_schema_attempt_{attempt + 1:03d}.json", raw)
            dump(out / "layout_constraints_raw.json", raw)
            try:
                validate_schema(raw, context)
                break
            except ValueError as exc:
                if attempt == 2:
                    raise
                prompt += (
                    "\nSCHEMA CORRECTION: "
                    + str(exc)
                    + ". Return a complete replacement."
                )
    elif mode == "rules":
        raw = local_schema(context)
        provenance = {"source": "local_rules", "live_model_called": False}
    else:
        raise ValueError(
            "Carrier layout_constraints.mode must be llm or rules; replay requires an explicit proposal path"
        )
    dump(out / "layout_constraints_raw.json", raw)
    dump(out / "schema_provenance.json", provenance)
    compiled = CompiledRestrictions(raw, context)
    dump(out / "layout_constraints.json", compiled.schema)
    dump(
        out / "semantic_guardrails.json",
        {
            "guards": compiled.guards,
            "hard_constraints_never_relaxed": True,
            "scope": "Local clearance floor and configured poses/ownership preserved; bounded soft preferences only.",
        },
    )
    return compiled, provenance
