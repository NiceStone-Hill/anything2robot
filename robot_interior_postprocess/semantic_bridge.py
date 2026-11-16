"""Vision-model restricted constraints, with reproducible replay and safety guards."""

from dataclasses import replace
from pathlib import Path

import numpy as np
from common import ROOT, dump, read
from voxel_tool.application.llm_agent import (
    FunctionalAnnotationClient,
    load_agent_config,
)
from voxel_tool.presentation.shell_observation import export_shell_observation
from voxel_tool.stages.semantics.layout_constraints import (
    normalize_layout_constraint_proposal,
)

DEFAULT_ENV = Path(__file__).parent / ".env"


class RecordingClient(FunctionalAnnotationClient):
    def __init__(self, agent_config, out):
        super().__init__(
            agent_config, progress=lambda message: print(message, flush=True)
        )
        self.output = out
        self.attempt = 0

    def _request_json(self, **kwargs):
        value = super()._request_json(**kwargs)
        self.attempt += 1
        dump(self.output / f"layout_schema_attempt_{self.attempt:03d}.json", value)
        return value


def validate_proposal(raw, components, config):
    proposal = normalize_layout_constraint_proposal(raw, components)
    policies, guards = [], []
    for policy in proposal.policies:
        if policy.containment_mode != "interior_only" or policy.require_exterior_access:
            raise ValueError(
                f"{policy.component_id}: requested external access/partial containment is not supported by this closed-cavity adapter; no constraints were relaxed"
            )
        guarded = replace(
            policy,
            clearance_mm=max(policy.clearance_mm, config["component_clearance_mm"]),
            interface_access_mm=max(policy.interface_access_mm, 9),
            max_candidates=min(
                policy.max_candidates, config.get("semantic_candidate_budget", 128)
            ),
        )
        policies.append(guarded)
        guards.append(
            {
                "component": policy.component_id,
                "clearance_mm": guarded.clearance_mm,
                "interface_access_mm": guarded.interface_access_mm,
                "candidate_budget": guarded.max_candidates,
            }
        )
    return replace(proposal, policies=tuple(policies)), guards


def _cutaway(space, free, destination):
    from PIL import Image, ImageDraw

    xyz = np.argwhere(free)
    if not len(xyz):
        raise ValueError("No usable cavity for semantic observations")
    midpoint = np.rint(xyz.mean(axis=0)).astype(int)
    image = Image.new("RGB", (1500, 580), "white")
    draw = ImageDraw.Draw(image)
    draw.text(
        (20, 12),
        "CAVITY SECTIONS | cyan: usable | gray: material | red: fixed motor exclusion",
        fill="black",
    )
    for column, depth_axis in enumerate(range(3)):
        shape = [n for i, n in enumerate(free.shape) if i != depth_axis]
        data = np.full((*shape, 3), 245, np.uint8)
        sl = [slice(None)] * 3
        sl[depth_axis] = int(midpoint[depth_axis])
        sl = tuple(sl)
        data[space["material"][sl]] = [100, 110, 125]
        data[free[sl]] = [20, 190, 180]
        data[space["motor_block"][sl]] = [220, 60, 55]
        panel = Image.fromarray(np.flipud(data.transpose(1, 0, 2)))
        panel.thumbnail((470, 470), Image.Resampling.NEAREST)
        image.paste(panel, (column * 500 + 15, 65))
        draw.text(
            (column * 500 + 15, 40),
            f"Fixed {'XYZ'[depth_axis]} index {midpoint[depth_axis]}",
            fill="black",
        )
    path = destination / "06_cavity_sections.png"
    image.save(path)
    return path


def prepare_constraints(config, schema, grid, space, scene, fixed, free, out):
    settings = config.get("layout_constraints", {})
    mode = settings.get("mode", "rules")
    if mode == "rules":
        return None
    if mode not in {"llm", "replay"}:
        raise ValueError("layout_constraints.mode must be llm, replay or rules")
    movable = [
        c for c in schema["components"] if c["id"] not in config["motor_mapping"]
    ]
    source = out / "semantic_components.json"
    dump(source, {**schema, "components": movable})
    observation = export_shell_observation(
        grid,
        source,
        out / "layout_observation",
        title="Robot assembly: real hollow cavity, locked motors",
    )
    context = read(observation.context)
    sections = _cutaway(space, free, observation.directory)
    context["layout_image_files"].append(sections.name)
    context["fixed_mechanical_context"] = {
        "motors": scene["motors"],
        "locked_instances": [p.instance_id for p in fixed],
        "placement_part": config["placement_part"],
        "instruction": "Motors are locked obstacles, NOT movable BOM candidates. Never relocate them or add them to the output components.",
    }
    context["electrical_connectivity"] = read(out / "connectivity.json")
    context["adapter_contract"] = {
        "scope": "Static planning envelopes and provisional pin anchors, not manufacturing-certified CAD",
        "spatial_domain": "Only cyan usable cavity in the configured part; other air and fixed motor volumes are forbidden",
        "containment": "This adapter accepts interior_only. External operating access is not modeled; record it as unknown rather than falsely certified.",
        "clearance_floor_mm": config["component_clearance_mm"],
        "interface_access_floor_mm": 9,
        "max_candidates_per_component": config.get("semantic_candidate_budget", 128),
        "orientation": "Infer from available evidence; if ambiguous allow all plausible normals. Do not invent measured pins, front or gravity direction.",
    }
    dump(observation.context, context)
    context = read(observation.context)
    if mode == "llm":
        agent_config = load_agent_config(
            ROOT / settings.get("env_file", str(DEFAULT_ENV))
        )
        client = RecordingClient(agent_config, out)
        print("Requesting vision-model restricted layout schema", flush=True)
        images = (
            tuple(
                observation.directory / name for name in context["layout_image_files"]
            )
            + observation.component_sheets
        )
        raw = client.propose_layout_constraints(context=context, image_paths=images)
        provenance = {"source": "live_llm", "model": agent_config.model}
    else:
        proposal_path = (ROOT / settings["proposal"]).resolve()
        raw = read(proposal_path)
        provenance = {
            "source": "saved_schema_replay",
            "proposal": str(proposal_path),
            "note": "Replay is not evidence of a live LLM request",
        }
    dump(out / "layout_constraints_raw.json", raw)
    dump(out / "schema_provenance.json", provenance)
    proposal, guards = validate_proposal(raw, movable, config)
    dump(out / "layout_constraints.json", proposal.to_dict())
    dump(
        out / "semantic_guardrails.json",
        {
            "guards": guards,
            "note": "Minimum geometric clearances preserved; candidate caps only bound search resources. No semantic hard constraints silently relaxed.",
        },
    )
    return proposal
