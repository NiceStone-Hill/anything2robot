"""Restricted split strategy proposals using the existing layout model client."""

import json
from pathlib import Path

from .io import write_json


def validate_proposal(raw, catalog):
    if (
        not isinstance(raw, dict)
        or set(raw) != {"schema_version", "parts", "unknowns"}
        or raw["schema_version"] != 1
        or not isinstance(raw["parts"], list)
    ):
        raise ValueError("Expected schema_version=1, parts and unknowns only")
    received = set()
    for row in raw["parts"]:
        if not isinstance(row, dict) or set(row) != {
            "part_id",
            "ranked_candidate_ids",
            "rationale",
            "assembly_notes",
            "unknowns",
        }:
            raise ValueError("Invalid part strategy fields")
        part = row["part_id"]
        if not isinstance(part, str) or part not in catalog or part in received:
            raise ValueError("Unknown or repeated part ID")
        received.add(part)
        ranked = row["ranked_candidate_ids"]
        allowed = {c["id"] for c in catalog[part]}
        if (
            not isinstance(ranked, list)
            or not 3 <= len(ranked) <= 6
            or not all(isinstance(c, str) for c in ranked)
            or len(set(ranked)) != len(ranked)
            or not set(ranked) <= allowed
        ):
            raise ValueError(f"{part}: rank 3-6 unique supplied candidate IDs")
        if not isinstance(row["rationale"], str) or not row["rationale"].strip():
            raise ValueError("Each strategy needs a rationale")
        for field in ("assembly_notes", "unknowns"):
            if not isinstance(row[field], list) or not all(
                isinstance(s, str) for s in row[field]
            ):
                raise ValueError(f"{field} must be a string list")
    if received != set(catalog):
        raise ValueError("Strategy must cover every link, including motor-only links")
    if not isinstance(raw["unknowns"], list) or not all(
        isinstance(s, str) for s in raw["unknowns"]
    ):
        raise ValueError("unknowns must be a string list")
    return raw


def propose(context, catalog, images, out, env_file, replay=None):
    architecture = Path(__file__).with_name("architecture_prompt.txt").read_text()
    prompt = architecture + "\nCONTEXT_JSON=" + json.dumps(context, ensure_ascii=False)
    if context.get("support_design"):
        prompt += (
            "\nSUPPORT CO-DESIGN: Each ranked split is evaluated with newly generated supports. "
            "Consider one-piece support ownership, root attachment, component insertion, "
            "cover closure and the immutable wire tubes. Compare bearing-pad/tray templates "
            "listed in support_design. Existing fused mounts are indivisible and cannot "
            "be cut. Keep the same output schema; geometry searches piece ownership and "
            "template variants. Unknown motor interfaces, retention and strength stay pending."
        )
    (out / "model_prompt.txt").write_text(prompt)
    if replay:
        raw = json.loads(Path(replay).read_text())
        write_json(out / "proposal_raw.json", raw)
        validate_proposal(raw, catalog)
        provenance = {
            "source": "saved_schema_replay",
            "path": str(Path(replay).resolve()),
            "live_model_called": False,
        }
    else:
        # Only this optional adapter depends on voxel_tool; no upstream edits.
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
        write_json(out / "model_provenance.json", provenance)
        for attempt in range(3):
            print(f"VLM split architecture proposal, attempt {attempt + 1}", flush=True)
            (out / f"model_prompt_attempt_{attempt + 1}.txt").write_text(prompt)
            raw = client._request_json(
                prompt=prompt,
                image_paths=tuple(images),
                max_tokens=6000,
                system_prompt="You are a mechanical assembly architect. Return only the requested restricted JSON. Geometry remains unverified.",
            )
            write_json(out / f"proposal_attempt_{attempt + 1}.json", raw)
            try:
                validate_proposal(raw, catalog)
                break
            except ValueError as exc:
                if attempt == 2:
                    raise
                prompt += (
                    "\nSCHEMA CORRECTION: "
                    + str(exc)
                    + ". Return a complete replacement."
                )
        write_json(out / "proposal_raw.json", raw)
    write_json(out / "model_provenance.json", provenance)
    return raw
