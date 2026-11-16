from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from common import read
from optimization import evaluator_for
from semantic_bridge import validate_proposal
from voxel_tool.stages.placement.planner import Placement, PlacementResult
from voxel_tool.stages.placement.semantic import SemanticEvaluation

HERE = Path(__file__).resolve().parents[1]


def inputs():
    raw = read(HERE / "tests/fixtures/layout_schema_replay.json")
    schema = read(HERE / "outputs/cactus/components.json")
    components = [
        c
        for c in schema["components"]
        if c["id"] in {"U1", "S1", "BATTERY", "REGULATOR"}
    ]
    return raw, components, read(HERE / "configs/cactus.json")


def test_schema_cannot_relax_geometry_floor_or_add_unknown_components():
    raw, components, config = inputs()
    raw["components"][0]["clearance_mm"] = 0
    proposal, guards = validate_proposal(raw, components, config)
    assert proposal.policies[0].clearance_mm == config["component_clearance_mm"]
    assert guards[0]["clearance_mm"] == config["component_clearance_mm"]
    raw["components"][0]["component_id"] = "UNKNOWN"
    with pytest.raises(ValueError, match="Unknown"):
        validate_proposal(raw, components, config)


def test_unsupported_external_access_is_rejected_not_silently_relaxed():
    raw, components, config = inputs()
    raw["components"][0]["require_exterior_access"] = True
    with pytest.raises(ValueError, match="no constraints were relaxed"):
        validate_proposal(raw, components, config)


def test_eps_receives_semantic_hard_constraints_and_soft_cost_without_fixed_motors():
    motor = Placement(
        "M_1",
        "M",
        "motor",
        (1.0, 1.0, 1.0),
        (1.0, 1.0, 1.0),
        (1, 1, 1),
        (1, 1, 1),
        (1.0, 0.0, 0.0),
    )
    board = replace(
        motor,
        instance_id="U_1",
        component_id="U",
        voxel_start=(4, 4, 4),
        center_mm=(4.0, 4.0, 4.0),
    )
    initial = PlacementResult((motor, board), (), 0.0)

    def semantic(candidate):
        assert [p.component_id for p in candidate.placements] == ["U"]
        cost = candidate.placements[0].center_mm[0]
        return SemanticEvaluation(
            cost <= 4, cost, () if cost <= 4 else ("test_hard_relation",)
        )

    free = np.ones((8, 8, 8), bool)
    evaluate = evaluator_for(
        {"motor_mapping": {"M": "motor"}, "placement_part": "body"},
        {"parts": ["body"]},
        {"usable": free, "part_ids": np.ones(free.shape)},
        initial,
        semantic,
    )
    assert evaluate(initial).soft_cost == 4
    moved = replace(
        initial,
        placements=(
            motor,
            replace(board, center_mm=(5.0, 4.0, 4.0), voxel_start=(5, 4, 4)),
        ),
    )
    result = evaluate(moved)
    assert not result.feasible
    assert "test_hard_relation" in result.violations
    assert result.soft_cost == 5
