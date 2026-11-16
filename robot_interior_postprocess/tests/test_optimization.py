from dataclasses import replace

import numpy as np
from optimization import evaluator_for
from voxel_tool.domain.labels import VoxelLabel
from voxel_tool.domain.voxel import VoxelGrid
from voxel_tool.stages.placement.planner import Placement, PlacementResult
from voxel_tool.stages.routing.polish import polish_routing_bends
from voxel_tool.stages.routing.router import RouteBranch, RoutingResult


def test_eps_never_moves_fixed_motor_or_enters_blocked_cavity():
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
    initial = PlacementResult((motor, board), (), 0)
    free = np.ones((8, 8, 8), bool)
    free[6, 6, 6] = False
    evaluate = evaluator_for(
        {"motor_mapping": {"M": "motor"}, "placement_part": "body"},
        {"parts": ["body"]},
        {"usable": free, "part_ids": np.ones(free.shape)},
        initial,
    )
    assert evaluate(initial).feasible
    assert not evaluate(
        replace(initial, placements=(replace(motor, center_mm=(2.0, 1.0, 1.0)), board))
    ).feasible
    assert not evaluate(
        replace(initial, placements=(motor, replace(board, voxel_start=(6, 6, 6))))
    ).feasible


def test_polish_reduces_bends_and_preserves_endpoints_and_length():
    labels = np.full((7, 7, 3), int(VoxelLabel.INTERIOR_AIR), np.uint8)
    grid = VoxelGrid(np.zeros_like(labels, bool), labels, 1.0, np.zeros(3))
    path = ((1, 1, 1), (2, 1, 1), (2, 2, 1), (3, 2, 1), (3, 3, 1))
    branch = RouteBranch(
        "net",
        "net:1",
        "a",
        "b",
        "1",
        "1",
        None,
        None,
        path,
        path,
        4.0,
        3,
        (0.2, 0.4, 0.8),
    )
    polished, report = polish_routing_bends(
        grid, PlacementResult((), (), 0), RoutingResult((branch,), (), 4.0, 3)
    )
    assert polished.total_bends == 1
    assert polished.total_length_mm == 4.0
    assert polished.routes[0].voxel_path[0] == path[0]
    assert polished.routes[0].voxel_path[-1] == path[-1]
    assert report.total_bends_after < report.total_bends_before
