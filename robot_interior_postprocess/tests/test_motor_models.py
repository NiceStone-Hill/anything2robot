import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from motor_models import assign_motor_models


def test_mixed_models_preserve_geometry():
    motors = [{"id": "motor_base", "radius_mm": 26.5}, {"id": "motor_arm"}]
    result = assign_motor_models(motors, {"motor_model_mapping": {
        "motor_base": "DM6006", "motor_arm": "MG4005V2",
    }})
    assert [a["model_id"] for a in result["assignments"]] == ["DM6006", "MG4005V2"]
    assert motors[0]["radius_mm"] == 26.5
    assert motors[1]["hardware_assignment"]["compatibility_status"] == "unverified"


def test_no_implicit_model_inference():
    result = assign_motor_models([{"id": "m", "radius_mm": 26.5}], {})
    assert result["assignments"][0]["model_id"] is None


@pytest.mark.parametrize("motors,config", [
    ([{"id": "m"}], {"motor_model_mapping": {"typo": "MG4005V2"}}),
    ([{"id": "m"}], {"motor_model_mapping": {"m": "missing"}}),
    ([{"id": "m"}, {"id": "m"}], {}),
])
def test_reject_invalid_mapping(motors, config):
    with pytest.raises(ValueError):
        assign_motor_models(motors, config)


def test_custom_catalog():
    result = assign_motor_models([{"id": "m"}], {
        "motor_model_catalog": {"custom": {"manufacturer": "example"}},
        "motor_model_mapping": {"m": "custom"},
    })
    assert result["assignments"][0]["model"]["manufacturer"] == "example"
