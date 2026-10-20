from pathlib import Path
import re

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE.parent
GEOMETRY_PATH = PACKAGE / "config" / "bg1_geometry.yaml"
PHYSICS_PATH = PACKAGE / "config" / "bg1_physics.yaml"
FOOTPRINT_PATH = PACKAGE / "config" / "nav2_footprint.yaml"


def load(path):
    return yaml.safe_load(path.read_text())


def test_manual_dimensions_and_limits_are_authoritative():
    geometry = load(GEOMETRY_PATH)
    physics = load(PHYSICS_PATH)
    assert geometry["body_length_m"] == 1.120
    assert geometry["body_width_m"] == 0.660
    assert geometry["overall_length_m"] == 1.195
    assert geometry["overall_width_m"] == 0.760
    assert geometry["overall_height_m"] == 1.303
    assert geometry["max_linear_mps"] == 1.0
    assert geometry["routine_obstacle_height_m"] == 0.020
    assert geometry["maximum_obstacle_height_m"] == 0.025
    assert physics["total_mass_kg"] == 348.5


def test_axle_relative_envelope_closes_exactly():
    geometry = load(GEOMETRY_PATH)
    assert geometry["front_extent_m"] - geometry["rear_extent_m"] == pytest.approx(geometry["overall_length_m"])
    assert geometry["front_extent_m"] - geometry["body_rear_extent_m"] == pytest.approx(geometry["body_length_m"])
    assert 2 * geometry["half_width_m"] == pytest.approx(geometry["overall_width_m"])


def test_nav2_footprint_matches_maximum_envelope():
    geometry = load(GEOMETRY_PATH)
    footprint = load(FOOTPRINT_PATH)["footprint"]
    expected = [
        [geometry["front_extent_m"], geometry["half_width_m"]],
        [geometry["front_extent_m"], -geometry["half_width_m"]],
        [geometry["rear_extent_m"], -geometry["half_width_m"]],
        [geometry["rear_extent_m"], geometry["half_width_m"]],
    ]
    assert footprint == expected


def test_provisional_wheels_and_caster_fit_inside_envelope():
    geometry = load(GEOMETRY_PATH)
    wheel_outer_half_width = geometry["wheel_track_m"] / 2 + geometry["wheel_width_m"] / 2
    caster_front = geometry["caster_x_m"] + geometry["caster_radius_m"]
    assert wheel_outer_half_width <= geometry["half_width_m"]
    assert geometry["rear_extent_m"] < 0 < geometry["front_extent_m"]
    assert 0 < caster_front < geometry["front_extent_m"]
    assert geometry["caster_trail_m"] > 0
    assert geometry["parameter_status"] == "ESTIMATED_BASELINE"


def test_obsolete_model_values_are_absent():
    forbidden = {
        re.compile(r"\b344(?:\.0+)?\s*kg\b", re.IGNORECASE),
        re.compile(r"\b70\s*mm\b", re.IGNORECASE),
    }
    offenders = []
    for package in (PACKAGE, WORKSPACE_SRC / "bg1_gazebo"):
        if not package.exists():
            continue
        for path in package.rglob("*"):
            if not path.is_file() or path.suffix in {".pyc", ".png", ".dae"}:
                continue
            if path == Path(__file__).resolve():
                continue
            text = path.read_text(errors="ignore")
            if any(pattern.search(text) for pattern in forbidden):
                offenders.append(str(path.relative_to(WORKSPACE_SRC)))
    assert offenders == []
