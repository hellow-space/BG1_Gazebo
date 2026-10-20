import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from PIL import Image
import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
BUILDER = PACKAGE / "tools" / "build_visual_assets.py"
GEOMETRY = PACKAGE / "config" / "bg1_geometry.yaml"
PHYSICS = PACKAGE / "config" / "bg1_physics.yaml"
MODEL = PACKAGE / "urdf" / "bg1.urdf.xacro"


def load_builder():
    spec = importlib.util.spec_from_file_location("build_visual_assets", BUILDER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def build_into(root):
    module = load_builder()
    module.build_assets(GEOMETRY, root)


def generated_files(root):
    return {
        "mesh": root / "meshes" / "bg1_body.dae",
        "texture": root / "materials" / "textures" / "bg1_label.png",
        "manifest": root / "meshes" / "manifest.json",
    }


def expand(appearance):
    output = subprocess.check_output(
        [
            "xacro",
            str(MODEL),
            f"geometry:={GEOMETRY}",
            f"physics:={PHYSICS}",
            f"appearance:={appearance}",
        ],
        text=True,
    )
    return ET.fromstring(output)


def canonical_nonvisual(robot):
    root = copy.deepcopy(robot)
    for parent in root.iter():
        for child in list(parent):
            if child.tag == "visual":
                parent.remove(child)
    return ET.tostring(root, encoding="unicode")


def test_asset_build_is_deterministic(tmp_path):
    first = tmp_path / "first"
    second = tmp_path / "second"
    build_into(first)
    build_into(second)
    for key in generated_files(first):
        assert generated_files(first)[key].read_bytes() == generated_files(second)[key].read_bytes()


def test_wrapped_mesh_uses_authoritative_geometry(tmp_path):
    build_into(tmp_path)
    geometry = yaml.safe_load(GEOMETRY.read_text())
    manifest = json.loads(generated_files(tmp_path)["manifest"].read_text())
    assert manifest["geometry_sha256"] == hashlib.sha256(GEOMETRY.read_bytes()).hexdigest()
    assert manifest["body_size_m"] == pytest.approx(
        [
            geometry["body_length_m"],
            geometry["body_width_m"],
            geometry["overall_height_m"] - geometry["body_bottom_z_m"],
        ]
    )
    assert manifest["body_center_m"] == pytest.approx(
        [
            (geometry["front_extent_m"] + geometry["body_rear_extent_m"]) / 2,
            0,
            geometry["body_bottom_z_m"]
            + (geometry["overall_height_m"] - geometry["body_bottom_z_m"]) / 2,
        ]
    )


def test_mesh_triangle_budget_is_below_1500(tmp_path):
    build_into(tmp_path)
    manifest = json.loads(generated_files(tmp_path)["manifest"].read_text())
    assert 0 < manifest["triangle_count"] < 1500


def test_wrap_texture_is_present_and_legible_size(tmp_path):
    build_into(tmp_path)
    with Image.open(generated_files(tmp_path)["texture"]) as image:
        assert image.size == (1024, 320)
        assert image.mode == "RGBA"
        assert len(image.getcolors(maxcolors=1_000_000)) >= 4


def test_primitive_and_wrapped_nonvisual_signatures_match():
    primitive = expand("primitive")
    wrapped = expand("wrapped")
    assert canonical_nonvisual(primitive) == canonical_nonvisual(wrapped)
    wrapped_meshes = [mesh.get("filename") for mesh in wrapped.findall(".//visual/geometry/mesh")]
    primitive_meshes = [mesh.get("filename") for mesh in primitive.findall(".//visual/geometry/mesh")]
    assert "package://bg1_description/meshes/bg1_body.dae" in wrapped_meshes
    assert "package://bg1_description/meshes/bg1_body.dae" not in primitive_meshes
