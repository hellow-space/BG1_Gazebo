#!/usr/bin/env python3
"""Generate deterministic low-poly BG1 body and wrap assets."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile

from PIL import Image, ImageDraw, ImageFont
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mesh_primitives import collada_text, ShellBuilder  # noqa: E402


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def atomic_write(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as handle:
        temporary = Path(handle.name)
        handle.write(data)
    os.replace(temporary, path)


def label_png():
    image = Image.new("RGBA", (1024, 320), (231, 235, 237, 255))
    draw = ImageDraw.Draw(image)
    # Bottom atlas row is sampled only by solid surfaces; the upper field is a local decal.
    palette = [(231,235,237), (28,35,41), (64,74,80), (139,153,162),
               (15,22,29), (58,176,193), (199,44,42), (212,220,224)]
    for index, color in enumerate(palette):
        draw.rectangle((index*128, 280, (index+1)*128-1, 319), fill=(*color,255))
    font_path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf")
    if font_path.exists():
        name_font = ImageFont.truetype(str(font_path), 84)
        model_font = ImageFont.truetype(str(font_path), 46)
    else:
        name_font = ImageFont.load_default()
        model_font = ImageFont.load_default()
    draw.text((75, 70), "PUDU", font=name_font, fill=(53,64,72,255))
    draw.line((430, 86, 430, 165), fill=(153,165,171,255), width=3)
    draw.text((485, 96), "BG1 Pro", font=model_font, fill=(73,87,96,255))
    with tempfile.SpooledTemporaryFile() as stream:
        image.save(stream, format="PNG", optimize=False, compress_level=9)
        stream.seek(0)
        return stream.read()


def build_assets(geometry_path, output_root):
    geometry_path = Path(geometry_path)
    output_root = Path(output_root)
    geometry_bytes = geometry_path.read_bytes()
    geometry = yaml.safe_load(geometry_bytes)

    body_height = geometry["overall_height_m"] - geometry["body_bottom_z_m"]
    body_size = [geometry["body_length_m"], geometry["body_width_m"], body_height]
    body_center = [
        (geometry["front_extent_m"] + geometry["body_rear_extent_m"]) / 2.0,
        0.0,
        geometry["body_bottom_z_m"] + body_height / 2.0,
    ]
    shell = ShellBuilder()
    # Ratios preserve the manual envelope when dimensions are edited.
    sx, sy, sz = body_size[0]/1.120, body_size[1]/.660, geometry['overall_height_m']/1.303
    def part(cx, cy, rings, color):
        shell.rounded(cx*sx, cy*sy, [(z*sz, x*sx, y*sy, r*min(sx,sy)) for z,x,y,r in rings], color)

    # Floating dark chassis, pale body, seam, and tapered tank cover.
    part(.28, 0, [(.19,1.02,.58,.07),(.23,1.10,.64,.08),(.34,1.10,.64,.08)], 1)
    part(.28, 0, [(.34,1.08,.62,.07),(.39,1.10,.66,.085),
                 (1.12,1.10,.66,.085),(1.19,1.06,.62,.10)], 0)
    part(.28, 0, [(1.19,1.06,.62,.10),(1.202,1.06,.62,.10)], 2)
    part(.27, 0, [(1.202,1.06,.62,.10),(1.245,1.02,.60,.10),
                 (1.27,.94,.55,.10)], 7)
    # Recessed sensor spine on the front (+X), lower bumper and light bar.
    part(.833,0,[(.69,.014,.16,.006),(1.105,.014,.16,.006)],1)
    part(.840,0,[(.925,.009,.095,.004),(.99,.009,.095,.004)],4)
    part(.840,0,[(.795,.009,.060,.004),(.828,.009,.060,.004)],3)
    part(.805,0,[(.51,.065,.52,.025),(.55,.065,.52,.025)],1)
    part(.841,0,[(.526,.008,.34,.003),(.535,.008,.34,.003)],5)
    # Top lidar cap, rear console, red emergency stop; visual cues only.
    part(.56,0,[(1.27,.10,.10,.045),(1.303,.10,.10,.045)],1)
    part(-.205,0,[(1.12,.035,.38,.015),(1.18,.035,.38,.015)],1)
    part(-.225,0,[(1.135,.01,.22,.004),(1.17,.01,.22,.004)],4)
    part(-.15,.20,[(1.26,.035,.035,.016),(1.285,.035,.035,.016)],6)
    # Side branding is local and correctly oriented on both faces, never on the roof.
    for side in (-1,1):
        x0, x1 = (.08,.58) if side < 0 else (.58,.08)
        shell.decal([(x0*sx, side*.3305*sy,.965*sz),(x1*sx,side*.3305*sy,.965*sz),
                     (x1*sx,side*.3305*sy,1.085*sz),(x0*sx,side*.3305*sy,1.085*sz)])
        # Fine panel seam and recessed latch give scale without mechanical detail.
        def side_patch(x0, x1, z0, z1, color):
            if side > 0:
                x0, x1 = x1, x0
            shell.face([(x0*sx, side*.3307*sy,z0*sz),(x1*sx,side*.3307*sy,z0*sz),
                        (x1*sx,side*.3307*sy,z1*sz),(x0*sx,side*.3307*sy,z1*sz)],
                       ((color+.5)/8,.05))
        side_patch(-.15,.69,.59,.594,3)
        side_patch(-.15,-.146,.594,.92,3)
        side_patch(.686,.69,.594,.92,3)
        side_patch(-.15,.69,.918,.922,3)
        side_patch(.235,.30,.606,.63,2)
    mesh = shell.mesh()
    mesh_bytes = collada_text(mesh).encode("utf-8")
    texture_bytes = label_png()

    mesh_path = output_root / "meshes" / "bg1_body.dae"
    texture_path = output_root / "materials" / "textures" / "bg1_label.png"
    atomic_write(mesh_path, mesh_bytes)
    atomic_write(texture_path, texture_bytes)

    manifest = {
        "asset_version": 2,
        "style": "rounded_industrial_shell_local_decals",
        "body_center_m": body_center,
        "body_size_m": body_size,
        "geometry_sha256": sha256_bytes(geometry_bytes),
        "mesh_sha256": sha256_bytes(mesh_bytes),
        "texture_sha256": sha256_bytes(texture_bytes),
        "triangle_count": mesh.triangle_count,
    }
    manifest_bytes = (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode("utf-8")
    atomic_write(output_root / "meshes" / "manifest.json", manifest_bytes)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    args = parser.parse_args()
    manifest = build_assets(args.geometry, args.output_root)
    print(json.dumps(manifest, sort_keys=True))


if __name__ == "__main__":
    main()
