"""Small deterministic Collada helpers for the BG1 visual shell."""

from dataclasses import dataclass
from pathlib import Path
import math
from xml.sax.saxutils import escape


@dataclass(frozen=True)
class TexturedMesh:
    positions: tuple
    normals: tuple
    texcoords: tuple
    triangles: tuple

    @property
    def triangle_count(self):
        return len(self.triangles)


def textured_box(center, size):
    """Return a 24-vertex box so each face has its own normal and UV quad."""
    cx, cy, cz = center
    hx, hy, hz = (value / 2.0 for value in size)
    corners = {
        "nbl": (cx - hx, cy - hy, cz - hz),
        "nbr": (cx - hx, cy + hy, cz - hz),
        "ntl": (cx - hx, cy - hy, cz + hz),
        "ntr": (cx - hx, cy + hy, cz + hz),
        "fbl": (cx + hx, cy - hy, cz - hz),
        "fbr": (cx + hx, cy + hy, cz - hz),
        "ftl": (cx + hx, cy - hy, cz + hz),
        "ftr": (cx + hx, cy + hy, cz + hz),
    }
    faces = (
        (("fbl", "fbr", "ftr", "ftl"), (1.0, 0.0, 0.0)),
        (("nbr", "nbl", "ntl", "ntr"), (-1.0, 0.0, 0.0)),
        (("fbr", "nbr", "ntr", "ftr"), (0.0, 1.0, 0.0)),
        (("nbl", "fbl", "ftl", "ntl"), (0.0, -1.0, 0.0)),
        (("ftl", "ftr", "ntr", "ntl"), (0.0, 0.0, 1.0)),
        (("nbl", "nbr", "fbr", "fbl"), (0.0, 0.0, -1.0)),
    )
    uv_quad = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0))
    positions = []
    normals = []
    texcoords = []
    triangles = []
    for names, normal in faces:
        start = len(positions)
        positions.extend(corners[name] for name in names)
        normals.extend(normal for _ in names)
        texcoords.extend(uv_quad)
        triangles.extend(((start, start + 1, start + 2), (start, start + 2, start + 3)))
    return TexturedMesh(tuple(positions), tuple(normals), tuple(texcoords), tuple(triangles))


def _flat(values):
    return " ".join(f"{component:.9g}" for row in values for component in row)


class ShellBuilder:
    """Rounded lofts and local decals sharing a compact texture atlas."""

    def __init__(self):
        self.positions, self.normals, self.texcoords, self.triangles = [], [], [], []

    def face(self, points, uv):
        a, b, c = points[:3]
        u = [b[i] - a[i] for i in range(3)]
        v = [c[i] - a[i] for i in range(3)]
        n = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
        length = math.sqrt(sum(t*t for t in n))
        n = tuple(t / length for t in n)
        start = len(self.positions)
        self.positions.extend(points)
        self.normals.extend([n] * len(points))
        self.texcoords.extend(uv if isinstance(uv, list) else [uv] * len(points))
        self.triangles.extend((start, start+i, start+i+1) for i in range(1, len(points)-1))

    def rounded(self, cx, cy, rings, color):
        # Ring tuples: z, length, width, corner radius. Counterclockwise viewed from above.
        contours = []
        for z, length, width, radius in rings:
            contour = []
            for sx, sy, angle in ((1, 1, 0), (-1, 1, 90), (-1, -1, 180), (1, -1, 270)):
                for step in range(5):
                    theta = math.radians(angle + step * 22.5)
                    contour.append((cx+sx*(length/2-radius)+radius*math.cos(theta),
                                    cy+sy*(width/2-radius)+radius*math.sin(theta), z))
            contours.append(contour)
        uv = ((color+0.5)/8, 0.05)
        self.face(list(reversed(contours[0])), uv)
        self.face(contours[-1], uv)
        for low, high in zip(contours, contours[1:]):
            for i in range(len(low)):
                j = (i+1) % len(low)
                self.face([low[i], low[j], high[j], high[i]], uv)

    def decal(self, points):
        self.face(points, [(0.02, .25), (.98, .25), (.98, .97), (.02, .97)])

    def mesh(self):
        return TexturedMesh(tuple(self.positions), tuple(self.normals), tuple(self.texcoords), tuple(self.triangles))


def collada_text(mesh, texture_uri="../materials/textures/bg1_label.png"):
    """Serialize the mesh to a compact Collada 1.4.1 document."""
    index_stream = []
    for triangle in mesh.triangles:
        for index in triangle:
            index_stream.extend((str(index), str(index), str(index)))
    indices = " ".join(index_stream)
    positions = _flat(mesh.positions)
    normals = _flat(mesh.normals)
    texcoords = _flat(mesh.texcoords)
    texture_uri = escape(texture_uri)
    return f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit meter="1" name="meter"/><up_axis>Z_UP</up_axis></asset>
  <library_images><image id="bg1-label"><init_from>{texture_uri}</init_from></image></library_images>
  <library_effects>
    <effect id="bg1-wrap-effect"><profile_COMMON>
      <newparam sid="bg1-label-surface"><surface type="2D"><init_from>bg1-label</init_from></surface></newparam>
      <newparam sid="bg1-label-sampler"><sampler2D><source>bg1-label-surface</source></sampler2D></newparam>
      <technique sid="common"><phong>
        <ambient><color>0.7 0.7 0.7 1</color></ambient>
        <diffuse><texture texture="bg1-label-sampler" texcoord="UVSET0"/></diffuse>
        <specular><color>0.08 0.08 0.08 1</color></specular>
        <shininess><float>12</float></shininess>
      </phong></technique>
    </profile_COMMON></effect>
  </library_effects>
  <library_materials><material id="bg1-wrap-material"><instance_effect url="#bg1-wrap-effect"/></material></library_materials>
  <library_geometries>
    <geometry id="bg1-body-geometry" name="BG1 body shell"><mesh>
      <source id="bg1-positions"><float_array id="bg1-positions-array" count="{len(mesh.positions) * 3}">{positions}</float_array><technique_common><accessor source="#bg1-positions-array" count="{len(mesh.positions)}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
      <source id="bg1-normals"><float_array id="bg1-normals-array" count="{len(mesh.normals) * 3}">{normals}</float_array><technique_common><accessor source="#bg1-normals-array" count="{len(mesh.normals)}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
      <source id="bg1-texcoords"><float_array id="bg1-texcoords-array" count="{len(mesh.texcoords) * 2}">{texcoords}</float_array><technique_common><accessor source="#bg1-texcoords-array" count="{len(mesh.texcoords)}" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
      <vertices id="bg1-vertices"><input semantic="POSITION" source="#bg1-positions"/></vertices>
      <triangles material="BG1_WRAP" count="{mesh.triangle_count}">
        <input semantic="VERTEX" source="#bg1-vertices" offset="0"/>
        <input semantic="NORMAL" source="#bg1-normals" offset="1"/>
        <input semantic="TEXCOORD" source="#bg1-texcoords" offset="2" set="0"/>
        <p>{indices}</p>
      </triangles>
    </mesh></geometry>
  </library_geometries>
  <library_visual_scenes><visual_scene id="Scene" name="Scene"><node id="BG1Body" name="BG1Body"><instance_geometry url="#bg1-body-geometry"><bind_material><technique_common><instance_material symbol="BG1_WRAP" target="#bg1-wrap-material"><bind_vertex_input semantic="UVSET0" input_semantic="TEXCOORD" input_set="0"/></instance_material></technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>
'''


def write_textured_box(path, center, size, texture_uri="../materials/textures/bg1_label.png"):
    mesh = textured_box(center, size)
    Path(path).write_text(collada_text(mesh, texture_uri), encoding="utf-8")
    return mesh
