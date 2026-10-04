"""CAD export of the parametric assembly.

* STEP (B-rep, OpenCascade via CadQuery) - opens in Fusion 360, FreeCAD, SolidWorks, Onshape.
  Each component is a named, coloured body in an assembly; units millimetres.
* GLB (triangle meshes via trimesh) - for Blender / generic 3-D viewers.
* parts.json - compact 2-D outlines (integer micrometres) + z-bands + kinematic metadata, used by
  the three.js viewer, which extrudes them in the browser.

All geometry is generated from the same Part objects (twin.geometry.assembly), so CAD, viewer
and simulation can never disagree about dimensions.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from shapely.geometry import MultiPolygon, Polygon

from .assembly import Part

MM = 1e-3


def _polys(shape) -> list[Polygon]:
    if isinstance(shape, MultiPolygon):
        return list(shape.geoms)
    if isinstance(shape, Polygon):
        return [shape]
    return [g for g in getattr(shape, "geoms", []) if isinstance(g, Polygon)]


def _simplify(shape, tol_m: float):
    return shape.simplify(tol_m, preserve_topology=True) if tol_m > 0 else shape


# ---------------------------------------------------------------------------- CadQuery / STEP
def part_solid(part: Part, simplify: float = 0.5e-6):
    """CadQuery solid of a part, millimetres, placed at its assembly position."""
    import cadquery as cq
    solids = []
    for poly in _polys(_simplify(part.shape, simplify)):
        if poly.is_empty or poly.area <= 0:
            continue
        ext = [(x / MM, y / MM, 0.0) for x, y in list(poly.exterior.coords)[:-1]]
        outer = cq.Wire.makePolygon([cq.Vector(*p) for p in ext], close=True)
        inners = []
        for ring in poly.interiors:
            pts = [(x / MM, y / MM, 0.0) for x, y in list(ring.coords)[:-1]]
            if len(pts) >= 3:
                inners.append(cq.Wire.makePolygon([cq.Vector(*p) for p in pts], close=True))
        face = cq.Face.makeFromWires(outer, inners)
        h = (part.z[1] - part.z[0]) / MM
        s = cq.Solid.extrudeLinear(face, cq.Vector(0, 0, h))
        solids.append(s.translate(cq.Vector(part.centre[0] / MM, part.centre[1] / MM, part.z[0] / MM)))
    if not solids:
        return None
    return cq.Compound.makeCompound(solids) if len(solids) > 1 else solids[0]


def export_step(parts: list[Part], path: Path, skip_groups=("case",), simplify: float = 3.0e-6) -> dict:
    """STEP assembly. Outlines are simplified to `simplify` (3 um default: far below machining
    tolerances of watch parts) because every polyline segment becomes a B-rep face."""
    import cadquery as cq
    assy = cq.Assembly(name="Miyota_82S0_TWEG16716")
    vols = {}
    for p in parts:
        if p.group in skip_groups:
            continue
        s = part_solid(p, simplify=simplify)
        if s is None:
            continue
        vols[p.name] = s.Volume()
        c = p.color.lstrip("#")
        rgb = tuple(int(c[i:i + 2], 16) / 255 for i in (0, 2, 4))
        assy.add(s, name=p.name, color=cq.Color(*rgb, 1.0))
    path.parent.mkdir(parents=True, exist_ok=True)
    assy.export(str(path), exportType="STEP")
    return vols


# ---------------------------------------------------------------------------- GLB
def export_glb(parts: list[Part], path: Path, simplify: float = 1.0e-6) -> int:
    import trimesh
    scene = trimesh.Scene()
    n_tri = 0
    for p in parts:
        meshes = []
        for poly in _polys(_simplify(p.shape, simplify)):
            pm = Polygon([(x / MM, y / MM) for x, y in poly.exterior.coords],
                         [[(x / MM, y / MM) for x, y in r.coords] for r in poly.interiors])
            if not pm.is_valid or pm.area <= 0:
                pm = pm.buffer(0)
            for q in _polys(pm):
                mesh = trimesh.creation.extrude_polygon(q, (p.z[1] - p.z[0]) / MM)
                mesh.apply_translation([p.centre[0] / MM, p.centre[1] / MM, p.z[0] / MM])
                meshes.append(mesh)
        if not meshes:
            continue
        mesh = trimesh.util.concatenate(meshes)
        c = p.color.lstrip("#")
        rgba = [int(c[i:i + 2], 16) for i in (0, 2, 4)] + [int(255 * p.opacity)]
        mesh.visual.face_colors = np.tile(rgba, (len(mesh.faces), 1))
        n_tri += len(mesh.faces)
        scene.add_geometry(mesh, node_name=p.name, geom_name=p.name)
    path.parent.mkdir(parents=True, exist_ok=True)
    scene.export(str(path))
    return n_tri


# ---------------------------------------------------------------------------- web JSON
def _ring_um(coords, centre) -> list[int]:
    a = np.asarray(coords)[:-1]
    a = np.round(a / 1e-6).astype(int)
    return a.reshape(-1).tolist()


def export_web(parts: list[Part], path: Path | None = None, simplify: float = 1.2e-6) -> dict:
    """Compact geometry for the browser: rings in integer micrometres, local to each part axis."""
    out = []
    for p in parts:
        shapes = []
        for poly in _polys(_simplify(p.shape, simplify)):
            if poly.is_empty:
                continue
            shapes.append({"outer": _ring_um(poly.exterior.coords, p.centre),
                           "holes": [_ring_um(r.coords, p.centre) for r in poly.interiors if len(r.coords) > 3]})
        out.append({"name": p.name, "label": p.label, "group": p.group, "part_no": p.part_no,
                    "z": [round(p.z[0] * 1e6), round(p.z[1] * 1e6)], "c": [round(p.centre[0] * 1e6), round(p.centre[1] * 1e6)],
                    "motion": p.motion, "arbor": p.arbor, "ratio": p.ratio, "color": p.color, "opacity": p.opacity,
                    "material": p.material, "meta": {k: (float(v) if isinstance(v, (int, float, np.floating)) else v)
                                                     for k, v in p.meta.items()}, "shapes": shapes})
    data = {"units": "micrometre", "parts": out}
    if path:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, separators=(",", ":")))
    return data
