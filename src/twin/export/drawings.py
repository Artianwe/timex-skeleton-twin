"""2-D engineering views of the parametric assembly (matplotlib): plan view and exploded section."""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import PathPatch  # noqa: E402
from matplotlib.path import Path as MPath  # noqa: E402
from shapely import affinity  # noqa: E402
from shapely.geometry import MultiPolygon, Polygon  # noqa: E402

MM = 1e-3


def _patch(poly: Polygon, **kw):
    verts, codes = [], []
    for ring in [poly.exterior, *poly.interiors]:
        xy = list(ring.coords)
        verts += [(x / MM, y / MM) for x, y in xy]
        codes += [MPath.MOVETO] + [MPath.LINETO] * (len(xy) - 2) + [MPath.CLOSEPOLY]
    return PathPatch(MPath(verts, codes), **kw)


def plan_view(parts, path: Path, *, title: str = "", hide_groups=("case", "dial", "hands", "automatic winding"),
              labels: dict | None = None, dpi: int = 220):
    fig, ax = plt.subplots(figsize=(9, 9))
    order = sorted(parts, key=lambda p: p.z[1])
    for p in order:
        if p.group in hide_groups:
            continue
        g = affinity.translate(p.shape, p.centre[0], p.centre[1])
        geoms = g.geoms if isinstance(g, MultiPolygon) else [g]
        for q in geoms:
            alpha = 0.25 if p.group == "structure" else min(1.0, p.opacity)
            ax.add_patch(_patch(q, facecolor=p.color, edgecolor="#333333", lw=0.25, alpha=alpha))
    if labels:
        for name, (x, y) in labels.items():
            ax.annotate(name, (x / MM, y / MM), fontsize=8, ha="center", va="center",
                        bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="#888", alpha=0.85))
    ax.set_aspect("equal")
    lim = 14.5
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_xlabel("x [mm]  (→ 3 o'clock / crown)")
    ax.set_ylabel("y [mm]  (→ 12 o'clock)")
    ax.set_title(title or "Plan view (dial side), parametric CAD")
    ax.grid(alpha=0.2)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    return path


def section_view(parts, path: Path, dpi: int = 200):
    """Axial stack chart: z-band of every part (an 'exploded' side elevation)."""
    rows = [p for p in parts if p.group not in ("jewels",)]
    rows = sorted(rows, key=lambda p: (p.z[0]))
    fig, ax = plt.subplots(figsize=(10, 12))
    for i, p in enumerate(rows):
        ax.barh(i, (p.z[1] - p.z[0]) / MM, left=p.z[0] / MM, color=p.color, edgecolor="#333", lw=0.4,
                alpha=0.9 if p.opacity > 0.5 else 0.4)
        ax.text(p.z[1] / MM + 0.05, i, f"{p.label}", va="center", fontsize=6)
    ax.set_yticks([])
    ax.set_xlabel("z [mm]  (0 = dial-side face of main plate; − toward caseback)")
    ax.set_title("Axial stack (z-bands) of every component")
    ax.grid(axis="x", alpha=0.3)
    fig.savefig(path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    return path
