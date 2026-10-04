"""Single-view photogrammetry of the owner's photographs.

Converts hand-picked pixel coordinates (data/photo_points.yaml) into physical estimates using the
engraved 44 mm case diameter as the scale reference. This is deliberately simple (scaled
orthographic model, no lens calibration) and reports an uncertainty for every number.

Outputs (printed + data/photogrammetry_results.yaml):
  * balance outer diameter                  -> config balance.rim_outer_diameter (measured)
  * balance centre radius / angle (dial view) -> config layout.balance_centre_* (measured)

Coordinate convention for results: DIAL VIEW, x toward 3 o'clock (crown), y toward 12 o'clock,
angles measured counter-clockwise from 3 o'clock.

Author: Anwesh Ajitabh Dash
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
CASE_DIAMETER_MM = 44.0  # engraved on the caseback ("44 MM CASE")


def _case_scale(edges: dict) -> tuple[np.ndarray, float, float]:
    """Return (centre_px, px_per_mm, ellipticity) from four silhouette points."""
    l, r, t, b = (np.array(edges[k], float) for k in ("left", "right", "top", "bottom"))
    centre = np.array([(l[0] + r[0]) / 2.0, (t[1] + b[1]) / 2.0])
    dx = r[0] - l[0]
    dy = b[1] - t[1]
    major = max(dx, dy)                       # foreshortening shrinks only one axis
    return centre, float(major / CASE_DIAMETER_MM), float(min(dx, dy) / major)


def _to_dial_frame(v_img: np.ndarray, view: str) -> np.ndarray:
    """Map an image-space offset (x right, y down) to the dial-view frame (x->3h, y->12h)."""
    x, y = v_img
    if view == "dial":            # crown right, 12 at top: simple y flip
        return np.array([x, -y])
    if view == "caseback":        # crown right, flipped about the 3-9 axis: 12 at bottom
        return np.array([x, y])
    raise ValueError(view)


def analyse(points_file: Path = ROOT / "data" / "photo_points.yaml") -> dict:
    pts = yaml.safe_load(points_file.read_text())
    out: dict = {"estimates": {}}
    diam, rad, ang = [], [], []
    for view in ("caseback", "dial"):
        p = pts[view]
        c_case, s, ell = _case_scale(p["case_edge"])
        bc = np.array(p["balance_centre"], float)
        rim = np.array(p["balance_rim"], float)
        r_px = np.linalg.norm(rim - bc, axis=1)
        d_mm = 2.0 * float(np.mean(r_px)) / s
        diam.append(d_mm)
        centres = {"case_centre": c_case}
        if view == "caseback":
            centres["rotor_axis"] = np.array(p["rotor_axis"], float)
        else:
            centres["hands_pivot"] = np.array(p["hands_pivot"], float)
        rec = {"px_per_mm": round(float(s), 2), "case_ellipticity": round(float(ell), 4),
               "balance_diameter_mm": round(d_mm, 2), "balance_centre": {}}
        for name, c in centres.items():
            v = _to_dial_frame(bc - c, view) / s
            r_mm = float(np.hypot(*v))
            a_deg = math.degrees(math.atan2(v[1], v[0])) % 360.0
            rad.append(r_mm)
            ang.append(a_deg)
            rec["balance_centre"][name] = {"radius_mm": round(r_mm, 2), "angle_deg": round(a_deg, 1)}
        out[view] = rec
    out["estimates"] = {
        "balance_diameter_mm": {"value": round(float(np.mean(diam)), 2),
                                "spread": round(float(np.ptp(diam)) / 2 + 0.3, 2)},
        "balance_centre_radius_mm": {"value": round(float(np.median(rad)), 2),
                                     "min": round(min(rad), 2), "max": round(max(rad), 2)},
        "balance_centre_angle_deg": {"value": round(float(np.median(ang)), 1),
                                     "min": round(min(ang), 1), "max": round(max(ang), 1)},
        "note": ("Scaled-orthographic model; depth differences between case silhouette and "
                 "movement (~3-6 mm) add ~3% scale uncertainty."),
    }
    return out


if __name__ == "__main__":
    res = analyse()
    (ROOT / "data" / "photogrammetry_results.yaml").write_text(yaml.safe_dump(res, sort_keys=False))
    print(yaml.safe_dump(res, sort_keys=False))
