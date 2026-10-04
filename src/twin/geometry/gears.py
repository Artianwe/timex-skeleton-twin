"""Cycloidal watch-gear tooth profiles (2-D), generated from tooth counts and module.

Horological gearing is cycloidal, not involute:
  * the WHEEL addendum is an EPICYCLOID traced by a rolling circle of radius rho = r_p/2 (half the
    pinion pitch radius) rolling on the wheel pitch circle;
  * the same rolling circle rolling INSIDE the pinion pitch circle traces a hypocycloid that
    degenerates into a straight RADIAL line (the classical result for rho = r/2), so pinion
    flanks are radial;
  * pinion leaves get rounded (semicircular) tips; wheel teeth have pointed-ogival tips
    truncated at the addendum circle.
Proportions follow common watch practice (NIHS 20-02 style): wheel tooth thickness 0.5 p at the
pitch circle, pinion leaf thickness ~0.42 p (watch gearing runs with generous backlash).

Epicycloid (base radius R, rolling radius rho), parameter t:
    x = (R+rho) cos t - rho cos((R+rho) t / rho)
    y = (R+rho) sin t - rho sin((R+rho) t / rho)
"""
from __future__ import annotations

import math

import numpy as np


def _rot(pts: np.ndarray, a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return pts @ np.array([[c, s], [-s, c]])


def epicycloid(R: float, rho: float, t_max: float, n: int = 24) -> np.ndarray:
    t = np.linspace(0.0, t_max, n)
    x = (R + rho) * np.cos(t) - rho * np.cos((R + rho) / rho * t)
    y = (R + rho) * np.sin(t) - rho * np.sin((R + rho) / rho * t)
    return np.column_stack([x, y])


def wheel_profile(z: int, m: float, z_mate: int, *, thickness_frac: float = 0.5,
                  addendum: float = 1.35, dedendum: float = 1.6, n_flank: int = 14) -> np.ndarray:
    """Closed outline (N, 2) of a cycloidal wheel with z teeth, module m, meshing a z_mate pinion."""
    R = m * z / 2.0
    rho = m * z_mate / 4.0                 # half the mate's pitch radius
    r_tip = R + addendum * m
    r_root = R - dedendum * m
    pitch_ang = 2 * math.pi / z
    half_t = thickness_frac * pitch_ang / 2.0          # half tooth angle at pitch circle
    # epicycloid starting at the pitch point (angle 0), tooth flank on the +side
    t_hi = 1.2
    curve = epicycloid(R, rho, t_hi, 200)
    r_c = np.hypot(curve[:, 0], curve[:, 1])
    a_c = np.arctan2(curve[:, 1], curve[:, 0])
    # flank for the tooth's LEFT side: mirror so it leans toward the tooth centre
    # tooth centre at angle 0; right flank starts at pitch angle -half_t going outward toward 0
    flank_r, flank_a = [], []
    for rr, aa in zip(r_c, a_c):
        ang = -half_t + aa             # epicycloid advances toward the tooth centre
        if rr > r_tip or ang >= 0.0:
            break
        flank_r.append(rr)
        flank_a.append(ang)
    flank_r = np.array(flank_r)
    flank_a = np.array(flank_a)
    # resample flank
    idx = np.linspace(0, len(flank_r) - 1, n_flank).astype(int)
    flank_r, flank_a = flank_r[idx], flank_a[idx]
    tip_a = flank_a[-1]
    right = np.column_stack([flank_r, flank_a])                     # from pitch to tip
    root_pt = np.array([[r_root, -half_t]])                          # radial dedendum flank
    one = [np.array([[r_root, -half_t]]), np.array([[R, -half_t]]), right]
    # tip arc
    if tip_a < 0:
        ta = np.linspace(tip_a, -tip_a, 5)
        one.append(np.column_stack([np.full_like(ta, flank_r[-1]), ta]))
    left = right[::-1].copy()
    left[:, 1] = -left[:, 1]
    one += [left, np.array([[R, half_t]]), np.array([[r_root, half_t]])]
    tooth = np.vstack(one)
    del root_pt
    pts = []
    gap_n = 6
    for k in range(z):
        a0 = k * pitch_ang
        tp = tooth.copy()
        tp[:, 1] += a0
        pts.append(tp)
        # root arc to next tooth
        ra = np.linspace(a0 + half_t, a0 + pitch_ang - half_t, gap_n + 2)[1:-1]
        pts.append(np.column_stack([np.full_like(ra, r_root), ra]))
    polar = np.vstack(pts)
    return np.column_stack([polar[:, 0] * np.cos(polar[:, 1]), polar[:, 0] * np.sin(polar[:, 1])])


def pinion_profile(z: int, m: float, *, thickness_frac: float = 0.42, addendum: float = 0.7,
                   dedendum: float = 1.75, n_tip: int = 9) -> np.ndarray:
    """Closed outline of a watch pinion: radial flanks, semicircular leaf tips."""
    R = m * z / 2.0
    pitch_ang = 2 * math.pi / z
    half_t = thickness_frac * pitch_ang / 2.0
    r_root = max(R - dedendum * m, 0.35 * R)
    half_w = R * math.sin(half_t)                       # half leaf width at pitch circle
    r_tip_c = R + addendum * m - half_w                 # centre of tip semicircle
    pts = []
    for k in range(z):
        a0 = k * pitch_ang
        leaf = []
        # radial flank right (angle a0 - half_t) from root to pitch, then straight up to tip centre
        for rr in (r_root, R):
            leaf.append((rr * math.cos(a0 - half_t), rr * math.sin(a0 - half_t)))
        # semicircular tip
        cx, cy = r_tip_c * math.cos(a0), r_tip_c * math.sin(a0)
        for th in np.linspace(-math.pi / 2, math.pi / 2, n_tip):
            ang = a0 + th
            leaf.append((cx + half_w * math.cos(ang), cy + half_w * math.sin(ang)))
        for rr in (R, r_root):
            leaf.append((rr * math.cos(a0 + half_t), rr * math.sin(a0 + half_t)))
        pts.extend(leaf)
        ra = np.linspace(a0 + half_t, a0 + pitch_ang - half_t, 6)[1:-1]
        pts.extend([(r_root * math.cos(a), r_root * math.sin(a)) for a in ra])
    return np.asarray(pts)


def club_tooth_escape_wheel(z: int, r_tip: float, *, tooth_height: float = 0.32e-3,
                            lock_face_rake: float = math.radians(24), impulse_face_frac: float = 0.28,
                            club_width_frac: float = 0.16) -> np.ndarray:
    """Swiss club-tooth escape wheel outline. Teeth lean forward (rotation +), with a raked
    locking face, a flat 'club' impulse face and a curved back."""
    r_root = r_tip - tooth_height
    pitch = 2 * math.pi / z
    pts = []
    for k in range(z):
        a0 = k * pitch
        lock_tip = a0
        club_end = a0 - club_width_frac * pitch           # impulse face (club) behind the locking corner
        back_root = a0 - (0.5 + impulse_face_frac) * pitch
        root_front = a0 + tooth_height * math.tan(lock_face_rake) / r_root
        # going round counter-clockwise (increasing angle): back root -> club heel -> club -> lock tip -> lock root
        heel_r = r_tip - 0.30 * tooth_height
        seg = [(r_root, back_root)]
        for f in np.linspace(0.15, 1.0, 5):     # curved back of tooth
            seg.append((r_root + (heel_r - r_root) * f ** 0.7, back_root + (club_end - back_root) * f))
        seg += [(r_tip, a0 - 0.02 * pitch), (r_tip, lock_tip), (r_root, root_front)]
        ra = np.linspace(root_front, a0 + pitch - (0.5 + impulse_face_frac) * pitch, 5)[1:-1]
        seg += [(r_root, a) for a in ra]
        pts.extend([(r * math.cos(a), r * math.sin(a)) for r, a in seg])
    return np.asarray(pts)


def mesh_interference(wheel: np.ndarray, pinion: np.ndarray, a: float, ratio: float,
                      steps: int = 90, pinion_phase: float = 0.0, wheel_pitch: float | None = None):
    """Check a wheel/pinion pair at centre distance `a` over one wheel pitch.

    Returns (max_overlap_area, min_gap). Uses shapely. ratio = z_wheel / z_pinion.
    """
    from shapely import affinity
    from shapely.geometry import Polygon
    W = Polygon(wheel).buffer(0)
    Pn = Polygon(pinion).buffer(0)
    wheel_pitch = wheel_pitch or (2 * math.pi / 80)
    worst, min_gap = 0.0, float("inf")
    for i in range(steps):
        th = i / steps * wheel_pitch
        w = affinity.rotate(W, th, origin=(0, 0), use_radians=True)
        p = affinity.rotate(Pn, -th * ratio + pinion_phase, origin=(0, 0), use_radians=True)
        p = affinity.translate(p, a, 0.0)
        inter = w.intersection(p).area
        worst = max(worst, inter)
        min_gap = min(min_gap, w.distance(p))
    return worst, min_gap
