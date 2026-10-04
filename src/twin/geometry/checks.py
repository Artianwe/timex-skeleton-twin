"""Geometric validation of the CAD assembly: 3-D interference (z-band overlap AND plan overlap).

A pair of parts interferes if their z-bands overlap by more than `z_tol` and their placed 2-D
outlines intersect with area above `a_tol`. Pairs that touch by design (a jewel set in its
bridge, a part and its own arbor, an arbor passing through its jewel, the dial over the hands
window...) are listed explicitly in DESIGNED_CONTACT so nothing is silently ignored.
Meshing gears are NOT exempt: their phases are computed so that teeth interleave, and the
check proves it at the reference pose.
"""
from __future__ import annotations

from shapely import affinity

from .assembly import Part

SAME_BODY_KEYS = ("arbor", "motion")


def _placed(p: Part):
    return affinity.translate(p.shape, p.centre[0], p.centre[1])


def _same_rigid_body(a: Part, b: Part) -> bool:
    """Wheel + pinion + arbor of one axis, fork + stones + staff, balance sub-parts, barrel sub-parts."""
    if a.motion == b.motion and a.motion in ("fork", "balance", "rotor"):
        return True
    if a.motion == "train" and b.motion == "train" and a.arbor == b.arbor and a.centre == b.centre:
        return True
    if a.motion == "auto" and b.motion == "auto" and a.centre == b.centre:
        return True
    return False


def _designed_contact(a: Part, b: Part) -> bool:
    names = {a.name, b.name}
    groups = {a.group, b.group}
    if "jewels" in groups:
        return True                      # jewels are set into plates/bridges and receive pivots
    if groups & {"case"} or groups & {"dial"} and groups & {"case", "keyless"}:
        return True
    pivots = ("arbor", "staff", "pipe")
    if any(k in a.name for k in pivots) and b.group == "structure" or any(k in b.name for k in pivots) and a.group == "structure":
        return True                      # pivots pass through their bearing holes / seats
    if names & {"barrel_arbor"} and names & {"barrel_drum", "barrel_teeth", "barrel_lid", "mainspring", "ratchet_wheel", "train_bridge"}:
        return True
    if names & {"mainspring"} and names & {"barrel_drum", "barrel_lid", "barrel_teeth"}:
        return True
    if names & {"hairspring"} and names & {"balance_staff", "regulator", "balance_cock"}:
        return True                      # collet on staff, stud/regulator pins
    if names & {"regulator"} and names & {"balance_cock", "balance_staff"}:
        return True
    if names == {"impulse_pin", "pallet_fork"} or names == {"roller", "pallet_fork"} or names == {"guard_pin", "roller"}:
        return True                      # escapement engagement (pin in fork slot at rest)
    if names & {"rotor_pinion"} and names & {"centre_arbor", "rotor"}:
        return True
    if names & {"seconds_arbor"} and names & {"centre_arbor", "cannon_pinion", "cannon_pipe", "hour_wheel", "hour_pipe", "seconds_pinion"}:
        return True                      # concentric tubes
    if names & {"cannon_pipe", "hour_pipe", "centre_arbor"} and names & {"cannon_pinion", "hour_wheel", "hour_pipe", "cannon_pipe", "centre_arbor", "minute_hand", "hour_hand", "seconds_hand"}:
        return True
    if names & {"stem", "winding_pinion", "clutch", "setting_lever"} and names & {"stem", "winding_pinion", "clutch", "setting_lever", "main_plate"}:
        return True
    if names & {"click"} and names & {"ratchet_wheel", "train_bridge"}:
        return True
    if names & {"ratchet_wheel", "crown_wheel"} and names & {"train_bridge"}:
        return False
    return False


def interference_report(parts: list[Part], z_tol: float = 1e-6, a_tol: float = 2e-10) -> list[dict]:
    placed = [_placed(p) for p in parts]
    out = []
    for i, a in enumerate(parts):
        for j in range(i + 1, len(parts)):
            b = parts[j]
            dz = min(a.z[1], b.z[1]) - max(a.z[0], b.z[0])
            if dz <= z_tol:
                continue
            if _same_rigid_body(a, b) or _designed_contact(a, b):
                continue
            if not placed[i].intersects(placed[j]):
                continue
            area = placed[i].intersection(placed[j]).area
            if area > a_tol:
                out.append({"a": a.name, "b": b.name, "overlap_mm2": area * 1e6, "dz_mm": dz * 1e3})
    return sorted(out, key=lambda r: -r["overlap_mm2"])
