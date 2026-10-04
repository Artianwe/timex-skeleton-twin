"""Parametric assembly of the movement: every component as a 2-D profile extruded over a z-band.

Each Part carries
  * geometry: a shapely (Multi)Polygon in its LOCAL frame (rotation axis at the origin), already
    rotated by its assembly phase, plus an axis position and z-band;
  * kinematics: which signal drives it ('train' with a speed ratio to the escape wheel,
    'balance', 'fork', 'rotor', 'auto', or 'static') - used by the 3-D viewer and animations;
  * identification: label, group and Miyota part number for the component inventory.

Everything is derived from WatchModel + Layout (tooth counts, modules, centre distances,
escapement geometry, measured balance diameter...). Nothing is drawn by hand.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from shapely import affinity
from shapely.geometry import LineString, MultiPolygon, Point, Polygon
from shapely.ops import unary_union

from ..model import WatchModel
from .gears import club_tooth_escape_wheel, pinion_profile, wheel_profile
from .layout import Layout

MM = 1e-3


@dataclass
class Part:
    name: str
    label: str
    group: str
    shape: Polygon | MultiPolygon
    z: tuple[float, float]
    centre: tuple[float, float] = (0.0, 0.0)
    motion: str = "static"          # static | train | balance | fork | rotor | auto | crown
    arbor: str | None = None        # train arbor name (for ratio)
    ratio: float = 0.0              # omega / omega_escape (train) or omega / omega_rotor (auto)
    color: str = "#c9ccd1"
    opacity: float = 1.0
    part_no: str = ""
    material: str = "steel"
    meta: dict = field(default_factory=dict)


# ----------------------------------------------------------------------------------------------
def _poly(pts) -> Polygon:
    return Polygon(pts).buffer(0)


def _circle(r, c=(0.0, 0.0), res=48):
    return Point(c).buffer(r, res)


def _ring(r_in, r_out, c=(0.0, 0.0)):
    return _circle(r_out, c, 96).difference(_circle(r_in, c, 96))


def _bar(p0, p1, w):
    return LineString([p0, p1]).buffer(w / 2.0, cap_style=2)


def _rot(g, ang):
    return affinity.rotate(g, ang, origin=(0, 0), use_radians=True)


def _crossings(outline: Polygon, r_rim_in: float, r_hub: float, n_arms: int, arm_w: float, hole_r: float,
               arm_phase: float = 0.0):
    if r_rim_in > r_hub + 0.15 * MM:
        window = _ring(r_hub, r_rim_in)
        arms = unary_union([_bar((0, 0), (r_rim_in * 1.05 * math.cos(arm_phase + 2 * math.pi * k / n_arms),
                                          r_rim_in * 1.05 * math.sin(arm_phase + 2 * math.pi * k / n_arms)), arm_w)
                            for k in range(n_arms)])
        outline = outline.difference(window.difference(arms))
    return outline.difference(_circle(hole_r))


def mesh_phase(phi_driver: float, z_driver: int, z_driven: int, c_driver, c_driven) -> float:
    """Phase of a driven gear so that its teeth interleave with the driver at the reference pose."""
    beta = math.atan2(c_driven[1] - c_driver[1], c_driven[0] - c_driver[0])
    p1 = 2 * math.pi / z_driver
    p2 = 2 * math.pi / z_driven
    k = round((beta - phi_driver) / p1)
    delta = beta - (phi_driver + k * p1)
    return beta + math.pi - p2 / 2.0 + delta * z_driver / z_driven


# ----------------------------------------------------------------------------------------------
def build_parts(m: WatchModel, L: Layout) -> list[Part]:
    P = m.P
    tr = m.train
    meshes = {x.name: x for x in tr.meshes}
    band = lambda k: P.band(f"cad.{k}")  # noqa: E731
    pos = {k: tuple(map(float, v)) for k, v in L.pos.items()}
    w_e = tr.omega["escape"]
    ratio = lambda a: tr.omega[a] / w_e  # noqa: E731
    R_mov = P["movement.diameter"] / 2.0
    parts: list[Part] = []
    BRASS, STEEL, RUBY, GILT = "#d4b26a", "#c9ccd1", "#c2185b", "#e0c070"

    def gear(name, label, arbor, z, zmate, module, zband, phase, *, pinion=False, color=BRASS,
             arms=5, part_no="", hole=0.12 * MM):
        if pinion:
            prof = pinion_profile(z, module)
            shp = _poly(prof).difference(_circle(min(hole, 0.3 * module * z / 2)))
        else:
            prof = wheel_profile(z, module, zmate)
            R = module * z / 2.0
            shp = _crossings(_poly(prof), R - 1.6 * module - max(0.10 * R, 0.15 * MM), max(0.12 * R, 0.30 * MM),
                             arms, max(0.07 * R, 0.14 * MM), hole)
        parts.append(Part(name, label, "going train" if arbor in ("centre", "third", "fourth", "escape", "seconds", "barrel") else "motion works",
                          _rot(shp, phase), zband, pos.get(arbor, (0.0, 0.0)) if arbor in pos else (0.0, 0.0),
                          "train", arbor, ratio(arbor), color, 1.0, part_no, "brass" if not pinion else "steel"))

    def arbor(name, arb, zband, r=0.10 * MM, part_no=""):
        parts.append(Part(name, f"{arb} arbor", "going train", _circle(r), zband, pos[arb], "train", arb,
                          ratio(arb), STEEL, 1.0, part_no))

    # ---------------- phases along the train ------------------------------------------------
    bc, c3, s34, s3s, s4e = (meshes["barrel->centre"], meshes["centre->third"], meshes["third->fourth"],
                             meshes["third->seconds"], meshes["fourth->escape"])
    ph = {"barrel": 0.0, "centre_w": 0.0, "third_w": 0.0, "fourth_w": 0.0}
    ph["centre_p"] = mesh_phase(ph["barrel"], bc.z_driver, bc.z_driven, pos["barrel"], pos["centre"])
    ph["third_p"] = mesh_phase(ph["centre_w"], c3.z_driver, c3.z_driven, pos["centre"], pos["third"])
    ph["fourth_p"] = mesh_phase(ph["third_w"], s34.z_driver, s34.z_driven, pos["third"], pos["fourth"])
    ph["seconds_p"] = mesh_phase(ph["third_w"], s3s.z_driver, s3s.z_driven, pos["third"], pos["centre"])
    ph["escape_p"] = mesh_phase(ph["fourth_w"], s4e.z_driver, s4e.z_driven, pos["fourth"], pos["escape"])

    # ---------------- barrel, mainspring, ratchet -------------------------------------------
    sp = m.spring
    Rb = bc.r_driver
    drum_out = Rb - 1.7 * bc.module
    tb = band("barrel_teeth")
    drum_z = (band("barrel")[0], tb[0])
    parts.append(Part("barrel_drum", "Barrel drum", "energy", _ring(sp.R, drum_out), drum_z, pos["barrel"], "train", "barrel",
                      ratio("barrel"), BRASS, 1.0, "001-870", "brass"))
    teeth = _poly(wheel_profile(bc.z_driver, bc.module, bc.z_driven)).difference(_circle(sp.r + 0.25 * MM))
    parts.append(Part("barrel_teeth", "Barrel toothed rim", "energy", teeth, tb, pos["barrel"], "train", "barrel",
                      ratio("barrel"), BRASS, 1.0, "001-870", "brass"))
    parts.append(Part("barrel_lid", "Barrel cover", "energy", _ring(sp.r + 0.05 * MM, sp.R + 0.1 * MM),
                      (band("barrel")[0], band("barrel")[0] + 0.15 * MM), pos["barrel"], "train", "barrel",
                      ratio("barrel"), BRASS, 0.55, "001-870", "brass"))
    parts.append(Part("barrel_arbor", "Barrel arbor", "energy", _circle(sp.r), (band("ratchet")[0] + 0.01 * MM, band("plate")[0]),
                      pos["barrel"], "static", None, 0.0, STEEL, 1.0, "001-870"))
    zc = 0.5 * (band("barrel")[0] + band("barrel_teeth")[1])
    spiral = sp.spiral(sp.n_dev)
    ms = LineString(spiral).buffer(sp.e / 2.0, cap_style=1, quad_segs=2)
    parts.append(Part("mainspring", "Mainspring (fully wound)", "energy", ms, (zc - sp.h / 2, zc + sp.h / 2), pos["barrel"],
                      "static", None, 0.0, "#8fa3b8", 1.0, "001-870", "steel", {"parametric": "mainspring"}))
    # ratchet + crown wheel + click (keyless) ---------------------------------------------------
    am = P["autowinding.module"]
    zr = P.int("keyless.ratchet_wheel")
    rat = _poly(wheel_profile(zr, am, P.int("autowinding.reduction_pinion"), thickness_frac=0.5, addendum=1.0)).difference(_circle(0.5 * MM))
    parts.append(Part("ratchet_wheel", "Ratchet wheel", "energy", rat, band("ratchet"), pos["barrel"], "crown", None,
                      0.0, STEEL, 1.0, "059-560"))
    zcw = P.int("keyless.crown_wheel")
    a_cw = am * (zr + zcw) / 2.0
    bx, by = pos["barrel"]
    ang_cw = math.radians(-35.0)
    cw_c = (bx + a_cw * math.cos(ang_cw), by + a_cw * math.sin(ang_cw))
    ph_cw = mesh_phase(0.0, zr, zcw, pos["barrel"], cw_c)
    parts.append(Part("crown_wheel", "Crown wheel", "keyless", _rot(_poly(wheel_profile(zcw, am, zr, addendum=1.0)).difference(_circle(0.3 * MM)), ph_cw),
                      band("ratchet"), cw_c, "crown", None, 0.0, STEEL, 1.0, "058-360"))
    click = _bar((bx + (Rb - 0.2 * MM) * math.cos(1.9), by + (Rb - 0.2 * MM) * math.sin(1.9)),
                 (bx + (Rb + 1.6 * MM) * math.cos(2.3), by + (Rb + 1.6 * MM) * math.sin(2.3)), 0.5 * MM)
    parts.append(Part("click", "Click", "energy", click, band("ratchet"), (0, 0), "static", None, 0.0, STEEL, 1.0, "060-390"))

    # ---------------- going train -------------------------------------------------------------
    gear("centre_pinion", "Centre pinion", "centre", bc.z_driven, bc.z_driver, bc.module, band("center_pinion"), ph["centre_p"], pinion=True, part_no="012-116")
    gear("centre_wheel", "Centre wheel", "centre", c3.z_driver, c3.z_driven, c3.module, band("center_wheel"), ph["centre_w"], part_no="012-116")
    arbor("centre_arbor", "centre", (band("train_bridge")[0], band("motion_works")[1]), 0.30 * MM, "012-116")
    gear("third_pinion", "Third pinion", "third", c3.z_driven, c3.z_driver, c3.module, band("third_pinion"), ph["third_p"], pinion=True, part_no="017-760")
    gear("third_wheel", "Third wheel", "third", s34.z_driver, s34.z_driven, s34.module, band("third_wheel"), ph["third_w"], arms=4, part_no="017-760")
    arbor("third_arbor", "third", (band("train_bridge")[0] + 0.2 * MM, band("plate")[0] + 0.15 * MM), part_no="017-760")
    gear("fourth_pinion", "Fourth pinion", "fourth", s34.z_driven, s34.z_driver, s34.module, band("fourth_pinion"), ph["fourth_p"], pinion=True, part_no="023-940")
    gear("fourth_wheel", "Fourth wheel", "fourth", s4e.z_driver, s4e.z_driven, s4e.module, band("fourth_wheel"), ph["fourth_w"], arms=4, part_no="023-940")
    arbor("fourth_arbor", "fourth", (band("train_bridge")[0] + 0.2 * MM, band("plate")[0] + 0.15 * MM), part_no="023-940")
    gear("seconds_pinion", "Centre second pinion", "seconds", s3s.z_driven, s3s.z_driver, s3s.module, band("seconds_pinion"), ph["seconds_p"], pinion=True, part_no="025-670")
    sec_hole = P["movement.seconds_hand_hole"] / 2.0
    parts.append(Part("seconds_arbor", "Centre second pinion arbor", "going train", _circle(sec_hole),
                      (band("seconds_pinion")[0], P["cad.seconds_hand_z"]), (0.0, 0.0), "train", "seconds", ratio("seconds"),
                      STEEL, 1.0, "025-670"))
    gear("escape_pinion", "Escape pinion", "escape", s4e.z_driven, s4e.z_driver, s4e.module, band("escape_pinion"), ph["escape_p"], pinion=True, part_no="032-106")
    arbor("escape_arbor", "escape", (band("pallet_bridge")[1], band("plate")[0] + 0.15 * MM), 0.08 * MM, "032-106")

    # ---------------- escapement: escape wheel + pallet fork ------------------------------------
    esc = m.esc
    alpha6 = L.angles["escapement_line"]
    ew = _poly(club_tooth_escape_wheel(esc.z, esc.r_e))
    ew = _crossings(ew, esc.r_root - 0.18 * MM, 0.30 * MM, 4, 0.16 * MM, 0.08 * MM)
    phase_esc = alpha6 + esc.beta
    parts.append(Part("escape_wheel", "Escape wheel (15 club teeth)", "escapement", _rot(ew, phase_esc), band("escape_wheel"),
                      pos["escape"], "train", "escape", 1.0, STEEL, 1.0, "032-106", "steel",
                      {"phase": phase_esc}))
    # fork in local frame: +x from pallet staff toward balance, then rotated by alpha6
    lever_len = esc.d_bp - esc.r_roller
    fork = unary_union([
        _bar((0, 0), (lever_len + 0.05 * MM, 0), 0.34 * MM),
        _bar((lever_len - 0.25 * MM, -0.33 * MM), (lever_len + 0.12 * MM, -0.33 * MM), 0.30 * MM),
        _bar((lever_len - 0.25 * MM, 0.33 * MM), (lever_len + 0.12 * MM, 0.33 * MM), 0.30 * MM),
        _circle(0.45 * MM),
    ])
    slot = Polygon([(lever_len - 0.18 * MM, -0.11 * MM), (lever_len + 0.4 * MM, -0.11 * MM),
                    (lever_len + 0.4 * MM, 0.11 * MM), (lever_len - 0.18 * MM, 0.11 * MM)])
    fork = fork.difference(slot)
    stones = []
    Pe_local = np.array([-esc.d_ep, 0.0])
    for sgn in (+1, -1):
        Lp = np.array([esc.r_e * math.cos(esc.beta) - esc.d_ep, sgn * esc.r_e * math.sin(esc.beta)])
        u = (Lp - Pe_local) / np.linalg.norm(Lp - Pe_local)
        cst = Lp + 0.37 * MM * u          # locked depth ~0.08 mm at the banking pose
        fork = fork.union(_bar((0, 0), tuple(cst + 0.15 * MM * u), 0.30 * MM))
        st = _bar(tuple(cst - 0.33 * MM * u), tuple(cst + 0.33 * MM * u), 0.24 * MM)
        stones.append(st)
    fork = fork.difference(unary_union(stones)).difference(_circle(0.07 * MM))
    fork = fork.difference(_circle(esc.r_e + 0.05 * MM, tuple(Pe_local)))      # arms clear the wheel
    # rest pose = sim start: fork on its banking (physical rotation +phi_B), entry stone locking
    rest = esc.phi_B
    stones_g = _rot(unary_union(stones), alpha6 + rest)
    fork_g = _rot(fork, alpha6 + rest)
    Pe_rel = (pos["escape"][0] - pos["pallet"][0], pos["escape"][1] - pos["pallet"][1])
    ew_placed = affinity.translate(_rot(ew, phase_esc), Pe_rel[0], Pe_rel[1])
    lock_ofs = 0.0
    for k in range(400):                     # rotate wheel back (CW) until it just clears the stone
        trial = affinity.rotate(ew_placed, -k * 0.0005, origin=Pe_rel, use_radians=True)
        if trial.intersection(stones_g).area < 1e-13 and trial.intersection(fork_g).area < 1e-13:
            lock_ofs = -k * 0.0005
            break
    phase_esc += lock_ofs
    parts[-1].shape = _rot(ew, phase_esc)
    parts[-1].meta["phase"] = phase_esc
    parts[-1].meta["lock_offset"] = lock_ofs
    fz = band("pallet_fork")
    parts.append(Part("pallet_fork", "Pallet fork (lever)", "escapement", fork_g, fz, pos["pallet"], "fork", None, 0.0,
                      STEEL, 1.0, "035-560", "steel", {"alpha6": alpha6, "rest_rotation": rest}))
    parts.append(Part("pallet_stones", "Pallet stones (entry/exit)", "escapement", stones_g,
                      (band("escape_wheel")[0] - 0.03 * MM, fz[1] + 0.02 * MM), pos["pallet"], "fork", None, 0.0, RUBY, 0.9, "035-560", "ruby"))
    parts.append(Part("guard_pin", "Guard pin", "escapement", _rot(_circle(0.04 * MM, (lever_len - 0.42 * MM, 0)), alpha6 + rest),
                      (fz[0] - 0.12 * MM, fz[0]), pos["pallet"], "fork", None, 0.0, STEEL, 1.0, "035-560"))
    parts.append(Part("pallet_staff", "Pallet staff", "escapement", _circle(0.08 * MM), (band("pallet_bridge")[1], band("plate")[0] + 0.1 * MM),
                      pos["pallet"], "fork", None, 0.0, STEEL, 1.0, "035-560"))

    # ---------------- balance + hairspring -----------------------------------------------------
    bal = m.balance
    B = pos["balance"]
    ang_bp = math.atan2(pos["pallet"][1] - B[1], pos["pallet"][0] - B[0])
    pin_ang = ang_bp - bal.beat_offset              # impulse pin direction at theta = 0
    rim = _ring(bal.rim_inner, bal.rim_outer)
    rz = band("balance_rim")
    parts.append(Part("balance_rim", "Balance rim", "oscillator", rim, rz, B, "balance", None, 0.0, GILT, 1.0, "039-102", "CuBe",
                      {"pin_angle": pin_ang}))
    arms = unary_union([_bar((0, 0), (bal.rim_inner * math.cos(pin_ang + math.pi + 2 * math.pi * k / 3 + 0.5),
                                      bal.rim_inner * math.sin(pin_ang + math.pi + 2 * math.pi * k / 3 + 0.5)), P["balance.arm_width"])
                        for k in range(P.int("balance.arms"))]).union(_circle(0.4 * MM)).difference(_circle(0.1 * MM))
    t_arm = P["balance.arm_thickness"]
    parts.append(Part("balance_arms", "Balance arms", "oscillator", arms, (rz[1] - t_arm, rz[1]), B, "balance", None, 0.0, GILT, 1.0, "039-102", "CuBe"))
    parts.append(Part("balance_staff", "Balance staff", "oscillator", _circle(0.10 * MM), (band("balance_cock")[0] + 0.1 * MM, band("plate")[0] + 0.1 * MM),
                      B, "balance", None, 0.0, STEEL, 1.0, "039-102"))
    roller = _circle(esc.r_roller + 0.18 * MM).difference(_circle(0.1 * MM))
    roller = roller.difference(_circle(0.30 * MM, (esc.r_roller * math.cos(pin_ang + math.pi), esc.r_roller * math.sin(pin_ang + math.pi))))
    ro = band("roller")
    parts.append(Part("roller", "Impulse roller (double roller)", "oscillator", roller, (ro[0] + 0.08 * MM, ro[1]), B, "balance", None, 0.0, STEEL, 1.0, "039-102"))
    pin = _circle(0.10 * MM, (esc.r_roller * math.cos(pin_ang), esc.r_roller * math.sin(pin_ang)))
    parts.append(Part("impulse_pin", "Impulse jewel (roller pin)", "oscillator", pin, (ro[0] - 0.05 * MM, ro[1]), B, "balance", None, 0.0,
                      RUBY, 1.0, "039-102", "ruby"))
    # hairspring (Archimedean spiral at rest)
    hs_in, hs_out = P["hairspring.inner_radius"], P["hairspring.outer_radius"]
    turns = bal.hs_turns
    tt = np.linspace(0, 2 * math.pi * turns, int(140 * turns))
    rr = hs_in + (hs_out - hs_in) * tt / tt[-1]
    a0 = pin_ang
    hs_pts = np.column_stack([rr * np.cos(tt + a0), rr * np.sin(tt + a0)])
    hs = LineString(hs_pts).buffer(P["hairspring.thickness"] / 2.0, cap_style=2, quad_segs=2).union(_circle(hs_in + 0.05 * MM)).difference(_circle(0.1 * MM))
    parts.append(Part("hairspring", "Hairspring (flat spiral)", "oscillator", hs, band("hairspring"), B, "balance", None, 0.0, "#5b8fd6", 1.0,
                      "039-102", "Fe-Ni", {"parametric": "hairspring", "r_in": hs_in, "r_out": hs_out, "turns": turns, "a0": a0}))

    # ---------------- motion works & hands (dial side) -----------------------------------------
    mw1, mw2 = meshes["cannon->minute_wheel"], meshes["minute_pinion->hour"]
    mwz = band("motion_works")
    mw_ang = math.radians(150.0)
    mw_c = (mw1.centre_distance * math.cos(mw_ang), mw1.centre_distance * math.sin(mw_ang))
    pos["minute_wheel"] = mw_c
    zlo = (mwz[0], mwz[0] + 0.25 * MM)
    zhi = (mwz[0] + 0.33 * MM, mwz[1])
    ph_mw = mesh_phase(0.0, mw1.z_driver, mw1.z_driven, (0, 0), mw_c)
    ph_hw = mesh_phase(0.0, mw2.z_driver, mw2.z_driven, mw_c, (0, 0))
    gear("cannon_pinion", "Cannon pinion", "centre", mw1.z_driver, mw1.z_driven, mw1.module, zlo, 0.0, pinion=True, color=STEEL, hole=0.3 * MM)
    parts[-1].group = "motion works"
    parts.append(Part("minute_wheel", "Minute wheel", "motion works", _rot(_poly(wheel_profile(mw1.z_driven, mw1.module, mw1.z_driver)).difference(_circle(0.12 * MM)), ph_mw),
                      zlo, mw_c, "train", "minute_wheel", ratio("minute_wheel"), BRASS, 1.0, "072-520", "brass"))
    parts.append(Part("minute_pinion", "Minute pinion", "motion works", _rot(_poly(pinion_profile(mw2.z_driver, mw2.module)), 0.0),
                      zhi, mw_c, "train", "minute_wheel", ratio("minute_wheel"), STEEL, 1.0, "072-520"))
    hw = _poly(wheel_profile(mw2.z_driven, mw2.module, mw2.z_driver)).difference(_circle(P["movement.hour_hand_hole"] / 2 + 0.05 * MM))
    parts.append(Part("hour_wheel", "Hour wheel", "motion works", _rot(hw, ph_hw), zhi, (0, 0), "train", "hour", ratio("hour"), BRASS, 1.0, "075-124", "brass"))
    parts.append(Part("hour_pipe", "Hour wheel pipe", "motion works", _ring(P["movement.minute_hand_hole"] / 2 + 0.06 * MM, P["movement.hour_hand_hole"] / 2),
                      (zhi[1], P["cad.hour_hand_z"]), (0, 0), "train", "hour", ratio("hour"), BRASS, 1.0, "075-124"))
    parts.append(Part("cannon_pipe", "Cannon pinion pipe", "motion works", _ring(sec_hole + 0.05 * MM, P["movement.minute_hand_hole"] / 2),
                      (zlo[1], P["cad.minute_hand_z"]), (0, 0), "train", "centre", ratio("centre"), STEEL, 1.0))

    def hand(name, label, length, width, tail, z, arb, color, hole):
        poly = Polygon([(-tail, -width * 0.6), (length * 0.85, -width / 2), (length, 0), (length * 0.85, width / 2), (-tail, width * 0.6)])
        poly = poly.union(_circle(max(width * 0.9, hole + 0.15 * MM))).buffer(0).difference(_circle(hole))
        poly = _rot(poly, math.pi / 2)        # reference pose: pointing to 12 o'clock
        parts.append(Part(name, label, "hands", poly, (z, z + 0.12 * MM), (0, 0), "train", arb, ratio(arb), color, 1.0, "", "steel"))

    hand("hour_hand", "Hour hand", 9.5 * MM, 1.0 * MM, 1.5 * MM, P["cad.hour_hand_z"], "hour", "#f2f2f2", P["movement.hour_hand_hole"] / 2)
    hand("minute_hand", "Minute hand", 14.0 * MM, 0.8 * MM, 2.0 * MM, P["cad.minute_hand_z"], "centre", "#f2f2f2", P["movement.minute_hand_hole"] / 2)
    hand("seconds_hand", "Seconds hand", 15.5 * MM, 0.25 * MM, 4.0 * MM, P["cad.seconds_hand_z"], "seconds", "#e4a3b0", 0.0)
    parts[-1].shape = parts[-1].shape.union(_circle(0.25 * MM)).buffer(0)

    # ---------------- automatic winding train + rotor ------------------------------------------
    zrp, zrv, zrvp, zrd, zrdp = (P.int("autowinding.rotor_pinion"), P.int("autowinding.reversing_wheel"),
                                 P.int("autowinding.reversing_pinion"), P.int("autowinding.reduction_wheel"),
                                 P.int("autowinding.reduction_pinion"))
    a1, a2, a3 = am * (zrp + zrv) / 2, am * (zrvp + zrd) / 2, am * (zrdp + zr) / 2
    Pb = np.array(pos["barrel"])
    best = None
    for th1 in np.radians(np.arange(0, 360, 2)):
        R1 = np.array([a1 * math.cos(th1), a1 * math.sin(th1)])
        d = np.linalg.norm(Pb - R1)
        if not (abs(a2 - a3) < d < a2 + a3):
            continue
        aa = (a2**2 - a3**2 + d**2) / (2 * d)
        hh = math.sqrt(max(a2**2 - aa**2, 0))
        base = R1 + aa * (Pb - R1) / d
        perp = np.array([-(Pb - R1)[1], (Pb - R1)[0]]) / d
        for s in (1, -1):
            R2 = base + s * hh * perp
            score = min(np.linalg.norm(R2 - np.array(B)), np.linalg.norm(R1 - np.array(B))) + 0.3 * min(R_mov - np.linalg.norm(R2), 1e-3)
            if best is None or score > best[0]:
                best = (score, R1, R2)
    _, R1, R2 = best
    l1, l2, l3 = band("ratchet"), band("auto_l2"), band("auto_l3")
    rp_phase = 0.0
    ph_rv = mesh_phase(rp_phase, zrp, zrv, (0, 0), tuple(R1))
    ph_rd = mesh_phase(0.0, zrvp, zrd, tuple(R1), tuple(R2))
    ph_rdp = mesh_phase(0.0, zr, zrdp, pos["barrel"], tuple(R2))       # pinion phase from ratchet
    r_rv = -zrp / zrv
    r_rd = r_rv * (-zrvp / zrd)
    parts.append(Part("rotor_pinion", "Rotor pinion", "automatic winding", _poly(pinion_profile(zrp, am)).difference(_circle(0.25 * MM)),
                      l3, (0, 0), "rotor", None, 1.0, STEEL, 1.0, "119-A17"))
    parts.append(Part("reversing_wheel", "Reversing wheel", "automatic winding", _rot(_poly(wheel_profile(zrv, am, zrp)).difference(_circle(0.15 * MM)), ph_rv),
                      l3, tuple(R1), "auto", None, r_rv, BRASS, 1.0, "141-190", "brass"))
    parts.append(Part("sliding_wheel_pinion", "Ratchet sliding wheel (one-way clutch)", "automatic winding", _poly(pinion_profile(zrvp, am)).difference(_circle(0.12 * MM)),
                      l2, tuple(R1), "auto", None, r_rv, STEEL, 1.0, "087-250"))
    parts.append(Part("reduction_wheel", "Reduction wheel", "automatic winding", _rot(_poly(wheel_profile(zrd, am, zrvp)).difference(_circle(0.15 * MM)), ph_rd),
                      l2, tuple(R2), "auto", None, r_rd, BRASS, 1.0, "088-120", "brass"))
    parts.append(Part("reduction_pinion", "Reduction pinion", "automatic winding", _rot(_poly(pinion_profile(zrdp, am)).difference(_circle(0.12 * MM)), ph_rdp),
                      l1, tuple(R2), "auto", None, r_rd, STEEL, 1.0, "088-120"))
    rr_ = P["autowinding.rotor_radius"]
    sector = Polygon([(0, 0)] + [(rr_ * math.cos(a), rr_ * math.sin(a)) for a in np.linspace(0, math.pi, 90)]).buffer(0)
    rotor = sector.difference(_ring(2.2 * MM, rr_ - 2.6 * MM).intersection(_circle(rr_)).difference(
        unary_union([_bar((0, 0), (rr_ * math.cos(a), rr_ * math.sin(a)), 1.4 * MM) for a in (0.35, 1.5708, 2.79)]))).union(_circle(1.6 * MM))
    rotor = rotor.difference(_circle(0.4 * MM))
    parts.append(Part("rotor", "Oscillating weight (rotor)", "automatic winding", rotor, band("rotor"), (0, 0), "rotor", None, 1.0,
                      "#b8bcc4", 0.92, "119-A17", "brass"))

    # ---------------- plates, bridges, jewels (static) -----------------------------------------
    disk = _circle(R_mov, res=128)
    pivots = {k: v for k, v in pos.items() if k in ("centre", "third", "fourth", "escape", "pallet", "balance", "barrel")}
    holes = unary_union([_circle(0.22 * MM, c) for c in pivots.values()])
    open_heart = _circle(bal.rim_outer - 0.55 * MM, B).difference(_bar(B, pos["pallet"], 1.0 * MM).union(_circle(0.9 * MM, B)))
    windows = []
    for ang0, ang1, r0, r1 in ((1.15, 2.05, 7.2, 11.4), (2.4, 3.1, 3.2, 8.8), (5.55, 6.15, 4.0, 8.0)):
        win = Polygon([(r0 * MM * math.cos(a), r0 * MM * math.sin(a)) for a in np.linspace(ang0, ang1, 20)] +
                      [(r1 * MM * math.cos(a), r1 * MM * math.sin(a)) for a in np.linspace(ang1, ang0, 20)]).buffer(-0.3 * MM).buffer(0.3 * MM)
        windows.append(win)
    plate = disk.difference(holes).difference(open_heart).difference(unary_union(windows)).difference(_circle(0.40 * MM))
    parts.append(Part("main_plate", "Main plate (pillar plate)", "structure", plate, band("plate"), (0, 0), "static", None, 0.0, "#d9dce1", 0.85, "", "brass-rhodium"))

    avoid_bal = _circle(bal.rim_outer + 0.35 * MM, B)

    def bridge(name, label, pts, width, zb, part_no, avoid=(), color="#cfd3d8"):
        segs = [_bar(pts[i], pts[i + 1], width) for i in range(len(pts) - 1)]
        g = unary_union(segs + [_circle(width * 0.62, p) for p in pts])
        for a in avoid:
            g = g.difference(a)
        g = g.intersection(_circle(R_mov - 0.15 * MM, res=128)).difference(holes)
        if isinstance(g, MultiPolygon):     # keep the main body only
            g = max(g.geoms, key=lambda q: q.area)
        parts.append(Part(name, label, "structure", g, zb, (0, 0), "static", None, 0.0, color, 0.85, part_no, "brass-rhodium"))

    def edge(ang, r=R_mov - 1.0 * MM):
        return (r * math.cos(ang), r * math.sin(ang))

    bridge("train_bridge", "Barrel and train wheel bridge", [edge(math.radians(25)), pos["barrel"], (4.2 * MM, 0.8 * MM), pos["third"],
                                                              pos["fourth"], pos["escape"], edge(math.radians(275))],
           1.9 * MM, band("train_bridge"), "701-F52", (avoid_bal, _circle(1.3 * MM)))
    bridge("centre_cock", "Centre wheel cock", [(0.0, 0.0), edge(math.radians(165))], 1.5 * MM, band("train_bridge"), "711-074", (avoid_bal,))
    bridge("pallet_bridge", "Pallet bridge", [pos["pallet"], edge(math.atan2(pos["pallet"][1], pos["pallet"][0]) + 0.25)], 1.3 * MM,
           band("pallet_bridge"), "708-095")
    ang_b = math.atan2(B[1], B[0])
    bridge("balance_cock", "Balance bridge (cock)", [B, edge(ang_b - 0.35, R_mov - 0.7 * MM)], 2.0 * MM, band("balance_cock"), "350-020")
    reg_arm = _bar(B, (B[0] + 3.4 * MM * math.cos(ang_b + 2.2), B[1] + 3.4 * MM * math.sin(ang_b + 2.2)), 0.35 * MM).union(_circle(0.7 * MM, B)).difference(_circle(0.25 * MM, B))
    bc_z = band("balance_cock")
    parts.append(Part("regulator", "Regulator index", "oscillator", reg_arm, (bc_z[0] - 0.12 * MM, bc_z[0]), (0, 0), "static", None, 0.0, STEEL, 1.0, "350-020"))
    # jewels (rubies) at pivots on plate and bridges
    jewel_sets = {"centre": ["train_bridge"], "third": ["plate", "train_bridge"], "fourth": ["plate", "train_bridge"],
                  "escape": ["plate", "train_bridge"], "pallet": ["plate", "pallet_bridge"], "balance": ["plate", "balance_cock"]}
    for arb, where in jewel_sets.items():
        for wb in where:
            zb = band(wb)
            z_j = (zb[1] - 0.12 * MM, zb[1] + 0.02 * MM) if wb == "plate" else (zb[0] - 0.02 * MM, zb[0] + 0.12 * MM)
            parts.append(Part(f"jewel_{arb}_{wb}", f"Jewel ({arb}, {wb})", "jewels", _ring(0.07 * MM, 0.42 * MM, pos[arb]), z_j,
                              (0, 0), "static", None, 0.0, RUBY, 0.9, "", "ruby"))

    # ---------------- keyless works (simplified) -----------------------------------------------
    zs = -0.55 * MM
    parts.append(Part("stem", "Setting stem", "keyless", _bar((7.4 * MM, 0), (21.5 * MM, 0), 0.9 * MM), (zs - 0.45 * MM, zs + 0.45 * MM),
                      (0, 0), "static", None, 0.0, STEEL, 1.0, "065-212"))
    parts.append(Part("winding_pinion", "Winding pinion", "keyless", _bar((9.2 * MM, 0), (10.6 * MM, 0), 1.6 * MM), (zs - 0.8 * MM, zs + 0.8 * MM),
                      (0, 0), "static", None, 0.0, STEEL, 1.0, "064-450"))
    parts.append(Part("clutch", "Clutch wheel", "keyless", _bar((7.8 * MM, 0), (9.0 * MM, 0), 1.4 * MM), (zs - 0.7 * MM, zs + 0.7 * MM),
                      (0, 0), "static", None, 0.0, STEEL, 1.0, "064-450"))
    sl = Polygon([(6.5 * MM, 1.0 * MM), (9.5 * MM, 1.6 * MM), (9.8 * MM, 3.2 * MM), (6.0 * MM, 2.6 * MM)])
    parts.append(Part("setting_lever", "Setting lever + yoke", "keyless", sl, (0.0, 0.25 * MM), (0, 0), "static", None, 0.0, STEEL, 1.0, "067-860"))
    crown = _bar((22.0 * MM, 0), (24.6 * MM, 0), 4.6 * MM)
    parts.append(Part("crown", "Crown", "case", crown, (zs - 2.3 * MM, zs + 2.3 * MM), (0, 0), "static", None, 0.0, "#c0c4ca", 1.0, "", "steel"))

    # ---------------- dial & case ----------------------------------------------------------------
    dz = band("dial")
    r_open = 9.8 * MM
    dial = _ring(r_open, 19.6 * MM)
    parts.append(Part("dial", "Skeleton dial (ring)", "dial", dial, dz, (0, 0), "static", None, 0.0, "#24262b", 0.97, "", "brass-lacquer"))
    idx = unary_union([_rot(_bar((0, 16.2 * MM), (0, 18.6 * MM), 0.55 * MM), 2 * math.pi * k / 12) for k in range(12)])
    parts.append(Part("indices", "Hour indices", "dial", idx, (dz[1], dz[1] + 0.15 * MM), (0, 0), "static", None, 0.0, "#f5f5f5", 1.0))
    parts.append(Part("inner_ring", "Inner chapter ring", "dial", _ring(r_open, r_open + 1.1 * MM), (dz[1], dz[1] + 0.12 * MM), (0, 0), "static", None, 0.0, "#101114", 1.0))
    case_t = P["watch.case_thickness"]
    case_z = (band("rotor")[0] - 1.6 * MM, band("rotor")[0] - 1.6 * MM + case_t)
    parts.append(Part("case", "Case middle (44 mm)", "case", _ring(19.9 * MM, P["watch.case_diameter"] / 2), case_z, (0, 0), "static", None, 0.0,
                      "#c7cbd1", 0.35, "", "316L"))
    parts.append(Part("crystal", "Crystal", "case", _circle(19.9 * MM, res=128), (case_z[1] - 0.9 * MM, case_z[1] - 0.2 * MM), (0, 0), "static", None, 0.0,
                      "#bfe3ff", 0.10, "", "sapphire"))
    parts.append(Part("caseback_glass", "Exhibition caseback glass", "case", _circle(14.5 * MM, res=128), (case_z[0], case_z[0] + 0.8 * MM), (0, 0), "static",
                      None, 0.0, "#bfe3ff", 0.10, "", "mineral glass"))
    return parts


def part_table(parts: list[Part]) -> list[dict]:
    return [{"name": p.name, "label": p.label, "group": p.group, "part_no": p.part_no, "motion": p.motion,
             "arbor": p.arbor, "ratio_to_escape": p.ratio, "z0_mm": p.z[0] * 1e3, "z1_mm": p.z[1] * 1e3,
             "x_mm": p.centre[0] * 1e3, "y_mm": p.centre[1] * 1e3, "area_mm2": p.shape.area * 1e6} for p in parts]
