"""Planar layout synthesis of the movement (positions of every arbor).

Known: centre distances of every mesh (from tooth counts and modules), the straight-line
escapement (escape wheel, pallet staff and balance staff collinear), and the MEASURED balance
position (photogrammetry). Unknown: the angles at which the train winds around the plate.

The chain  centre -> third -> fourth -> escape -> (pallet) -> balance  has two free angles
(third, fourth); the last two links are solved in closed form as a two-link inverse-kinematics
problem so that the balance lands exactly on its measured position:

    a5 u(alpha5) + L u(alpha6) = B - F,   L = d_ep + d_bp
    alpha5 = angle(D) +/- acos((a5^2 + |D|^2 - L^2) / (2 a5 |D|))

All (alpha3, alpha4, branch, barrel angle) combinations are scored by the minimum clearance
between bodies and foreign arbors that overlap axially (z-aware), and parts must fit inside the
movement. The layout maximising the minimum clearance is selected - a small, deterministic
mechanism-synthesis problem.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ..model import WatchModel


@dataclass
class Body:
    name: str
    centre: np.ndarray
    r: float
    z: tuple[float, float]
    kind: str = "body"        # body | arbor


@dataclass
class Layout:
    pos: dict[str, np.ndarray]
    angles: dict[str, float]
    min_clearance: float
    bodies: list[Body] = field(default_factory=list)
    checks: list[dict] = field(default_factory=list)

    def xy(self, name: str) -> tuple[float, float]:
        p = self.pos[name]
        return float(p[0]), float(p[1])


def _u(a):
    return np.stack([np.cos(a), np.sin(a)], axis=-1)


def _z_overlap(a, b) -> bool:
    return min(a[1], b[1]) > max(a[0], b[0])


def body_list(m: WatchModel, pos: dict, P) -> list[Body]:
    tr = m.train
    mm = {x.name: x for x in tr.meshes}
    b = lambda k: P.band(f"cad.{k}")  # noqa: E731
    arb = 0.15e-3
    bodies = [
        Body("barrel", pos["barrel"], mm["barrel->centre"].r_driver + 1.1 * mm["barrel->centre"].module, b("barrel")),
        Body("center_wheel", pos["centre"], mm["centre->third"].r_driver + 1.35 * mm["centre->third"].module, b("center_wheel")),
        Body("third_wheel", pos["third"], mm["third->fourth"].r_driver + 1.35 * mm["third->fourth"].module, b("third_wheel")),
        Body("fourth_wheel", pos["fourth"], mm["fourth->escape"].r_driver + 1.35 * mm["fourth->escape"].module, b("fourth_wheel")),
        Body("escape_wheel", pos["escape"], m.esc.r_e, b("escape_wheel")),
        Body("pallet_fork", pos["pallet"], m.esc.rho_lock + 0.35e-3, b("pallet_fork")),
        Body("balance_rim", pos["balance"], m.balance.rim_outer, b("balance_rim")),
        Body("arbor_centre", pos["centre"], arb + 0.10e-3, (b("train_bridge")[0], 0.0), "arbor"),
        Body("arbor_third", pos["third"], arb, (b("train_bridge")[0], b("plate")[0]), "arbor"),
        Body("arbor_fourth", pos["fourth"], arb, (b("train_bridge")[0], b("plate")[0]), "arbor"),
        Body("arbor_escape", pos["escape"], arb, (b("train_bridge")[0], b("plate")[0]), "arbor"),
        Body("arbor_pallet", pos["pallet"], arb, (b("pallet_bridge")[0], b("plate")[0]), "arbor"),
        Body("arbor_balance", pos["balance"], arb, (b("balance_cock")[0], b("plate")[0]), "arbor"),
        Body("arbor_barrel", pos["barrel"], 0.5e-3, (b("ratchet")[0], b("plate")[0]), "arbor"),
    ]
    return bodies


_OWN = {"barrel": "arbor_barrel", "center_wheel": "arbor_centre", "third_wheel": "arbor_third",
        "fourth_wheel": "arbor_fourth", "escape_wheel": "arbor_escape", "pallet_fork": "arbor_pallet",
        "balance_rim": "arbor_balance"}
# pairs that interpenetrate in plan BY DESIGN (a wheel's teeth reach its partner's pinion leaves)
_MESHING = {frozenset(p) for p in [("escape_wheel", "pallet_fork"), ("center_wheel", "arbor_third"),
                                    ("third_wheel", "arbor_fourth"), ("third_wheel", "arbor_centre"),
                                    ("fourth_wheel", "arbor_escape"), ("barrel", "arbor_centre"),
                                    ("pallet_fork", "arbor_balance"), ("balance_rim", "arbor_pallet")]}


def clearance_report(bodies: list[Body], R_move: float, margin: float) -> list[dict]:
    out = []
    for i, a in enumerate(bodies):
        if a.kind == "body":
            fit = R_move - margin - (np.hypot(*a.centre) + a.r)
            out.append({"a": a.name, "b": "movement edge", "clearance": float(fit)})
        for b2 in bodies[i + 1:]:
            if a.kind == "arbor" and b2.kind == "arbor":
                continue
            if _OWN.get(a.name) == b2.name or _OWN.get(b2.name) == a.name:
                continue
            if frozenset((a.name, b2.name)) in _MESHING:
                continue
            if not _z_overlap(a.z, b2.z):
                continue
            d = float(np.hypot(*(a.centre - b2.centre)))
            out.append({"a": a.name, "b": b2.name, "clearance": d - a.r - b2.r})
    return out


def solve_layout(m: WatchModel, step_deg: float = 1.0) -> Layout:
    P = m.P
    tr = m.train
    mm = {x.name: x for x in tr.meshes}
    a3 = mm["centre->third"].centre_distance
    a4 = mm["third->fourth"].centre_distance
    a5 = mm["fourth->escape"].centre_distance
    L = m.esc.d_ep + m.esc.d_bp
    rb = P["layout.balance_centre_radius"]
    thb = P["layout.balance_centre_angle"]
    B = np.array([rb * math.cos(thb), rb * math.sin(thb)])
    R_move = P["movement.diameter"] / 2.0
    margin = P["layout.margin_to_edge"]
    abar = mm["barrel->centre"].centre_distance

    al3 = np.radians(np.arange(0.0, 360.0, step_deg))
    al4 = np.radians(np.arange(0.0, 360.0, step_deg))
    A3, A4 = np.meshgrid(al3, al4, indexing="ij")
    T3 = a3 * _u(A3)
    F = T3 + a4 * _u(A4)
    D = B - F
    dn = np.hypot(D[..., 0], D[..., 1])
    cosg = (a5**2 + dn**2 - L**2) / (2 * a5 * dn)
    ok = np.abs(cosg) <= 1.0
    g = np.arccos(np.clip(cosg, -1, 1))
    phD = np.arctan2(D[..., 1], D[..., 0])
    best = None
    bar_angles = [P["layout.barrel_angle"]] + list(np.radians(np.arange(30.0, 130.0, 10.0)))
    for br, sgn in [(0, 1.0), (0, -1.0)]:
        A5 = phD + sgn * g
        E = F + a5 * _u(A5)
        U6 = (B - E) / L
        A6 = np.arctan2(U6[..., 1], U6[..., 0])
        Pp = E + m.esc.d_ep * _u(A6)
        # vectorised quick screening with the main clearance terms
        r3 = mm["third->fourth"].r_driver + 1.35 * mm["third->fourth"].module
        r4 = mm["fourth->escape"].r_driver + 1.35 * mm["fourth->escape"].module
        re = m.esc.r_e
        rbal = m.balance.rim_outer
        arb = 0.15e-3
        fit = np.minimum.reduce([
            R_move - margin - (np.hypot(*np.moveaxis(T3, -1, 0)) + r3),
            R_move - margin - (np.hypot(*np.moveaxis(F, -1, 0)) + r4),
            R_move - margin - (np.hypot(*np.moveaxis(E, -1, 0)) + re),
        ])
        dist = lambda X, Y: np.hypot(*np.moveaxis(X - Y, -1, 0))  # noqa: E731
        cl = np.minimum.reduce([
            fit,
            dist(T3, B) - rbal - arb, dist(F, B) - rbal - arb, dist(E, B) - rbal - arb,   # arbors vs balance rim
            dist(F, np.zeros(2)) - r4 - arb - 0.1e-3,                                      # 4th wheel vs centre arbor
            dist(E, np.zeros(2)) - re - arb - 0.1e-3,
            dist(E, T3) - re - arb, dist(F, Pp) - r4 - arb, dist(T3, Pp) - r3 - arb,
        ])
        cl = np.where(ok, cl, -np.inf)
        idx = np.unravel_index(np.argsort(cl, axis=None)[::-1][:400], cl.shape)
        for i, j in zip(*idx):
            if not np.isfinite(cl[i, j]):
                continue
            pos0 = {"centre": np.zeros(2), "third": T3[i, j], "fourth": F[i, j], "escape": E[i, j],
                    "pallet": Pp[i, j], "balance": B}
            for ba in bar_angles:
                pos = dict(pos0)
                pos["barrel"] = abar * np.array([math.cos(ba), math.sin(ba)])
                bodies = body_list(m, pos, P)
                rep = clearance_report(bodies, R_move, margin)
                mc = min(r["clearance"] for r in rep)
                # prefer the configured barrel angle unless it is infeasible
                score = mc + (2e-5 if ba == bar_angles[0] else 0.0)
                if best is None or score > best[0]:
                    ang = {"third": float(A3[i, j]), "fourth": float(A4[i, j]), "escape": float(A5[i, j]),
                           "escapement_line": float(A6[i, j]), "barrel": float(ba)}
                    best = (score, pos, ang, bodies, rep, mc)
    score, pos, ang, bodies, rep, mc = best
    return Layout(pos=pos, angles=ang, min_clearance=mc, bodies=bodies, checks=rep)
