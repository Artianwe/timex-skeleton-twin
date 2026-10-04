"""Sensitivity analysis and uncertainty propagation.

1. One-at-a-time (OAT) perturbation of each parameter by +/-10 % (clipped to its plausible range),
   reporting the change of every output and the normalised sensitivity (elasticity)
        S = (dY / Y) / (dX / X)          (central difference)
   For the daily rate, whose baseline is near zero, the absolute change dRate per +10 % is used.

2. Monte-Carlo propagation: every ESTIMATED parameter that carries a range is sampled uniformly
   inside it; the resulting output distributions are compared with the VERIFIED specifications
   (power reserve 42 h, rate -20...+40 s/day, posture difference < 50 s/day).

The evaluation of one parameter set (`evaluate`) is self-contained so it can run in parallel
worker processes.
"""
from __future__ import annotations

import math
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from ..dynamics.power_reserve import find_stop
from ..dynamics.simulator import Simulator
from ..model import build_model
from ..params import ParamSet, load_params
from .energy import energy_flow
from .positional import posture_difference

OUTPUTS = ("amplitude_DU", "rate_DU", "amplitude_CD", "rate_CD", "posture_difference", "beat_error_DU",
           "power_reserve_h", "eta_total", "eta_escapement", "rate_mean_4pos")

OAT_KEYS = [
    "mainspring.youngs_modulus", "mainspring.working_stress", "mainspring.thickness", "mainspring.coil_friction",
    "mainspring.development_factor", "train.barrel_teeth",
    "balance.rim_thickness", "balance.inertia_scale", "hairspring.stiffness_scale", "hairspring.cubic_stiffness",
    "balance.pivot_friction_coeff", "balance.pivot_radius", "balance.unpoise_offset", "balance.beat_offset",
    "damping.q_air", "damping.q_oil", "damping.q_hairspring",
    "escapement.pallet_friction_coeff", "escapement.impulse_plane_angle", "escapement.drop_angle",
    "escapement.lock_angle", "escapement.draw_angle", "escapement.pallet_to_balance", "escapement.escape_tip_radius",
    "train_friction.mesh_friction_coeff", "train_friction.pivot_friction_coeff",
]


def evaluate(overrides: dict | None = None, config: str | None = None, reserve: bool = True) -> dict:
    P = load_params(config, overrides=overrides)
    m = build_model(P)
    sim = Simulator(m)
    T = float(m.spring.torque_out(m.spring.n_dev))
    out = {}
    rows = []
    for pos in ("DU", "CD", "CL", "CR"):
        r = sim.steady_state_fast(T, pos, measure_periods=20)
        rows.append({"position": pos, "rate_s_per_day": r.get("rate_s_per_day", np.nan),
                     "amplitude_deg": r.get("amplitude_deg", 0.0)})
        if pos == "DU":
            out["amplitude_DU"] = r.get("amplitude_deg", 0.0)
            out["rate_DU"] = r.get("rate_s_per_day", np.nan)
            out["beat_error_DU"] = r.get("beat_error_ms", np.nan)
            if r.get("running"):
                fl = energy_flow(m, r)
                out["eta_total"] = fl["eta_total"]
                out["eta_escapement"] = fl["eta_escapement"]
            else:
                out["eta_total"] = out["eta_escapement"] = 0.0
        if pos == "CD":
            out["amplitude_CD"] = r.get("amplitude_deg", 0.0)
            out["rate_CD"] = r.get("rate_s_per_day", np.nan)
    out["posture_difference"] = posture_difference(rows)
    out["rate_mean_4pos"] = float(np.nanmean([x["rate_s_per_day"] for x in rows]))
    if reserve:
        n_stop = find_stop(sim, "DU", tol=5e-3)
        out["power_reserve_h"] = (m.spring.n_dev - n_stop) * 2 * math.pi / m.barrel_omega / 3600.0
    return out


def _perturbed_value(P: ParamSet, key: str, sign: int, rel: float = 0.10) -> float:
    prm = P.info(key)
    v = prm.raw
    if v != 0:
        new = v * (1 + sign * rel)
    else:
        lo, hi = prm.range_raw
        new = v + sign * 0.25 * (hi - lo) / 2
    if prm.range_raw:
        lo, hi = prm.range_raw
        new = min(max(new, lo), hi)
    if key.startswith("train.") and prm.unit == "1":
        new = float(round(new))
    return new


def oat(keys=OAT_KEYS, rel: float = 0.10, workers: int = 2, config: str | None = None) -> dict:
    P = load_params(config)
    jobs = [("base", None, 0.0)]
    for k in keys:
        for s in (-1, 1):
            jobs.append((k, {k: _perturbed_value(P, k, s, rel)}, s))
    with ProcessPoolExecutor(max_workers=workers) as ex:
        res = list(ex.map(evaluate, [j[1] for j in jobs], [config] * len(jobs)))
    base = res[0]
    table = []
    for i, k in enumerate(keys):
        lo, hi = res[1 + 2 * i], res[2 + 2 * i]
        x0 = P.info(k).raw
        x_lo, x_hi = jobs[1 + 2 * i][1][k], jobs[2 + 2 * i][1][k]
        row = {"param": k, "x0": x0, "x_lo": x_lo, "x_hi": x_hi, "prov": P.info(k).prov}
        for o in OUTPUTS:
            y0, yl, yh = base.get(o, np.nan), lo.get(o, np.nan), hi.get(o, np.nan)
            row[f"{o}_lo"], row[f"{o}_hi"] = yl, yh
            row[f"d_{o}"] = yh - yl
            if x0 != 0 and y0 not in (0, np.nan) and abs(y0) > 1e-12 and x_hi != x_lo:
                row[f"S_{o}"] = ((yh - yl) / y0) / ((x_hi - x_lo) / x0)
            else:
                row[f"S_{o}"] = np.nan
        table.append(row)
    return {"base": base, "table": table, "rel": rel}


def ranking(oat_res: dict, output: str, by: str = "d") -> list[tuple[str, float]]:
    key = f"{by}_{output}"
    rows = [(r["param"], r[key]) for r in oat_res["table"] if np.isfinite(r[key])]
    return sorted(rows, key=lambda t: -abs(t[1]))


MC_EXCLUDE_PREFIX = ("layout.", "cad.", "simulation.", "watch.", "movement.", "autowinding.", "keyless.", "train.")
# perturbation knobs of an existing watch (trimmed away by regulation at assembly), not estimation
# uncertainties -> excluded from uncertainty propagation
MC_EXCLUDE_KEYS = ("hairspring.regulator_offset", "balance.unpoise_angle", "balance.inertia_scale", "hairspring.stiffness_scale")


def monte_carlo(n: int = 96, seed: int = 7, workers: int = 2, config: str | None = None) -> dict:
    P = load_params(config)
    keys = [p.key for p in P.ranged() if p.prov in ("assumed", "inferred") and not p.key.startswith(MC_EXCLUDE_PREFIX)
            and p.key not in MC_EXCLUDE_KEYS]
    rng = np.random.default_rng(seed)
    samples = []
    for _ in range(n):
        ov = {}
        for k in keys:
            lo, hi = P.info(k).range_raw
            ov[k] = float(rng.uniform(lo, hi))
        samples.append(ov)
    with ProcessPoolExecutor(max_workers=workers) as ex:
        res = list(ex.map(evaluate, samples, [config] * n))
    arr = {o: np.array([r.get(o, np.nan) for r in res], float) for o in OUTPUTS}
    return {"keys": keys, "samples": samples, "outputs": arr, "n": n}
