"""Positional analysis: the six standard test positions.

Gravity enters through
  1. pivot friction: horizontal positions load the pivot END on the cap jewel (small radius),
     vertical positions load the journal in the hole jewel (larger radius) -> lower amplitude;
  2. out-of-poise torque  -m g e sin(gamma - beta - theta)  in vertical positions, whose
     rate effect depends on amplitude as J1(A)/A (it changes sign near A = 220 deg).
Dial up and dial down differ only if the two pivots are made different (kept equal here).
"""
from __future__ import annotations

import numpy as np

from ..dynamics.simulator import Simulator

POSITIONS = ("DU", "DD", "CU", "CD", "CL", "CR")


def positional_table(sim: Simulator, T_barrel: float, measure_periods: int = 30) -> list[dict]:
    rows = []
    for pos in POSITIONS:
        r = sim.steady_state_fast(T_barrel, pos, measure_periods=measure_periods)
        rows.append({"position": pos, "label": sim.P.positions[pos]["label"],
                     "amplitude_deg": r.get("amplitude_deg", np.nan), "rate_s_per_day": r.get("rate_s_per_day", np.nan),
                     "beat_error_ms": r.get("beat_error_ms", np.nan), "running": r.get("running", False),
                     "power_uW": r.get("power_escape_wheel", 0.0) * 1e6})
    return rows


def posture_difference(rows: list[dict], positions=("DU", "CD", "CL", "CR")) -> float:
    """Max - min rate over the positions used by the Miyota spec (dial up + three verticals)."""
    r = [x["rate_s_per_day"] for x in rows if x["position"] in positions]
    return float(np.max(r) - np.min(r))


def mean_rate(rows: list[dict]) -> float:
    return float(np.mean([x["rate_s_per_day"] for x in rows]))


def unpoise_amplitude_sweep(sim: Simulator, position: str = "CD", torques=None) -> dict:
    """Rate with and without unpoise vs amplitude -> isolates the J1(A)/A signature."""
    from ..dynamics import hybrid as H
    spring = sim.m.spring
    torques = torques if torques is not None else spring.torque_out(np.linspace(0.05, 1.0, 12) * spring.n_dev)
    A, d = [], []
    for T in torques:
        p = sim.params_vector(float(T), position)
        r1 = sim.steady_state_fast(float(T), position, p=p, measure_periods=20)
        p0 = p.copy()
        p0[H.P_UON] = 0.0
        r0 = sim.steady_state_fast(float(T), position, p=p0, measure_periods=20)
        if r1.get("running") and r0.get("running"):
            A.append(r1["amplitude_deg"])
            d.append(r1["rate_s_per_day"] - r0["rate_s_per_day"])
    return {"amplitude_deg": np.array(A), "unpoise_rate_effect": np.array(d)}
