"""Timekeeping analysis: rate, amplitude, beat error, isochronism and error decomposition.

Rate definition (what a timegrapher reports):
    rate [s/day] = 86400 * (f_actual / f_nominal - 1),   f_nominal = 3 Hz
because the hands advance exactly one escape tooth per balance period.

Error decomposition at a given mainspring torque and position:
    free oscillator  : linear balance + hairspring, no damping       -> 0 (by construction)
    + damping        : viscous: Delta f/f = -1/(8 Q^2) (negligible); Coulomb: 0 (period unchanged)
    + gravity        : out-of-poise torque in vertical positions (amplitude dependent)
    + escapement     : unlocking resistance before the dead point and impulse around it
                       (Airy's theorem) - the dominant isochronism error of a lever escapement
Each contribution is isolated by switching it off in the parameter vector and re-running.
"""
from __future__ import annotations

import numpy as np

from ..dynamics import hybrid as H
from ..dynamics.simulator import Simulator


def isochronism_curve(sim: Simulator, position: str = "DU", torques=None, measure_periods: int = 30) -> dict:
    spring = sim.m.spring
    if torques is None:
        torques = spring.torque_out(np.linspace(0.08, 1.0, 14) * spring.n_dev)
    rows = [sim.steady_state_fast(float(T), position, measure_periods=measure_periods) for T in torques]
    ok = [r for r in rows if r.get("running")]
    return {"T_barrel": np.array([r["T_barrel"] for r in ok]),
            "amplitude_deg": np.array([r["amplitude_deg"] for r in ok]),
            "rate": np.array([r["rate_s_per_day"] for r in ok]),
            "beat_error_ms": np.array([r["beat_error_ms"] for r in ok]),
            "timegrapher_amplitude_deg": np.array([r.get("timegrapher_amplitude_deg", np.nan) for r in ok])}


def error_decomposition(sim: Simulator, T_barrel: float, position: str = "CD") -> dict:
    """Contribution of each physical effect to the rate (s/day) at one operating point."""
    base = sim.params_vector(T_barrel, position)
    out = {}

    def rate(p):
        r = sim.steady_state_fast(T_barrel, position, p=p, measure_periods=30)
        return r.get("rate_s_per_day", np.nan), r.get("amplitude_deg", np.nan)

    full, A = rate(base)
    out["total"] = full
    out["amplitude_deg"] = A
    p = base.copy()
    p[H.P_UON] = 0.0
    no_grav, _ = rate(p)
    out["gravity_unpoise"] = full - no_grav
    p2 = p.copy()
    p2[H.P_TBE] = 0.0
    no_be, _ = rate(p2)
    out["beat_offset_asymmetry"] = no_grav - no_be
    out["escapement_and_damping"] = no_be
    out["regulator_offset"] = (sim.m.balance.f_free / sim.m.balance.f_design - 1.0) * 86400.0
    return out
