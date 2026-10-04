"""Power reserve: multi-time-scale coupling of mainspring (hours) and escapement (milliseconds).

Separation of time scales
-------------------------
The balance amplitude relaxes with time constant tau = 2Q/omega_0 ~ 30 s, whereas the barrel
torque changes by <0.1 % per minute. The fast subsystem is therefore always on its limit cycle
for the instantaneous torque (quasi-static / averaging approximation). The slow subsystem is
kinematic: the escapement lets the train advance exactly one escape tooth per balance period,
so while the watch runs the barrel unwinds at the constant rate

        dn/dt = - omega_barrel / (2 pi) = - 1 / (T_barrel_rev)          (turns per second)

independent of amplitude. Hence n(t) = n_0 - t / T_rev, T_b(t) = M_out(n(t)), and every fast
quantity (amplitude, rate, losses) follows from the limit-cycle map evaluated at T_b(t).

The power reserve is the time until the limit cycle ceases to exist (the impulse can no longer
replace the losses) - found by bisection on n.

`brute_force()` integrates the full hybrid model through the whole reserve (minutes of CPU)
and is used to validate the quasi-static result.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ..units import DEG
from . import hybrid as H
from .simulator import Simulator


@dataclass
class ReserveMap:
    position: str
    n: np.ndarray                 # wound turns (descending from full)
    T_barrel: np.ndarray
    T_escape: np.ndarray
    amplitude: np.ndarray         # deg
    rate: np.ndarray              # s/day
    beat_error: np.ndarray        # ms
    power_escape: np.ndarray      # W
    running: np.ndarray
    ledgers: list = field(default_factory=list)
    n_stop: float = 0.0
    loss_powers: dict = field(default_factory=dict)   # category -> W at each map point
    power_balance: np.ndarray | None = None


def reserve_map(sim: Simulator, position: str = "DU", n_points: int | None = None,
                measure_periods: int = 30) -> ReserveMap:
    spring = sim.m.spring
    n_points = n_points or int(sim.P["simulation.torque_grid_points"])
    n_stop = find_stop(sim, position)
    # geometric clustering toward the stop point, where torque and amplitude change fastest
    s = np.linspace(0.0, 1.0, n_points)
    beta = 4.0
    n = n_stop + (spring.n_dev - n_stop) * np.expm1(beta * s) / np.expm1(beta)
    n[0] = n_stop + 1e-4
    n = np.sort(n)[::-1]
    rows = []
    for ni in n:
        T = float(spring.torque_out(ni))
        r = sim.steady_state_fast(T, position, measure_periods=measure_periods)
        rows.append(r)
    get = lambda k, d=np.nan: np.array([r.get(k, d) for r in rows], float)  # noqa: E731
    cats = [k for k in (rows[0].get("ledger") or {}) if k not in ("escape_wheel_input", "delta_mechanical_energy",
                                                                   "total_losses", "closure_error")]
    lp = {c: np.array([(r["ledger"][c] / r["duration"]) if r.get("ledger") else 0.0 for r in rows]) for c in cats}
    pbal = sum(lp[c] for c in cats if c.startswith("balance_")) if cats else None
    return ReserveMap(position, n, get("T_barrel"), get("T_escape"), get("amplitude_deg", 0.0),
                      get("rate_s_per_day"), get("beat_error_ms"), get("power_escape_wheel", 0.0),
                      np.array([bool(r.get("running")) for r in rows]),
                      [r.get("ledger", {}) for r in rows], n_stop, lp, pbal)


def find_stop(sim: Simulator, position: str = "DU", tol: float = 1e-3) -> float:
    """Wound turns n at which the limit cycle disappears (watch stops)."""
    spring = sim.m.spring
    runs = lambda n: sim.limit_cycle(float(spring.torque_out(n)), position) is not None  # noqa: E731
    lo, hi = 0.0, spring.n_dev
    if not runs(hi):
        return hi
    if runs(lo + 1e-6):
        return 0.0
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        if runs(mid):
            hi = mid
        else:
            lo = mid
    return hi


@dataclass
class ReserveCurve:
    position: str
    t_h: np.ndarray
    n: np.ndarray
    T_barrel: np.ndarray
    amplitude: np.ndarray
    rate: np.ndarray
    beat_error: np.ndarray
    power_escape: np.ndarray
    power_barrel: np.ndarray
    energy_remaining: np.ndarray      # J (elastic energy above let-down)
    cumulative_error_s: np.ndarray
    reserve_h: float
    power_balance: np.ndarray | None = None
    loss_powers: dict | None = None


def reserve_curve(sim: Simulator, rmap: ReserveMap, n0: float | None = None, dt_h: float = 0.25) -> ReserveCurve:
    """Quasi-static time history from wound state n0 (default: fully wound)."""
    m = sim.m
    spring = m.spring
    n0 = spring.n_dev if n0 is None else n0
    T_rev = 2.0 * math.pi / m.barrel_omega            # s per barrel revolution
    reserve_h = (n0 - rmap.n_stop) * T_rev / 3600.0
    t = np.arange(0.0, reserve_h + 1e-9, dt_h)
    n = n0 - t * 3600.0 / T_rev
    order = np.argsort(rmap.n)
    xi, interp = rmap.n[order], lambda arr: np.interp(n, xi, arr[order])  # noqa: E731
    A = interp(rmap.amplitude)
    rate = interp(rmap.rate)
    be = interp(rmap.beat_error)
    pe = interp(rmap.power_escape)
    Tb = spring.torque_out(n)
    pb = Tb * m.barrel_omega
    E = spring.energy_elastic(n)
    cum = np.concatenate([[0.0], np.cumsum(0.5 * (rate[1:] + rate[:-1]) * np.diff(t) / 24.0)])
    pbal = interp(rmap.power_balance) if rmap.power_balance is not None else None
    lp = {k: interp(v) for k, v in rmap.loss_powers.items()} if rmap.loss_powers else None
    return ReserveCurve(rmap.position, t, n, Tb, A, rate, be, pe, pb, E, cum, reserve_h, pbal, lp)


def brute_force(sim: Simulator, position: str = "DU", n0: float | None = None, chunk_s: float = 60.0,
                max_hours: float = 60.0, progress=None) -> dict:
    """Full hybrid integration through the power reserve (validation of the quasi-static model).

    The barrel torque is updated after every `chunk_s` seconds from the exact escape-wheel
    advance (the train is rigid, so barrel turns = escape-wheel turns / i)."""
    m = sim.m
    spring = m.spring
    i_tot = m.train.escape_to_barrel
    n = spring.n_dev if n0 is None else n0
    T = float(spring.torque_out(n))
    p = sim.params_vector(T, position)
    A0 = sim.limit_cycle(T, position, p=p) or 250 * DEG
    state = sim.initial_state(A0, p)
    out = {k: [] for k in ("t_h", "n", "T_barrel", "amplitude", "rate", "beat_error", "W_escape")}
    t = 0.0
    while t < max_hours * 3600.0:
        p[H.P_TE] = max(m.losses.escape_torque(T), 0.0)
        p[H.P_TEB] = m.losses.escape_torque_backdrive(T)
        r = sim.run(T, position, chunk_s, state=state, p=p)
        dpsi = r.state.y[H.S_PSI] - r.state0.y[H.S_PSI]
        n -= dpsi / (2.0 * math.pi * i_tot)
        t = r.state.t
        met = r.metrics()
        out["t_h"].append(t / 3600.0)
        out["n"].append(n)
        out["T_barrel"].append(T)
        out["amplitude"].append(met.get("amplitude_deg", 0.0))
        out["rate"].append(met.get("rate_s_per_day", np.nan))
        out["beat_error"].append(met.get("beat_error_ms", np.nan))
        out["W_escape"].append(r.ledger["escape_wheel_input"])
        state = r.state
        T = float(spring.torque_out(n))
        if progress:
            progress(t, n, met)
        ok = r.good_beats
        if r.status != 0 or len(ok) < 0.5 * chunk_s * 2 * m.balance.f_design:
            break
    res = {k: np.array(v) for k, v in out.items()}
    res["reserve_h"] = float(res["t_h"][-1]) if len(res["t_h"]) else 0.0
    return res
