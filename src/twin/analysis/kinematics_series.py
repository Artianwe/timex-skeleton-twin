"""Kinematic time series of every wheel, derived from the escape-wheel motion of a hybrid run.

The going train is rigid (backlash ignored), so every arbor angle is an exact multiple of the
escape-wheel angle:  theta_j(t) = (omega_j/omega_e) * psi(t).  Angular velocity and acceleration
follow by the same ratio. The escape wheel itself moves in steps (rest -> recoil -> impulse ->
drop -> lock), so wheel accelerations are impulsive at the lock instants - this module exposes
that stepping nature instead of pretending wheels rotate smoothly.
"""
from __future__ import annotations

import numpy as np

from ..dynamics import hybrid as H
from ..dynamics.simulator import SimResult
from ..model import WatchModel

TRAIN_ARBORS = ("barrel", "centre", "third", "fourth", "seconds", "escape", "minute_wheel", "hour")


def wheel_series(model: WatchModel, res: SimResult) -> dict[str, np.ndarray]:
    rec = res.rec
    t = rec[:, 0]
    psi, psid = rec[:, 4], rec[:, 5]
    psidd = np.gradient(psid, t)
    out = {"t": t, "balance_angle": rec[:, 1], "balance_velocity": rec[:, 2],
           "balance_acceleration": np.gradient(rec[:, 2], t),
           "fork_angle": rec[:, 3], "mode": rec[:, 6], "wheel_state": rec[:, 7]}
    w_e = model.train.omega["escape"]
    for a in TRAIN_ARBORS:
        k = model.train.omega[a] / w_e
        out[f"{a}_angle"] = k * psi
        out[f"{a}_velocity"] = k * psid
        out[f"{a}_acceleration"] = k * psidd
    return out


def mean_speeds(model: WatchModel, res: SimResult) -> dict[str, float]:
    """Average angular speeds measured from the simulation (rad/s) vs the kinematic prediction."""
    dpsi = res.state.y[H.S_PSI] - res.state0.y[H.S_PSI]
    dt = res.state.t - res.state0.t
    w_e_sim = dpsi / dt
    w_e = model.train.omega["escape"]
    return {a: w_e_sim * model.train.omega[a] / w_e for a in TRAIN_ARBORS}


EVENT_NAMES = [
    ("t_entry", "Impulse pin enters fork slot - UNLOCKING starts (1st timegrapher sound)"),
    ("t_unlock", "Unlocking complete - tooth leaves locking face, wheel released"),
    ("t_catch", "Wheel catches impulse plane - IMPULSE contact begins"),
    ("t_letoff", "Let-off - tooth leaves impulse plane, DROP begins"),
    ("t_lock", "Tooth lands on opposite pallet - LOCK (main 'tick')"),
    ("t_exit", "Pin leaves fork, fork rests on banking - FREE (supplementary) arc"),
]


def escapement_event_table(res: SimResult, beat_index: int = -2) -> list[dict]:
    """Ordered events of one beat with times relative to the beat start and the balance angle."""
    b = res.good_beats[beat_index]
    cols = {"t_entry": H.B_TENTRY, "t_unlock": H.B_TUNL, "t_catch": H.B_TCATCH, "t_letoff": H.B_TLETOFF,
            "t_lock": H.B_TLOCK, "t_exit": H.B_TEXIT}
    t0 = b[H.B_TENTRY]
    rec = res.rec
    rows = []
    for key, desc in EVENT_NAMES:
        t = b[cols[key]]
        if t <= 0:
            continue
        th = float(np.interp(t, rec[:, 0], rec[:, 1])) if len(rec) else float("nan")
        om = float(np.interp(t, rec[:, 0], rec[:, 2])) if len(rec) else float("nan")
        rows.append({"event": key, "description": desc, "t_ms": (t - t0) * 1e3, "balance_deg": np.degrees(th),
                     "balance_rad_s": om})
    return sorted(rows, key=lambda r: r["t_ms"])
