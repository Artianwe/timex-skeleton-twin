"""Automatic (self-winding) system: rotor dynamics with a one-way clutch.

The oscillating weight is an eccentric mass (m, centre of mass at r_cm) pivoting in the case
plane. The case moves with the wrist: its dial plane is tilted by tau from horizontal (so gravity
has an in-plane component g sin tau) and it rotates in its own plane by chi(t). Relative to the
case, the rotor angle psi obeys

    I_r (psi'' + chi'') = m g sin(tau) r_cm sin(gamma - chi - psi) - T_b sgn(psi') - T_w(psi')

where gamma is the in-plane direction of gravity in the inertial frame. The Miyota 82 family
winds in ONE direction only (ratchet sliding wheel = one-way clutch): when psi' has the winding
sense the rotor must drive the reduction train against the mainspring,

    T_w = M_wind(n) / (R * eta_auto)     (R = rotor turns per barrel-arbor turn)

otherwise it freewheels (bearing drag only). Barrel winding:  dn/dt = |psi'| / (2 pi R).

Wrist motion is a modelling ASSUMPTION (documented activity profiles); the output of interest is
the net winding rate compared with the escapement-governed consumption of 1/7.5 turns per hour.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numba import njit

from ..model import WatchModel

PROFILES = {
    # name: (tilt deg, in-plane swing amplitude deg, swing frequency Hz, gesture rate per min, gesture amplitude deg)
    "off-wrist (desk)": (0.0, 0.0, 0.5, 0.0, 0.0),
    "desk work": (35.0, 4.0, 0.25, 1.5, 70.0),
    "walking": (70.0, 28.0, 0.95, 2.0, 90.0),
    "brisk walking / chores": (75.0, 40.0, 1.1, 4.0, 120.0),
    "sport": (80.0, 60.0, 1.4, 8.0, 150.0),
}


@dataclass
class RotorParams:
    m: float
    r_cm: float
    I: float
    R: float
    eta: float
    T_bearing: float
    direction: float


def rotor_params(model: WatchModel) -> RotorParams:
    P = model.P
    m = P["autowinding.rotor_mass"]
    Rr = P["autowinding.rotor_radius"]
    R = (P.int("autowinding.reversing_wheel") / P.int("autowinding.rotor_pinion")) * \
        (P.int("autowinding.reduction_wheel") / P.int("autowinding.reversing_pinion")) * \
        (P.int("keyless.ratchet_wheel") / P.int("autowinding.reduction_pinion"))
    return RotorParams(m, P["autowinding.rotor_cm_radius"], 0.5 * m * Rr**2, R, P["autowinding.efficiency"],
                       P["autowinding.bearing_friction"], P["autowinding.winding_direction"])


@njit(cache=True)
def _rotor_run(m, r_cm, I, T_w, T_b, direction, tilt, amp, freq, gest_t, gest_amp, dur, dt, g):
    """Integrate rotor motion; returns winding angle (rad of rotor in winding sense) and work."""
    psi = 0.0
    om = 0.0
    wound = 0.0
    n = int(dur / dt)
    gp = g * math.sin(tilt)
    gamma = -math.pi / 2.0
    gi = 0
    ng = gest_t.shape[0]
    for i in range(n):
        t = i * dt
        # in-plane case rotation: sinusoidal swing + smooth gestures (raised-cosine, 0.6 s)
        w = 2.0 * math.pi * freq
        chi = amp * math.sin(w * t)
        chidd = -amp * w * w * math.sin(w * t)
        for k in range(gi, ng):
            tk = gest_t[k]
            if t < tk:
                break
            if t < tk + 0.6:
                s = (t - tk) / 0.6
                chi += gest_amp[k] * 0.5 * (1.0 - math.cos(math.pi * s))
                chidd += gest_amp[k] * 0.5 * (math.pi / 0.6) ** 2 * math.cos(math.pi * s)
            elif k == gi:
                gi += 1
        for k in range(0, gi):
            chi += gest_amp[k]
        Tg = m * gp * r_cm * math.sin(gamma - chi - psi)
        winding = om * direction > 0.0
        Tl = T_b * (1.0 if om > 0 else (-1.0 if om < 0 else 0.0))
        if winding:
            Tl += T_w * (1.0 if om > 0 else -1.0)
        acc = (Tg - Tl) / I - chidd
        # semi-implicit Euler with friction stick check
        om_new = om + acc * dt
        if om != 0.0 and om * om_new < 0.0:
            om_new = 0.0
        if om == 0.0 and abs(Tg - I * chidd) < T_b:
            om_new = 0.0
        psi += om_new * dt
        if om_new * direction > 0.0:
            wound += abs(om_new) * dt
        om = om_new
    return wound


def winding_rate(model: WatchModel, profile: str, n_wound: float | None = None, duration: float = 300.0,
                 dt: float = 2e-4, seed: int = 1) -> dict:
    """Mean barrel winding rate (turns/h) for an activity profile at wind state n."""
    rp = rotor_params(model)
    sp = model.spring
    n = sp.n_dev * 0.5 if n_wound is None else n_wound
    T_w = float(sp.torque_wind(n)) / (rp.R * rp.eta)
    tilt, amp, f, grate, gamp = PROFILES[profile]
    rng = np.random.default_rng(seed)
    k = rng.poisson(grate * duration / 60.0)
    gt = np.sort(rng.uniform(0, duration, k))
    ga = np.radians(gamp) * rng.choice([-1.0, 1.0], k) * rng.uniform(0.4, 1.0, k)
    wound = _rotor_run(rp.m, rp.r_cm, rp.I, T_w, rp.T_bearing, rp.direction, math.radians(tilt), math.radians(amp),
                       f, gt, ga, duration, dt, model.P["simulation.gravity"])
    turns_per_h = wound / (2 * math.pi * rp.R) / duration * 3600.0
    return {"profile": profile, "turns_per_hour": turns_per_h, "consumption_turns_per_hour": 1.0 / (2 * math.pi / model.barrel_omega / 3600.0),
            "rotor_turns_per_hour_winding": wound / (2 * math.pi) / duration * 3600.0, "T_wind_at_rotor": T_w}


def daily_scenario(model: WatchModel, schedule=None, n0: float | None = None, days: int = 3) -> dict:
    """Wind state over several days of a repeating activity schedule (hours, profile)."""
    schedule = schedule or [(7.0, "off-wrist (desk)"), (1.0, "brisk walking / chores"), (8.0, "desk work"),
                            (1.5, "walking"), (5.5, "desk work"), (1.0, "walking")]
    sp = model.spring
    n = sp.n_dev * 0.6 if n0 is None else n0
    cons = model.barrel_omega / (2 * math.pi) * 3600.0          # turns per hour
    cache = {}
    t, ts, ns = 0.0, [0.0], [n]
    for _ in range(days):
        for hours, prof in schedule:
            steps = max(int(hours * 4), 1)
            for _s in range(steps):
                key = (prof, round(n / sp.n_dev, 1))
                if key not in cache:
                    cache[key] = winding_rate(model, prof, n_wound=max(n, 0.05), duration=120.0)["turns_per_hour"] if prof != "off-wrist (desk)" else 0.0
                dn = (cache[key] - cons) * hours / steps
                n = float(np.clip(n + dn, 0.0, sp.n_dev))
                t += hours / steps
                ts.append(t)
                ns.append(n)
    return {"t_h": np.array(ts), "n": np.array(ns), "n_dev": sp.n_dev, "consumption": cons}
