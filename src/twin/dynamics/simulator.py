"""High-level interface to the hybrid escapement/balance simulation.

    from twin.model import build_model
    from twin.dynamics.simulator import Simulator
    sim = Simulator(build_model())
    res = sim.run(T_barrel=3.3e-3, position="DU", duration=20.0)
    ss  = sim.steady_state(T_barrel=3.3e-3, position="DU")

All quantities SI. The barrel torque is held constant during a run (it changes by < 1e-4 per
minute); long runs are chained by twin.dynamics.power_reserve.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ..model import WatchModel
from ..units import DEG, SECONDS_PER_DAY
from . import hybrid as H


@dataclass
class SimState:
    y: np.ndarray
    mode: int = H.FREE
    wst: int = H.W_LOCKED
    sc: float = 1.0
    sdir: float = 1.0
    fork_side: float = -1.0
    psiL: float = 0.0
    t: float = 0.0

    def copy(self) -> "SimState":
        return SimState(self.y.copy(), self.mode, self.wst, self.sc, self.sdir, self.fork_side, self.psiL, self.t)


@dataclass
class SimResult:
    p: np.ndarray
    state0: SimState
    state: SimState
    rec: np.ndarray            # [t, theta, omega, phi, psi, psidot, mode, wheel_state]
    beats: np.ndarray          # see hybrid.B_* columns
    turns: np.ndarray          # [t, theta]
    impacts: np.ndarray        # hybrid.I_* categories (J)
    status: int
    E0_mech: float
    E1_mech: float
    T_barrel: float
    T_escape: float
    f_design: float
    visc_split: dict = field(default_factory=dict)

    # ------------------------------------------------------------------------------
    @property
    def ledger(self) -> dict:
        """Energy flows over the run (J). Positive = energy delivered / dissipated."""
        y0, y1 = self.state0.y, self.state.y
        d = lambda i: float(y1[i] - y0[i])  # noqa: E731
        visc = d(H.L_VISC)
        out = {
            "escape_wheel_input": d(H.L_WIN),
            "balance_air": visc * self.visc_split.get("air", 1.0),
            "balance_hairspring_internal": visc * self.visc_split.get("hairspring", 0.0),
            "balance_pivot_oil": visc * self.visc_split.get("oil", 0.0),
            "balance_pivot_coulomb": d(H.L_COUL),
            "unlocking_friction": d(H.L_UNL),
            "impulse_plane_friction": d(H.L_IMPF),
            "impulse_pin_friction": d(H.L_PIN),
            "impact_pin_entry": float(self.impacts[H.I_ENTRY]),
            "impact_fork_banking": float(self.impacts[H.I_BANK]),
            "impact_drop_lock": float(self.impacts[H.I_DROP]),
            "impact_wheel_catch_up": float(self.impacts[H.I_CATCH]),
            "impact_knocking": float(self.impacts[H.I_KNOCK]),
            "impact_failed_unlock": float(self.impacts[H.I_UNLFAIL]),
            "delta_mechanical_energy": self.E1_mech - self.E0_mech,
        }
        losses = sum(v for k, v in out.items() if k not in ("escape_wheel_input", "delta_mechanical_energy"))
        out["total_losses"] = losses
        out["closure_error"] = out["escape_wheel_input"] - losses - out["delta_mechanical_energy"]
        return out

    @property
    def good_beats(self) -> np.ndarray:
        return self.beats[self.beats[:, H.B_OK] > 0.5]

    def amplitudes(self) -> np.ndarray:
        """|theta| at successive turning points (rad)."""
        return np.abs(self.turns[:, 1])

    def metrics(self, last_n_beats: int | None = None) -> dict:
        """Timegrapher-style metrics from the (last part of the) run."""
        b = self.good_beats
        if last_n_beats:
            b = b[-last_n_beats:]
        out = {"status": self.status, "n_beats": int(len(b))}
        tl = b[:, H.B_TLOCK]
        tl = tl[tl > 0]
        if len(tl) >= 5:
            iv = np.diff(tl)
            # period from same-direction events (every second lock)
            n2 = (len(tl) - 1) // 2 * 2
            T = (tl[n2] - tl[0]) / (n2 / 2)
            out["period"] = T
            out["frequency"] = 1.0 / T
            out["rate_s_per_day"] = (1.0 / (T * self.f_design) - 1.0) * SECONDS_PER_DAY
            a, c = iv[0::2], iv[1::2]
            m = min(len(a), len(c))
            out["beat_error_ms"] = abs(float(np.mean(a[:m]) - np.mean(c[:m]))) / 2.0 * 1e3
            out["beats_per_hour"] = 3600.0 / float(np.mean(iv))
        tt = self.turns
        if len(tt) >= 4:
            k = min(len(tt), 2 * (last_n_beats or len(tt)))
            amp = np.abs(tt[-k:, 1])
            out["amplitude_deg"] = float(np.mean(amp)) / DEG
            pos = tt[-k:][tt[-k:, 1] > 0, 1]
            neg = tt[-k:][tt[-k:, 1] < 0, 1]
            if len(pos) and len(neg):
                out["amplitude_pos_deg"] = float(np.mean(pos)) / DEG
                out["amplitude_neg_deg"] = float(-np.mean(neg)) / DEG
        if len(b) >= 4 and "period" in out:
            lift = self.p[H.P_LIFT]
            tlift = b[:, H.B_TLOCK] - b[:, H.B_TENTRY]
            tlift = tlift[(b[:, H.B_TLOCK] > 0)]
            if len(tlift):
                out["timegrapher_amplitude_deg"] = lift / (2.0 * math.sin(math.pi * float(np.mean(tlift)) / out["period"])) / DEG
            out["energy_per_beat_from_train"] = float(np.mean(b[:, H.B_WESC]))
            dt_imp = b[:, H.B_TLETOFF] - b[:, H.B_TUNL]
            out["impulse_duration_ms"] = float(np.mean(dt_imp)) * 1e3
            dt_unl = b[:, H.B_TUNL] - b[:, H.B_TENTRY]
            out["unlock_duration_ms"] = float(np.mean(dt_unl)) * 1e3
            cu = b[:, H.B_TCATCH]
            okc = cu > 0
            if okc.any():
                out["impulse_lag_ms"] = float(np.mean((cu - b[:, H.B_TUNL])[okc])) * 1e3
            dr = b[:, H.B_TLOCK] - b[:, H.B_TLETOFF]
            out["drop_duration_ms"] = float(np.mean(dr[b[:, H.B_TLOCK] > 0])) * 1e3
        return out


class Simulator:
    """Builds the numba parameter vector from a WatchModel and runs the hybrid solver."""

    def __init__(self, model: WatchModel):
        self.m = model
        self.P = model.P
        bal = model.balance
        c = bal.c
        self.visc_split = {"air": bal.c_air / c, "hairspring": bal.c_hairspring / c, "oil": bal.c_oil / c}

    # ------------------------------------------------------------------------------
    def params_vector(self, T_barrel: float, position: str = "DU", *, T_escape: float | None = None,
                      escapement: bool = True) -> np.ndarray:
        m, P = self.m, self.P
        bal, esc = m.balance, m.esc
        pos = P.positions[position]
        horizontal = bool(pos["horizontal"])
        g = P["simulation.gravity"]
        p = np.zeros(H.N_PARAMS)
        p[H.P_IB] = bal.inertia
        p[H.P_K] = bal.k
        p[H.P_CUB] = bal.cubic
        p[H.P_C] = bal.c
        p[H.P_TC] = bal.coulomb_torque(horizontal, g)
        p[H.P_U] = bal.unpoise
        gamma = math.radians(float(pos["gravity_angle_deg"]))
        p[H.P_UPH] = gamma - bal.unpoise_angle
        p[H.P_UON] = 0.0 if horizontal else 1.0
        p[H.P_TBE] = bal.beat_offset
        p[H.P_LIFT] = esc.lift if escapement else 1e-9
        p[H.P_RR] = esc.r_roller
        p[H.P_DBP] = esc.d_bp
        p[H.P_PHIB] = esc.phi_B
        p[H.P_UUE] = esc.u_unlock_end
        p[H.P_UIE] = esc.u_impulse_end
        p[H.P_KU] = esc.kappa_U
        p[H.P_KI] = esc.kappa_I
        p[H.P_PSID] = esc.psi_D
        p[H.P_PSIREC] = esc.psi_rec
        p[H.P_PSIH] = esc.psi_half
        p[H.P_FUF] = esc.f_U_fwd
        p[H.P_FUB] = esc.f_U_back
        p[H.P_ETAI] = esc.eta_I
        p[H.P_ETAP] = esc.eta_pin
        p[H.P_IF] = esc.I_fork
        p[H.P_IE] = m.I_escape_eff
        Te = m.losses.escape_torque(T_barrel) if T_escape is None else T_escape
        p[H.P_TE] = max(Te, 0.0)
        p[H.P_TEB] = m.losses.escape_torque_backdrive(T_barrel) if T_escape is None else T_escape * 1.6
        p[H.P_KNOCK] = esc.knock_angle if escapement else 1e9
        p[H.P_KNE] = esc.knock_e
        p[H.P_DTF] = P["simulation.dt_free"]
        p[H.P_DTC] = P["simulation.dt_contact"]
        p[H.P_TOL] = P["simulation.event_tolerance"]
        if not escapement:   # free oscillator: make the fork unreachable
            p[H.P_LIFT] = -10.0
        return p

    def initial_state(self, amplitude: float, p: np.ndarray) -> SimState:
        """Balance at its negative extreme, about to swing positive; fork on the -banking."""
        y = np.zeros(H.NSTATE)
        y[H.S_TH] = -abs(amplitude)
        return SimState(y=y, mode=H.FREE, wst=H.W_LOCKED, sc=1.0, sdir=1.0, fork_side=-1.0, psiL=0.0, t=0.0)

    # ------------------------------------------------------------------------------
    def run(self, T_barrel: float, position: str = "DU", duration: float = 10.0, *,
            amplitude0: float = 250 * DEG, state: SimState | None = None, record_dt: float = 0.0,
            max_beats: int | None = None, p: np.ndarray | None = None, escapement: bool = True) -> SimResult:
        if p is None:
            p = self.params_vector(T_barrel, position, escapement=escapement)
        st0 = state.copy() if state is not None else self.initial_state(amplitude0, p)
        f = self.m.balance.f_design
        if max_beats is None:
            max_beats = int(duration * 2 * f * 1.5) + 20
        max_rec = int(duration / record_dt) + 10 if record_dt > 0 else 1
        out = H.simulate(p, st0.y, st0.mode, st0.wst, st0.sc, st0.sdir, st0.fork_side, st0.psiL,
                         st0.t, st0.t + duration, max_beats, record_dt, max_rec)
        (y, mode, wst, sc, sdir, fork_side, psiL, t, rec, nrec, beats, nbeats, turns, nturn,
         imp, status, E0) = out
        st1 = SimState(y.copy(), int(mode), int(wst), float(sc), float(sdir), float(fork_side), float(psiL), float(t))
        E1 = H.mech_energy(y, mode, wst, sc, p)
        return SimResult(p=p, state0=st0, state=st1, rec=rec[:nrec].copy(), beats=beats[:nbeats].copy(),
                         turns=turns[:nturn].copy(), impacts=imp.copy(), status=int(status), E0_mech=float(E0),
                         E1_mech=float(E1), T_barrel=T_barrel, T_escape=float(p[H.P_TE]), f_design=f,
                         visc_split=self.visc_split)

    def steady_state(self, T_barrel: float, position: str = "DU", *, amplitude0: float | None = None,
                     settle: float | None = None, measure: float | None = None,
                     p: np.ndarray | None = None) -> dict:
        """Settle to the limit cycle, then measure over `measure` seconds."""
        P = self.P
        f = self.m.balance.f_design
        settle = settle if settle is not None else P["simulation.settle_periods"] / f
        measure = measure if measure is not None else P["simulation.measure_periods"] / f
        if p is None:
            p = self.params_vector(T_barrel, position)
        a0 = amplitude0 if amplitude0 is not None else self.amplitude_guess(T_barrel, position)
        r1 = self.run(T_barrel, position, settle, amplitude0=a0, p=p)
        if r1.status != 0:
            return {"status": r1.status, "running": False, "T_barrel": T_barrel, "position": position}
        r2 = self.run(T_barrel, position, measure, state=r1.state, p=p)
        met = r2.metrics()
        led = r2.ledger
        met.update({"T_barrel": T_barrel, "T_escape": r2.T_escape, "position": position,
                    "running": r2.status == 0 and met.get("n_beats", 0) > 0.8 * measure * 2 * f
                    and met.get("amplitude_deg", 0) > math.degrees(self.m.esc.lift) / 2,
                    "ledger": led, "duration": measure, "final_state": r2.state})
        met["power_escape_wheel"] = led["escape_wheel_input"] / measure
        return met

    # ------------------------------------------------------------------------------
    def period_map(self, A: float, p: np.ndarray) -> float:
        """Poincare map of the limit cycle: start at theta=-A (at rest), integrate one full
        oscillation (two beats) and return |theta| at the next same-side turning point."""
        st = self.initial_state(A, p)
        f = self.m.balance.f_design
        out = H.simulate(p, st.y, st.mode, st.wst, st.sc, st.sdir, st.fork_side, st.psiL, 0.0,
                         1.6 / f, 3, 0.0, 1)
        turns, nturn, status = out[12], out[13], out[15]
        if status != 0 or nturn < 2:
            return 0.0
        return float(abs(turns[1, 1]))

    def limit_cycle(self, T_barrel: float, position: str = "DU", *, p: np.ndarray | None = None,
                    tol: float = 1e-6) -> float | None:
        """Steady amplitude A* solving F(A) = map(A) - A = 0 (Brent). None if the watch stops."""
        from scipy.optimize import brentq
        if p is None:
            p = self.params_vector(T_barrel, position)
        F = lambda A: self.period_map(A, p) - A  # noqa: E731
        lo = self.m.esc.lift / 2.0 + 8 * DEG
        hi = 345 * DEG
        f_lo = F(lo)
        if f_lo <= 0.0:
            return None
        f_hi = F(hi)
        if f_hi >= 0.0:
            return hi
        return float(brentq(F, lo, hi, xtol=tol, rtol=1e-10, maxiter=60))

    def steady_state_fast(self, T_barrel: float, position: str = "DU", *, measure_periods: int | None = None,
                          p: np.ndarray | None = None, record_dt: float = 0.0) -> dict:
        """Limit cycle by shooting, then a short measurement run on the cycle."""
        if p is None:
            p = self.params_vector(T_barrel, position)
        A = self.limit_cycle(T_barrel, position, p=p)
        f = self.m.balance.f_design
        if A is None:
            return {"running": False, "status": 2, "T_barrel": T_barrel, "position": position,
                    "T_escape": float(p[H.P_TE]), "amplitude_deg": 0.0}
        n = measure_periods or int(self.P["simulation.measure_periods"])
        r = self.run(T_barrel, position, n / f + 1e-4, amplitude0=A, p=p, record_dt=record_dt)
        met = r.metrics()
        led = r.ledger
        met.update({"running": r.status == 0, "T_barrel": T_barrel, "T_escape": r.T_escape,
                    "position": position, "amplitude_limit_cycle_deg": A / DEG, "ledger": led,
                    "duration": r.state.t - r.state0.t, "result": r})
        met["power_escape_wheel"] = led["escape_wheel_input"] / met["duration"]
        return met

    def amplitude_guess(self, T_barrel: float, position: str) -> float:
        """Energy-balance estimate used as initial condition (speeds up settling)."""
        bal, esc = self.m.balance, self.m.esc
        Te = self.m.losses.escape_torque(T_barrel)
        E_in = 2.0 * Te * esc.psi_I * esc.eta_I * esc.eta_pin * 0.85      # per period
        # losses per period: viscous pi c w A^2 + coulomb 4 Tc A
        w = bal.omega0
        Tc = bal.coulomb_torque(bool(self.P.positions[position]["horizontal"]))
        a, b, c = math.pi * bal.c * w, 4.0 * Tc, -E_in
        A = (-b + math.sqrt(b * b - 4 * a * c)) / (2 * a)
        return float(min(max(A, 120 * DEG), 330 * DEG))
