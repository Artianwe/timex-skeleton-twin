"""Milestones 5-7 - balance oscillator, escapement event model, energy conservation."""
import math

import numpy as np
import pytest

from twin.dynamics import hybrid as H
from twin.units import DEG


def _free_params(sim, T, **zero):
    p = sim.params_vector(T, "DU", escapement=False)
    for k in zero:
        p[getattr(H, k)] = 0.0
    return p


def test_free_undamped_period_equals_2pi_sqrt_I_over_k(sim, T_full):
    p = _free_params(sim, T_full, P_C=0, P_TC=0, P_CUB=0)
    r = sim.run(T_full, "DU", 3.0, amplitude0=200 * DEG, p=p, max_beats=10)
    t = r.turns[:, 0]
    T_sim = (t[-1] - t[1]) / ((len(t) - 2) / 2)  # same-side turning points every period
    T_th = 2 * math.pi * math.sqrt(p[H.P_IB] / p[H.P_K])
    assert T_sim == pytest.approx(T_th, rel=1e-8)
    assert T_th == pytest.approx(1 / 3, rel=1e-12)       # hairspring sized for 3 Hz


def test_viscous_decay_matches_quality_factor(sim, T_full, model):
    p = _free_params(sim, T_full, P_TC=0, P_CUB=0)
    r = sim.run(T_full, "DU", 3.0, amplitude0=250 * DEG, p=p, max_beats=10)
    A = r.amplitudes()
    Q = model.balance.q_viscous
    # amplitude ratio per half period = exp(-pi / (2 Q))  (light damping)
    ratio = A[1:] / A[:-1]
    assert np.mean(ratio) == pytest.approx(math.exp(-math.pi / (2 * Q)), rel=2e-5)


def test_coulomb_decay_is_linear(sim, T_full):
    p = _free_params(sim, T_full, P_C=0, P_CUB=0)
    p[H.P_TC] = 5e-9
    r = sim.run(T_full, "DU", 3.0, amplitude0=250 * DEG, p=p, max_beats=10)
    A = r.amplitudes()
    dA = -np.diff(A)
    assert np.allclose(dA, 2 * p[H.P_TC] / p[H.P_K], rtol=1e-6)   # 2 T_c / k per half swing


def test_energy_ledger_closes(sim, T_full):
    for pos in ("DU", "CL"):
        r = sim.run(T_full, pos, 10.0, amplitude0=250 * DEG)
        L = r.ledger
        assert r.status == 0
        assert abs(L["closure_error"]) < 1e-10 * L["escape_wheel_input"] + 1e-15


def test_wheel_advances_half_pitch_per_beat(sim, T_full, model):
    r = sim.run(T_full, "DU", 4.0, amplitude0=260 * DEG)
    # psi grows by exactly pi/Z per beat once locked
    n = len(r.good_beats)
    total = r.state.y[H.S_PSI] - r.state0.y[H.S_PSI]
    assert total == pytest.approx(n * math.pi / model.esc.z, rel=0, abs=1e-9)


def test_lift_angle_geometry(model):
    e = model.esc
    assert e.fork_angle(e.lift / 2) == pytest.approx(e.phi_B, rel=1e-12)
    assert e.fork_angle(-e.lift / 2) == pytest.approx(-e.phi_B, rel=1e-12)


def test_draw_exceeds_friction_angle(model):
    """Safety: draw must pull the fork to the banking, i.e. delta > atan(mu)."""
    e = model.esc
    assert e.delta > math.atan(e.mu)
    assert e.f_U_back > 0


def test_symmetric_escapement_has_no_beat_error(sim, T_full):
    p = sim.params_vector(T_full, "DU")
    p[H.P_TBE] = 0.0
    s = sim.steady_state_fast(T_full, "DU", p=p, measure_periods=20)
    assert s["beat_error_ms"] < 2e-3


def test_beat_offset_creates_beat_error(sim, T_full):
    s = sim.steady_state_fast(T_full, "DU", measure_periods=20)
    assert 0.2 < s["beat_error_ms"] < 2.0


def test_ideal_oscillator_keeps_perfect_time(sim, T_full):
    """No escapement, no damping: rate error must vanish (validates the rate measurement).
    Numerical floor of RK4 at dt=1e-4 s is ~3e-4 s/day (3e-9 relative), far below the 0.1 s/day
    resolution of a timegrapher."""
    p = _free_params(sim, T_full, P_C=0, P_TC=0, P_CUB=0)
    r = sim.run(T_full, "DU", 10.0, amplitude0=250 * DEG, p=p, max_beats=10)
    t = r.turns[r.turns[:, 1] > 0, 0]
    T = np.mean(np.diff(t))
    rate = (1 / (T * 3.0) - 1) * 86400
    assert abs(rate) < 1e-3


def test_limit_cycle_is_a_fixed_point(sim, T_full):
    p = sim.params_vector(T_full, "DU")
    A = sim.limit_cycle(T_full, "DU", p=p)
    assert sim.period_map(A, p) == pytest.approx(A, abs=1e-6)
    r = sim.run(T_full, "DU", 30.0, amplitude0=A, p=p)
    assert np.degrees(r.amplitudes()[-1]) == pytest.approx(np.degrees(A), abs=0.05)


def test_amplitude_increases_with_torque(sim, T_full):
    A = [sim.limit_cycle(T_full * f, "DU") for f in (0.5, 0.75, 1.0)]
    assert A[0] < A[1] < A[2]


def test_vertical_positions_have_lower_amplitude(sim, T_full):
    du = sim.limit_cycle(T_full, "DU")
    for pos in ("CU", "CD", "CL", "CR"):
        assert sim.limit_cycle(T_full, pos) < du


def test_unlocking_failure_when_torque_vanishes(sim):
    assert sim.limit_cycle(1e-6, "DU") is None


def test_inertia_and_stiffness_scales_shift_rate_as_theory(P):
    """Free-oscillator rate shift: Delta f/f = 0.5 (Delta k/k - Delta I/I)  => 1% -> 432 s/day."""
    from twin.model import build_model
    b0 = build_model(P).balance
    b1 = build_model(P.with_overrides({"balance.inertia_scale": 1.001})).balance
    b2 = build_model(P.with_overrides({"hairspring.stiffness_scale": 1.001})).balance
    assert b1.rate_from_frequency(b1.f_free) == pytest.approx(-43.2, rel=2e-3)
    assert b2.rate_from_frequency(b2.f_free) == pytest.approx(+43.2, rel=2e-3)
    assert b0.f_free == pytest.approx(3.0)


@pytest.mark.parametrize("frac,pos", [(1.0, "CD"), (0.5, "CL"), (0.15, "CU")])
def test_unpoise_rate_matches_averaging_theory(sim, model, frac, pos):
    """Krylov-Bogoliubov averaging of an out-of-poise balance predicts
        Delta rate = 86400 * (m g e cos(gamma - beta) / k) * J1(A) / A
    (sign reversal at A = 219.5 deg, the first zero of J1). The hybrid model must reproduce it."""
    from scipy.special import j1
    T = float(model.spring.torque_out(frac * model.spring.n_dev))
    p = sim.params_vector(T, pos)
    r1 = sim.steady_state_fast(T, pos, p=p, measure_periods=20)
    p0 = p.copy()
    p0[H.P_UON] = 0.0
    r0 = sim.steady_state_fast(T, pos, p=p0, measure_periods=20)
    A = math.radians(r1["amplitude_deg"])
    theory = 86400 * (model.balance.unpoise * math.cos(p[H.P_UPH]) / model.balance.k) * j1(A) / A
    assert r1["rate_s_per_day"] - r0["rate_s_per_day"] == pytest.approx(theory, rel=0.01, abs=0.02)
