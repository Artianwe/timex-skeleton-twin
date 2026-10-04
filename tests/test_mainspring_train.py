"""Milestone 4 - mainspring, energy storage and torque transmission."""
import math

import numpy as np
import pytest


def test_full_wind_stress_equals_working_stress(model, P):
    sp = model.spring
    assert sp.stress(sp.M_full) == pytest.approx(P["mainspring.working_stress"])


def test_stored_energy_equals_integral_of_torque(model):
    sp = model.spring
    n = np.linspace(0, sp.n_dev, 20001)
    E_num = np.trapezoid(sp.torque_elastic(n), n) * 2 * math.pi
    assert sp.energy_elastic(sp.n_dev) == pytest.approx(E_num, rel=1e-6)


def test_torque_monotonic_and_hysteresis(model):
    sp = model.spring
    n = np.linspace(0.01, sp.n_dev, 200)
    assert np.all(np.diff(sp.torque_out(n)) > 0)
    assert np.all(sp.torque_wind(n) > sp.torque_out(n))
    assert sp.energy_deliverable(sp.n_dev) < sp.energy_elastic(sp.n_dev)


def test_development_geometry(model):
    sp = model.spring
    assert 0 < sp.n_dev < sp.n_theoretical
    # spring fits: area L e equals fill * annulus
    assert sp.length * sp.e == pytest.approx(sp.fill * math.pi * (sp.R**2 - sp.r**2))


def test_escape_torque_transmission(model, T_full):
    L = model.losses
    Te = L.escape_torque(T_full)
    ideal = T_full / L.i_total
    assert 0.6 * ideal < Te < ideal                 # losses reduce torque
    # affine relation T_e = a T_b - b with a = eta_total / i
    a, b = L.affine()
    assert a == pytest.approx(L.eta_total / L.i_total, rel=1e-9)
    assert Te == pytest.approx(a * T_full - b)


def test_backdrive_needs_more_torque_than_forward(model, T_full):
    L = model.losses
    assert L.escape_torque_backdrive(T_full) > T_full / L.i_total > L.escape_torque(T_full)


def test_stage_power_balance(model, T_full):
    rows = model.losses.forward(T_full)
    for r in rows:
        assert r["P_in"] - r["P_out"] == pytest.approx(r["P_loss"])
        assert r["P_loss"] >= 0
    total_loss = sum(r["P_loss"] for r in rows)
    assert rows[0]["P_in"] - rows[-1]["P_out"] == pytest.approx(total_loss)
