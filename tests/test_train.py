"""Milestones 2-3 - gear ratios, hand rates, directions, centre distances (kinematics)."""
import math

import pytest


def test_hand_periods_follow_from_tooth_counts(model):
    tr = model.train
    assert tr.period("seconds") == pytest.approx(60.0, rel=1e-12)
    assert tr.period("centre") == pytest.approx(3600.0, rel=1e-12)
    assert tr.period("hour") == pytest.approx(12 * 3600.0, rel=1e-12)


def test_escape_wheel_speed_matches_beat_rate(model):
    tr = model.train
    # 21,600 vph -> 3 Hz -> one tooth per period -> 3/15 rev/s = 12 rpm
    assert abs(tr.rpm("escape")) == pytest.approx(12.0)
    beats_per_hour = 2 * tr.f_balance * 3600
    assert beats_per_hour == pytest.approx(21600)


def test_rotation_directions(model):
    tr = model.train
    for hand_arbor in ("seconds", "centre", "hour"):
        assert tr.omega[hand_arbor] < 0, f"{hand_arbor} hand must turn clockwise"
    # every external mesh reverses direction
    for m in tr.meshes:
        assert math.copysign(1, tr.omega[m.driver]) != math.copysign(1, tr.omega[m.driven]), m.name


def test_mesh_speed_ratios(model):
    tr = model.train
    for m in tr.meshes:
        assert tr.omega[m.driven] / tr.omega[m.driver] == pytest.approx(-m.z_driver / m.z_driven)


def test_total_train_ratio(model):
    tr = model.train
    assert tr.escape_to_barrel == pytest.approx(75 / 10 * 80 / 10 * 75 / 10 * 84 / 7)


def test_coaxial_seconds_pinion_centre_distances_agree(model):
    tr = model.train
    a1 = tr.mesh("centre->third").centre_distance
    a2 = tr.mesh("third->seconds").centre_distance
    assert a1 == pytest.approx(a2, rel=1e-12)


def test_meshing_pairs_share_module(model):
    tr = model.train
    assert tr.mesh("third->fourth").module == tr.mesh("third->seconds").module


def test_keyless_full_wind_within_spec(model, P):
    crown_turns = model.spring.n_dev / model.keyless.arbor_turns_per_crown_turn
    assert crown_turns <= P["movement.full_wind_crown_turns"]  # Miyota: 40 turns give full wind


def test_kinematic_angle_map_is_exact(model):
    tr = model.train
    psi = 2 * math.pi * 12 * 60  # one hour of escape-wheel rotation (12 rpm)
    ang = tr.angles_from_escape(psi * math.copysign(1, tr.omega["escape"]))
    assert abs(ang["centre"]) == pytest.approx(2 * math.pi)
    assert abs(ang["seconds"]) == pytest.approx(2 * math.pi * 60)
    assert abs(ang["hour"]) == pytest.approx(2 * math.pi / 12)


def test_motion_works_coaxial_constraint(P, model):
    """Minute wheel meshes two COAXIAL gears (cannon pinion, hour wheel) with one module:
    tooth sums must be equal, and the overall ratio must be 12."""
    z = lambda k: P.int(f"motion_works.{k}")  # noqa: E731
    assert z("cannon_pinion") + z("minute_wheel") == z("minute_pinion") + z("hour_wheel")
    tr = model.train
    assert tr.mesh("cannon->minute_wheel").centre_distance == pytest.approx(
        tr.mesh("minute_pinion->hour").centre_distance, rel=1e-12)
    assert abs(tr.omega["centre"] / tr.omega["hour"]) == pytest.approx(12.0)
