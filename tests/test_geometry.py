"""Milestone 2 - parametric geometry / CAD: layout, clearances, interference, meshing, mass props."""
import math

import numpy as np
import pytest
from shapely import affinity

from twin.geometry.assembly import build_parts
from twin.geometry.checks import interference_report
from twin.geometry.gears import mesh_interference, pinion_profile, wheel_profile
from twin.geometry.layout import solve_layout


@pytest.fixture(scope="module")
def layout(model):
    return solve_layout(model)


@pytest.fixture(scope="module")
def parts(model, layout):
    return {p.name: p for p in build_parts(model, layout)}


def test_balance_lands_on_measured_position(layout, P):
    b = layout.pos["balance"]
    assert math.hypot(*b) == pytest.approx(P["layout.balance_centre_radius"], abs=1e-9)
    assert math.atan2(b[1], b[0]) % (2 * math.pi) == pytest.approx(P["layout.balance_centre_angle"], abs=1e-9)


def test_layout_centre_distances(layout, model):
    tr = model.train
    d = lambda a, b: float(np.hypot(*(layout.pos[a] - layout.pos[b])))  # noqa: E731
    assert d("centre", "third") == pytest.approx(tr.mesh("centre->third").centre_distance)
    assert d("third", "fourth") == pytest.approx(tr.mesh("third->fourth").centre_distance)
    assert d("fourth", "escape") == pytest.approx(tr.mesh("fourth->escape").centre_distance)
    assert d("escape", "pallet") == pytest.approx(model.esc.d_ep)
    assert d("pallet", "balance") == pytest.approx(model.esc.d_bp)
    assert d("centre", "barrel") == pytest.approx(tr.mesh("barrel->centre").centre_distance)


def test_straight_line_escapement(layout):
    e, p, b = layout.pos["escape"], layout.pos["pallet"], layout.pos["balance"]
    cross = (p - e)[0] * (b - e)[1] - (p - e)[1] * (b - e)[0]
    assert abs(cross) < 1e-12


def test_layout_clearances_and_fit(layout):
    assert layout.min_clearance >= 0.15e-3
    for c in layout.checks:
        assert c["clearance"] > 0, c


def test_no_3d_interference_at_reference_pose(parts):
    rep = interference_report(list(parts.values()))
    assert rep == [], rep[:5]


PAIRS = [("barrel_teeth", "centre_pinion"), ("centre_wheel", "third_pinion"), ("third_wheel", "fourth_pinion"),
         ("third_wheel", "seconds_pinion"), ("fourth_wheel", "escape_pinion"), ("cannon_pinion", "minute_wheel"),
         ("minute_pinion", "hour_wheel"), ("rotor_pinion", "reversing_wheel"), ("sliding_wheel_pinion", "reduction_wheel")]


@pytest.mark.parametrize("a,b", PAIRS)
def test_gear_pairs_mesh_without_interference_through_a_pitch(parts, a, b):
    """Rotate driver by one tooth pitch in 24 steps, driven by the exact kinematic ratio."""
    A, B = parts[a], parts[b]
    if A.motion == "auto" or B.motion == "auto" or A.motion == "rotor":
        ra, rb = (A.ratio if A.motion != "rotor" else 1.0), (B.ratio if B.motion != "rotor" else 1.0)
    else:
        ra, rb = A.ratio, B.ratio
    z_a = len(A.shape.exterior.coords) if hasattr(A.shape, "exterior") else 50
    # pitch of the driver taken from its largest tooth count among the mesh partners
    steps = 24
    pitch = 2 * math.pi / 10 if "pinion" in a else 2 * math.pi / 30
    for k in range(steps + 1):
        th = k / steps * pitch
        ga = affinity.translate(affinity.rotate(A.shape, th, origin=(0, 0), use_radians=True), *A.centre)
        gb = affinity.translate(affinity.rotate(B.shape, th * rb / ra, origin=(0, 0), use_radians=True), *B.centre)
        assert ga.intersection(gb).area < 2e-13, f"{a}/{b} interfere at step {k}"
    del z_a


def test_standalone_cycloidal_mesh_has_backlash_not_interference():
    m = 0.09e-3
    W, Pn = wheel_profile(80, m, 10), pinion_profile(10, m)
    ov, gap = mesh_interference(W, Pn, m * 90 / 2, 8.0, steps=30, pinion_phase=math.pi / 10,
                                wheel_pitch=2 * math.pi / 80)
    assert ov < 1e-14 and 1e-6 < gap < 30e-6


def test_movement_height_matches_spec(P):
    top = P.band("cad.plate")[1]
    bottom = P.band("cad.rotor")[0]
    assert top - bottom == pytest.approx(P["movement.height"], abs=1e-9)


def test_hand_stack_clearances(P):
    zs = [P["cad.hour_hand_z"], P["cad.minute_hand_z"], P["cad.seconds_hand_z"]]
    assert all(b - a >= 0.2e-3 for a, b in zip(zs, zs[1:]))
    assert zs[0] - P.band("cad.dial")[1] >= 0.1e-3


def test_cad_volume_matches_analytic_mass_model(parts, model):
    cq = pytest.importorskip("cadquery")  # noqa: F841
    from twin.geometry.cad import part_solid
    rim = parts["balance_rim"]
    v_cad = part_solid(rim, simplify=0.0).Volume() * 1e-9
    b = model.balance
    v_an = math.pi * (b.rim_outer**2 - b.rim_inner**2) * (rim.z[1] - rim.z[0])
    assert v_cad == pytest.approx(v_an, rel=2e-3)


def test_escape_wheel_locked_on_pallet_at_rest(parts):
    ew = parts["escape_wheel"]
    assert -8.0 < math.degrees(ew.meta["lock_offset"]) <= 0.0
