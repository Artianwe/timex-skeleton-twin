"""Milestone 1 - parameter database integrity, provenance and units."""
import math

import pytest

from twin.params import PROVENANCE, load_params
from twin.units import UNITS, dimension, from_si, to_si


def test_every_parameter_has_valid_provenance_and_unit(P):
    for key, prm in P.items():
        assert prm.prov in PROVENANCE, key
        assert prm.unit in UNITS, key


def test_estimated_parameters_carry_a_source_or_rationale(P):
    for key, prm in P.items():
        if prm.prov in ("assumed", "inferred"):
            assert prm.src.strip() != "", key


def test_values_lie_inside_declared_ranges(P):
    for prm in P.ranged():
        lo, hi = prm.range_raw
        assert lo <= prm.raw <= hi, f"{prm.key}={prm.raw} outside [{lo},{hi}]"


def test_verified_core_specs(P):
    """The verified anchors of the twin (owner photos + Miyota 82S0 documents)."""
    assert P.info("watch.case_diameter").prov == "measured"
    assert P["watch.case_diameter"] == pytest.approx(0.044)
    assert P.info("movement.frequency").prov == "measured"
    assert P["movement.frequency"] == pytest.approx(21600 / 3600)          # beats per second
    assert P["movement.lift_angle"] == pytest.approx(math.radians(49))
    assert P.info("movement.lift_angle").prov == "sourced"
    assert P.int("watch.jewels") == 21


def test_si_conversion_roundtrip():
    for u in UNITS:
        assert from_si(to_si(1.2345, u), u) == pytest.approx(1.2345)


@pytest.mark.parametrize("unit,dim", [("N*mm", (1, 2, -2)), ("mg*cm^2", (1, 2, 0)), ("GPa", (1, -1, -2)),
                                      ("mm", (0, 1, 0)), ("vph", (0, 0, -1)), ("deg", (0, 0, 0))])
def test_unit_dimensions(unit, dim):
    assert dimension(unit) == dim


def test_dimensional_homogeneity_of_core_equations():
    """k = I w^2 ; M = sigma h e^2/6 ; S = E h e^3/(12 L) ; P = T w  -- check dimension vectors."""
    add = lambda a, b: tuple(x + y for x, y in zip(a, b))  # noqa: E731
    I, w = dimension("kg*m^2"), dimension("Hz")
    assert add(I, add(w, w)) == dimension("N*m/rad")
    L = dimension("m")
    assert add(dimension("Pa"), add(L, add(L, L))) == dimension("N*m")
    assert add(dimension("Pa"), add(L, add(L, add(L, L)))) == add(dimension("N*m"), L)  # E I = S L
    assert add(dimension("N*m"), dimension("Hz")) == dimension("W")


def test_overrides_do_not_mutate_original(P):
    P2 = P.with_overrides({"balance.rim_thickness": 0.5})
    assert P2["balance.rim_thickness"] == pytest.approx(0.5e-3)
    assert P["balance.rim_thickness"] == pytest.approx(0.40e-3)


def test_provenance_inventory_counts(P):
    bp = P.by_provenance()
    assert len(bp["measured"]) >= 6
    assert len(bp["sourced"]) >= 10
    assert sum(len(v) for v in bp.values()) == len(list(P.keys()))
