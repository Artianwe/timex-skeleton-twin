"""Analytic mass properties of simple watch-part primitives (SI units).

All inertias are polar moments about the part's rotation axis (z). These closed-form results
are cross-checked against CadQuery/OpenCascade solids in tests/test_geometry.py.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class MassProps:
    mass: float      # kg
    inertia: float   # kg m^2 about own axis

    def __add__(self, other: "MassProps") -> "MassProps":
        return MassProps(self.mass + other.mass, self.inertia + other.inertia)


ZERO = MassProps(0.0, 0.0)


def annulus(r_in: float, r_out: float, thickness: float, density: float, fill: float = 1.0) -> MassProps:
    """Flat ring (or disc if r_in=0). I = m (r_o^2 + r_i^2) / 2."""
    m = density * math.pi * (r_out**2 - r_in**2) * thickness * fill
    return MassProps(m, 0.5 * m * (r_out**2 + r_in**2))


def cylinder(r: float, length: float, density: float) -> MassProps:
    return annulus(0.0, r, length, density)


def radial_bar(r_from: float, r_to: float, width: float, thickness: float, density: float) -> MassProps:
    """Straight arm along a radius from r_from to r_to. I = m (r1^2 + r1 r2 + r2^2)/3 + m w^2/12."""
    length = r_to - r_from
    m = density * length * width * thickness
    i = m * (r_from**2 + r_from * r_to + r_to**2) / 3.0 + m * width**2 / 12.0
    return MassProps(m, i)


def point_mass(m: float, r: float) -> MassProps:
    return MassProps(m, m * r * r)


def spoked_wheel(r_tip: float, r_root: float, thickness: float, density: float, *,
                 rim_width: float, n_arms: int, arm_width: float, hub_radius: float,
                 tooth_fill: float = 0.5) -> MassProps:
    """Train/escape wheel: tooth band + rim + arms (crossings) + hub."""
    teeth = annulus(r_root, r_tip, thickness, density, fill=tooth_fill)
    rim = annulus(max(r_root - rim_width, hub_radius), r_root, thickness, density)
    arms = ZERO
    r_arm_out = max(r_root - rim_width, hub_radius)
    for _ in range(n_arms):
        arms = arms + radial_bar(hub_radius, r_arm_out, arm_width, thickness, density)
    hub = annulus(0.0, hub_radius, thickness, density)
    return teeth + rim + arms + hub


def spiral_strip(length: float, height: float, thickness: float, density: float,
                 r_in: float, r_out: float, attached_fraction: float = 1.0 / 3.0) -> MassProps:
    """Hairspring: mass and effective inertia (classical 1/3 rule for a spring attached at the
    collet and fixed at the stud): I_eff ~ m_eff * r_mean^2."""
    m = density * length * height * thickness
    r_mean2 = (r_in**2 + r_out**2) / 2.0
    return MassProps(m, attached_fraction * m * r_mean2)
