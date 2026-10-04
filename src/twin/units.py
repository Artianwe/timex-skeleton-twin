"""Unit handling: every configuration value is converted to SI on load.

Each unit maps to (scale_to_SI, dimension) where dimension is a tuple of exponents over the
base quantities (M, L, T). Angles are dimensionless (radians); 'deg' only carries a scale.
The dimension tuple is used by the test-suite to verify dimensional consistency.
"""
from __future__ import annotations

import math

# (factor to SI, (M, L, T))
_DIM_NONE = (0, 0, 0)
UNITS: dict[str, tuple[float, tuple[int, int, int]]] = {
    "1": (1.0, _DIM_NONE),
    "bool": (1.0, _DIM_NONE),
    "rad": (1.0, _DIM_NONE),
    "deg": (math.pi / 180.0, _DIM_NONE),
    "turn": (2.0 * math.pi, _DIM_NONE),
    "1/rad^2": (1.0, _DIM_NONE),
    # length
    "m": (1.0, (0, 1, 0)),
    "mm": (1e-3, (0, 1, 0)),
    "um": (1e-6, (0, 1, 0)),
    # mass
    "kg": (1.0, (1, 0, 0)),
    "g": (1e-3, (1, 0, 0)),
    "mg": (1e-6, (1, 0, 0)),
    # time / frequency
    "s": (1.0, (0, 0, 1)),
    "ms": (1e-3, (0, 0, 1)),
    "h": (3600.0, (0, 0, 1)),
    "Hz": (1.0, (0, 0, -1)),
    "vph": (1.0 / 3600.0, (0, 0, -1)),       # vibrations (beats) per hour -> beats per second
    "rpm": (2.0 * math.pi / 60.0, (0, 0, -1)),
    "s/day": (1.0 / 86400.0, _DIM_NONE),     # rate expressed as fractional frequency error
    # mechanics
    "N": (1.0, (1, 1, -2)),
    "N*m": (1.0, (1, 2, -2)),
    "N*mm": (1e-3, (1, 2, -2)),
    "uN*m": (1e-6, (1, 2, -2)),
    "J": (1.0, (1, 2, -2)),
    "W": (1.0, (1, 2, -3)),
    "uW": (1e-6, (1, 2, -3)),
    "kg*m^2": (1.0, (1, 2, 0)),
    "mg*cm^2": (1e-10, (1, 2, 0)),
    "Pa": (1.0, (1, -1, -2)),
    "MPa": (1e6, (1, -1, -2)),
    "GPa": (1e9, (1, -1, -2)),
    "kg/m^3": (1.0, (1, -3, 0)),
    "m/s^2": (1.0, (0, 1, -2)),
    "N*m/rad": (1.0, (1, 2, -2)),
    "uN*m/rad": (1e-6, (1, 2, -2)),
    "N*mm/rad": (1e-3, (1, 2, -2)),
    "uJ": (1e-6, (1, 2, -2)),
    "N*m*s": (1.0, (1, 2, -1)),
}


def to_si(value: float, unit: str) -> float:
    """Convert a value expressed in `unit` to SI."""
    try:
        factor, _ = UNITS[unit]
    except KeyError as exc:  # pragma: no cover - configuration error
        raise KeyError(f"Unknown unit '{unit}'. Add it to twin.units.UNITS.") from exc
    return float(value) * factor


def from_si(value: float, unit: str) -> float:
    """Convert an SI value into `unit`."""
    return float(value) / UNITS[unit][0]


def dimension(unit: str) -> tuple[int, int, int]:
    return UNITS[unit][1]


# convenient constants
DEG = math.pi / 180.0
MG_CM2 = 1e-10          # kg m^2
SECONDS_PER_DAY = 86400.0
