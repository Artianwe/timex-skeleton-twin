"""Gear-train kinematics derived purely from tooth counts.

Nothing here is hard-coded: wheel speeds follow from the balance frequency (the time base) and the
tooth counts via
        omega_driven = - omega_driver * z_driver / z_driven      (external mesh)
The minus sign encodes the reversal of rotation at every external mesh.

Sign convention: angular velocity about +z (z points toward the dial/viewer), i.e. POSITIVE =
counter-clockwise seen from the dial. Hands must turn clockwise => negative.

The escape wheel advances exactly one tooth per balance period (two beats), so
        omega_escape = 2*pi * f_balance / Z_escape .
Everything else (centre wheel 1 rev/h, seconds 1 rev/min, hour 1 rev/12h) is then a
*consequence* that the validation suite checks.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from ..params import ParamSet


@dataclass(frozen=True)
class Mesh:
    name: str
    driver: str          # arbor carrying the driving gear
    driven: str          # arbor carrying the driven gear
    z_driver: int
    z_driven: int
    module: float        # m (SI, metres)
    kind: str = "going"  # going | motion | keyless | auto

    @property
    def ratio(self) -> float:
        """Speed ratio omega_driven / omega_driver (signed, external mesh)."""
        return -self.z_driver / self.z_driven

    @property
    def centre_distance(self) -> float:
        return self.module * (self.z_driver + self.z_driven) / 2.0

    @property
    def r_driver(self) -> float:
        return self.module * self.z_driver / 2.0

    @property
    def r_driven(self) -> float:
        return self.module * self.z_driven / 2.0


@dataclass
class GearTrain:
    meshes: list[Mesh]
    f_balance: float                     # Hz
    z_escape: int
    omega: dict[str, float] = field(default_factory=dict)   # rad/s, signed

    # ------------------------------------------------------------------------------
    @classmethod
    def from_params(cls, P: ParamSet) -> "GearTrain":
        z = lambda k: P.int(k)  # noqa: E731
        m_c = P["train.module_center"]
        # third-wheel mesh shares both axes with the centre/third-pinion mesh (coaxial seconds
        # pinion) => its module is fixed by equal centre distances:
        m_3 = m_c * (z("train.center_wheel") + z("train.third_pinion")) / (
            z("train.third_wheel") + z("train.seconds_pinion"))
        meshes = [
            Mesh("barrel->centre", "barrel", "centre", z("train.barrel_teeth"), z("train.center_pinion"), P["train.module_barrel"]),
            Mesh("centre->third", "centre", "third", z("train.center_wheel"), z("train.third_pinion"), m_c),
            Mesh("third->fourth", "third", "fourth", z("train.third_wheel"), z("train.fourth_pinion"), m_3),
            Mesh("third->seconds", "third", "seconds", z("train.third_wheel"), z("train.seconds_pinion"), m_3),
            Mesh("fourth->escape", "fourth", "escape", z("train.fourth_wheel"), z("train.escape_pinion"), P["train.module_fourth"]),
            # motion works (cannon pinion is friction-fitted on the centre arbor)
            Mesh("cannon->minute_wheel", "centre", "minute_wheel", z("motion_works.cannon_pinion"), z("motion_works.minute_wheel"), P["motion_works.module"], "motion"),
            Mesh("minute_pinion->hour", "minute_wheel", "hour", z("motion_works.minute_pinion"), z("motion_works.hour_wheel"), P["motion_works.module"], "motion"),
        ]
        f_bal = P["movement.frequency"] / 2.0          # vph -> beats/s ; 2 beats per period
        tr = cls(meshes, f_bal, z("train.escape_teeth"))
        tr.solve()
        return tr

    # ------------------------------------------------------------------------------
    def solve(self) -> None:
        """Propagate speeds through the mesh graph starting from the escape wheel.

        The escape wheel's *sense* of rotation is not imposed: we impose that the centre wheel
        (minute hand) turns clockwise, propagate magnitudes from the escape wheel, and let the
        signs follow from the mesh graph.
        """
        # 1) magnitudes from the time base, signs relative to centre = -1 (clockwise)
        rel: dict[str, float] = {"centre": -1.0}
        changed = True
        while changed:
            changed = False
            for m in self.meshes:
                if m.driver in rel and m.driven not in rel:
                    rel[m.driven] = rel[m.driver] * m.ratio
                    changed = True
                elif m.driven in rel and m.driver not in rel:
                    rel[m.driver] = rel[m.driven] / m.ratio
                    changed = True
        omega_esc_mag = 2.0 * math.pi * self.f_balance / self.z_escape
        scale = omega_esc_mag / abs(rel["escape"])
        self.omega = {k: v * scale for k, v in rel.items()}

    # ------------------------------------------------------------------------------
    def ratio(self, a: str, b: str) -> float:
        """Signed speed ratio omega_a / omega_b."""
        return self.omega[a] / self.omega[b]

    def period(self, arbor: str) -> float:
        """Time for one revolution (s)."""
        return 2.0 * math.pi / abs(self.omega[arbor])

    def rpm(self, arbor: str) -> float:
        return self.omega[arbor] * 60.0 / (2.0 * math.pi)

    def direction(self, arbor: str) -> str:
        return "CCW (dial view)" if self.omega[arbor] > 0 else "CW (dial view)"

    @property
    def escape_to_barrel(self) -> float:
        """Total going-train multiplication i = omega_escape / omega_barrel (positive magnitude)."""
        return abs(self.omega["escape"] / self.omega["barrel"])

    def mesh(self, name: str) -> Mesh:
        return next(m for m in self.meshes if m.name == name)

    def angles_from_escape(self, psi):
        """Exact kinematic map: arbor angles given the escape-wheel angle psi (rad, signed).

        Valid for a rigid, backlash-free train. Returns {arbor: angle}.
        """
        k = {a: w / self.omega["escape"] for a, w in self.omega.items()}
        return {a: psi * r for a, r in k.items()}

    def hands(self) -> dict[str, dict[str, float]]:
        return {
            "seconds": {"arbor": "seconds", "period_s": self.period("seconds"), "omega": self.omega["seconds"]},
            "minute": {"arbor": "centre", "period_s": self.period("centre"), "omega": self.omega["centre"]},
            "hour": {"arbor": "hour", "period_s": self.period("hour"), "omega": self.omega["hour"]},
        }

    def table(self) -> list[dict]:
        rows = []
        for a, w in self.omega.items():
            rows.append({"arbor": a, "omega_rad_s": w, "rpm": self.rpm(a), "rev_per_hour": abs(w) * 3600 / (2 * math.pi),
                         "period_s": self.period(a), "direction": self.direction(a)})
        return rows


@dataclass
class KeylessTrain:
    """Crown -> winding pinion -> crown wheel (idler) -> ratchet (barrel arbor)."""
    z_winding_pinion: int
    z_crown_wheel: int
    z_ratchet: int
    efficiency: float

    @classmethod
    def from_params(cls, P: ParamSet) -> "KeylessTrain":
        return cls(P.int("keyless.winding_pinion"), P.int("keyless.crown_wheel"),
                   P.int("keyless.ratchet_wheel"), P["keyless.efficiency"])

    @property
    def arbor_turns_per_crown_turn(self) -> float:
        # idler does not change the overall ratio
        return self.z_winding_pinion / self.z_ratchet

    def crown_torque(self, spring_torque: float) -> float:
        """Torque the wearer must apply at the crown (N m)."""
        return spring_torque * self.arbor_turns_per_crown_turn / self.efficiency
