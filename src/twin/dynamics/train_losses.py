"""Going-train torque transmission and losses.

Stage model (forward drive, barrel -> escape wheel)
--------------------------------------------------
For every mesh i the tooth-friction efficiency is the classical sliding-loss estimate

        eta_mesh = 1 - f * mu_m * pi * (1/z_driver + 1/z_driven)                     (1)

(f < 1 accounts for cycloidal watch gearing acting mostly after the line of centres).
For every arbor j carrying an input pinion (pitch radius r_in) and output wheel (r_out), the
pivot friction torque is mu_p * r_pivot * |F_radial| with F_radial ~ T/r_in + T/r_out, giving

        eta_pivot = 1 - mu_p * r_pivot * (1/r_in + 1/r_out)                            (2)

Load-independent drags (seconds-pinion friction spring, motion-works drag) are subtracted as
constant torques. The torque reaching the escape wheel is therefore an affine function of the
barrel torque:  T_e = a * T_b - b.   Losses per stage follow from power balance.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from ..kinematics.train import GearTrain
from ..params import ParamSet


@dataclass
class Stage:
    name: str
    eta: float            # multiplicative efficiency of this stage
    ratio: float          # |omega_out / omega_in| speed-up of the stage (torque divided by it)
    drag_out: float = 0.0  # constant drag torque subtracted at the stage output (N m)
    kind: str = "mesh"    # mesh | pivot | drag


@dataclass
class TrainLosses:
    stages: list[Stage]
    i_total: float
    omega_barrel: float
    detail: dict = field(default_factory=dict)

    @classmethod
    def from_params(cls, P: ParamSet, train: GearTrain) -> "TrainLosses":
        mu_m = P["train_friction.mesh_friction_coeff"]
        f = P["train_friction.mesh_loss_factor"]
        mu_p = P["train_friction.pivot_friction_coeff"]

        def eta_mesh(z1, z2):
            return 1.0 - f * mu_m * math.pi * (1.0 / z1 + 1.0 / z2)

        def eta_pivot(r_p, r_in, r_out):
            return 1.0 - mu_p * r_p * (1.0 / r_in + 1.0 / r_out)

        m = {mm.name: mm for mm in train.meshes}
        bc, c3, s34, s3s, s4e = (m["barrel->centre"], m["centre->third"], m["third->fourth"],
                                 m["third->seconds"], m["fourth->escape"])
        r_esc_tip = P["escapement.escape_tip_radius"]
        stages = [
            Stage("barrel drum on arbor", eta_pivot(P["train_friction.pivot_radius_barrel"], 1e9, bc.r_driver), 1.0, kind="pivot"),
            Stage("mesh barrel/centre pinion", eta_mesh(bc.z_driver, bc.z_driven), bc.z_driver / bc.z_driven),
            Stage("centre arbor pivots", eta_pivot(P["train_friction.pivot_radius_center"], bc.r_driven, c3.r_driver), 1.0,
                  drag_out=P["train_friction.motion_works_drag"], kind="pivot"),
            Stage("mesh centre/third pinion", eta_mesh(c3.z_driver, c3.z_driven), c3.z_driver / c3.z_driven),
            Stage("third arbor pivots", eta_pivot(P["train_friction.pivot_radius_third"], c3.r_driven, s34.r_driver), 1.0,
                  drag_out=P["train_friction.seconds_brake_torque"] * (s3s.z_driver / s3s.z_driven) / eta_mesh(s3s.z_driver, s3s.z_driven),
                  kind="pivot"),
            Stage("mesh third/fourth pinion", eta_mesh(s34.z_driver, s34.z_driven), s34.z_driver / s34.z_driven),
            Stage("fourth arbor pivots", eta_pivot(P["train_friction.pivot_radius_fourth"], s34.r_driven, s4e.r_driver), 1.0, kind="pivot"),
            Stage("mesh fourth/escape pinion", eta_mesh(s4e.z_driver, s4e.z_driven), s4e.z_driver / s4e.z_driven),
            Stage("escape arbor pivots", eta_pivot(P["train_friction.pivot_radius_escape"], s4e.r_driven, r_esc_tip), 1.0, kind="pivot"),
        ]
        tl = cls(stages, train.escape_to_barrel, abs(train.omega["barrel"]))
        tl.detail = {"eta_seconds_branch_mesh": eta_mesh(s3s.z_driver, s3s.z_driven)}
        return tl

    # ------------------------------------------------------------------------------
    @property
    def eta_total(self) -> float:
        """Product of multiplicative efficiencies (excludes constant drags)."""
        e = 1.0
        for s in self.stages:
            e *= s.eta
        return e

    def forward(self, T_barrel: float) -> list[dict]:
        """Torque/power at each stage output for forward running at nominal speed."""
        T = T_barrel
        w = self.omega_barrel
        rows = []
        for s in self.stages:
            P_in = T * w
            T_out = T * s.eta / s.ratio - s.drag_out
            w_out = w * s.ratio
            P_out = T_out * w_out
            rows.append({"stage": s.name, "kind": s.kind, "T_in": T, "T_out": T_out, "omega_out": w_out,
                         "P_in": P_in, "P_out": P_out, "P_loss": P_in - P_out, "eta": s.eta})
            T, w = T_out, w_out
        return rows

    def escape_torque(self, T_barrel: float) -> float:
        """Torque available at the escape wheel while the train runs forward (N m)."""
        return self.forward(T_barrel)[-1]["T_out"]

    def escape_torque_backdrive(self, T_barrel: float) -> float:
        """Torque the escapement must apply to drive the train BACKWARD (recoil during unlocking).
        Friction now opposes the reversed motion, so efficiencies divide and drags add."""
        T = T_barrel
        for s in self.stages:
            T = T / (s.eta * s.ratio) + s.drag_out
        return T

    def affine(self) -> tuple[float, float]:
        """Return (a, b) with T_e = a*T_b - b."""
        t0 = self.escape_torque(0.0)
        t1 = self.escape_torque(1.0)
        return t1 - t0, -t0
