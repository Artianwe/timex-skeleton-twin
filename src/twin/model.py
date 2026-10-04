"""Assemble the complete physical model of the movement from the parameter database.

`build_model(P)` returns a WatchModel holding the kinematic train, train losses, mainspring,
balance and escapement sub-models, plus a DerivedRegistry documenting every derived number
with its governing equation and input dependencies.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .derived import DerivedRegistry
from .dynamics.balance import BalanceProps
from .dynamics.escapement_geometry import EscapementGeometry
from .dynamics.mainspring import Mainspring
from .dynamics.train_losses import TrainLosses
from .geometry.massprops import MassProps, cylinder, spoked_wheel
from .kinematics.train import GearTrain, KeylessTrain
from .params import ParamSet, load_params
from .units import SECONDS_PER_DAY


@dataclass
class WatchModel:
    P: ParamSet
    train: GearTrain
    keyless: KeylessTrain
    losses: TrainLosses
    spring: Mainspring
    balance: BalanceProps
    esc: EscapementGeometry
    arbor_inertia: dict
    I_escape_eff: float
    reg: DerivedRegistry

    # convenience ---------------------------------------------------------------------
    def escape_torque(self, T_barrel: float) -> float:
        return self.losses.escape_torque(T_barrel)

    @property
    def barrel_omega(self) -> float:
        return abs(self.train.omega["barrel"])

    def runtime_for_turns(self, turns: float) -> float:
        """Seconds of running that unwind `turns` barrel turns (escapement-governed)."""
        return turns * 2.0 * math.pi / self.barrel_omega


def _wheel_props(P: ParamSet, r_pitch: float, module: float, n_arms: int = 5) -> MassProps:
    t = P["train_friction.wheel_thickness"]
    rho = P["train_friction.wheel_density"]
    return spoked_wheel(r_pitch + module, r_pitch - 1.25 * module, t, rho,
                        rim_width=max(0.10 * r_pitch, 0.15e-3), n_arms=n_arms,
                        arm_width=max(0.07 * r_pitch, 0.12e-3), hub_radius=max(0.10 * r_pitch, 0.25e-3),
                        tooth_fill=0.5)


def _pinion_props(r_pitch: float, length: float = 0.7e-3) -> MassProps:
    return cylinder(r_pitch, length, 7850.0) + cylinder(0.12e-3, 2.5e-3, 7850.0)


def build_model(P: ParamSet | None = None) -> WatchModel:
    P = P or load_params()
    reg = DerivedRegistry()
    train = GearTrain.from_params(P)
    keyless = KeylessTrain.from_params(P)
    losses = TrainLosses.from_params(P, train)
    spring = Mainspring.from_params(P)
    bal = BalanceProps.from_params(P)
    esc = EscapementGeometry.from_params(P)

    # ---------------- arbor inertias & reflected inertia at the escape wheel --------------
    m = {mm.name: mm for mm in train.meshes}
    I = {
        "centre": _wheel_props(P, m["centre->third"].r_driver, m["centre->third"].module) + _pinion_props(m["barrel->centre"].r_driven),
        "third": _wheel_props(P, m["third->fourth"].r_driver, m["third->fourth"].module) + _pinion_props(m["centre->third"].r_driven),
        "fourth": _wheel_props(P, m["fourth->escape"].r_driver, m["fourth->escape"].module) + _pinion_props(m["third->fourth"].r_driven),
        "seconds": _pinion_props(m["third->seconds"].r_driven, 0.5e-3),
        "escape": MassProps(esc.escape_mass, esc.I_escape),
    }
    w_e = train.omega["escape"]
    I_eff = sum(I[a].inertia * (train.omega[a] / w_e) ** 2 for a in I)

    # ---------------- registry of derived quantities --------------------------------------
    f_bal = train.f_balance
    reg.add("balance_frequency", f_bal, "Hz", "f = vph / 7200  (2 beats per oscillation)", ["movement.frequency"])
    reg.add("beat_period", 1.0 / (2 * f_bal), "ms", "t_beat = 1/(2 f)", ["movement.frequency"])
    reg.add("escape_wheel_speed", abs(w_e), "rpm", "omega_e = 2 pi f / Z_e", ["movement.frequency", "train.escape_teeth"])
    for a in ("barrel", "centre", "third", "fourth", "seconds", "escape", "minute_wheel", "hour"):
        reg.add(f"period_{a}", train.period(a), "h" if train.period(a) > 3600 else "s",
                "T = 2 pi / |omega|, omega from tooth-count chain", ["train.*"])
    reg.add("train_ratio_escape_to_barrel", train.escape_to_barrel, "1", "i = prod(z_wheel/z_pinion)", ["train.*"])
    reg.add("module_third_mesh", m["third->fourth"].module, "mm",
            "m3 = m_c (Z_c + z_3p)/(Z_3 + z_sp)  (coaxial seconds pinion)", ["train.module_center"])
    for mesh in train.meshes:
        reg.add(f"centre_distance[{mesh.name}]", mesh.centre_distance, "mm", "a = m (z1+z2)/2", ["train.*"])
    reg.add("keyless_ratio", keyless.arbor_turns_per_crown_turn, "1", "arbor turns per crown turn = z_wp / z_ratchet", ["keyless.*"])

    reg.add("mainspring_length", spring.length, "mm", "L = f pi (R^2 - r^2)/e", ["mainspring.*"])
    reg.add("mainspring_turns_theoretical", spring.n_theoretical, "1", "n_th = [(R_o-r)-(R-R_i)]/e", ["mainspring.*"])
    reg.add("mainspring_turns_usable", spring.n_dev, "1", "n_dev = k_dev n_th", ["mainspring.*"])
    reg.add("mainspring_stiffness", spring.stiffness, "N*mm/rad", "S = E h e^3 / (12 L)", ["mainspring.*"])
    reg.add("mainspring_torque_full", spring.M_full, "N*mm", "M_full = sigma_w h e^2 / 6", ["mainspring.*"])
    reg.add("mainspring_torque_letdown", spring.M_down, "N*mm", "M_down = M_full - 2 pi S n_dev", ["mainspring.*"])
    reg.add("mainspring_energy_full", spring.energy_elastic(spring.n_dev), "J", "E = 2 pi (M_down n + pi S n^2)", ["mainspring.*"])
    reg.add("crown_turns_full_wind", spring.n_dev / keyless.arbor_turns_per_crown_turn, "1",
            "N_crown = n_dev / (z_wp/z_ratchet)", ["mainspring.*", "keyless.*"])
    reg.add("theoretical_runtime_full_development", spring.n_dev * 2 * math.pi / abs(train.omega["barrel"]), "h",
            "t = 2 pi n_dev / omega_barrel", ["mainspring.*", "train.*"])

    reg.add("balance_mass", bal.mass, "mg", "rim + arms + staff + hairspring", ["balance.*"])
    reg.add("balance_inertia", bal.inertia, "mg*cm^2", "I = m_rim (r_o^2+r_i^2)/2 + sum I_arm + I_staff + I_hs/3", ["balance.*"])
    reg.add("hairspring_stiffness", bal.k, "uN*m/rad", "k = I (2 pi f)^2", ["balance.*", "hairspring.design_frequency"])
    reg.add("hairspring_length", bal.hs_length, "mm", "L_h = E_h h t^3 / (12 k)", ["hairspring.*"])
    reg.add("hairspring_turns", bal.hs_turns, "1", "N = L_h / (2 pi r_mean)", ["hairspring.*"])
    reg.add("balance_Q_viscous", bal.q_viscous, "1", "1/Q = 1/Q_air + 1/Q_hs + 1/Q_oil", ["damping.*"])
    reg.add("pivot_friction_torque_horizontal", bal.coulomb_torque(True), "uN*m", "T_c = mu m g r_end", ["balance.*"])
    reg.add("pivot_friction_torque_vertical", bal.coulomb_torque(False), "uN*m", "T_c = mu m g r_pivot", ["balance.*"])
    reg.add("unpoise_torque", bal.unpoise, "uN*m", "U = m g e_cg", ["balance.unpoise_offset"])

    reg.add("fork_banking_angle", esc.phi_B, "deg", "phi_B = (2(phi_L+phi_R)+phi_I)/2", ["escapement.*"])
    reg.add("impulse_pin_radius", esc.r_roller, "mm", "r_r = d sin(phi_B)/sin(lambda/2+phi_B)", ["movement.lift_angle", "escapement.*"])
    reg.add("escape_to_pallet_distance", esc.d_ep, "mm", "d_ep = r_e / cos(beta)", ["escapement.*"])
    reg.add("escape_recoil", esc.psi_rec, "deg", "psi_rec = (phi_L+phi_R) rho_lock tan(delta)/r_e", ["escapement.*"])
    reg.add("escape_impulse_rotation", esc.psi_I, "deg", "psi_I = pi/Z + psi_rec - psi_D", ["escapement.*"])
    reg.add("impulse_ratio_kappa_I", esc.kappa_I, "1", "kappa_I = psi_I / phi_I", ["escapement.*"])
    reg.add("impulse_efficiency", esc.eta_I, "1", "eta_I = tan(alpha)/tan(alpha+atan(mu))", ["escapement.*"])
    reg.add("unlock_friction_factor", esc.f_U_fwd, "1", "f_U = tan(delta+atan(mu))/tan(delta)", ["escapement.*"])
    reg.add("fork_inertia", esc.I_fork, "mg*cm^2", "lever + pallet arms + stones + staff", ["escapement.*"])
    reg.add("escape_wheel_inertia", esc.I_escape, "mg*cm^2", "spoked wheel + pinion + arbor", ["escapement.*"])
    reg.add("escape_inertia_reflected", I_eff, "mg*cm^2", "I_eff = sum I_j (omega_j/omega_e)^2", ["train.*", "escapement.*"])
    reg.add("train_efficiency_multiplicative", losses.eta_total, "1", "prod(eta_mesh * eta_pivot)", ["train_friction.*"])
    T_full = spring.torque_out(spring.n_dev)
    reg.add("escape_torque_full_wind", losses.escape_torque(T_full), "uN*m", "T_e = T_b prod(eta)/i - drags", ["mainspring.*", "train_friction.*"])
    reg.add("nominal_rate_free_oscillator", (bal.f_free / bal.f_design - 1.0) * SECONDS_PER_DAY, "s/day",
            "rate = (f_free/f_design - 1) * 86400", ["hairspring.regulator_offset"])

    return WatchModel(P, train, keyless, losses, spring, bal, esc, I, I_eff, reg)
