"""Swiss lever escapement: reduced geometric model.

The escapement is described by angles (degrees in config, radians here):

  Fork (pallet) angles, measured at the pallet arbor
    phi_L  lock          phi_R  run to banking        phi_I  impulse
    phi_B = (2 (phi_L + phi_R) + phi_I) / 2         half total fork swing (banking angle)    (1)
    A swing from one banking to the other is split into
       u in [0, phi_L+phi_R]                    UNLOCKING  (tooth slides off locking face)
       u in [phi_L+phi_R, phi_L+phi_R+phi_I]    IMPULSE    (tooth slides along impulse plane)
       u in [.., 2 phi_B]                       DROP / LOCK / RUN (fork follows the roller)
    where u = s*phi + phi_B is the fork travel from its starting banking (s = swing direction).

  Roller / fork coupling (exact plane geometry, impulse pin at radius r_r, pallet arbor at
  distance d from the balance staff):
        phi = g(theta') = atan2( r_r sin theta', d - r_r cos theta' ),  theta' = theta - theta_be   (2)
    The impulse pin engages the fork while |theta'| <= lambda/2 (lambda = LIFT ANGLE, 49 deg,
    sourced). Requiring g(lambda/2) = phi_B fixes the impulse-pin radius:
        r_r = d sin(phi_B) / sin(lambda/2 + phi_B)                                         (3)

  Escape wheel (Z teeth, tip radius r_e), straight-line tangent geometry with pallets spanning
  angle 2*beta (60 deg => 2.5 tooth pitches):
        d_ep = r_e / cos(beta),     rho_lock = r_e tan(beta)                               (4)
    Recoil while unlocking (draw angle delta):
        psi_rec = (phi_L + phi_R) * rho_lock * tan(delta) / r_e                            (5)
    Per beat the wheel must advance exactly half a pitch:
        -psi_rec + psi_I + psi_D = pi / Z          =>   psi_I = pi/Z + psi_rec - psi_D     (6)
    Transmission ratios (wheel angle per fork angle):
        kappa_U = psi_rec/(phi_L+phi_R)   (backwards),   kappa_I = psi_I/phi_I              (7)

  Sliding friction on inclined planes (friction angle varphi = atan mu):
        impulse efficiency      eta_I = tan(alpha) / tan(alpha + varphi)                  (8)
        unlocking resistance    f_U   = tan(delta + varphi) / tan(delta)   (fork advancing) (9)
                                f_U'  = tan(delta - varphi) / tan(delta)   (fork returning)
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from ..geometry.massprops import MassProps, annulus, cylinder, point_mass, radial_bar, spoked_wheel
from ..params import ParamSet


@dataclass
class EscapementGeometry:
    z: int
    r_e: float
    r_root: float
    beta: float
    d_ep: float
    rho_lock: float
    d_bp: float
    lift: float          # balance lift angle (rad)
    phi_L: float
    phi_R: float
    phi_I: float
    phi_B: float
    r_roller: float
    psi_half: float
    psi_D: float
    psi_rec: float
    psi_I: float
    kappa_U: float
    kappa_I: float
    delta: float
    mu: float
    alpha: float
    eta_I: float
    f_U_fwd: float
    f_U_back: float
    eta_pin: float
    knock_angle: float
    knock_e: float
    I_fork: float
    I_escape: float
    fork_mass: float
    escape_mass: float

    @classmethod
    def from_params(cls, P: ParamSet) -> "EscapementGeometry":
        e = lambda k: P[f"escapement.{k}"]  # noqa: E731
        z = P.int("train.escape_teeth")
        r_e = e("escape_tip_radius")
        beta = e("pallet_span") / 2.0
        d_ep = r_e / math.cos(beta)
        rho_lock = r_e * math.tan(beta)
        phi_L, phi_R, phi_I = e("lock_angle"), e("run_angle"), e("impulse_angle")
        phi_B = (2.0 * (phi_L + phi_R) + phi_I) / 2.0
        lift = P["movement.lift_angle"]
        d_bp = e("pallet_to_balance")
        r_roller = d_bp * math.sin(phi_B) / math.sin(lift / 2.0 + phi_B)
        delta = e("draw_angle")
        psi_half = math.pi / z
        psi_D = e("drop_angle")
        psi_rec = (phi_L + phi_R) * rho_lock * math.tan(delta) / r_e
        psi_I = psi_half + psi_rec - psi_D
        mu = e("pallet_friction_coeff")
        vphi = math.atan(mu)
        alpha = e("impulse_plane_angle")

        # ---- mass properties --------------------------------------------------------
        t_w, rho_w = e("wheel_thickness"), e("wheel_density")
        tooth_h = 0.32e-3
        r_root = r_e - tooth_h
        wheel = spoked_wheel(r_e, r_root, t_w, rho_w, rim_width=0.18e-3, n_arms=4,
                             arm_width=0.16e-3, hub_radius=0.30e-3, tooth_fill=0.30)
        esc_pinion = cylinder(P["train.module_fourth"] * P.int("train.escape_pinion") / 2.0, 0.6e-3, 7850.0)
        esc_arbor = cylinder(0.08e-3, 1.8e-3, 7850.0)
        escape = wheel + esc_pinion + esc_arbor

        t_f, rho_f, w_f = e("fork_thickness"), e("fork_density"), e("fork_width")
        lever_len = d_bp - r_roller + 0.35e-3          # past the slot to the horns
        lever = radial_bar(0.0, lever_len, w_f, t_f, rho_f)
        frame = radial_bar(0.0, rho_lock + 0.15e-3, 0.30e-3, t_f, rho_f)
        frame = MassProps(2 * frame.mass, 2 * frame.inertia)          # two pallet arms
        stone_m = 3980.0 * 0.30e-3 * 0.25e-3 * 0.80e-3                 # ruby pallet stone
        stones = point_mass(2 * stone_m, rho_lock)
        staff = cylinder(0.10e-3, 1.6e-3, 7850.0)
        guard = point_mass(7850.0 * math.pi * (0.04e-3) ** 2 * 0.4e-3, lever_len - 0.2e-3)
        fork = lever + frame + stones + staff + guard

        return cls(
            z=z, r_e=r_e, r_root=r_root, beta=beta, d_ep=d_ep, rho_lock=rho_lock, d_bp=d_bp,
            lift=lift, phi_L=phi_L, phi_R=phi_R, phi_I=phi_I, phi_B=phi_B, r_roller=r_roller,
            psi_half=psi_half, psi_D=psi_D, psi_rec=psi_rec, psi_I=psi_I,
            kappa_U=psi_rec / (phi_L + phi_R), kappa_I=psi_I / phi_I,
            delta=delta, mu=mu, alpha=alpha,
            eta_I=math.tan(alpha) / math.tan(alpha + vphi),
            f_U_fwd=math.tan(delta + vphi) / math.tan(delta),
            f_U_back=max(math.tan(delta - vphi), 0.0) / math.tan(delta),
            eta_pin=e("pin_efficiency"),
            knock_angle=2.0 * math.pi - lift / 2.0,
            knock_e=e("knock_restitution"),
            I_fork=fork.inertia, I_escape=escape.inertia,
            fork_mass=fork.mass, escape_mass=escape.mass,
        )

    # ------------------------------------------------------------------------------
    def fork_angle(self, theta_rel: float) -> float:
        """Equation (2): fork angle for balance angle theta' (relative to escapement centre)."""
        return math.atan2(self.r_roller * math.sin(theta_rel), self.d_bp - self.r_roller * math.cos(theta_rel))

    def fork_ratio_at_centre(self) -> float:
        """d(phi)/d(theta) at theta'=0: r_r/(d - r_r)."""
        return self.r_roller / (self.d_bp - self.r_roller)

    @property
    def u_unlock_end(self) -> float:
        return self.phi_L + self.phi_R

    @property
    def u_impulse_end(self) -> float:
        return self.phi_L + self.phi_R + self.phi_I
