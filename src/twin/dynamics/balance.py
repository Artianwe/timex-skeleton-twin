"""Balance wheel + hairspring oscillator properties.

Governing equation of the free balance (between escapement contacts):

    I theta'' + c theta' + T_c sgn(theta') + k theta (1 + b theta^2) + T_g(theta) = 0     (1)

  I      polar moment of inertia of balance (+1/3 of hairspring, classical result)
  k      hairspring torsional stiffness;  k = I (2 pi f)^2  for design frequency f       (2)
  c      equivalent viscous damping, sum of air, hairspring-internal and oil terms:
            c_i = I omega_0 / Q_i                                                       (3)
  T_c    Coulomb pivot friction, orientation dependent
            horizontal: T_c = mu m g r_end      (pivot end on cap jewel)
            vertical  : T_c = mu m g r_pivot    (journal in hole jewel)                 (4)
  T_g    gravity torque of an out-of-poise balance (vertical positions only):
            T_g = - m g e_cg sin(gamma - beta_h - theta)   (see positional docs)       (5)

Hairspring geometry (flat spiral): k = E_h I_h / L_h with I_h = h t^3/12, hence
            L_h = E_h h t^3 / (12 k)                                                    (6)
which is iterated together with the hairspring's own effective inertia.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from ..geometry.massprops import MassProps, annulus, point_mass, radial_bar, spiral_strip
from ..params import ParamSet
from ..units import SECONDS_PER_DAY


@dataclass
class BalanceProps:
    mass: float
    inertia: float               # total effective (kg m^2)
    inertia_wheel: float         # rim + arms + staff (no hairspring)
    k: float                     # N m / rad (including regulator offset)
    k_design: float              # stiffness for exact design frequency
    f_design: float
    f_free: float                # free (undamped, linear) frequency after regulator offset
    c_air: float
    c_hairspring: float
    c_oil: float
    cubic: float                 # b in k(1+b theta^2)
    mu_pivot: float
    r_pivot: float
    r_end: float
    unpoise: float               # m g e  (N m)
    unpoise_angle: float         # rad
    beat_offset: float           # rad
    hs_length: float
    hs_turns: float
    hs_mass: float
    rim_outer: float
    rim_inner: float
    parts: dict

    @classmethod
    def from_params(cls, P: ParamSet) -> "BalanceProps":
        b = lambda k: P[f"balance.{k}"]  # noqa: E731
        hs = lambda k: P[f"hairspring.{k}"]  # noqa: E731
        g = P["simulation.gravity"]
        r_o = b("rim_outer_diameter") / 2.0
        r_i = r_o - b("rim_radial_width")
        rho = b("density")
        rim = annulus(r_i, r_o, b("rim_thickness"), rho)
        hub_r = 0.35e-3
        arms = MassProps(0.0, 0.0)
        for _ in range(P.int("balance.arms")):
            arms = arms + radial_bar(hub_r, r_i, b("arm_width"), b("arm_thickness"), rho)
        staff = point_mass(b("staff_assembly_mass"), b("staff_assembly_radius_of_gyration"))
        wheel = rim + arms + staff

        f0 = hs("design_frequency")
        w0 = 2.0 * math.pi * f0
        # iterate hairspring length <-> stiffness <-> effective inertia
        I_tot = wheel.inertia
        hsp = MassProps(0.0, 0.0)
        for _ in range(20):
            k_design = I_tot * w0**2
            I_h = hs("height") * hs("thickness") ** 3 / 12.0
            L_h = hs("youngs_modulus") * I_h / k_design
            hsp = spiral_strip(L_h, hs("height"), hs("thickness"), hs("density"),
                               hs("inner_radius"), hs("outer_radius"))
            I_new = wheel.inertia + hsp.inertia
            if abs(I_new - I_tot) < 1e-18:
                break
            I_tot = I_new
        k_design = I_tot * w0**2
        # regulator: rate offset rho (fraction) -> f = f0 (1 + rho)  => k = I (2 pi f)^2
        rho_rate = P["hairspring.regulator_offset"]           # already fractional (s/day -> 1)
        # hairspring is matched to the as-designed balance; scale factors then perturb an existing
        # watch (inertia change without re-matching, or a stiffness change of the fitted spring)
        k = I_tot * (2.0 * math.pi * f0 * (1.0 + rho_rate)) ** 2 * P["hairspring.stiffness_scale"]
        I_tot = I_tot * P["balance.inertia_scale"]
        f_free = math.sqrt(k / I_tot) / (2.0 * math.pi)
        r_mean = 0.5 * (hs("inner_radius") + hs("outer_radius"))
        m_total = wheel.mass + hsp.mass
        return cls(
            mass=m_total, inertia=I_tot, inertia_wheel=wheel.inertia, k=k, k_design=k_design,
            f_design=f0, f_free=f_free,
            c_air=I_tot * w0 / P["damping.q_air"],
            c_hairspring=I_tot * w0 / P["damping.q_hairspring"],
            c_oil=I_tot * w0 / P["damping.q_oil"],          # Q defined at the design frequency
            cubic=P["hairspring.cubic_stiffness"],
            mu_pivot=b("pivot_friction_coeff"), r_pivot=b("pivot_radius"), r_end=b("end_contact_radius"),
            unpoise=m_total * g * b("unpoise_offset"), unpoise_angle=b("unpoise_angle"),
            beat_offset=b("beat_offset"),
            hs_length=L_h, hs_turns=L_h / (2.0 * math.pi * r_mean), hs_mass=hsp.mass,
            rim_outer=r_o, rim_inner=r_i,
            parts={"rim": rim, "arms": arms, "staff": staff, "hairspring_eff": hsp},
        )

    # ------------------------------------------------------------------------------
    @property
    def c(self) -> float:
        return self.c_air + self.c_hairspring + self.c_oil

    @property
    def omega0(self) -> float:
        return math.sqrt(self.k / self.inertia)

    @property
    def q_viscous(self) -> float:
        return self.inertia * self.omega0 / self.c

    def coulomb_torque(self, horizontal: bool, g: float = 9.80665) -> float:
        r = self.r_end if horizontal else self.r_pivot
        return self.mu_pivot * self.mass * g * r

    def energy(self, amplitude: float) -> float:
        """Oscillator energy at amplitude A (rad): E = k A^2 / 2 (linear spring)."""
        return 0.5 * self.k * amplitude**2

    def rate_from_frequency(self, f: float) -> float:
        """Daily rate (s/day) of a watch whose balance runs at f instead of f_design."""
        return (f / self.f_design - 1.0) * SECONDS_PER_DAY
