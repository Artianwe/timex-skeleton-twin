"""Mainspring and barrel model.

State variable
--------------
n  = wound turns of the barrel arbor relative to the barrel drum, measured from the let-down
     state (0 <= n <= n_dev). Running the watch turns the drum and decreases n; winding (crown or
     rotor) turns the arbor and increases n until the slipping bridle limits it at n_dev.

Governing relations
-------------------
Spiral spring under uniform bending moment (Euler-Bernoulli, every section carries the same
moment M):
        M = (E I_z / L) * (Theta - Theta_0),   I_z = h e^3 / 12                     (1)
so the torque grows linearly with wound angle with stiffness S = E I_z / L.

Strength limit at full wind (outer fibre stress):
        M_full = sigma_w * h * e^2 / 6                                              (2)

Geometry (classical barrel design, spring cross-section area L*e filling a fraction f of the
annulus between arbor radius r and barrel radius R):
        L     = f * pi * (R^2 - r^2) / e                                            (3)
        n_th  = [ (R_o - r) - (R - R_i) ] / e,   R_i^2 = R^2 - L e/pi,  R_o^2 = r^2 + L e/pi  (4)
        n_dev = k_dev * n_th                                                         (5)

Delivered torque while unwinding includes inter-coil friction (hysteresis) and the collapse of
torque as the outer coils settle on the barrel wall near let-down:
        M_out(n) = (1 - xi) * M_el(n) * (1 - exp(-n / (x_c * n_dev)))             (6)
Winding requires M_wind(n) = (1 + xi) * M_el(n).
Stored elastic energy from let-down:  E(n) = 2*pi * (M_down n + pi S n^2).          (7)
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..params import ParamSet


@dataclass
class Mainspring:
    E: float
    sigma_w: float
    h: float
    e: float
    R: float
    r: float
    fill: float
    k_dev: float
    xi: float
    x_c: float
    density: float

    @classmethod
    def from_params(cls, P: ParamSet) -> "Mainspring":
        g = lambda k: P[f"mainspring.{k}"]  # noqa: E731
        return cls(g("youngs_modulus"), g("working_stress"), g("height"), g("thickness"),
                   g("barrel_inner_radius"), g("arbor_radius"), g("fill_factor"),
                   g("development_factor"), g("coil_friction"), g("end_rolloff"), g("density"))

    # ---- geometry ------------------------------------------------------------------
    @property
    def length(self) -> float:
        return self.fill * math.pi * (self.R**2 - self.r**2) / self.e

    @property
    def n_theoretical(self) -> float:
        a = self.length * self.e / math.pi
        R_i = math.sqrt(max(self.R**2 - a, 0.0))
        R_o = math.sqrt(self.r**2 + a)
        return ((R_o - self.r) - (self.R - R_i)) / self.e

    @property
    def n_dev(self) -> float:
        return self.k_dev * self.n_theoretical

    @property
    def I_z(self) -> float:
        return self.h * self.e**3 / 12.0

    @property
    def stiffness(self) -> float:
        """S = E I / L  (N m / rad)."""
        return self.E * self.I_z / self.length

    @property
    def M_full(self) -> float:
        return self.sigma_w * self.h * self.e**2 / 6.0

    @property
    def M_down(self) -> float:
        return max(self.M_full - self.stiffness * 2.0 * math.pi * self.n_dev, 0.0)

    @property
    def mass(self) -> float:
        return self.density * self.length * self.h * self.e

    # ---- torque & energy -------------------------------------------------------------
    def torque_elastic(self, n):
        n = np.clip(n, 0.0, self.n_dev)
        return self.M_down + self.stiffness * 2.0 * math.pi * n

    def rolloff(self, n):
        n = np.maximum(n, 0.0)
        return 1.0 - np.exp(-n / (self.x_c * self.n_dev))

    def torque_out(self, n):
        """Torque delivered to the train while running (N m)."""
        return (1.0 - self.xi) * self.torque_elastic(n) * self.rolloff(n)

    def torque_wind(self, n):
        """Torque required at the barrel arbor to wind further (N m)."""
        return (1.0 + self.xi) * self.torque_elastic(n)

    def energy_elastic(self, n):
        """Elastic energy stored relative to the let-down state (J)."""
        n = np.clip(n, 0.0, self.n_dev)
        return 2.0 * math.pi * (self.M_down * n + math.pi * self.stiffness * n**2)

    def energy_deliverable(self, n, samples: int = 2001) -> float:
        """Energy delivered to the train when unwinding from n to 0 (J)."""
        x = np.linspace(0.0, float(n), samples)
        return float(np.trapezoid(self.torque_out(x), x) * 2.0 * math.pi)

    def stress(self, M: float) -> float:
        return 6.0 * M / (self.h * self.e**2)

    # ---- geometry for visualisation ---------------------------------------------------
    def spiral(self, n: float, pts_per_turn: int = 60) -> np.ndarray:
        """Approximate centre-line of the spring (x, y) in the barrel frame for wind state n.

        Two-pack model: a pack wrapped on the arbor and a pack against the barrel wall, joined by
        a free span. The fraction of length on the arbor grows linearly with n.
        """
        x = float(np.clip(n / self.n_dev, 0.0, 1.0)) if self.n_dev > 0 else 0.0
        L = self.length
        L_arb = (0.08 + 0.84 * x) * L
        L_wall = L - L_arb
        pts = []
        # arbor pack: Archimedean spiral r = r0 + e*phi/2pi
        s, phi = 0.0, 0.0
        while s < L_arb:
            rr = self.r + self.e * phi / (2 * math.pi)
            pts.append((rr * math.cos(phi), rr * math.sin(phi)))
            dphi = 2 * math.pi / pts_per_turn
            s += rr * dphi
            phi += dphi
        # wall pack: inward spiral from R
        s2, phi2 = 0.0, phi + 0.6
        wall = []
        while s2 < L_wall:
            rr = self.R - self.e * (s2 / (2 * math.pi * self.R)) * 1.0
            wall.append((rr, phi2))
            dphi = 2 * math.pi / pts_per_turn
            s2 += rr * dphi
            phi2 += dphi
        for rr, ph in reversed(wall):
            pts.append((rr * math.cos(ph), rr * math.sin(ph)))
        return np.asarray(pts)
