# Theory: governing equations and physical basis

Every calculated result in the twin comes from one of the relations below. Symbols are SI. The sign
convention is dial view: *x* points to 3 o'clock, *y* to 12 o'clock, and *z* toward the viewer, so
counter-clockwise (CCW) rotation is positive.

---

## 1. Kinematics of the going train

**External mesh.** Two gears in mesh roll without slip at the pitch circle (radii *r = m z / 2*):

$$\omega_{\text{driven}} = -\,\omega_{\text{driver}}\,\frac{z_{\text{driver}}}{z_{\text{driven}}}$$

The minus sign is the reversal of sense at every external mesh. The chain is solved as a graph,
anchored by the time base:

$$\omega_e = \frac{2\pi f}{Z_e}, \qquad f = \frac{\text{vph}}{7200} = 3\ \text{Hz}$$

The escape wheel advances one tooth per balance period (one half-pitch per beat). Hand periods are
therefore *outputs*:

- centre (minute) wheel: 3600 s
- seconds pinion: 60 s
- hour wheel: 43 200 s

The validation checks these to 10⁻⁹ s.

**Constraints that fixed the assumed tooth counts.**

- *Hand rates:* (Z_c/z₃)(Z₃/z₄) = 60 and Z₄/z_e = 12 for 21 600 vph with a 15-tooth escape wheel.
- *Indirect seconds:* the centre-second pinion is driven by the third wheel, so z_sp = z₄ for 1 rpm.
- *Coaxial meshes on two shared axes:* m_c (Z_c + z₃) = m₃ (Z₃ + z_sp). This fixes the third-wheel module.
- *Motion works on one module:* z_cp + Z_mw = z_mp + Z_hw and (Z_mw / z_cp)(Z_hw / z_mp) = 12, giving 10/30 · 8/32.

**Centre distance:** a = m (z₁ + z₂)/2.

**Tooth profiles.** These are cycloidal, as in horological practice. The wheel addendum is the epicycloid
traced by a rolling circle of radius ρ = r_pinion/2:

$$x = (R+\rho)\cos t - \rho\cos\!\left(\tfrac{R+\rho}{\rho}t\right),\quad y = (R+\rho)\sin t - \rho\sin\!\left(\tfrac{R+\rho}{\rho}t\right)$$

The same circle rolling inside the pinion pitch circle degenerates to a radial line, which is why
watch pinion flanks are radial. Every mesh is checked to run through a full tooth pitch with zero
interference.

---

## 2. Mainspring and barrel

Euler–Bernoulli bending of a spiral strip carrying the same moment at every section gives:

$$M = \frac{E I_z}{L}\,(\Theta-\Theta_0), \qquad I_z = \frac{h e^3}{12} \;\Rightarrow\; S = \frac{E h e^3}{12 L}$$

**Strength limit at full wind:**

$$M_{\text{full}} = \sigma_w\,\frac{h e^2}{6}$$

**Barrel geometry.** The spring occupies a fraction *f* of the annulus between arbor radius *r* and barrel radius *R*:

$$L = \frac{f\pi(R^2-r^2)}{e}, \qquad n_{\text{th}} = \frac{(R_o - r) - (R - R_i)}{e},$$

$$R_i^2 = R^2 - \frac{Le}{\pi}, \qquad R_o^2 = r^2 + \frac{Le}{\pi}$$

**Delivered torque.** Inter-coil friction (hysteresis ξ) and roll-off near let-down reduce the elastic torque:

$$M_{\text{out}}(n) = (1-\xi)\,M_{\text{el}}(n)\left(1-e^{-n/(x_c n_{\text{dev}})}\right)$$

**Stored energy:**

$$E(n) = 2\pi\left(M_{\text{down}}\, n + \pi S n^2\right)$$

---

## 3. Torque transmission and train losses

**Sliding friction at a cycloidal mesh**, with coefficient μ_m and a recess factor *f*:

$$\eta_{\text{mesh}} = 1 - f\,\mu_m\,\pi\left(\frac{1}{z_1}+\frac{1}{z_2}\right)$$

**Pivot friction on an arbor** carrying an input pinion (r_in) and an output wheel (r_out):

$$\eta_{\text{pivot}} = 1 - \mu_p\, r_p\left(\frac{1}{r_{\text{in}}}+\frac{1}{r_{\text{out}}}\right)$$

**Torque at the escape wheel.** Constant drags (the seconds-pinion friction spring and the motion works) are subtracted at their arbors:

$$T_e = \frac{T_b\prod\eta}{i} - \text{drags}, \qquad i = \frac{\omega_e}{\omega_b} = 5400$$

**Back-driving** (recoil during unlocking) needs T_b/(i Πη) plus the drags, because friction reverses.

---

## 4. Balance and hairspring

**Equation of motion between contacts:**

$$I\ddot\theta + c\,\dot\theta + T_c\,\mathrm{sgn}\,\dot\theta + k\,\theta\,(1+b\theta^2) = U\sin(\gamma-\beta-\theta)$$

**Inertia** (rim, arms, staff and ⅓ of the hairspring):

$$I = \tfrac12 m_{\text{rim}}(r_o^2+r_i^2) + \sum I_{\text{arm}} + I_{\text{staff}} + \tfrac13 m_{hs} \bar r^2$$

**Stiffness** for the design frequency:

$$k = I(2\pi f)^2$$

**Hairspring length:**

$$L_h = \frac{E_h h t^3}{12 k}$$

This is a plausibility check: it comes out at 13.4 turns.

**Viscous damping** from quality factors:

$$c_i = \frac{I\omega_0}{Q_i}, \qquad \frac{1}{Q} = \sum \frac{1}{Q_i}$$

A free oscillation then decays by the factor e^(−π/2Q) per half-period. This is tested.

**Pivot friction.** Horizontal positions load the pivot end on the cap jewel; vertical positions load the journal in the hole jewel:

$$T_c = \mu m g\, r_{\text{end}} \;\;(\text{horizontal}), \qquad T_c = \mu m g\, r_{\text{pivot}} \;\;(\text{vertical})$$

Coulomb friction reduces the amplitude linearly, by 2T_c/k per half-swing (tested), and leaves the period unchanged.

**Out-of-poise torque** in vertical positions, with U = m g e:

$$U\sin(\gamma-\beta-\theta)$$

---

## 5. Swiss lever escapement (hybrid model)

**Fork angles.** The fork swings through lock φ_L, run φ_R and impulse φ_I, so the banking angle is:

$$\phi_B = \frac{2(\phi_L+\phi_R)+\phi_I}{2}$$

**Roller–fork coupling.** This is exact plane geometry, with impulse-pin radius r_r and pallet-staff distance d:

$$\phi = g(\theta') = \operatorname{atan2}\!\left(r_r\sin\theta',\, d - r_r\cos\theta'\right), \qquad \theta' = \theta-\theta_{be}$$

The **lift angle** λ = 49° (sourced) fixes the pin radius:

$$r_r = d\,\frac{\sin\phi_B}{\sin(\lambda/2+\phi_B)}$$

**Escape wheel.** The straight-line tangent geometry is:

$$d_{ep} = \frac{r_e}{\cos\beta}, \qquad \rho_{\text{lock}} = r_e\tan\beta$$

The recoil during unlocking is:

$$\psi_{\text{rec}} = \frac{(\phi_L+\phi_R)\,\rho_{\text{lock}}\tan\delta}{r_e}$$

Kinematic closure per beat:

$$-\psi_{\text{rec}}+\psi_I+\psi_D = \frac{\pi}{Z}$$

**Friction on inclined planes**, with friction angle φ = atan μ:

$$\eta_I = \frac{\tan\alpha}{\tan(\alpha+\varphi)}, \qquad f_U = \frac{\tan(\delta+\varphi)}{\tan\delta}$$

The safety rule δ > φ ensures that draw pulls the fork back to the banking. This is tested.

**Coupled equation in each contact phase.** With G = du/dθ = s g′, fork travel u and lumped inertia I_c:

$$J(\theta)\,\ddot\theta + I_c\, g'g''\,\dot\theta^2 = F_{\text{bal}} + G\,T_u, \qquad J = I_b + I_c g'^2$$

**Unilateral impulse contact.** The wheel pushes only while the contact torque is non-negative:

$$\lambda_c = T_e - I_e\ddot\psi \geq 0$$

Otherwise the wheel separates, free-runs under T_e/I_e, and catches the receding pallet with a plastic impact.

**Impact laws.** Generalised momentum is conserved along the active constraint:

- *pin entry:* I_b ω⁻ = J ω⁺
- *catch-up:* Λ = (ψ̇⁻ − κ_I G ω⁻)/(1/I_e + η κ_I² G²/J_bf)
- *drop and lock:* the wheel's kinetic energy is lost
- *banking:* the fork's kinetic energy is lost

**Energy ledger.** All dissipation rates are integrated as extra state variables, so that

$$W_{\text{escape}} = \sum \text{losses} + \Delta E_{\text{mech}}$$

closes to machine precision.

---

## 6. Timekeeping

**Rate**, as a timegrapher reports it:

$$\text{rate} = 86400\left(\frac{f}{f_0}-1\right)\ \text{s/day}$$

The hands advance one escape tooth per balance period, so this is exactly the error of the displayed time.

**Beat error:** half the difference between successive beat intervals.

**Amplitude** is measured from turning points. The virtual timegrapher value is:

$$A_{tg} = \frac{\lambda}{2\sin(\pi t_{\text{lift}}/T)}$$

**Krylov–Bogoliubov averaging** for a weak perturbing torque F(θ, θ̇), with θ = A cos ψ:

$$\frac{\Delta\omega}{\omega_0} = -\frac{\langle F\cos\psi\rangle}{kA}$$

For the out-of-poise torque this gives:

$$\Delta\text{rate} = 86400\,\frac{U\cos(\gamma-\beta)}{k}\,\frac{J_1(A)}{A}$$

The effect vanishes at A = 219.5°, the first zero of J₁. The hybrid simulation reproduces this to within 0.03 s/day at all 12 amplitudes tested. This is an independent check of the dynamics.

**Duffing hairspring** (isochronism):

$$\frac{\Delta f}{f} = \tfrac38\, b A^2$$

**Free oscillator.** With no escapement, no damping and a linear spring, the rate is 0. This is tested to 10⁻³ s/day, the RK4 floor.

**Perturbations of an existing watch.** A change of inertia or hairspring stiffness that is not re-matched shifts the rate by:

$$\frac{\Delta f}{f} = \tfrac12\left(\frac{\Delta k}{k} - \frac{\Delta I}{I}\right)$$

This is 432 s/day per 1 %, and it is tested.

---

## 7. Two time scales and the power reserve

**Amplitude relaxation:**

$$\tau = \frac{2Q}{\omega_0} \approx 30\ \text{s}$$

The mainspring changes over hours, so the balance is always on its limit cycle for the current torque. The limit cycle is found as the fixed point of the Poincaré (period) map, A* = P(A*), by Brent's method.

**Barrel unwinding** is escapement-governed, independent of amplitude:

$$\frac{dn}{dt} = -\frac{\omega_b}{2\pi}$$

**Power reserve:**

$$t_{PR} = \frac{(n_0 - n_{\text{stop}})\,2\pi}{\omega_b}$$

where n_stop is the wind state at which the limit cycle ceases to exist.

**Validation.** A full event-by-event integration through the whole reserve (≈10⁶ escapement events)
agrees with this model to 0.15 % in reserve. Amplitude agrees to better than 1°, and rate to better than 0.1 s/day, over the first 95 % of the run.

---

## 8. Self-winding

**Rotor relative to the case:**

$$I_r(\ddot\psi_r + \ddot\chi) = m g\sin\tau\, r_{cm}\sin(\gamma-\chi-\psi_r) - T_b\,\mathrm{sgn}\dot\psi_r - T_w$$

Here τ is the tilt of the dial plane and χ(t) the in-plane wrist rotation (an assumed activity profile). The winding load is active only in the winding sense, because the ratchet sliding wheel is a one-way clutch:

$$T_w = \frac{M_{\text{wind}}}{R\,\eta_{\text{auto}}}, \qquad \dot n = \frac{|\dot\psi_r|}{2\pi R}$$
