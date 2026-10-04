# Assumptions, approximations and limitations

This page states plainly what the twin does *not* know or does not model, so its results are read at
the right level of confidence.

## What is verified

These quantities are verified against the real watch or its manufacturer:

- **From the case:** 44 mm case, 21 jewels, automatic, 50 m water resistance, reference TWEG16716.
- **From the owner:** hacking, and 21,600 vph from a count of 6 beats per second.
- **From two photographs, agreeing to 0.05 mm:** balance diameter 9.54 mm.
- **From photographs plus the Grail reference:** balance position.
- **From Miyota's documents:** frequency, 49° lift angle, 42 h reserve, rate and posture specifications, dimensions and the parts list.

## What is estimated, and how it is constrained

| Estimated | Constrained by | Remaining freedom |
|---|---|---|
| Tooth counts | verified hand rates (60 s, 3600 s, 12 h) and 21,600 vph; two coaxial-mesh conditions | Individual factorisations (80/10 vs 72/9, …). The ratios are certain; the counts are not. |
| Modules, centre distances | barrel size, movement diameter, coaxial meshes | ±15 % on wheel sizes |
| Mainspring section | stress limit, barrel geometry, the 42 h spec (calibration) | Torque level ±30 %; the reserve itself is calibrated |
| Balance inertia | measured diameter; assumed rim section and alloy density | ±25 % |
| Hairspring stiffness | k = I(2πf)², exact by definition for the as-built watch | none for the rate; geometry is only a plausibility check |
| Escapement angles, friction | Swiss lever design practice; the 49° lift angle fixes the pin radius | ±15 % on impulse; drives amplitude and efficiency |
| Damping Q values | order of magnitude from horological literature | ±50 %; drives amplitude |
| Unpoise, beat offset | typical unadjusted movement; Miyota's posture-difference spec bounds unpoise | magnitudes only |
| Wrist motion (self-winding) | assumed activity profiles | qualitative only |

The Monte-Carlo study propagates all of these ranges to the outputs (`docs/05_results.md`).

## Modelling approximations

1. **Reduced escapement geometry.** Tooth and pallet contacts are described by kinematic ratios (κ_U, κ_I), draw and impulse-plane angles, and inclined-plane friction efficiencies. The model does not compute 2-D contact geometry of the real club-tooth profiles. Impact, separation and catch-up are modelled; the tooth-tip "slide" on the locking face and the elasticity of the parts are not. The run-to-banking after lock is purely kinematic: about 1 % of the impulse energy is neglected.
2. **Rigid, backlash-free train.** Wheel angles are exact multiples of the escape-wheel angle. Tooth backlash, the indirect-seconds stutter and train elasticity are not simulated. The stutter is documented and its friction spring is included as a drag.
3. **Constant-coefficient friction.** Mesh and pivot friction are Coulomb with fixed μ, and oil viscosity and temperature are ignored. Static friction (breakaway) at restart is not modelled. That is one reason a real watch stops slightly earlier than a model running on pure kinetic friction.
4. **Linear viscous balance damping.** Air, oil and hairspring losses use equivalent viscous terms at 3 Hz (Q values). Real aerodynamic drag has a quadratic component.
5. **Hairspring.** It is a flat spiral with uniform twist (for the animation) and linear or cubic stiffness. Spring breathing, centre-of-gravity shift, regulator-pin play and temperature effects are not modelled.
6. **Positional effects.** Only pivot friction and balance unpoise are modelled. Pallet-fork poise, hairspring sag and escape-wheel unbalance are neglected.
7. **Mainspring.** The linear Euler–Bernoulli torque law uses empirical hysteresis and roll-off factors. The bridle-slip characteristic is idealised as a hard limit at full wind.
8. **Time-scale separation.** This holds until the final minutes of run-down, which the brute-force comparison quantifies.
9. **CAD geometry.** Bridges and plates are parametric approximations around the solved arbor positions; their exact Miyota outlines are not public. The z-stack is assumed but consistent with the 5.67 mm height and free of interference.
10. **Self-winding.** The rotor is a rigid pendulum with idealised wrist motion. Results are qualitative.

## How to tighten the model

1. **Timegrapher readings** (rate, amplitude and beat error in dial-up and crown-down at full wind and after 24 h). These directly calibrate Q, unpoise and beat offset and validate the escapement.
2. **A full-wind-to-stop run time in dial-up.** This independently checks the reserve; the current value is calibrated.
3. **Macro photographs of the open heart and dial-side openings.** These allow counting visible teeth.
4. **Side and top photographs with a scale.** These give case thickness, bridge outlines and rotor shape.
