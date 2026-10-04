"""System-level validation: every major result checked against a specification, a closed-form
relationship, or an independent computation. Writes docs/04_validation_report.md.

Status:  PASS / FAIL against a stated tolerance;  INFO where the reference is itself an
estimate or the quantity was calibrated (so it cannot validate anything).
"""
from __future__ import annotations

import math
import subprocess
import sys

import numpy as np

from ..dynamics import hybrid as H
from ..params import ROOT, ParamSet


def parameter_rows(P: ParamSet) -> list[dict]:
    return [{"key": p.key, "value": p.raw, "unit": p.unit, "prov": p.prov, "range": p.range_raw, "src": p.src}
            for _, p in P.items()]


def _row(cid, cat, check, eq, expected, value, ok, tol="", basis="", status=None):
    st = status or ("PASS" if ok else "FAIL")
    return {"id": cid, "category": cat, "check": check, "equation": eq, "expected": expected, "value": value,
            "tolerance": tol, "status": st, "basis": basis}


def run_validation(m, sim, R: dict, run_pytest: bool = True) -> list[dict]:
    P = m.P
    tr = m.train
    out = []
    f = m.balance.f_design
    # ---------------------------------------------------------------- kinematics
    out.append(_row("K1", "kinematics", "Seconds-hand period", "T = 2π/|ω_sec|, ω from tooth-count chain",
                    "60 s", f"{tr.period('seconds'):.9g} s", abs(tr.period("seconds") - 60) < 1e-9, "1e-9 s", "watchmaking definition"))
    out.append(_row("K2", "kinematics", "Minute-hand (centre wheel) period", "T = 2π/|ω_c|", "3600 s",
                    f"{tr.period('centre'):.9g} s", abs(tr.period("centre") - 3600) < 1e-6, "1e-6 s", "watchmaking definition"))
    out.append(_row("K3", "kinematics", "Hour-hand period", "T = 2π/|ω_h|", "43200 s", f"{tr.period('hour'):.9g} s",
                    abs(tr.period("hour") - 43200) < 1e-5, "1e-5 s", "watchmaking definition"))
    out.append(_row("K4", "kinematics", "Escape-wheel speed", "ω_e = 2π f / Z_e", "12 rpm", f"{abs(tr.rpm('escape')):.6g} rpm",
                    abs(abs(tr.rpm("escape")) - 12) < 1e-9, "exact", "21,600 vph (measured) with 15 teeth"))
    out.append(_row("K5", "kinematics", "Hands turn clockwise", "sign(ω) < 0 about +z", "all CW",
                    ", ".join(f"{a}:{'CW' if tr.omega[a] < 0 else 'CCW'}" for a in ("seconds", "centre", "hour")),
                    all(tr.omega[a] < 0 for a in ("seconds", "centre", "hour")), "", "external meshes reverse sense"))
    st = R.get("events")
    ss_rows = R.get("positional", {}).get("full", [])
    # ---------------------------------------------------------------- oscillator
    T_free = 2 * math.pi * math.sqrt(m.balance.inertia / m.balance.k)
    out.append(_row("O1", "oscillator", "Free balance period", "T = 2π √(I/k)", "1/3 s (3 Hz)", f"{T_free:.9g} s",
                    abs(T_free - 1 / 3) < 1e-9, "1e-9 s", "sourced frequency; k sized from I"))
    T_full = float(m.spring.torque_out(m.spring.n_dev))
    ss = sim.steady_state_fast(T_full, "DU", measure_periods=40)
    bph = ss.get("beats_per_hour", float("nan"))
    out.append(_row("O2", "oscillator", "Simulated beat rate (dial up, full wind)", "beats/h = 3600 / mean beat interval",
                    "21,600 vph (±0.1 %)", f"{bph:.1f} vph", abs(bph - 21600) / 21600 < 1e-3, "0.1 %", "owner measurement + Miyota spec"))
    led = ss["ledger"]
    rel = abs(led["closure_error"]) / led["escape_wheel_input"]
    out.append(_row("E1", "energy", "Energy ledger closure (40 periods)",
                    "W_escape = Σ losses + ΔE_mech", "0", f"{rel:.2e} (relative)", rel < 1e-9, "1e-9", "conservation of energy"))
    p_kin = m.losses.forward(T_full)[-1]["P_out"]
    p_sim = ss["power_escape_wheel"]
    out.append(_row("E2", "energy", "Torque transmission barrel → escape wheel", "P_e = T_e ω_e, T_e = T_b Πη / i − drags",
                    f"{p_kin * 1e6:.4f} µW", f"{p_sim * 1e6:.4f} µW", abs(p_sim / p_kin - 1) < 0.02, "2 %",
                    "kinematic train model vs measured work on the wheel (difference = recoil back-drive)"))
    # escapement geometry
    e = m.esc
    out.append(_row("X1", "escapement", "Per-beat wheel advance", "−ψ_rec + ψ_I + ψ_D = π / Z", f"{math.degrees(math.pi / e.z):.6g}°",
                    f"{math.degrees(-e.psi_rec + e.psi_I + e.psi_D):.6g}°", abs(-e.psi_rec + e.psi_I + e.psi_D - math.pi / e.z) < 1e-12,
                    "exact", "kinematic closure of the escapement"))
    if st:
        ev = {r["event"]: r for r in st}
        lift_sim = ev["t_exit"]["balance_deg"] - ev["t_entry"]["balance_deg"]
        out.append(_row("X2", "escapement", "Balance lift angle (pin entry → exit)", "λ = θ_exit − θ_entry",
                        "49°", f"{lift_sim:.4f}°", abs(lift_sim - 49) < 1e-3, "0.001°", "Miyota 82S spec sheet"))
    out.append(_row("X3", "escapement", "Draw exceeds friction angle (fork pulled to banking)", "δ > atan μ",
                    f"> {math.degrees(math.atan(e.mu)):.2f}°", f"{math.degrees(e.delta):.2f}°", e.delta > math.atan(e.mu), "",
                    "lever-escapement safety rule"))
    # timekeeping vs spec
    pos = R.get("positional", {})
    if ss_rows:
        r_du = next(r for r in ss_rows if r["position"] == "DU")["rate_s_per_day"]
        out.append(_row("T1", "timekeeping", "Daily rate, dial up, full wind", "rate = 86400 (f/f₀ − 1)",
                        "−20 … +40 s/day", f"{r_du:+.2f} s/day", -20 <= r_du <= 40, "spec band", "Miyota 82S0 spec (regulator at neutral)"))
        pd = pos["posture_difference_full"]
        out.append(_row("T2", "timekeeping", "Posture difference (DU, CD, CL, CR)", "max − min rate", "< 50 s/day",
                        f"{pd:.2f} s/day", pd < 50, "spec", "Miyota 82S0 spec"))
        a_du = next(r for r in ss_rows if r["position"] == "DU")["amplitude_deg"]
        a_v = np.mean([r["amplitude_deg"] for r in ss_rows if r["position"] in ("CU", "CD", "CL", "CR")])
        out.append(_row("T3", "timekeeping", "Amplitude, dial up, full wind", "limit cycle of the hybrid model",
                        "250–310° (typical healthy watch)", f"{a_du:.1f}°", 250 <= a_du <= 310, "band", "general horology - not caliber-specific",
                        status="PASS" if 250 <= a_du <= 310 else "FAIL"))
        out.append(_row("T4", "timekeeping", "Horizontal-to-vertical amplitude drop", "A_DU − mean(A_vertical)",
                        "20–60° (typical)", f"{a_du - a_v:.1f}°", 20 <= a_du - a_v <= 60, "band", "general horology"))
    un = R.get("unpoise")
    if un:
        from scipy.special import j1
        A = np.radians(un["sim"]["amplitude_deg"])
        th = 86400 * (un["U"] * math.cos(un["phase"]) / un["k"]) * j1(A) / A
        err = float(np.max(np.abs(un["sim"]["unpoise_rate_effect"] - th)))
        out.append(_row("T5", "timekeeping", "Out-of-poise error vs averaging theory (12 amplitudes)",
                        "Δrate = 86400 (U cos γ′ / k) J₁(A)/A", "theory", f"max |Δ| = {err:.3f} s/day", err < 0.1, "0.1 s/day",
                        "Krylov–Bogoliubov averaging (independent of the simulator)"))
    if "timegrapher_amplitude_deg" in ss:
        out.append(_row("T6", "timekeeping", "Virtual timegrapher amplitude (49° lift, entry→lock)",
                        "A = λ / (2 sin(π t_lift / T))", f"true {ss['amplitude_deg']:.1f}°", f"{ss['timegrapher_amplitude_deg']:.1f}°",
                        True, "", "shows the bias of timegrapher amplitude when the lock sound is used as end of lift", status="INFO"))
    # power reserve
    rv = R.get("reserve", {})
    if "DU" in rv:
        res_h = rv["DU"]["reserve_h"]
        out.append(_row("PR1", "power reserve", "Predicted reserve, dial up", "t = (n_dev − n_stop) · 2π / ω_barrel",
                        "42 h (Miyota) / 40 h (Timex)", f"{res_h:.2f} h", True, "", "CALIBRATED via mainspring development factor (blind: 47.7 h)", status="INFO"))
        vert = [rv[p]["reserve_h"] for p in rv if p in ("CU", "CD", "CL", "CR")]
        if vert:
            out.append(_row("PR2", "power reserve", "Reserve in vertical positions (not calibrated)", "same, vertical friction",
                            "≤ dial-up reserve", f"{min(vert):.2f}–{max(vert):.2f} h", max(vert) <= res_h + 1e-6, "", "physics: more friction stops earlier"))
    bf = R.get("bruteforce")
    if bf is not None and "qs_t" in bf:
        rel = abs(bf["qs_reserve_h"] - bf["reserve_h"]) / bf["reserve_h"]
        out.append(_row("PR3", "power reserve", "Reserve: multi-time-scale model vs full event-by-event integration",
                        "|t_qs − t_brute| / t_brute", "< 1 %", f"{bf['qs_reserve_h']:.2f} h vs {bf['reserve_h']:.2f} h ({100 * rel:.2f} %)",
                        rel < 0.01, "1 %", "independent brute-force run (~10⁶ escapement events, blind mainspring)"))
        qs_A = np.interp(bf["t_h"], bf["qs_t"], bf["qs_A"])
        qs_r = np.interp(bf["t_h"], bf["qs_t"], bf["qs_rate"])
        mask = bf["t_h"] < 0.95 * bf["reserve_h"]
        dA = float(np.max(np.abs(bf["amplitude"][mask] - qs_A[mask])))
        dr = float(np.nanmax(np.abs(bf["rate"][mask] - qs_r[mask])))
        out.append(_row("PR4", "power reserve", "Amplitude & rate: quasi-static vs brute force (first 95 % of reserve)",
                        "max |A_b − A_qs|, max |r_b − r_qs|", "< 1°, < 0.1 s/day", f"{dA:.3f}°, {dr:.3f} s/day", dA < 1.0 and dr < 0.1,
                        "1°, 0.1 s/day", "time-scale separation holds until the final minutes of run-down"))
    # geometry
    lay = R.get("layout", {})
    if lay:
        out.append(_row("G1", "geometry", "3-D interference between parts (reference pose)", "z-band overlap ∧ plan overlap",
                        "0", f"{len(lay['interferences'])}", len(lay["interferences"]) == 0, "", "parametric CAD"))
        out.append(_row("G2", "geometry", "Minimum layout clearance", "distance − radii", "≥ 0.15 mm", f"{lay['min_clearance_mm']:.3f} mm",
                        lay["min_clearance_mm"] >= 0.15, "", "layout synthesis"))
    h = P.band("cad.plate")[1] - P.band("cad.rotor")[0]
    out.append(_row("G3", "geometry", "Movement height of CAD stack", "z_plate − z_rotor", f"{P['movement.height'] * 1e3:.2f} mm",
                    f"{h * 1e3:.2f} mm", abs(h - P["movement.height"]) < 1e-9, "exact", "Miyota spec"))
    ct = m.spring.n_dev / m.keyless.arbor_turns_per_crown_turn
    out.append(_row("G4", "geometry", "Crown turns from let-down to full wind", "N = n_dev z_ratchet / z_wp", "≤ 40",
                    f"{ct:.1f}", ct <= 40, "", "Miyota spec measurement condition"))
    out.append(_row("G5", "geometry", "Hairspring length plausibility", "L = E h t³/(12 k); N = L / (2π r_mean)",
                    "10–16 turns", f"{m.balance.hs_turns:.1f} turns", 10 <= m.balance.hs_turns <= 16, "", "typical flat hairspring"))
    mw = (P.int("motion_works.cannon_pinion") + P.int("motion_works.minute_wheel"),
          P.int("motion_works.minute_pinion") + P.int("motion_works.hour_wheel"))
    out.append(_row("G6", "geometry", "Coaxial motion-works constraint", "z_cp + Z_mw = z_mp + Z_hw", "equal", f"{mw[0]} / {mw[1]}",
                    mw[0] == mw[1], "", "single module, two coaxial meshes"))
    # test-suite
    if run_pytest:
        try:
            cp = subprocess.run([sys.executable, "-m", "pytest", "-q", str(ROOT / "tests")], capture_output=True, text=True,
                                timeout=1800, cwd=str(ROOT))
            last = [ln for ln in cp.stdout.strip().splitlines() if "passed" in ln or "failed" in ln]
            summ = last[-1] if last else cp.stdout[-200:]
            out.append(_row("S1", "software", "Automated test-suite", "pytest", "all pass", summ, cp.returncode == 0, "", "tests/"))
        except Exception as exc:  # pragma: no cover
            out.append(_row("S1", "software", "Automated test-suite", "pytest", "all pass", str(exc), False))
    write_report(out)
    return out


def write_report(rows: list[dict]) -> None:
    n_pass = sum(r["status"] == "PASS" for r in rows)
    n_fail = sum(r["status"] == "FAIL" for r in rows)
    n_info = sum(r["status"] == "INFO" for r in rows)
    lines = ["# Validation report", "",
             "_Generated by `python -m twin run-all` (twin.analysis.validation). Do not edit by hand._", "",
             f"**{n_pass} PASS · {n_fail} FAIL · {n_info} INFO** (INFO = reference is calibrated or is itself an estimate)", "",
             "| ID | Category | Check | Governing relation | Expected | Model | Tol. | Status | Basis |",
             "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        lines.append(f"| {r['id']} | {r['category']} | {r['check']} | `{r['equation']}` | {r['expected']} | {r['value']} | "
                     f"{r['tolerance']} | **{r['status']}** | {r['basis']} |")
    (ROOT / "docs" / "04_validation_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
