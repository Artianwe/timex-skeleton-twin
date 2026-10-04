"""End-to-end pipeline: model -> CAD -> kinematics -> dynamics -> analyses -> figures -> web data.

    python -m twin run-all            # everything except the long studies
    python -m twin sensitivity        # OAT study (parallel)
    python -m twin montecarlo         # uncertainty propagation (parallel)
    python -m twin experiment --set balance.inertia_scale=1.01 --set damping.q_air=400

Outputs go to outputs/ (figures, CSV, JSON, CAD) and web/data/ (viewer + dashboard bundle).
"""
from __future__ import annotations

import math
import pickle
import time
from pathlib import Path

import numpy as np

from .analysis.energy import energy_flow, sankey_links
from .analysis.kinematics_series import escapement_event_table, wheel_series
from .analysis.positional import POSITIONS, positional_table, posture_difference, unpoise_amplitude_sweep
from .analysis.timekeeping import error_decomposition, isochronism_curve
from .dynamics import hybrid as H
from .dynamics.autowinding import PROFILES, daily_scenario, winding_rate
from .dynamics.power_reserve import reserve_curve, reserve_map
from .dynamics.simulator import Simulator
from .export import plots
from .export.drawings import plan_view, section_view
from .export.results import write_columns, write_csv, write_json
from .geometry.assembly import build_parts, part_table
from .geometry.cad import export_glb, export_step, export_web
from .geometry.checks import interference_report
from .geometry.layout import solve_layout
from .model import WatchModel, build_model
from .params import ROOT, load_params

OUT = ROOT / "outputs"
WEB = ROOT / "web" / "data"


def log(msg: str):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


# ------------------------------------------------------------------------------------------
def run_cad(m: WatchModel, step: bool = True) -> dict:
    L = solve_layout(m)
    parts = build_parts(m, L)
    rep = interference_report(parts)
    cad = OUT / "cad"
    web_geo = export_web(parts, WEB / "parts.json")
    n_tri = export_glb(parts, cad / "movement_82S0.glb")
    if step:
        export_step(parts, cad / "movement_82S0.step")
    write_csv(part_table(parts), cad / "component_table.csv")
    lab = {k.capitalize(): L.pos[k] for k in ("barrel", "centre", "third", "fourth", "escape", "pallet", "balance")}
    plan_view(parts, OUT / "figures" / "00a_cad_plan_view.png", labels=lab)
    plan_view(parts, OUT / "figures" / "00b_cad_train_escapement.png",
              hide_groups=("case", "dial", "hands", "automatic winding", "structure", "jewels", "keyless", "motion works"),
              title="Going train, escapement and oscillator (structure hidden)")
    section_view(parts, OUT / "figures" / "00c_cad_axial_stack.png")
    layout = {"positions_mm": {k: [float(v[0] * 1e3), float(v[1] * 1e3)] for k, v in L.pos.items()},
              "angles_deg": {k: math.degrees(v) % 360 for k, v in L.angles.items()},
              "min_clearance_mm": L.min_clearance * 1e3, "interferences": rep, "triangles": n_tri}
    write_json(layout, OUT / "cad" / "layout.json")
    return {"layout": L, "parts": parts, "geo": web_geo, "layout_json": layout}


def run_kinematics(m: WatchModel) -> dict:
    rows = m.train.table()
    write_csv(rows, OUT / "data" / "kinematics_train_speeds.csv")
    meshes = [{"mesh": x.name, "z_driver": x.z_driver, "z_driven": x.z_driven, "module_mm": x.module * 1e3,
               "centre_distance_mm": x.centre_distance * 1e3, "speed_ratio": x.ratio} for x in m.train.meshes]
    write_csv(meshes, OUT / "data" / "kinematics_meshes.csv")
    return {"speeds": rows, "meshes": meshes, "hands": m.train.hands()}


def run_reference_cycle(m: WatchModel, sim: Simulator, T: float, position: str = "DU") -> dict:
    """High-resolution limit cycle (2 periods) for the escapement study and the 3-D viewer."""
    ss = sim.steady_state_fast(T, position, measure_periods=4, record_dt=1e-5)
    res = ss["result"]
    ser = wheel_series(m, res)
    ev = escapement_event_table(res)
    write_csv(ev, OUT / "data" / "escapement_events_one_beat.csv")
    keep = ["t", "balance_angle", "balance_velocity", "balance_acceleration", "fork_angle", "escape_angle",
            "escape_velocity", "fourth_angle", "third_angle", "centre_angle", "seconds_angle", "mode"]
    dec = slice(None, None, 5)
    write_columns({k: ser[k][dec] for k in keep}, OUT / "data" / "kinematics_timeseries_2_periods.csv")
    b = res.good_beats[-2]
    beat = {"t_entry": b[H.B_TENTRY], "t_unlock": b[H.B_TUNL], "t_catch": b[H.B_TCATCH], "t_letoff": b[H.B_TLETOFF],
            "t_lock": b[H.B_TLOCK], "t_exit": b[H.B_TEXIT]}
    return {"steady": ss, "series": ser, "events": ev, "beat": beat}


def cycle_for_viewer(sim: Simulator, T: float, position: str = "DU") -> dict:
    """One full period (two beats) starting at a balance extreme, adaptively decimated."""
    p = sim.params_vector(T, position)
    A = sim.limit_cycle(T, position, p=p)
    if A is None:
        return {}
    f = sim.m.balance.f_design
    r = sim.run(T, position, 1.0 / f + 0.004, amplitude0=A, p=p, record_dt=1e-5)
    rec = r.rec
    t = rec[:, 0]
    mode = rec[:, 6]
    turn_t = r.turns[1, 0] if len(r.turns) > 1 else 1.0 / f
    m = t <= turn_t
    rec, t, mode = rec[m], t[m], mode[m]
    near = np.zeros(len(t), bool)
    active = (mode != 0) | (rec[:, 7] == 1)
    idx = np.where(active)[0]
    for i in idx:
        near[max(i - 20, 0):i + 20] = True
    keep = near | (np.arange(len(t)) % 25 == 0)
    keep[-1] = True
    rec = rec[keep]
    beats = [{k: float(bb[c] - r.state0.t) for k, c in (("t_entry", H.B_TENTRY), ("t_unlock", H.B_TUNL), ("t_catch", H.B_TCATCH),
                                                       ("t_letoff", H.B_TLETOFF), ("t_lock", H.B_TLOCK), ("t_exit", H.B_TEXIT))}
             for bb in r.good_beats]
    return {"T_barrel": T, "amplitude_deg": math.degrees(A), "period": float(turn_t), "dpsi": float(r.state.y[H.S_PSI]),
            "t": np.round(rec[:, 0], 7), "theta": np.round(rec[:, 1], 6), "phi": np.round(rec[:, 3], 6),
            "psi": np.round(rec[:, 4], 6), "mode": rec[:, 6].astype(int), "beats": beats}


def run_reserve(sim: Simulator, positions=POSITIONS, n_points: int = 24) -> dict:
    maps, curves = {}, {}
    for pos in positions:
        log(f"  reserve map {pos}")
        rm = reserve_map(sim, pos, n_points=n_points)
        maps[pos] = rm
        curves[pos] = reserve_curve(sim, rm)
        c = curves[pos]
        write_columns({"t_h": c.t_h, "wound_turns": c.n, "barrel_torque_Nm": c.T_barrel, "amplitude_deg": c.amplitude,
                       "rate_s_day": c.rate, "beat_error_ms": c.beat_error, "power_barrel_W": c.power_barrel,
                       "power_escape_W": c.power_escape, "power_balance_W": c.power_balance,
                       "energy_J": c.energy_remaining, "cumulative_error_s": c.cumulative_error_s},
                      OUT / "data" / f"power_reserve_{pos}.csv")
    return {"maps": maps, "curves": curves}


def run_all(step: bool = True, positions=POSITIONS) -> dict:
    t0 = time.time()
    P = load_params()
    m = build_model(P)
    sim = Simulator(m)
    T_full = float(m.spring.torque_out(m.spring.n_dev))
    figs = OUT / "figures"
    R: dict = {"meta": P.meta, "identification": P.identification}

    log("parameter tables")
    from .analysis.validation import parameter_rows
    R["parameters"] = parameter_rows(P)
    R["derived"] = m.reg.rows()

    log("CAD geometry + exports")
    cad = run_cad(m, step=step)
    R["layout"] = cad["layout_json"]

    log("kinematics")
    R["kinematics"] = run_kinematics(m)

    log("reference escapement cycle")
    ref = run_reference_cycle(m, sim, T_full)
    R["events"] = ref["events"]
    plots.escapement_cycle_figure(ref["series"], ref["beat"], figs, m)

    log("power reserve maps")
    rv = run_reserve(sim, positions)
    curves = rv["curves"]
    R["reserve"] = {pos: {"t_h": c.t_h, "n": c.n, "T": c.T_barrel, "A": c.amplitude, "rate": c.rate, "be": c.beat_error,
                          "P_barrel": c.power_barrel, "P_escape": c.power_escape, "P_balance": c.power_balance,
                          "E": c.energy_remaining, "cum_err": c.cumulative_error_s, "reserve_h": c.reserve_h,
                          "losses": c.loss_powers} for pos, c in curves.items()}
    plots.reserve_figures(curves, figs)

    log("energy flow")
    flow_full = energy_flow(m, ref["steady"])
    R["energy_flow_full"] = flow_full
    R["sankey_full"] = sankey_links(flow_full)
    plots.losses_figure(flow_full, figs)
    stage_rows = m.losses.forward(T_full)
    R["stage_torques"] = stage_rows
    plots.stage_torque_figure(stage_rows, figs)

    log("positional analysis")
    pos_full = positional_table(sim, T_full)
    n24 = m.spring.n_dev - 24.0 * 3600.0 * m.barrel_omega / (2 * math.pi)
    pos_24 = positional_table(sim, float(m.spring.torque_out(n24)))
    R["positional"] = {"full": pos_full, "after_24h": pos_24, "posture_difference_full": posture_difference(pos_full),
                       "posture_difference_24h": posture_difference(pos_24)}
    write_csv(pos_full, OUT / "data" / "positional_full_wind.csv")
    write_csv(pos_24, OUT / "data" / "positional_after_24h.csv")
    plots.positional_figure(pos_full, figs)

    log("isochronism + unpoise + error decomposition")
    iso = {pos: isochronism_curve(sim, pos) for pos in ("DU", "CD")}
    R["isochronism"] = iso
    plots.isochronism_figure(iso, figs)
    sw = unpoise_amplitude_sweep(sim, "CD")
    from scipy.special import j1
    U, k = m.balance.unpoise, m.balance.k
    gp = sim.params_vector(T_full, "CD")[H.P_UPH]
    R["unpoise"] = {"sim": sw, "U": U, "k": k, "phase": gp}
    plots.unpoise_figure(sw, lambda A: 86400 * (U * math.cos(gp) / k) * j1(A) / A, figs)
    R["error_decomposition"] = {pos: error_decomposition(sim, T_full, pos) for pos in ("DU", "CD", "CL")}

    log("automatic winding")
    rates = [winding_rate(m, prof, duration=180.0) for prof in PROFILES]
    scen = daily_scenario(m)
    R["autowinding"] = {"rates": rates, "scenario": scen}
    plots.autowinding_figure(rates, scen, figs)

    R["viewer"] = {
        "n_dev": m.spring.n_dev, "T_rev_s": 2 * math.pi / m.barrel_omega, "f": m.balance.f_design,
        "phi_B": m.esc.phi_B, "lift": m.esc.lift, "z_escape": m.esc.z, "psi_half": m.esc.psi_half,
        "n_stop": {pos: float(rv["maps"][pos].n_stop) for pos in rv["maps"]},
        "omega": m.train.omega, "spring": {"R": m.spring.R, "r": m.spring.r, "e": m.spring.e, "h": m.spring.h,
                                            "L": m.spring.length, "n_dev": m.spring.n_dev},
        "hairspring": {"r_in": P["hairspring.inner_radius"], "r_out": P["hairspring.outer_radius"], "turns": m.balance.hs_turns,
                       "z": list(P.band("cad.hairspring"))},
        "barrel_z": list(P.band("cad.barrel")), "beat_offset": m.balance.beat_offset,
        "M_full": float(m.spring.torque_out(m.spring.n_dev)), "keyless_ratio": m.keyless.arbor_turns_per_crown_turn,
        "rotor_R": float(m.P.int("autowinding.reversing_wheel") / m.P.int("autowinding.rotor_pinion") *
                         m.P.int("autowinding.reduction_wheel") / m.P.int("autowinding.reversing_pinion") *
                         m.P.int("keyless.ratchet_wheel") / m.P.int("autowinding.reduction_pinion")),
    }

    log("viewer cycles")
    cycles = []
    for frac in (1.0, 0.8, 0.6, 0.4, 0.2, 0.06):
        n = m.spring.n_dev * frac
        c = cycle_for_viewer(sim, float(m.spring.torque_out(n)))
        if c:
            c["wound_fraction"] = frac
            cycles.append(c)
    R["cycles"] = cycles

    # long studies (computed separately, optional)
    oat_p, mc_p, bf_p = OUT / "analysis" / "oat.pkl", OUT / "analysis" / "mc.pkl", OUT / "validation" / "bruteforce_DU.npz"
    if oat_p.exists():
        oat = pickle.loads(oat_p.read_bytes())
        R["oat"] = oat
        for o, lab in (("rate_DU", "rate DU [s/day]"), ("amplitude_DU", "amplitude DU [deg]"),
                       ("power_reserve_h", "power reserve [h]"), ("eta_total", "total efficiency [-]"),
                       ("posture_difference", "posture difference [s/day]")):
            plots.tornado_figure(oat, o, lab, figs)
    if mc_p.exists():
        mc = pickle.loads(mc_p.read_bytes())
        R["mc"] = {"n": mc["n"], "keys": mc["keys"], "outputs": mc["outputs"]}
        plots.mc_figure(mc, {"power_reserve_h": [40.0, 42.0], "rate_DU": [-20.0, 40.0], "posture_difference": [50.0]}, figs)
    if bf_p.exists():
        d = dict(np.load(bf_p))
        # the brute-force run used the blind (uncalibrated) mainspring: compare like with like
        import json
        cal_p = OUT / "validation" / "calibration.json"
        k_blind = json.loads(cal_p.read_text())["blind_development_factor"] if cal_p.exists() else P["mainspring.development_factor"]
        sim_b = Simulator(build_model(load_params(overrides={"mainspring.development_factor": k_blind})))
        qs = reserve_curve(sim_b, reserve_map(sim_b, "DU", n_points=24), dt_h=0.05)
        R["bruteforce"] = {"t_h": d["t_h"], "amplitude": d["amplitude"], "rate": d["rate"], "n": d["n"],
                           "qs_t": qs.t_h, "qs_A": qs.amplitude, "qs_rate": qs.rate, "qs_reserve_h": qs.reserve_h,
                           "reserve_h": float(d["t_h"][-1]), "development_factor": k_blind}
        plots.bruteforce_figure(d, qs, figs)

    log("validation report")
    from .analysis.validation import run_validation
    R["validation"] = run_validation(m, sim, R)

    log("results summary")
    from .export.report import write_results
    write_results(R)

    log("web bundle")
    web = {k: R[k] for k in R if k not in ("oat",)}
    if "oat" in R:
        web["oat"] = {"base": R["oat"]["base"], "table": R["oat"]["table"]}
    write_json(web, WEB / "results.json", compact=True)
    write_json({k: R[k] for k in ("positional", "energy_flow_full", "error_decomposition", "events", "kinematics", "validation")},
               OUT / "data" / "summary.json")
    from .export.web import build as build_web
    build_web()
    log(f"done in {time.time() - t0:.0f} s")
    return R
