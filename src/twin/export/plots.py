"""Publication figures (matplotlib).

Style: validated categorical palette (fixed slot order), 2 px lines, solid hairline grid,
recessive axes, legends for >= 2 series, no dual axes (different units -> separate panels).
"""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948"]
SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
SPEC_BAND = "#f0efec"


def _style():
    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "axes.edgecolor": AXIS, "axes.linewidth": 0.8, "axes.labelcolor": INK2, "axes.titlecolor": INK,
        "axes.titlesize": 12, "axes.titleweight": "bold", "axes.labelsize": 10,
        "xtick.color": MUTED, "ytick.color": MUTED, "xtick.labelsize": 9, "ytick.labelsize": 9,
        "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8, "grid.linestyle": "-",
        "axes.spines.top": False, "axes.spines.right": False, "lines.linewidth": 2.0,
        "lines.solid_capstyle": "round", "legend.frameon": False, "legend.fontsize": 9,
        "font.family": "DejaVu Sans", "text.color": INK,
    })


_style()


def _save(fig, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return path


def _end_label(ax, x, y, text):
    ax.annotate(text, (x, y), xytext=(6, 0), textcoords="offset points", va="center", fontsize=8.5, color=INK2)


# ------------------------------------------------------------------------- reserve family
def reserve_figures(curves: dict, out: Path) -> list[Path]:
    """curves: {position: ReserveCurve}. Produces the six required time-history plots."""
    paths = []
    du = curves["DU"]
    # 1 torque vs time (single series)
    fig, ax = plt.subplots(figsize=(7, 3.6))
    ax.plot(du.t_h, du.T_barrel * 1e3, color=SERIES[0])
    ax.set_xlabel("time since full wind [h]")
    ax.set_ylabel("barrel torque [N·mm]")
    ax.set_title("Mainspring torque vs time")
    ax.set_ylim(bottom=0)
    _end_label(ax, du.t_h[-1], du.T_barrel[-1] * 1e3, f"stop {du.reserve_h:.1f} h")
    paths.append(_save(fig, out / "01_mainspring_torque_vs_time.png"))
    # 2 stored energy
    fig, ax = plt.subplots(figsize=(7, 3.6))
    ax.plot(du.t_h, du.energy_remaining * 1e3, color=SERIES[0])
    ax.set_xlabel("time since full wind [h]")
    ax.set_ylabel("elastic energy above let-down [mJ]")
    ax.set_title("Stored mainspring energy vs time")
    ax.set_ylim(bottom=0)
    paths.append(_save(fig, out / "02_stored_energy_vs_time.png"))
    # 3 amplitude (horizontal vs vertical)
    fig, ax = plt.subplots(figsize=(7, 3.6))
    for i, (pos, lab) in enumerate((("DU", "Dial up"), ("CD", "Crown down"))):
        c = curves[pos]
        ax.plot(c.t_h, c.amplitude, color=SERIES[i], label=lab)
        _end_label(ax, c.t_h[-1], c.amplitude[-1], lab)
    ax.axhline(49 / 2, color=AXIS, lw=0.8)
    ax.set_xlabel("time since full wind [h]")
    ax.set_ylabel("balance amplitude [deg]")
    ax.set_title("Balance amplitude vs time")
    ax.legend(loc="lower left")
    paths.append(_save(fig, out / "03_balance_amplitude_vs_time.png"))
    # 4 power transmitted (one unit, three points of the flow)
    fig, ax = plt.subplots(figsize=(7, 3.6))
    ax.plot(du.t_h, du.power_barrel * 1e6, color=SERIES[0], label="mainspring → train")
    ax.plot(du.t_h, du.power_escape * 1e6, color=SERIES[1], label="train → escape wheel")
    if hasattr(du, "power_balance") and du.power_balance is not None:
        ax.plot(du.t_h, du.power_balance * 1e6, color=SERIES[2], label="escapement → balance")
    ax.set_xlabel("time since full wind [h]")
    ax.set_ylabel("power [µW]")
    ax.set_title("Power transmitted through the movement (dial up)")
    ax.set_ylim(bottom=0)
    ax.legend(loc="lower left")
    paths.append(_save(fig, out / "04_power_flow_vs_time.png"))
    # 5 rate vs time
    fig, ax = plt.subplots(figsize=(7, 3.6))
    ax.axhspan(-20, 40, color=SPEC_BAND, zorder=0)
    ax.text(0.3, 37, "Miyota spec −20…+40 s/day", fontsize=8, color=MUTED, va="top")
    for i, (pos, lab) in enumerate((("DU", "Dial up"), ("CD", "Crown down"))):
        c = curves[pos]
        m = c.t_h <= 0.97 * c.reserve_h
        ax.plot(c.t_h[m], c.rate[m], color=SERIES[i], label=lab)
    ax.set_xlabel("time since full wind [h]")
    ax.set_ylabel("rate [s/day]")
    ax.set_title("Simulated rate vs time")
    ax.legend(loc="lower left")
    paths.append(_save(fig, out / "05_rate_vs_time.png"))
    return paths


def losses_figure(flow: dict, out: Path) -> Path:
    """Power loss by component at one operating point; subsystem = colour (3 slots)."""
    rows = [(s["stage"], s["P_loss"], 0) for s in flow["train_losses"]]
    rows += [(k.replace("_", " "), v, 1) for k, v in flow["escapement_losses"].items() if v > 0]
    rows += [(k.replace("balance_", "balance ").replace("_", " "), v, 2) for k, v in flow["balance_losses"].items()]
    rows.sort(key=lambda r: r[1])
    fig, ax = plt.subplots(figsize=(8, 6.2))
    y = np.arange(len(rows))
    ax.barh(y, [r[1] * 1e9 for r in rows], height=0.62, color=[SERIES[r[2]] for r in rows])
    ax.set_yticks(y, [r[0] for r in rows], fontsize=8.5)
    for yi, r in zip(y, rows):
        ax.text(r[1] * 1e9 + 1.5, yi, f"{r[1] * 1e9:.1f}", va="center", fontsize=7.5, color=INK2)
    ax.set_xlabel("dissipated power [nW]")
    ax.set_title(f"Energy losses by component (full wind, dial up) — total efficiency {flow['eta_total'] * 100:.1f} %")
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(color=SERIES[i], label=l) for i, l in enumerate(("going train", "escapement", "balance / oscillator"))],
              loc="lower right")
    ax.grid(axis="y", visible=False)
    return _save(fig, out / "06_energy_losses_by_stage.png")


def stage_torque_figure(rows: list[dict], out: Path) -> Path:
    fig, ax = plt.subplots(figsize=(8, 3.8))
    names = ["barrel"] + [r["stage"] for r in rows]
    T = [rows[0]["T_in"]] + [r["T_out"] for r in rows]
    x = np.arange(len(names))
    ax.bar(x, np.array(T) * 1e6, width=0.55, color=SERIES[0])
    ax.set_yscale("log")
    ax.set_xticks(x, names, rotation=35, ha="right", fontsize=8)
    ax.set_ylabel("torque [µN·m] (log)")
    ax.set_title("Torque delivered to each stage (full wind)")
    ax.grid(axis="x", visible=False)
    return _save(fig, out / "07_torque_per_stage.png")


# ------------------------------------------------------------------------- escapement
def escapement_cycle_figure(series: dict, beat: dict, out: Path, model) -> Path:
    """Three stacked panels over one beat: balance angle, fork angle, escape-wheel angle."""
    t = series["t"]
    t0 = beat["t_entry"] - 0.004
    t1 = beat["t_exit"] + 0.004
    m = (t >= t0) & (t <= t1)
    tt = (t[m] - beat["t_entry"]) * 1e3
    fig, axs = plt.subplots(3, 1, figsize=(8, 6.6), sharex=True)
    spans = [("unlocking", beat["t_entry"], beat["t_unlock"]), ("wheel lag", beat["t_unlock"], beat["t_catch"]),
             ("impulse", beat["t_catch"], beat["t_letoff"]), ("drop", beat["t_letoff"], beat["t_lock"]),
             ("run to banking", beat["t_lock"], beat["t_exit"])]
    shades = ["#eef4fc", "#fbefe9", "#e8f7f1", "#fdf5e3", "#f7f6f2"]
    for ax in axs:
        for (lab, a, b), sh in zip(spans, shades):
            if b > a > 0:
                ax.axvspan((a - beat["t_entry"]) * 1e3, (b - beat["t_entry"]) * 1e3, color=sh, zorder=0)
    axs[0].plot(tt, np.degrees(series["balance_angle"][m]), color=SERIES[0])
    axs[0].set_ylabel("balance θ [deg]")
    axs[1].plot(tt, np.degrees(series["fork_angle"][m]), color=SERIES[1])
    axs[1].set_ylabel("fork φ [deg]")
    psi = np.degrees(series["escape_angle"][m])
    axs[2].plot(tt, psi - psi[0], color=SERIES[2])
    axs[2].set_ylabel("escape wheel ψ [deg]")
    axs[2].set_xlabel("time from impulse-pin entry [ms]")
    from matplotlib.patches import Patch
    handles = [Patch(color=sh, label=f"{lab} ({(b - a) * 1e3:.2f} ms)") for (lab, a, b), sh in zip(spans, shades) if b > a > 0]
    fig.legend(handles=handles, loc="lower center", ncol=3, bbox_to_anchor=(0.5, -0.06), fontsize=8.5)
    axs[0].set_title("One beat of the Swiss lever escapement (event-resolved)")
    return _save(fig, out / "08_escapement_one_beat.png")


def isochronism_figure(iso: dict, out: Path) -> Path:
    fig, ax = plt.subplots(figsize=(7, 3.8))
    for i, (pos, lab) in enumerate((("DU", "Dial up"), ("CD", "Crown down"))):
        d = iso[pos]
        o = np.argsort(d["amplitude_deg"])
        ax.plot(d["amplitude_deg"][o], d["rate"][o], color=SERIES[i], marker="o", ms=4, label=lab)
    ax.set_xlabel("balance amplitude [deg]")
    ax.set_ylabel("rate [s/day]")
    ax.set_title("Isochronism: rate vs amplitude")
    ax.legend()
    return _save(fig, out / "09_isochronism_rate_vs_amplitude.png")


def unpoise_figure(sweep: dict, theory_fn, out: Path) -> Path:
    fig, ax = plt.subplots(figsize=(7, 3.8))
    A = np.linspace(100, 330, 300)
    ax.plot(A, theory_fn(np.radians(A)), color=SERIES[0], label="averaging theory  ∝ J₁(A)/A")
    ax.plot(sweep["amplitude_deg"], sweep["unpoise_rate_effect"], "o", color=SERIES[1], ms=6,
            markeredgecolor=SURFACE, markeredgewidth=1.5, label="hybrid simulation")
    ax.axhline(0, color=AXIS, lw=0.8)
    ax.axvline(np.degrees(3.8317), color=AXIS, lw=0.8)
    ax.text(np.degrees(3.8317) + 2, ax.get_ylim()[1] * 0.85, "J₁ zero: 219.5°", fontsize=8, color=MUTED)
    ax.set_xlabel("balance amplitude [deg]")
    ax.set_ylabel("rate change from unpoise [s/day]")
    ax.set_title("Out-of-poise error, crown down: simulation vs theory")
    ax.legend()
    return _save(fig, out / "10_unpoise_vs_theory.png")


def positional_figure(rows: list[dict], out: Path) -> list[Path]:
    labels = [r["label"] for r in rows]
    x = np.arange(len(rows))
    paths = []
    fig, ax = plt.subplots(figsize=(7, 3.4))
    ax.bar(x, [r["rate_s_per_day"] for r in rows], width=0.5, color=SERIES[0])
    ax.axhline(0, color=AXIS, lw=0.8)
    for xi, r in zip(x, rows):
        v = r["rate_s_per_day"]
        ax.text(xi, v + (0.3 if v >= 0 else -0.3), f"{v:+.1f}", ha="center", va="bottom" if v >= 0 else "top", fontsize=8, color=INK2)
    ax.set_xticks(x, labels)
    ax.set_ylabel("rate [s/day]")
    ax.set_title("Rate in the six positions (full wind)")
    ax.grid(axis="x", visible=False)
    paths.append(_save(fig, out / "11a_positional_rate.png"))
    fig, ax = plt.subplots(figsize=(7, 3.4))
    ax.bar(x, [r["amplitude_deg"] for r in rows], width=0.5, color=SERIES[0])
    for xi, r in zip(x, rows):
        ax.text(xi, r["amplitude_deg"] + 3, f"{r['amplitude_deg']:.0f}°", ha="center", fontsize=8, color=INK2)
    ax.set_xticks(x, labels)
    ax.set_ylabel("amplitude [deg]")
    ax.set_title("Amplitude in the six positions (full wind)")
    ax.grid(axis="x", visible=False)
    paths.append(_save(fig, out / "11b_positional_amplitude.png"))
    return paths


def tornado_figure(oat_res: dict, output: str, label: str, out: Path, top: int = 12) -> Path:
    base = oat_res["base"][output]
    rows = [r for r in oat_res["table"] if np.isfinite(r[f"{output}_lo"]) and np.isfinite(r[f"{output}_hi"])]
    rows.sort(key=lambda r: max(abs(r[f"{output}_lo"] - base), abs(r[f"{output}_hi"] - base)), reverse=True)
    rows = rows[:top][::-1]
    fig, ax = plt.subplots(figsize=(8, 0.42 * len(rows) + 1.4))
    y = np.arange(len(rows))
    lo = [r[f"{output}_lo"] - base for r in rows]
    hi = [r[f"{output}_hi"] - base for r in rows]
    ax.barh(y + 0.17, lo, height=0.32, color=SERIES[0], label="parameter −10 %")
    ax.barh(y - 0.17, hi, height=0.32, color=SERIES[1], label="parameter +10 %")
    ax.axvline(0, color=AXIS, lw=0.8)
    ax.set_yticks(y, [r["param"] for r in rows], fontsize=8)
    ax.set_xlabel(f"change in {label}")
    ax.set_title(f"Sensitivity of {label} (baseline {base:.3g})")
    ax.legend(loc="lower right")
    ax.grid(axis="y", visible=False)
    return _save(fig, out / f"12_tornado_{output}.png")


def mc_figure(mc: dict, specs: dict, out: Path) -> Path:
    keys = [("power_reserve_h", "power reserve [h]"), ("amplitude_DU", "amplitude DU [deg]"),
            ("rate_DU", "rate DU [s/day]"), ("posture_difference", "posture difference [s/day]")]
    fig, axs = plt.subplots(2, 2, figsize=(9, 6))
    for ax, (k, lab) in zip(axs.ravel(), keys):
        v = mc["outputs"][k]
        v = v[np.isfinite(v)]
        ax.hist(v, bins=18, color=SERIES[0], edgecolor=SURFACE, linewidth=1.5)
        if k in specs:
            for s in np.atleast_1d(specs[k]):
                ax.axvline(s, color=SERIES[7], lw=1.5)
        ax.set_xlabel(lab)
        ax.set_ylabel("count")
        ax.grid(axis="x", visible=False)
        ax.set_title(f"median {np.median(v):.3g}, 5–95 % [{np.percentile(v, 5):.3g}, {np.percentile(v, 95):.3g}]", fontsize=9)
    fig.suptitle(f"Monte-Carlo propagation of estimated-parameter uncertainty (n = {mc['n']}); red = verified spec", fontsize=11)
    fig.tight_layout()
    return _save(fig, out / "13_monte_carlo_vs_specs.png")


def autowinding_figure(rates: list[dict], scenario: dict, out: Path) -> list[Path]:
    paths = []
    fig, ax = plt.subplots(figsize=(7, 3.4))
    names = [r["profile"] for r in rates]
    v = [r["turns_per_hour"] for r in rates]
    x = np.arange(len(rates))
    ax.bar(x, v, width=0.5, color=SERIES[0])
    cons = rates[0]["consumption_turns_per_hour"]
    ax.axhline(cons, color=SERIES[1], lw=2, label=f"consumption {cons:.3f} turns/h")
    ax.set_xticks(x, names, fontsize=8)
    ax.set_ylabel("barrel winding [turns/h]")
    ax.set_title("Self-winding rate by wrist activity (model)")
    ax.legend()
    ax.grid(axis="x", visible=False)
    paths.append(_save(fig, out / "14a_autowinding_rates.png"))
    fig, ax = plt.subplots(figsize=(7, 3.4))
    ax.plot(scenario["t_h"], scenario["n"] / scenario["n_dev"] * 100, color=SERIES[0])
    ax.set_xlabel("time [h]")
    ax.set_ylabel("state of wind [% of full]")
    ax.set_ylim(0, 105)
    ax.set_title("Wind state over three days of a typical schedule (7 h off-wrist nightly)")
    paths.append(_save(fig, out / "14b_autowinding_daily.png"))
    return paths


def bruteforce_figure(bf: dict, qs, out: Path) -> Path:
    """Brute-force (blind mainspring) vs quasi-static curve computed with the same parameters."""
    fig, axs = plt.subplots(2, 1, figsize=(7, 5.6), sharex=True)
    axs[0].plot(qs.t_h, qs.amplitude, color=SERIES[0], label="quasi-static (limit-cycle map)")
    axs[0].plot(bf["t_h"], bf["amplitude"], color=SERIES[1], lw=1.2, label="full hybrid integration")
    axs[0].set_ylabel("amplitude [deg]")
    axs[0].legend(loc="lower left")
    m = qs.t_h <= 0.95 * qs.reserve_h
    mb = bf["t_h"] <= 0.95 * qs.reserve_h
    axs[1].plot(qs.t_h[m], qs.rate[m], color=SERIES[0])
    axs[1].plot(bf["t_h"][mb], bf["rate"][mb], color=SERIES[1], lw=1.2)
    axs[1].set_ylabel("rate [s/day]")
    axs[1].set_xlabel("time since full wind [h]")
    axs[0].set_title("Validation: full event-by-event integration vs multi-time-scale model")
    return _save(fig, out / "15_validation_bruteforce_vs_quasistatic.png")
