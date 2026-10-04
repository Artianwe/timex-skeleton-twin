"""Validation: full hybrid integration through the entire power reserve vs quasi-static model.

Integrates ~170,000 s of watch time event-by-event (about 1 million escapement events) and
compares amplitude, rate and power reserve with the quasi-static (limit-cycle) prediction.
Writes outputs/validation/bruteforce_<position>.npz and a short JSON summary.

Usage:  python tools/validate_reserve_bruteforce.py [POSITION]
Author: Anwesh Ajitabh Dash
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from twin.dynamics.power_reserve import brute_force, reserve_curve, reserve_map  # noqa: E402
from twin.dynamics.simulator import Simulator  # noqa: E402
from twin.model import build_model  # noqa: E402
from twin.params import load_params  # noqa: E402


def main(position: str = "DU", config: str | None = None) -> dict:
    out_dir = ROOT / "outputs" / "validation"
    out_dir.mkdir(parents=True, exist_ok=True)
    P = load_params(config) if config else load_params()
    sim = Simulator(build_model(P))
    t0 = time.time()
    last = [0.0]

    def prog(t, n, met):
        if t - last[0] >= 3600.0:
            last[0] = t
            print(f"  t={t/3600:6.2f} h  n={n:6.3f}  A={met.get('amplitude_deg', 0):6.1f}  "
                  f"rate={met.get('rate_s_per_day', float('nan')):7.2f}  wall={time.time()-t0:6.0f}s", flush=True)

    bf = brute_force(sim, position, progress=prog)
    rm = reserve_map(sim, position)
    rc = reserve_curve(sim, rm)
    np.savez(out_dir / f"bruteforce_{position}.npz", **{k: v for k, v in bf.items() if isinstance(v, np.ndarray)},
             qs_t=rc.t_h, qs_A=rc.amplitude, qs_rate=rc.rate)
    A_qs = np.interp(bf["t_h"], rc.t_h, rc.amplitude)
    r_qs = np.interp(bf["t_h"], rc.t_h, rc.rate)
    mask = bf["t_h"] < 0.95 * min(bf["reserve_h"], rc.reserve_h)
    summary = {
        "position": position,
        "reserve_bruteforce_h": bf["reserve_h"],
        "reserve_quasistatic_h": rc.reserve_h,
        "reserve_rel_diff": (rc.reserve_h - bf["reserve_h"]) / bf["reserve_h"],
        "amplitude_max_abs_diff_deg": float(np.max(np.abs(bf["amplitude"][mask] - A_qs[mask]))),
        "rate_max_abs_diff_s_day": float(np.nanmax(np.abs(bf["rate"][mask] - r_qs[mask]))),
        "wall_time_s": time.time() - t0,
    }
    (out_dir / f"bruteforce_{position}.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    return summary


if __name__ == "__main__":
    main(*(sys.argv[1:2] or ["DU"]))
