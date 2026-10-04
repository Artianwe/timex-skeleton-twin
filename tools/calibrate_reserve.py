"""Calibrate ONE assumed parameter so the predicted dial-up power reserve equals the sourced spec.

Blind prediction (all mainspring parameters assumed): see outputs/validation/calibration.json.
The usable-development factor k_dev (assumed range 0.75-0.92) is the least-constrained input
that sets the reserve, so it is the calibration knob. Everything else stays untouched. The
result is written to config/watch.yaml with provenance 'inferred' and a note.

Author: Anwesh Ajitabh Dash
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from twin.dynamics.power_reserve import find_stop  # noqa: E402
from twin.dynamics.simulator import Simulator  # noqa: E402
from twin.model import build_model  # noqa: E402
from twin.params import load_params  # noqa: E402


def reserve_h(P) -> float:
    m = build_model(P)
    s = Simulator(m)
    n_stop = find_stop(s, "DU")
    return (m.spring.n_dev - n_stop) * 2 * 3.141592653589793 / m.barrel_omega / 3600.0


def main(target_h: float | None = None, write: bool = True) -> dict:
    P0 = load_params(ROOT / "outputs" / "validation" / "watch_uncalibrated.yaml") \
        if (ROOT / "outputs" / "validation" / "watch_uncalibrated.yaml").exists() else load_params()
    target_h = target_h or P0["movement.power_reserve"] / 3600.0
    blind_k = P0.info("mainspring.development_factor").raw
    blind = reserve_h(P0)
    lo, hi = 0.5, 1.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if reserve_h(P0.with_overrides({"mainspring.development_factor": mid})) > target_h:
            hi = mid
        else:
            lo = mid
    k = round(0.5 * (lo + hi), 4)
    final = reserve_h(P0.with_overrides({"mainspring.development_factor": k}))
    res = {"target_h": target_h, "blind_development_factor": blind_k, "blind_reserve_h": blind,
           "blind_error_pct": 100 * (blind - target_h) / target_h,
           "calibrated_development_factor": k, "calibrated_reserve_h": final}
    out = ROOT / "outputs" / "validation"
    out.mkdir(parents=True, exist_ok=True)
    (out / "calibration.json").write_text(json.dumps(res, indent=2))
    if write:
        cfg = ROOT / "config" / "watch.yaml"
        txt = cfg.read_text()
        new = (f'  development_factor:  {{value: {k}, unit: "1", prov: inferred, src: "CALIBRATED: usable fraction of '
               f'theoretical turns set so DU power reserve = {target_h:.0f} h (Miyota spec). Blind estimate '
               f'{blind_k} predicted {blind:.1f} h ({res["blind_error_pct"]:+.1f}%)", range: [0.70, 0.92]}}')
        txt = re.sub(r"  development_factor:.*\n", new + "\n", txt, count=1)
        cfg.write_text(txt)
    print(json.dumps(res, indent=2))
    return res


if __name__ == "__main__":
    main()
