"""Command-line interface.

    python -m twin run-all [--no-step]          full pipeline (figures, data, CAD, web bundle, report)
    python -m twin sensitivity [--workers 2]    one-at-a-time sensitivity study
    python -m twin montecarlo [--n 96]          uncertainty propagation
    python -m twin bruteforce [POSITION]        full event-by-event power-reserve integration
    python -m twin experiment --set KEY=VALUE   rerun key outputs with modified parameters
    python -m twin params                       regenerate docs/02_parameter_database.md
"""
from __future__ import annotations

import argparse
import pickle
import sys

from .params import ROOT


def main(argv=None):
    ap = argparse.ArgumentParser(prog="twin")
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("run-all")
    a.add_argument("--no-step", action="store_true", help="skip the (slow) STEP export")
    s = sub.add_parser("sensitivity")
    s.add_argument("--workers", type=int, default=2)
    mc = sub.add_parser("montecarlo")
    mc.add_argument("--n", type=int, default=96)
    mc.add_argument("--workers", type=int, default=2)
    bf = sub.add_parser("bruteforce")
    bf.add_argument("position", nargs="?", default="DU")
    ex = sub.add_parser("experiment")
    ex.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="parameter override in config units, e.g. balance.inertia_scale=1.01")
    ex.add_argument("--no-reserve", action="store_true")
    sub.add_parser("params")
    args = ap.parse_args(argv)

    if args.cmd == "run-all":
        from .pipeline import run_all
        run_all(step=not args.no_step)
    elif args.cmd == "sensitivity":
        from .analysis.sensitivity import oat
        r = oat(workers=args.workers)
        (ROOT / "outputs" / "analysis").mkdir(parents=True, exist_ok=True)
        (ROOT / "outputs" / "analysis" / "oat.pkl").write_bytes(pickle.dumps(r))
        print("saved outputs/analysis/oat.pkl")
    elif args.cmd == "montecarlo":
        from .analysis.sensitivity import monte_carlo
        r = monte_carlo(n=args.n, workers=args.workers)
        (ROOT / "outputs" / "analysis").mkdir(parents=True, exist_ok=True)
        (ROOT / "outputs" / "analysis" / "mc.pkl").write_bytes(pickle.dumps(r))
        print("saved outputs/analysis/mc.pkl")
    elif args.cmd == "bruteforce":
        sys.path.insert(0, str(ROOT / "tools"))
        from validate_reserve_bruteforce import main as bfm
        bfm(args.position)
    elif args.cmd == "experiment":
        from .analysis.sensitivity import OUTPUTS, evaluate
        ov = {}
        for kv in args.set:
            k, v = kv.split("=", 1)
            ov[k.strip()] = float(v)
        base = evaluate(None, reserve=not args.no_reserve)
        new = evaluate(ov, reserve=not args.no_reserve)
        print(f"\nExperiment: {ov}\n")
        print(f"{'output':24s} {'baseline':>12s} {'modified':>12s} {'change':>12s}")
        for o in OUTPUTS:
            if o in base:
                b, n = base[o], new.get(o, float('nan'))
                print(f"{o:24s} {b:12.4g} {n:12.4g} {n - b:+12.4g}")
    elif args.cmd == "params":
        sys.path.insert(0, str(ROOT / "tools"))
        from gen_param_docs import main as gp
        gp()
        print("docs/02_parameter_database.md regenerated")


if __name__ == "__main__":
    main()
