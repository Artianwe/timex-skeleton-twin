"""CSV / JSON export helpers (plain stdlib + numpy, no pandas dependency)."""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np


def _clean(v):
    if isinstance(v, (np.floating, float)):
        f = float(v)
        return None if not math.isfinite(f) else f
    if isinstance(v, (np.integer,)):
        return int(v)
    if isinstance(v, np.ndarray):
        return [_clean(x) for x in v.tolist()]
    if isinstance(v, dict):
        return {str(k): _clean(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_clean(x) for x in v]
    if isinstance(v, (bool, int, str)) or v is None:
        return v
    return str(v)


def write_json(obj, path: Path, compact: bool = False) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_clean(obj), indent=None if compact else 1, separators=(",", ":") if compact else None))
    return path


def write_csv(rows: list[dict], path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("")
        return path
    keys = list(rows[0].keys())
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: _clean(r.get(k)) for k in keys})
    return path


def write_columns(cols: dict, path: Path) -> Path:
    """Dict of equal-length arrays -> CSV."""
    keys = list(cols.keys())
    n = len(cols[keys[0]])
    rows = [{k: cols[k][i] for k in keys} for i in range(n)]
    return write_csv(rows, path)
