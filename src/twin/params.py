"""Parameter database: loads config/watch.yaml, converts to SI, tracks provenance.

Usage
-----
>>> from twin.params import load_params
>>> P = load_params()
>>> P["balance.rim_outer_diameter"]          # SI value (m)
0.00954
>>> P.info("balance.rim_outer_diameter").prov
'measured'
>>> P2 = P.with_overrides({"balance.rim_thickness": 0.45})   # value in the config's unit

Provenance categories are defined in config/watch.yaml. Derived quantities are recorded by
twin.derived.DerivedRegistry (with their governing equations) rather than here.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator, Mapping

import yaml

from .units import UNITS, to_si

PROVENANCE = ("measured", "sourced", "inferred", "assumed")
VERIFIED = ("measured", "sourced")
ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "config" / "watch.yaml"

# sections whose leaves are not physical parameters
_NON_PARAM_SECTIONS = {"meta", "identification", "positions"}


@dataclass(frozen=True)
class Param:
    key: str
    value: float            # SI
    raw: float              # value as written in config
    unit: str
    prov: str
    src: str
    range_raw: tuple[float, float] | None = None

    @property
    def range_si(self) -> tuple[float, float] | None:
        if self.range_raw is None:
            return None
        return (to_si(self.range_raw[0], self.unit), to_si(self.range_raw[1], self.unit))

    @property
    def verified(self) -> bool:
        return self.prov in VERIFIED


def _is_leaf(node: Any) -> bool:
    return isinstance(node, Mapping) and "value" in node and "unit" in node


def _walk(node: Mapping, prefix: str = "") -> Iterator[tuple[str, Mapping]]:
    for k, v in node.items():
        key = f"{prefix}.{k}" if prefix else k
        if _is_leaf(v):
            yield key, v
        elif isinstance(v, Mapping):
            yield from _walk(v, key)


class ParamSet:
    """Immutable-ish view over the configuration with SI conversion."""

    def __init__(self, raw: dict, source: Path | None = None):
        self._raw = raw
        self.source = source
        self._params: dict[str, Param] = {}
        self._bands: dict[str, tuple] = {}
        for key, leaf in _walk({k: v for k, v in raw.items() if k not in _NON_PARAM_SECTIONS}):
            prov = leaf.get("prov", "assumed")
            if prov not in PROVENANCE:
                raise ValueError(f"{key}: provenance '{prov}' not in {PROVENANCE}")
            unit = str(leaf["unit"])
            if unit not in UNITS:
                raise KeyError(f"{key}: unknown unit '{unit}'")
            rng = leaf.get("range")
            if isinstance(leaf["value"], (list, tuple)):
                self._bands[key] = tuple(to_si(v, unit) for v in leaf["value"])
                continue
            self._params[key] = Param(
                key=key,
                value=to_si(leaf["value"], unit),
                raw=float(leaf["value"]),
                unit=unit,
                prov=prov,
                src=str(leaf.get("src", "")),
                range_raw=(float(rng[0]), float(rng[1])) if rng else None,
            )

    # ---- access -------------------------------------------------------------------
    def __getitem__(self, key: str) -> float:
        return self._params[key].value

    def __contains__(self, key: str) -> bool:
        return key in self._params

    def band(self, key: str) -> tuple:
        """Vector-valued geometric parameter (e.g. a z-band), SI."""
        return self._bands[key]

    def info(self, key: str) -> Param:
        return self._params[key]

    def int(self, key: str) -> int:
        return int(round(self._params[key].value))

    def keys(self):
        return self._params.keys()

    def items(self):
        return self._params.items()

    @property
    def meta(self) -> dict:
        return dict(self._raw.get("meta", {}))

    @property
    def identification(self) -> dict:
        return dict(self._raw.get("identification", {}))

    @property
    def positions(self) -> dict:
        return dict(self._raw.get("positions", {}))

    def by_provenance(self) -> dict[str, list[Param]]:
        out: dict[str, list[Param]] = {p: [] for p in PROVENANCE}
        for prm in self._params.values():
            out[prm.prov].append(prm)
        return out

    def ranged(self) -> list[Param]:
        """Parameters that carry an uncertainty range (candidates for sensitivity studies)."""
        return [p for p in self._params.values() if p.range_raw is not None]

    # ---- modification ---------------------------------------------------------------
    def with_overrides(self, overrides: Mapping[str, float] | None = None, *,
                       si: bool = False) -> "ParamSet":
        """Return a new ParamSet with selected values replaced.

        overrides: {dotted_key: value}. Values are in the config unit unless si=True.
        """
        if not overrides:
            return self
        raw = copy.deepcopy(self._raw)
        for key, val in overrides.items():
            node = raw
            parts = key.split(".")
            for p in parts[:-1]:
                node = node[p]
            leaf = node[parts[-1]]
            if not _is_leaf(leaf):
                raise KeyError(f"{key} is not a parameter leaf")
            if si:
                val = float(val) / UNITS[leaf["unit"]][0]
            leaf["value"] = float(val)
        return ParamSet(raw, self.source)

    def scaled(self, key: str, factor: float) -> "ParamSet":
        return self.with_overrides({key: self.info(key).raw * factor})

    def to_rows(self) -> list[dict]:
        rows = []
        for p in self._params.values():
            rows.append({"key": p.key, "value": p.raw, "unit": p.unit, "prov": p.prov,
                         "range": p.range_raw, "src": p.src})
        return rows


def load_params(path: str | Path | None = None,
                overrides: Mapping[str, float] | None = None) -> ParamSet:
    path = Path(path) if path else DEFAULT_CONFIG
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    ps = ParamSet(raw, path)
    return ps.with_overrides(overrides) if overrides else ps
