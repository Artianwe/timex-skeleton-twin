"""Registry of derived (inferred) quantities with their governing equations.

Every quantity computed from configuration parameters is recorded with
  * value (SI) and a display unit
  * the governing equation (plain text / LaTeX-like)
  * the configuration keys it depends on
so that documentation and the dashboard can show *why* a number has the value it has, and so
that its provenance can be traced back to measured / sourced / assumed inputs.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from .units import from_si


@dataclass
class Derived:
    name: str
    value: float
    unit: str
    equation: str
    deps: tuple[str, ...] = ()
    note: str = ""

    @property
    def display(self) -> float:
        return from_si(self.value, self.unit)


@dataclass
class DerivedRegistry:
    items: dict[str, Derived] = field(default_factory=dict)

    def add(self, name: str, value: float, unit: str, equation: str,
            deps: tuple[str, ...] | list[str] = (), note: str = "") -> float:
        self.items[name] = Derived(name, float(value), unit, equation, tuple(deps), note)
        return float(value)

    def __getitem__(self, name: str) -> float:
        return self.items[name].value

    def __contains__(self, name: str) -> bool:
        return name in self.items

    def get(self, name: str) -> Derived:
        return self.items[name]

    def rows(self) -> list[dict]:
        return [{"name": d.name, "value": d.display, "unit": d.unit, "equation": d.equation,
                 "deps": ", ".join(d.deps), "note": d.note} for d in self.items.values()]

    def provenance(self, params) -> dict[str, str]:
        """Weakest provenance among inputs: verified only if all deps are verified."""
        out = {}
        for d in self.items.values():
            provs = [params.info(k).prov for k in d.deps if k in params]
            if provs and all(p in ("measured", "sourced") for p in provs):
                out[d.name] = "inferred-from-verified"
            elif any(p == "assumed" for p in provs):
                out[d.name] = "inferred-from-assumed"
            else:
                out[d.name] = "inferred"
        return out
