"""Energy-flow accounting from mainspring to balance.

Per unit time at a steady operating point:
    P_barrel = T_b * omega_b                               (mainspring output)
    P_escape = P_barrel - sum(train stage losses)          (= measured work on escape wheel)
    P_escape = escapement losses + balance losses          (steady state, Delta E = 0)
The escapement losses (unlocking friction, impulse-plane friction, impulse-pin friction, drop
impact, catch-up impact, pin-entry and banking impacts) come from the hybrid simulation ledger;
the balance losses (air, hairspring internal, pivot oil, pivot Coulomb) likewise.

Efficiencies:
    eta_train      = P_escape / P_barrel
    eta_escapement = (energy delivered to balance per beat) / (energy from wheel per beat)
    eta_total      = P_balance_dissipated / P_barrel        ('useful' energy that sustains
                                                            the oscillator)
"""
from __future__ import annotations

from ..model import WatchModel

ESC_KEYS = ["unlocking_friction", "impulse_plane_friction", "impulse_pin_friction", "impact_drop_lock",
            "impact_wheel_catch_up", "impact_pin_entry", "impact_fork_banking", "impact_knocking",
            "impact_failed_unlock"]
BAL_KEYS = ["balance_air", "balance_hairspring_internal", "balance_pivot_oil", "balance_pivot_coulomb"]


def energy_flow(model: WatchModel, steady: dict) -> dict:
    """Power flow (W) through every stage at a steady operating point."""
    led = steady["ledger"]
    dur = steady["duration"]
    T_b = steady["T_barrel"]
    stages = model.losses.forward(T_b)
    P_barrel = stages[0]["P_in"]
    P_esc_kin = stages[-1]["P_out"]
    P_esc_sim = led["escape_wheel_input"] / dur
    train = [{"stage": s["stage"], "P_loss": s["P_loss"]} for s in stages]
    esc = {k: led[k] / dur for k in ESC_KEYS}
    bal = {k: led[k] / dur for k in BAL_KEYS}
    P_bal = sum(bal.values())
    P_esc_loss = sum(esc.values())
    return {
        "P_barrel": P_barrel, "P_escape_wheel": P_esc_sim, "P_escape_wheel_kinematic": P_esc_kin,
        "train_losses": train, "escapement_losses": esc, "balance_losses": bal,
        "P_balance": P_bal, "P_escapement_loss": P_esc_loss,
        "eta_train": P_esc_kin / P_barrel,
        "eta_escapement": P_bal / P_esc_sim if P_esc_sim > 0 else 0.0,
        "eta_total": P_bal / P_barrel,
        "closure_W": P_esc_sim - P_esc_loss - P_bal - led["delta_mechanical_energy"] / dur,
    }


def sankey_links(flow: dict) -> list[tuple[str, str, float]]:
    """(source, target, power uW) links for a Sankey diagram."""
    L = []
    L.append(("Mainspring", "Going train", flow["P_barrel"] * 1e6))
    for s in flow["train_losses"]:
        L.append(("Going train", f"Loss: {s['stage']}", s["P_loss"] * 1e6))
    L.append(("Going train", "Escape wheel", flow["P_escape_wheel_kinematic"] * 1e6))
    for k, v in flow["escapement_losses"].items():
        if v > 0:
            L.append(("Escape wheel", f"Loss: {k.replace('_', ' ')}", v * 1e6))
    L.append(("Escape wheel", "Balance", flow["P_balance"] * 1e6))
    for k, v in flow["balance_losses"].items():
        L.append(("Balance", f"Loss: {k.replace('_', ' ')}", v * 1e6))
    return L
