"""Event-driven hybrid simulation of balance + Swiss lever escapement + escape wheel.

This is the numerical core (compiled with numba). It integrates the piecewise-smooth equations
of motion with fixed-step RK4, locates every discontinuity (contact change, impact, turning
point) by bisection to `event_tolerance`, and applies impact laws at events. All energy flows
are integrated alongside the state so the energy ledger closes to integration accuracy.

Phases of one beat (fork travel u = s*phi + phi_B measured from the starting banking):

  FREE      balance free; fork resting on its banking; escape wheel locked (or still dropping)
  UNLOCK    impulse pin drives the fork through lock+run; wheel recoils kinematically
            psi = psi_L - kappa_U * u,   locking-face friction factor f_U
  IMPULSE   wheel tooth on the impulse plane, UNILATERAL contact:
              contact:   psi = psi_c(u) = psi_L - psi_rec + kappa_I (u - u_U), lambda_c >= 0
              separated: wheel accelerates freely under T_e until it catches the face (impact)
  POST      after let-off: fork follows the roller to the banking; wheel drops psi_D and
            locks (plastic impact) on the opposite pallet
  pin exit  fork stops on the banking (its kinetic energy is lost), balance continues FREE

Generalised equation of the coupled balance+fork(+wheel) system, theta the balance angle:
    J(theta) theta'' + I_c g' g'' theta'^2 = F_bal(theta, theta') + G * T_u          (H1)
    J = I_b + I_c g'^2,   G = du/dtheta = s g'(theta - theta_be)
    F_bal = -k theta (1 + b theta^2) - c theta' - T_c sgn(theta') + U sin(gamma - beta - theta)
Contact torque on the wheel during impulse:
    lambda_c = T_e - I_e psi''                                                         (H2)
Impacts are plastic and conserve generalised momentum along the active constraint.
"""
from __future__ import annotations

import math

import numpy as np
from numba import njit

# ----------------------------------------------------------------------------------------------
# parameter vector layout
(P_IB, P_K, P_CUB, P_C, P_TC, P_U, P_UPH, P_UON, P_TBE, P_LIFT, P_RR, P_DBP, P_PHIB, P_UUE,
 P_UIE, P_KU, P_KI, P_PSID, P_PSIREC, P_PSIH, P_FUF, P_FUB, P_ETAI, P_ETAP, P_IF, P_IE, P_TE,
 P_TEB, P_KNOCK, P_KNE, P_DTF, P_DTC, P_TOL) = range(33)
N_PARAMS = 33

# modes / wheel states
FREE, UNLOCK, IMPULSE, POST = 0, 1, 2, 3
W_LOCKED, W_FREE, W_CONTACT, W_COUPLED = 0, 1, 2, 3

# state vector layout: theta, omega, psi, psidot, ledger...
S_TH, S_OM, S_PSI, S_PSID, L_VISC, L_COUL, L_UNL, L_IMPF, L_PIN, L_WIN = range(10)
NSTATE = 10

# impact-loss categories
I_ENTRY, I_BANK, I_DROP, I_CATCH, I_KNOCK, I_UNLFAIL = range(6)
N_IMP = 6

# beat log columns
(B_DIR, B_TENTRY, B_OMENTRY, B_TUNL, B_TCATCH, B_TLETOFF, B_TLOCK, B_TEXIT, B_OK, B_WESC,
 B_WBAL, B_AMP_IN) = range(12)
N_BCOL = 12


@njit(cache=True, inline="always")
def _fork(thr, rr, d):
    """g, g', g'' of the roller-fork map phi = atan2(r sin t, d - r cos t)."""
    st = math.sin(thr)
    ct = math.cos(thr)
    B = rr * rr + d * d - 2.0 * rr * d * ct
    g = math.atan2(rr * st, d - rr * ct)
    g1 = (rr * d * ct - rr * rr) / B
    g2 = -rr * d * st * (d * d - rr * rr) / (B * B)
    return g, g1, g2


@njit(cache=True, inline="always")
def _fbal(th, om, sdir, p):
    spring = -p[P_K] * th * (1.0 + p[P_CUB] * th * th)
    visc = -p[P_C] * om
    sg = sdir if om == 0.0 else (1.0 if om > 0.0 else -1.0)
    coul = -p[P_TC] * sg
    grav = p[P_UON] * p[P_U] * math.sin(p[P_UPH] - th)
    return spring + visc + coul + grav


@njit(cache=True)
def _deriv(y, mode, wst, sc, sdir, psiL, p, dy):
    """Fill dy with time derivatives; return contact torque lambda_c (0 if no contact)."""
    th = y[S_TH]
    om = y[S_OM]
    for i in range(NSTATE):
        dy[i] = 0.0
    dy[S_TH] = om
    F = _fbal(th, om, sdir, p)
    sg = sdir if om == 0.0 else (1.0 if om > 0.0 else -1.0)
    dy[L_VISC] = p[P_C] * om * om
    dy[L_COUL] = p[P_TC] * om * sg
    lam = 0.0
    TE = p[P_TE]
    IE = p[P_IE]
    IF = p[P_IF]
    if mode == FREE:
        dy[S_OM] = F / p[P_IB]
    else:
        g, g1, g2 = _fork(th - p[P_TBE], p[P_RR], p[P_DBP])
        G = sc * g1
        G2 = sc * g2
        udot = G * om
        if mode == UNLOCK:
            ku = p[P_KU]
            Ic = IF + ku * ku * IE
            if udot >= 0.0:
                Ttr = p[P_TEB]          # wheel back-driven: train resists more
                fU = p[P_FUF]
                Tu = -ku * Ttr * fU      # resisting torque on fork (+u)
                Tb = Tu / p[P_ETAP]      # balance must supply extra through pin friction
            else:
                Ttr = TE
                fU = p[P_FUB]
                Tu = -ku * Ttr * fU      # negative torque, motion negative => drives fork back
                Tb = Tu * p[P_ETAP]
            J = p[P_IB] + Ic * g1 * g1
            omd = (F + G * Tb - Ic * g1 * g2 * om * om) / J
            dy[S_OM] = omd
            udd = G * omd + G2 * om * om
            dy[S_PSI] = -ku * udot
            dy[S_PSID] = -ku * udd
            # ledger: train work on wheel, friction on locking face, pin friction
            psid = -ku * udot
            dy[L_WIN] = Ttr * psid
            dy[L_UNL] = abs(ku * Ttr * udot) * abs(fU - 1.0)
            dy[L_PIN] = abs(Tb * udot - Tu * udot)
        elif mode == IMPULSE and wst == W_CONTACT:
            ki = p[P_KI]
            eta = p[P_ETAI] * p[P_ETAP]
            J = p[P_IB] + IF * g1 * g1 + eta * ki * ki * IE * g1 * g1
            omd = (F + G * eta * ki * TE - (IF + eta * ki * ki * IE) * g1 * g2 * om * om) / J
            dy[S_OM] = omd
            udd = G * omd + G2 * om * om
            psid = ki * udot
            dy[S_PSI] = psid
            dy[S_PSID] = ki * udd
            lam = TE - IE * ki * udd
            dy[L_WIN] = TE * psid
            dy[L_IMPF] = (1.0 - p[P_ETAI]) * lam * psid
            dy[L_PIN] = (1.0 - p[P_ETAP]) * p[P_ETAI] * lam * psid
        else:
            # IMPULSE separated or POST: balance + fork coupled, wheel independent
            J = p[P_IB] + IF * g1 * g1
            dy[S_OM] = (F - IF * g1 * g2 * om * om) / J
    if mode == FREE or mode == POST or (mode == IMPULSE and wst == W_FREE):
        if wst == W_FREE:
            dy[S_PSI] = y[S_PSID]
            dy[S_PSID] = TE / IE
            dy[L_WIN] = TE * y[S_PSID]
    return lam


@njit(cache=True)
def _rk4(y, h, mode, wst, sc, sdir, psiL, p, out, k1, k2, k3, k4, tmp):
    _deriv(y, mode, wst, sc, sdir, psiL, p, k1)
    for i in range(NSTATE):
        tmp[i] = y[i] + 0.5 * h * k1[i]
    _deriv(tmp, mode, wst, sc, sdir, psiL, p, k2)
    for i in range(NSTATE):
        tmp[i] = y[i] + 0.5 * h * k2[i]
    _deriv(tmp, mode, wst, sc, sdir, psiL, p, k3)
    for i in range(NSTATE):
        tmp[i] = y[i] + h * k3[i]
    _deriv(tmp, mode, wst, sc, sdir, psiL, p, k4)
    for i in range(NSTATE):
        out[i] = y[i] + h * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0


@njit(cache=True)
def _project(y, mode, wst, sc, psiL, p):
    """Enforce kinematic wheel constraints exactly after a step."""
    if mode == UNLOCK or (mode == IMPULSE and wst == W_CONTACT):
        g, g1, g2 = _fork(y[S_TH] - p[P_TBE], p[P_RR], p[P_DBP])
        u = sc * g + p[P_PHIB]
        ud = sc * g1 * y[S_OM]
        if mode == UNLOCK:
            y[S_PSI] = psiL - p[P_KU] * u
            y[S_PSID] = -p[P_KU] * ud
        else:
            y[S_PSI] = psiL - p[P_PSIREC] + p[P_KI] * (u - p[P_UUE])
            y[S_PSID] = p[P_KI] * ud


@njit(cache=True)
def _events(y, mode, wst, sc, sdir, fork_side, psiL, psi_next, p, ev, dtmp):
    """Event functions; an event fires when a value crosses from < 0 to >= 0."""
    for i in range(8):
        ev[i] = -1.0
    th = y[S_TH]
    thr = th - p[P_TBE]
    ev[0] = -sdir * y[S_OM]                       # turning point
    if mode == FREE:
        if fork_side == -sdir:
            ev[1] = sdir * thr + 0.5 * p[P_LIFT]  # impulse pin enters fork slot
        ev[2] = sdir * thr - p[P_KNOCK]           # knocking on the fork horn
        if wst == W_FREE:
            ev[3] = y[S_PSI] - psi_next           # wheel lands on locking face
    else:
        g, g1, g2 = _fork(thr, p[P_RR], p[P_DBP])
        u = sc * g + p[P_PHIB]
        if mode == UNLOCK:
            ev[4] = u - p[P_UUE]
            ev[5] = -u                            # fork back on its banking (failed unlock)
        elif mode == IMPULSE:
            ev[4] = u - p[P_UIE]                  # let-off
            if wst == W_CONTACT:
                lam = _deriv(y, mode, wst, sc, sdir, psiL, p, dtmp)
                ev[6] = -lam                      # contact force vanishes -> separation
            else:
                ev[7] = y[S_PSI] - (psiL - p[P_PSIREC] + p[P_KI] * (u - p[P_UUE]))  # catch-up
        else:  # POST
            ev[4] = u - 2.0 * p[P_PHIB]           # impulse pin leaves the fork
            if wst == W_FREE:
                ev[3] = y[S_PSI] - psi_next


@njit(cache=True)
def _mech_energy(y, mode, wst, sc, p):
    th = y[S_TH]
    om = y[S_OM]
    E = 0.5 * p[P_IB] * om * om + 0.5 * p[P_K] * th * th + 0.25 * p[P_K] * p[P_CUB] * th ** 4
    E += -p[P_UON] * p[P_U] * math.cos(p[P_UPH] - th)
    if mode != FREE:
        g, g1, g2 = _fork(th - p[P_TBE], p[P_RR], p[P_DBP])
        E += 0.5 * p[P_IF] * (g1 * om) ** 2
    E += 0.5 * p[P_IE] * y[S_PSID] ** 2
    return E


@njit(cache=True)
def simulate(p, y0, mode0, wst0, sc0, sdir0, fork_side0, psiL0, t0, t_end, max_beats,
             rec_dt, max_rec):
    """Run the hybrid simulation.

    Returns
    -------
    y, mode, wst, sc, sdir, fork_side, psiL, t     final state (for continuation)
    rec (n,8)      samples [t, theta, omega, phi_fork, psi, psidot, mode, wheel_state]
    nrec
    beats (m, N_BCOL), nbeats
    turns (k, 2)   turning points [t, theta], nturn
    imp (N_IMP,)   accumulated impact losses (J)
    status         0 ok, 1 fault (reversal inside impulse/post), 2 stopped (amplitude died)
    E0_mech        mechanical energy at start (for the ledger)
    """
    y = y0.copy()
    mode = mode0
    wst = wst0
    sc = sc0
    sdir = sdir0
    fork_side = fork_side0
    psiL = psiL0
    psi_next = psiL + p[P_PSIH]
    t = t0
    k1 = np.zeros(NSTATE); k2 = np.zeros(NSTATE); k3 = np.zeros(NSTATE); k4 = np.zeros(NSTATE)
    tmp = np.zeros(NSTATE); ytry = np.zeros(NSTATE); ylo = np.zeros(NSTATE); dtmp = np.zeros(NSTATE)
    ev0 = np.zeros(8); ev1 = np.zeros(8)
    rec = np.zeros((max_rec, 8))
    nrec = 0
    next_rec = t0
    beats = np.zeros((max_beats + 2, N_BCOL))
    nbeats = 0
    turns = np.zeros((4 * max_beats + 8, 2))
    nturn = 0
    imp = np.zeros(N_IMP)
    status = 0
    E0 = _mech_energy(y, mode, wst, sc, p)
    cur = np.zeros(N_BCOL)
    last_amp = 0.0
    wesc_at_entry = 0.0
    ebal_at_entry = 0.0
    last_turn_t = t0
    while t < t_end and nbeats < max_beats:
        if mode == IMPULSE and wst == W_CONTACT:
            if _deriv(y, mode, wst, sc, sdir, psiL, p, dtmp) < 0.0:
                wst = W_FREE                       # contact cannot pull: separate
                y[S_PSI] -= 1e-12
        h = p[P_DTF] if (mode == FREE and wst != W_FREE) else p[P_DTC]
        if t + h > t_end:
            h = t_end - t
        # record
        if rec_dt > 0.0 and t >= next_rec and nrec < max_rec:
            rec[nrec, 0] = t
            rec[nrec, 1] = y[S_TH]
            rec[nrec, 2] = y[S_OM]
            if mode == FREE:
                rec[nrec, 3] = fork_side * p[P_PHIB]
            else:
                g, g1, g2 = _fork(y[S_TH] - p[P_TBE], p[P_RR], p[P_DBP])
                rec[nrec, 3] = g
            rec[nrec, 4] = y[S_PSI]
            rec[nrec, 5] = y[S_PSID]
            rec[nrec, 6] = mode
            rec[nrec, 7] = wst
            nrec += 1
            next_rec += rec_dt
        _events(y, mode, wst, sc, sdir, fork_side, psiL, psi_next, p, ev0, dtmp)
        _rk4(y, h, mode, wst, sc, sdir, psiL, p, ytry, k1, k2, k3, k4, tmp)
        _project(ytry, mode, wst, sc, psiL, p)
        _events(ytry, mode, wst, sc, sdir, fork_side, psiL, psi_next, p, ev1, dtmp)
        fired = -1
        for i in range(8):
            if ev0[i] < 0.0 and ev1[i] >= 0.0:
                fired = 1
        if fired < 0:
            for i in range(NSTATE):
                y[i] = ytry[i]
            t += h
            # detect a dead oscillator (never reaches the fork any more)
            if mode == FREE and t - last_turn_t > 2.0:
                status = 2
                break
            continue
        # ---- locate earliest event by bisection on the step size ----
        hlo = 0.0
        hhi = h
        which = -1
        for it in range(60):
            if hhi - hlo <= p[P_TOL]:
                break
            hm = 0.5 * (hlo + hhi)
            _rk4(y, hm, mode, wst, sc, sdir, psiL, p, ytry, k1, k2, k3, k4, tmp)
            _project(ytry, mode, wst, sc, psiL, p)
            _events(ytry, mode, wst, sc, sdir, fork_side, psiL, psi_next, p, ev1, dtmp)
            any_f = False
            for i in range(8):
                if ev0[i] < 0.0 and ev1[i] >= 0.0:
                    any_f = True
            if any_f:
                hhi = hm
            else:
                hlo = hm
        _rk4(y, hhi, mode, wst, sc, sdir, psiL, p, ytry, k1, k2, k3, k4, tmp)
        _project(ytry, mode, wst, sc, psiL, p)
        _events(ytry, mode, wst, sc, sdir, fork_side, psiL, psi_next, p, ev1, dtmp)
        best = 1e300
        for i in range(8):
            if ev0[i] < 0.0 and ev1[i] >= 0.0:
                # choose the event that was 'closest' at the start (earliest) - all fired in
                # the same tiny interval; priority by index order except turning point last
                pri = i if i != 0 else 99
                if pri < best:
                    best = pri
                    which = i
        for i in range(NSTATE):
            y[i] = ytry[i]
        t += hhi
        # ---------------------------- event handlers ----------------------------------
        if which == 0:                                     # turning point
            if nturn < turns.shape[0]:
                turns[nturn, 0] = t
                turns[nturn, 1] = y[S_TH]
                nturn += 1
            last_amp = abs(y[S_TH])
            last_turn_t = t
            sdir = -sdir
            y[S_OM] = 0.0
            if mode == IMPULSE or mode == POST:
                status = 1
                break
        elif which == 1:                                   # pin enters fork -> UNLOCK
            if wst == W_FREE:
                # wheel still dropping when the next unlock starts: land it first (rare)
                imp[I_DROP] += 0.5 * p[P_IE] * y[S_PSID] ** 2
                y[S_PSI] = psi_next
                y[S_PSID] = 0.0
                wst = W_LOCKED
                psiL = psi_next
                psi_next = psiL + p[P_PSIH]
            g, g1, g2 = _fork(y[S_TH] - p[P_TBE], p[P_RR], p[P_DBP])
            Jb = p[P_IB]
            Ja = p[P_IB] + (p[P_IF] + p[P_KU] ** 2 * p[P_IE]) * g1 * g1
            om_new = Jb * y[S_OM] / Ja
            imp[I_ENTRY] += 0.5 * Jb * y[S_OM] ** 2 - 0.5 * Ja * om_new ** 2
            y[S_OM] = om_new
            mode = UNLOCK
            wst = W_COUPLED
            sc = sdir
            for c in range(N_BCOL):
                cur[c] = 0.0
            cur[B_DIR] = sc
            cur[B_TENTRY] = t
            cur[B_OMENTRY] = y[S_OM]
            cur[B_AMP_IN] = last_amp
            wesc_at_entry = y[L_WIN]
            ebal_at_entry = 0.5 * p[P_IB] * y[S_OM] ** 2 + 0.5 * p[P_K] * y[S_TH] ** 2
            _project(y, mode, wst, sc, psiL, p)
        elif which == 2:                                   # knock (overbanking)
            imp[I_KNOCK] += 0.5 * p[P_IB] * y[S_OM] ** 2 * (1.0 - p[P_KNE] ** 2)
            y[S_OM] = -p[P_KNE] * y[S_OM]
            sdir = -sdir
        elif which == 3:                                   # wheel locks (drop ends)
            imp[I_DROP] += 0.5 * p[P_IE] * y[S_PSID] ** 2
            y[S_PSI] = psi_next
            y[S_PSID] = 0.0
            wst = W_LOCKED
            psiL = psi_next
            psi_next = psiL + p[P_PSIH]
            if mode == FREE and nbeats > 0 and beats[nbeats - 1, B_TLOCK] == 0.0:
                beats[nbeats - 1, B_TLOCK] = t
            elif cur[B_TLOCK] == 0.0:
                cur[B_TLOCK] = t
        elif which == 4:
            if mode == UNLOCK:                             # unlocking complete -> IMPULSE
                mode = IMPULSE
                wst = W_FREE                               # wheel lags the receding face
                y[S_PSI] = psiL - p[P_PSIREC] - 1e-12
                cur[B_TUNL] = t
            elif mode == IMPULSE:                          # let-off -> POST, wheel drops
                mode = POST
                wst = W_FREE
                cur[B_TLETOFF] = t
            else:                                          # pin exits -> FREE
                g, g1, g2 = _fork(y[S_TH] - p[P_TBE], p[P_RR], p[P_DBP])
                imp[I_BANK] += 0.5 * p[P_IF] * (g1 * y[S_OM]) ** 2
                mode = FREE
                fork_side = sc
                cur[B_TEXIT] = t
                cur[B_OK] = 1.0
                cur[B_WESC] = y[L_WIN] - wesc_at_entry
                cur[B_WBAL] = (0.5 * p[P_IB] * y[S_OM] ** 2 + 0.5 * p[P_K] * y[S_TH] ** 2) - ebal_at_entry
                for c in range(N_BCOL):
                    beats[nbeats, c] = cur[c]
                nbeats += 1
        elif which == 5:                                   # failed unlock: fork back to banking
            g, g1, g2 = _fork(y[S_TH] - p[P_TBE], p[P_RR], p[P_DBP])
            imp[I_UNLFAIL] += 0.5 * (p[P_IF] + p[P_KU] ** 2 * p[P_IE]) * (g1 * y[S_OM]) ** 2
            mode = FREE
            wst = W_LOCKED
            y[S_PSI] = psiL
            y[S_PSID] = 0.0
            cur[B_TEXIT] = t
            cur[B_OK] = 0.0
            for c in range(N_BCOL):
                beats[nbeats, c] = cur[c]
            nbeats += 1
        elif which == 6:                                   # separation during impulse
            wst = W_FREE
        elif which == 7:                                   # wheel catches the impulse face
            g, g1, g2 = _fork(y[S_TH] - p[P_TBE], p[P_RR], p[P_DBP])
            G = sc * g1
            eta = p[P_ETAI] * p[P_ETAP]
            Jbf = p[P_IB] + p[P_IF] * g1 * g1
            ki = p[P_KI]
            E_before = 0.5 * Jbf * y[S_OM] ** 2 + 0.5 * p[P_IE] * y[S_PSID] ** 2
            Lam = (y[S_PSID] - ki * G * y[S_OM]) / (1.0 / p[P_IE] + eta * ki * ki * G * G / Jbf)
            if Lam > 0.0:
                y[S_PSID] = y[S_PSID] - Lam / p[P_IE]
                y[S_OM] = y[S_OM] + G * eta * ki * Lam / Jbf
            E_after = 0.5 * Jbf * y[S_OM] ** 2 + 0.5 * p[P_IE] * y[S_PSID] ** 2
            imp[I_CATCH] += E_before - E_after
            wst = W_CONTACT
            if cur[B_TCATCH] == 0.0:
                cur[B_TCATCH] = t
            _project(y, mode, wst, sc, psiL, p)
    return (y, mode, wst, sc, sdir, fork_side, psiL, t, rec, nrec, beats, nbeats, turns, nturn,
            imp, status, E0)


@njit(cache=True)
def mech_energy(y, mode, wst, sc, p):
    return _mech_energy(y, mode, wst, sc, p)
