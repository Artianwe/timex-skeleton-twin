'use strict';
/* ============================================================================================
   sim.js - live state of the digital twin in the browser.
   The physics is NOT re-solved here: it plays back and interpolates results computed by the
   Python model (twin.pipeline). Motion of every wheel comes from the escape-wheel angle and the
   exact tooth-count ratios; the balance, fork and wheel inside one oscillation come from the
   event-resolved hybrid limit cycle at the current state of wind.
   ============================================================================================ */
const D = JSON.parse(document.getElementById('twin-data').textContent);
const G = JSON.parse(document.getElementById('twin-geo').textContent);
const V = D.viewer;
const POS_LABEL = { DU: 'Dial up', DD: 'Dial down', CU: 'Crown up', CD: 'Crown down', CL: 'Crown left', CR: 'Crown right' };
const TAU = Math.PI * 2;

function interp(x, xs, ys) {
  const n = xs.length;
  if (!n) return NaN;
  const asc = xs[n - 1] >= xs[0];
  if (asc ? x <= xs[0] : x >= xs[0]) return ys[0];
  if (asc ? x >= xs[n - 1] : x <= xs[n - 1]) return ys[n - 1];
  let lo = 0, hi = n - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (asc ? xs[mid] <= x : xs[mid] >= x) lo = mid; else hi = mid;
  }
  const t = (x - xs[lo]) / (xs[hi] - xs[lo]);
  return ys[lo] + t * (ys[hi] - ys[lo]);
}

const CYCLES = D.cycles.slice().sort((a, b) => b.wound_fraction - a.wound_fraction);
const WALK = (D.autowinding.rates.find(r => r.profile === 'walking') || { turns_per_hour: 4 }).turns_per_hour;
const OMEGA_E = V.omega.escape;
const RATIO = Object.fromEntries(Object.entries(V.omega).map(([k, w]) => [k, w / OMEGA_E]));

const Twin = {
  pos: 'DU',
  n: V.n_dev,              // wound turns of the barrel arbor
  tSim: 0,                 // simulated seconds since the page opened
  tOsc: 0,                 // oscillation clock (advances only while the balance runs)
  playing: true,
  speed: 1,
  hacked: false,
  wrist: false,
  rotor: 0,                // rotor angle (rad)
  ratchet: 0,              // ratchet angle from winding (rad)
  wound: 0,                // turns added by winding since load
  clock0: (() => { const d = new Date(); return d.getHours() * 3600 + d.getMinutes() * 60 + d.getSeconds(); })(),
  eventMsg: '',

  reserve() { return D.reserve[this.pos]; },
  nStop() { return V.n_stop[this.pos] ?? 0; },
  running() { return !this.hacked && this.n > this.nStop() + 1e-6; },
  /** equivalent hours since full wind for the current state of wind */
  hoursFromFull() { return (V.n_dev - this.n) * V.T_rev_s / 3600; },
  at(key) { const r = this.reserve(); return interp(this.n, r.n, r[key]); },
  lossesNow() {
    const r = this.reserve();
    const out = {};
    for (const [k, arr] of Object.entries(r.losses || {})) out[k] = interp(this.n, r.n, arr);
    return out;
  },
  reserveLeftH() { return Math.max(0, (this.n - this.nStop()) * V.T_rev_s / 3600); },
  cycle() {
    const f = this.n / V.n_dev;
    let best = CYCLES[0], bd = 1e9;
    for (const c of CYCLES) { const d = Math.abs(c.wound_fraction - f); if (d < bd) { bd = d; best = c; } }
    return best;
  },
  /** balance angle, fork angle and total escape-wheel angle at the current oscillation time */
  kinematics() {
    const c = this.cycle();
    const T = c.period;
    const k = Math.floor(this.tOsc / T);
    const tau = this.tOsc - k * T;
    const theta = this.running() || this.hacked ? interp(tau, c.t, c.theta) : 0;
    const phi = interp(tau, c.t, c.phi);
    const psi = k * 2 * V.psi_half + interp(tau, c.t, c.psi);
    return { theta: this.hacked ? this._hackTheta ?? theta : theta, phi, psi, tau, cyc: c };
  },
  step(dtReal) {
    if (!this.playing) return;
    const dt = Math.min(dtReal, 0.1) * this.speed;
    this.tSim += dt;
    if (this.wrist) {
      const dn = WALK / 3600 * dt;
      this.n = Math.min(V.n_dev, this.n + dn);
      this.ratchet += dn * TAU;
    }
    if (this.running()) {
      this.tOsc += dt;
      this.n -= dt / V.T_rev_s;
    }
  },
  wind(turns) {
    const before = this.n;
    this.n = Math.min(V.n_dev, this.n + turns);
    this.ratchet += (this.n - before) * TAU;
    this.wound += this.n - before;
  },
  /** event list of the current cycle, as [{t, key, label}] sorted in time */
  events() {
    const c = this.cycle();
    const names = { t_entry: 'Unlocking begins: impulse pin enters the fork', t_unlock: 'Unlocked: escape wheel released',
      t_catch: 'Impulse: wheel tooth catches the pallet', t_letoff: 'Let-off: drop begins', t_lock: 'Lock: tooth lands on the other pallet (tick)',
      t_exit: 'Fork on banking: free supplementary arc' };
    const ev = [];
    for (const b of c.beats) for (const [k, lab] of Object.entries(names)) if (b[k] > 0) ev.push({ t: b[k], key: k, label: lab });
    return ev.sort((a, b) => a.t - b.t);
  },
  jumpEvent(dir) {
    const c = this.cycle();
    const T = c.period;
    const k = Math.floor(this.tOsc / T);
    const tau = this.tOsc - k * T;
    const ev = this.events();
    let target;
    if (dir > 0) {
      target = ev.find(e => e.t > tau + 1e-7);
      if (!target) { target = ev[0]; this.tOsc = (k + 1) * T + target.t; }
      else this.tOsc = k * T + target.t;
    } else {
      const prev = ev.filter(e => e.t < tau - 1e-7);
      target = prev.length ? prev[prev.length - 1] : ev[ev.length - 1];
      this.tOsc = (prev.length ? k : k - 1) * T + target.t;
      if (this.tOsc < 0) this.tOsc += T;
    }
    this.playing = false;
    this.eventMsg = target.label;
    return target;
  },
  watchTimeSeconds() {
    const { psi } = this.kinematics();
    return this.clock0 + psi / OMEGA_E;
  },
};
