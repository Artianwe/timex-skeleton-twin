/* ============================================================================================
   dashboard.js - engineering dashboard (Plotly) + tables. Theme-aware, validated palette,
   one axis per chart, legends for >= 2 series, hover readouts, table views.
   ============================================================================================ */
const Dash = (() => {
  const $ = id => document.getElementById(id);
  const tok = name => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  const S = () => [1, 2, 3, 4, 5, 6, 7, 8].map(i => tok('--s' + i));
  const fmt = (v, d = 2) => (v === null || v === undefined || Number.isNaN(v)) ? '–' : Number(v).toFixed(d);
  const live = {};       // live chart divs
  let ptState = 'full';

  function baseLayout(extra = {}) {
    const ink2 = tok('--ink-2'), muted = tok('--muted'), grid = tok('--line'), surf = tok('--surface');
    return Object.assign({
      paper_bgcolor: surf, plot_bgcolor: surf, font: { family: 'Archivo, Helvetica, Arial, sans-serif', size: 11, color: ink2 },
      margin: { l: 52, r: 14, t: 30, b: 40 }, hovermode: 'x unified',
      hoverlabel: { bgcolor: surf, bordercolor: grid, font: { color: tok('--ink'), size: 11 } },
      xaxis: { gridcolor: grid, linecolor: tok('--axis'), zeroline: false, tickfont: { color: muted }, title: { font: { size: 11 } } },
      yaxis: { gridcolor: grid, linecolor: tok('--axis'), zeroline: false, tickfont: { color: muted }, title: { font: { size: 11 } } },
      legend: { orientation: 'h', y: -0.3, yanchor: 'top', x: 0, font: { size: 10.5 } }, showlegend: true,
      title: { font: { size: 12.5, color: tok('--ink') }, x: 0.01, xanchor: 'left', y: 0.98, yanchor: 'top' },
    }, extra, { margin: Object.assign({ l: 52, r: 14, t: 34, b: (extra.showlegend === false ? 46 : 104) }, extra.margin || {}),
      legend: Object.assign({ orientation: 'h', y: -0.3, yanchor: 'top', x: 0, font: { size: 10.5 } }, extra.legend || {}) });
  }
  const CFG = { displayModeBar: false, responsive: true };

  const ro = new ResizeObserver(entries => { for (const e of entries) if (e.target.data) Plotly.Plots.resize(e.target); });
  function card(container, id, cls = 'chart') {
    const d = document.createElement('div');
    d.className = cls; d.id = id;
    container.appendChild(d);
    ro.observe(d);
    return d;
  }

  /* ---------------------------------------------------------------- KPIs */
  const KPI = [
    ['sim', 'Simulated time', ''], ['torque', 'Mainspring torque', 'N·mm'], ['energy', 'Stored energy', 'mJ'],
    ['reserve', 'Power reserve left', 'h'], ['rpm_c', 'Centre wheel', 'rpm'], ['rpm_3', 'Third wheel', 'rpm'],
    ['rpm_4', 'Fourth wheel', 'rpm'], ['rpm_e', 'Escape wheel', 'rpm'], ['amp', 'Balance amplitude', '°'],
    ['freq', 'Beat frequency', 'Hz'], ['be', 'Beat error', 'ms'], ['rate', 'Rate error', 's/day'], ['eff', 'Total efficiency', '%'],
  ];
  function buildKpis() {
    const box = $('kpis');
    for (const [id, label, unit] of KPI) {
      const k = document.createElement('div'); k.className = 'kpi'; k.id = 'k-' + id;
      const l = document.createElement('div'); l.className = 'k-label'; l.textContent = label;
      const v = document.createElement('div'); v.className = 'k-value';
      const n = document.createElement('span'); n.className = 'k-num'; n.textContent = '–';
      const u = document.createElement('span'); u.className = 'k-unit'; u.textContent = unit;
      v.append(n, u); k.append(l, v); box.appendChild(k);
    }
  }
  function setKpi(id, value, alert = false) {
    const k = $('k-' + id); if (!k) return;
    k.querySelector('.k-num').textContent = value;
    k.classList.toggle('alert', alert);
  }
  function updateKpis() {
    const run = Twin.running();
    const h = Twin.tSim / 3600;
    setKpi('sim', h < 1 ? `${Math.floor(Twin.tSim / 60)}m ${Math.floor(Twin.tSim % 60)}s` : `${Math.floor(h)}h ${Math.floor((h % 1) * 60)}m`);
    setKpi('torque', fmt(Twin.at('T') * 1e3, 3));
    setKpi('energy', fmt(Twin.at('E') * 1e3, 1));
    setKpi('reserve', fmt(Twin.reserveLeftH(), 1), Twin.reserveLeftH() < 4);
    const rpm = a => run ? fmtSmall(V.omega[a] * 60 / TAU) : '0';
    setKpi('rpm_c', rpm('centre')); setKpi('rpm_3', rpm('third')); setKpi('rpm_4', rpm('fourth')); setKpi('rpm_e', rpm('escape'));
    setKpi('amp', run ? fmt(Twin.at('A'), 1) : '0', run && Twin.at('A') < 180);
    setKpi('freq', run ? (1 / Twin.cycle().period).toFixed(5) : '0');
    setKpi('be', run ? fmt(Twin.at('be'), 2) : '–');
    const rate = Twin.at('rate');
    setKpi('rate', run ? (rate >= 0 ? '+' : '') + fmt(rate, 1) : '–', run && (rate < -20 || rate > 40));
    const pb = Twin.at('P_barrel'), pbal = Twin.at('P_balance');
    setKpi('eff', run && pb > 0 ? fmt(100 * pbal / pb, 1) : '–');
  }
  function fmtSmall(r) {
    const a = Math.abs(r);
    const s = a >= 1 ? a.toFixed(3) : a >= 0.01 ? a.toFixed(4) : a.toExponential(2);
    return (r < 0 ? '↻ ' : '↺ ') + s;
  }

  /* ---------------------------------------------------------------- live charts */
  function liveCharts() {
    const c = $('live-charts');
    c.replaceChildren();
    const s = S();
    const r = Twin.reserve();
    const du = D.reserve.DU;
    const posLab = POS_LABEL[Twin.pos];
    const mk = (id, traces, layout) => { const d = card(c, id); Plotly.newPlot(d, traces, baseLayout(layout), CFG); live[id] = d; };
    mk('c-torque', [{ x: r.t_h, y: r.T.map(v => v * 1e3), name: 'barrel torque', line: { color: s[0], width: 2 }, hovertemplate: '%{y:.3f} N·mm<extra></extra>' }],
      { title: { text: 'Mainspring torque vs time' }, xaxis: { title: { text: 'hours since full wind' } }, yaxis: { title: { text: 'N·mm' }, rangemode: 'tozero' }, showlegend: false });
    mk('c-energy', [{ x: r.t_h, y: r.E.map(v => v * 1e3), name: 'stored energy', line: { color: s[0], width: 2 }, hovertemplate: '%{y:.1f} mJ<extra></extra>' }],
      { title: { text: 'Stored energy vs time' }, xaxis: { title: { text: 'hours since full wind' } }, yaxis: { title: { text: 'mJ' }, rangemode: 'tozero' }, showlegend: false });
    const ampTr = [{ x: r.t_h, y: r.A, name: posLab, line: { color: s[0], width: 2 }, hovertemplate: '%{y:.1f}°' }];
    if (Twin.pos !== 'DU') ampTr.push({ x: du.t_h, y: du.A, name: 'Dial up', line: { color: s[1], width: 2 }, hovertemplate: '%{y:.1f}°' });
    mk('c-amp', ampTr, { title: { text: 'Balance amplitude vs time' }, xaxis: { title: { text: 'hours since full wind' } }, yaxis: { title: { text: 'degrees' } }, showlegend: Twin.pos !== 'DU',
      legend: { x: 0.99, xanchor: 'right', y: 0.99, yanchor: 'top', bgcolor: tok('--surface') + 'e6' }, margin: { b: 46 } });
    mk('c-power', [
      { x: r.t_h, y: r.P_barrel.map(v => v * 1e6), name: 'mainspring → train', line: { color: s[0], width: 2 }, hovertemplate: '%{y:.3f} µW' },
      { x: r.t_h, y: r.P_escape.map(v => v * 1e6), name: 'train → escape wheel', line: { color: s[1], width: 2 }, hovertemplate: '%{y:.3f} µW' },
      { x: r.t_h, y: r.P_balance.map(v => v * 1e6), name: 'escapement → balance', line: { color: s[2], width: 2 }, hovertemplate: '%{y:.3f} µW' },
    ], { title: { text: 'Power transmitted through the movement' }, xaxis: { title: { text: 'hours since full wind' } }, yaxis: { title: { text: 'µW' }, rangemode: 'tozero' },
      legend: { x: 0.99, xanchor: 'right', y: 0.99, yanchor: 'top', bgcolor: tok('--surface') + 'e6' }, margin: { b: 46 } });
    const m = r.t_h.map((t, i) => t <= 0.97 * r.reserve_h ? i : -1).filter(i => i >= 0);
    mk('c-rate', [{ x: m.map(i => r.t_h[i]), y: m.map(i => r.rate[i]), name: 'rate', line: { color: s[0], width: 2 }, hovertemplate: '%{y:+.2f} s/day<extra></extra>' }],
      { title: { text: `Simulated rate vs time (${posLab})` }, xaxis: { title: { text: 'hours since full wind' } }, yaxis: { title: { text: 's/day' } }, showlegend: false,
        shapes: [{ type: 'rect', xref: 'paper', x0: 0, x1: 1, y0: -20, y1: 40, fillcolor: tok('--surface-2'), line: { width: 0 }, layer: 'below' }] });
    const lb = lossBars();
    const ld = card(c, 'c-loss');
    Plotly.newPlot(ld, lb.traces, baseLayout({ title: { text: 'Energy losses by component (now)' }, barmode: 'stack', hovermode: 'closest',
      margin: { l: 150, r: 14, t: 34, b: 46 }, legend: { x: 0.99, xanchor: 'right', y: 0.02, yanchor: 'bottom', bgcolor: 'rgba(0,0,0,0)' }, xaxis: { title: { text: 'nW' } }, yaxis: { automargin: true, tickfont: { size: 10 } } }), CFG);
    live['c-loss'] = ld;
    liveTable();
  }

  function lossBars() {
    const s = S();
    const L = Twin.lossesNow();
    const scale = Twin.at('T') / V.M_full;
    const rows = [];
    for (const st of D.energy_flow_full.train_losses) rows.push([st.stage, st.P_loss * scale * 1e9, 0]);
    for (const [k, v] of Object.entries(L)) {
      if (v <= 0) continue;
      const grp = k.startsWith('balance_') ? 2 : 1;
      rows.push([k.replace('balance_', 'balance: ').replace(/_/g, ' '), v * 1e9, grp]);
    }
    rows.sort((a, b) => a[1] - b[1]);
    const names = ['going train', 'escapement', 'balance / oscillator'];
    const traces = [0, 1, 2].map(g => ({
      type: 'bar', orientation: 'h', name: names[g], marker: { color: s[g] },
      y: rows.map(r => r[0]), x: rows.map(r => (r[2] === g ? r[1] : 0)),
      hovertemplate: '%{x:.2f} nW<extra>%{y}</extra>',
    }));
    return { traces, rows };
  }

  function cursorShapes(div) {
    const t = Twin.hoursFromFull();
    const base = (div.layout.shapes || []).filter(sh => !sh._cursor);
    return base.concat([{ _cursor: true, type: 'line', xref: 'x', yref: 'paper', x0: t, x1: t, y0: 0, y1: 1, line: { color: tok('--accent'), width: 1.5 } }]);
  }
  function updateLive() {
    for (const id of ['c-torque', 'c-energy', 'c-amp', 'c-power', 'c-rate']) {
      const d = live[id]; if (!d || !d.layout) continue;
      Plotly.relayout(d, { shapes: cursorShapes(d) });
    }
    const d = live['c-loss'];
    if (d) { const lb = lossBars(); Plotly.react(d, lb.traces, d.layout, CFG); }
  }

  function liveTable() {
    const r = Twin.reserve();
    const rows = [];
    for (let h = 0; h <= r.reserve_h; h += 2) {
      const n = interp(h, r.t_h, r.n);
      rows.push([h, interp(h, r.t_h, r.T) * 1e3, interp(h, r.t_h, r.E) * 1e3, interp(h, r.t_h, r.A), interp(h, r.t_h, r.rate),
        interp(h, r.t_h, r.P_barrel) * 1e6, interp(h, r.t_h, r.P_balance) * 1e6, n]);
    }
    table($('live-table'), ['h', 'Torque N·mm', 'Energy mJ', 'Amplitude °', 'Rate s/day', 'P barrel µW', 'P balance µW', 'Wound turns'],
      rows.map(r => r.map((v, i) => i === 0 ? v.toFixed(0) : fmt(v, i === 4 ? 2 : 3))), true);
  }

  /* ---------------------------------------------------------------- tables */
  function table(container, head, rows, numeric = false, opts = {}) {
    const wrap = document.createElement('div'); wrap.className = 'tbl-wrap';
    const t = document.createElement('table');
    const thead = document.createElement('thead'); const tr = document.createElement('tr');
    head.forEach(h => { const th = document.createElement('th'); th.textContent = h; tr.appendChild(th); });
    thead.appendChild(tr); t.appendChild(thead);
    const tb = document.createElement('tbody');
    for (const r of rows) {
      const row = document.createElement('tr');
      r.forEach((v, i) => {
        const td = document.createElement('td');
        if (v instanceof Node) td.appendChild(v); else td.textContent = v;
        if (numeric && i > 0 && !(v instanceof Node)) td.className = 'num';
        if (opts.numCols && opts.numCols.includes(i)) td.className = 'num';
        row.appendChild(td);
      });
      tb.appendChild(row);
    }
    t.appendChild(tb); wrap.appendChild(t);
    container.replaceChildren(wrap);
  }
  const PROV_COLOR = { measured: '--s1', sourced: '--s2', inferred: '--s3', assumed: '--s4' };
  function badge(prov) {
    const b = document.createElement('span'); b.className = 'badge'; b.textContent = prov;
    b.style.setProperty('--c', `var(${PROV_COLOR[prov] || '--muted'})`);
    return b;
  }
  function code(txt) { const c = document.createElement('code'); c.textContent = txt; return c; }
  function statusEl(st) { const s = document.createElement('span'); s.className = 'status ' + st; s.textContent = st; return s; }

  /* ---------------------------------------------------------------- escapement tab */
  function escTab() {
    const cyc = CYCLES[0];
    const b = cyc.beats[0];
    const t0 = b.t_entry - 0.003, t1 = b.t_exit + 0.003;
    const idx = cyc.t.map((t, i) => (t >= t0 && t <= t1 ? i : -1)).filter(i => i >= 0);
    const x = idx.map(i => (cyc.t[i] - b.t_entry) * 1e3);
    const s = S();
    const deg = v => v * 180 / Math.PI;
    const psi0 = cyc.psi[idx[0]];
    const traces = [
      { x, y: idx.map(i => deg(cyc.theta[i])), name: 'balance θ [°]', yaxis: 'y', line: { color: s[0], width: 2 }, hovertemplate: '%{y:.2f}°' },
      { x, y: idx.map(i => deg(cyc.phi[i])), name: 'fork φ [°]', yaxis: 'y2', line: { color: s[1], width: 2 }, hovertemplate: '%{y:.3f}°' },
      { x, y: idx.map(i => deg(cyc.psi[i] - psi0)), name: 'escape wheel ψ [°]', yaxis: 'y3', line: { color: s[2], width: 2 }, hovertemplate: '%{y:.3f}°' },
    ];
    const ph = [['unlocking', b.t_entry, b.t_unlock], ['wheel lag', b.t_unlock, b.t_catch], ['impulse', b.t_catch, b.t_letoff],
      ['drop', b.t_letoff, b.t_lock], ['run to banking', b.t_lock, b.t_exit]];
    const shades = [tok('--surface-2'), 'rgba(235,104,52,0.10)', 'rgba(27,175,122,0.12)', 'rgba(237,161,0,0.12)', tok('--surface-2')];
    const shapes = ph.filter(p => p[2] > p[1] && p[1] > 0).map(([l, a, bb], i) => ({ type: 'rect', xref: 'x', yref: 'paper', x0: (a - b.t_entry) * 1e3,
      x1: (bb - b.t_entry) * 1e3, y0: 0, y1: 1, fillcolor: shades[i], line: { width: 0 }, layer: 'below' }));
    const ann = ph.filter(p => p[2] > p[1] && p[1] > 0).map(([l, a, bb], i) => ({ x: ((a + bb) / 2 - b.t_entry) * 1e3, y: i % 2 ? 1.08 : 1.02, xref: 'x', yref: 'paper',
      text: l, showarrow: false, font: { size: 10, color: tok('--ink-2') } }));
    const lay = baseLayout({ title: { text: `One beat at full wind (amplitude ${cyc.amplitude_deg.toFixed(1)}°)` }, shapes, annotations: ann,
      margin: { l: 60, r: 14, t: 62, b: 120 },
      yaxis: { domain: [0.68, 1], title: { text: 'θ [°]' }, gridcolor: tok('--line') },
      yaxis2: { domain: [0.35, 0.65], title: { text: 'φ [°]' }, gridcolor: tok('--line') },
      yaxis3: { domain: [0, 0.32], title: { text: 'ψ [°]' }, gridcolor: tok('--line') },
      xaxis: { title: { text: 'ms from impulse-pin entry' }, gridcolor: tok('--line'), anchor: 'y3' }, legend: { orientation: 'h', y: -0.2, yanchor: 'top' } });
    Plotly.newPlot($('esc-chart'), traces, lay, CFG);
    ro.observe($('esc-chart'));
    table($('esc-table'), ['Event', 'What happens', 't [ms]', 'Balance θ [°]', 'ω [rad/s]'],
      D.events.map(e => [e.event.replace('t_', ''), e.description, fmt(e.t_ms, 3), fmt(e.balance_deg, 2), fmt(e.balance_rad_s, 1)]), false, { numCols: [2, 3, 4] });
    const f = D.energy_flow_full;
    const el = $('esc-energy');
    el.textContent = `At full wind the escape wheel receives ${(f.P_escape_wheel * 1e6).toFixed(3)} µW; ${(f.P_balance * 1e6).toFixed(3)} µW reaches the balance ` +
      `(escapement efficiency ${(f.eta_escapement * 100).toFixed(1)} %). The largest escapement losses are impulse-plane friction ` +
      `(${(f.escapement_losses.impulse_plane_friction * 1e9).toFixed(0)} nW) and the drop impact (${(f.escapement_losses.impact_drop_lock * 1e9).toFixed(0)} nW).`;
  }

  /* ---------------------------------------------------------------- timekeeping tab */
  function timeTab() {
    const c = $('time-charts'); c.replaceChildren();
    const s = S();
    const rows = D.positional[ptState];
    const lab = rows.map(r => r.label);
    Plotly.newPlot(card(c, 'pos-rate'), [{ type: 'bar', x: lab, y: rows.map(r => r.rate_s_per_day), marker: { color: s[0] }, width: 0.5,
      text: rows.map(r => (r.rate_s_per_day >= 0 ? '+' : '') + r.rate_s_per_day.toFixed(1)), textposition: 'outside', cliponaxis: false,
      hovertemplate: '%{y:+.2f} s/day<extra>%{x}</extra>' }],
      baseLayout({ title: { text: `Rate by position (${ptState === 'full' ? 'full wind' : 'after 24 h'})` }, yaxis: { title: { text: 's/day' } }, showlegend: false, hovermode: 'closest' }), CFG);
    Plotly.newPlot(card(c, 'pos-amp'), [{ type: 'bar', x: lab, y: rows.map(r => r.amplitude_deg), marker: { color: s[0] }, width: 0.5,
      text: rows.map(r => r.amplitude_deg.toFixed(0) + '°'), textposition: 'outside', cliponaxis: false, hovertemplate: '%{y:.1f}°<extra>%{x}</extra>' }],
      baseLayout({ title: { text: 'Amplitude by position' }, yaxis: { title: { text: 'degrees' }, rangemode: 'tozero' }, showlegend: false, hovermode: 'closest' }), CFG);
    const iso = D.isochronism;
    const tr = [];
    [['DU', 'Dial up'], ['CD', 'Crown down']].forEach(([p, l], i) => {
      const d = iso[p]; const o = d.amplitude_deg.map((a, k) => k).sort((a, b) => d.amplitude_deg[a] - d.amplitude_deg[b]);
      tr.push({ x: o.map(k => d.amplitude_deg[k]), y: o.map(k => d.rate[k]), name: l, mode: 'lines+markers', line: { color: s[i], width: 2 }, marker: { size: 7 }, hovertemplate: '%{y:+.2f} s/day' });
    });
    Plotly.newPlot(card(c, 'iso'), tr, baseLayout({ title: { text: 'Isochronism: rate vs amplitude' }, xaxis: { title: { text: 'amplitude [°]' } }, yaxis: { title: { text: 's/day' } }, hovermode: 'closest' }), CFG);
    const un = D.unpoise;
    const A = []; for (let a = 100; a <= 330; a += 2) A.push(a);
    const th = A.map(a => { const r = a * Math.PI / 180; return 86400 * (un.U * Math.cos(un.phase) / un.k) * besselJ1(r) / r; });
    Plotly.newPlot(card(c, 'unpoise'), [
      { x: A, y: th, name: 'averaging theory ∝ J₁(A)/A', line: { color: s[0], width: 2 }, hovertemplate: '%{y:+.2f} s/day' },
      { x: un.sim.amplitude_deg, y: un.sim.unpoise_rate_effect, name: 'hybrid simulation', mode: 'markers', marker: { color: s[1], size: 9, line: { color: tok('--surface'), width: 2 } }, hovertemplate: '%{y:+.2f} s/day' },
    ], baseLayout({ title: { text: 'Out-of-poise error, crown down: theory vs simulation' }, xaxis: { title: { text: 'amplitude [°]' } }, yaxis: { title: { text: 's/day' } }, hovermode: 'closest',
      shapes: [{ type: 'line', x0: 219.5, x1: 219.5, yref: 'paper', y0: 0, y1: 1, line: { color: tok('--axis'), width: 1 } }] }), CFG);
    table($('pos-table'), ['Position', 'Amplitude °', 'Rate s/day', 'Beat error ms', 'Power to escape µW'],
      rows.map(r => [r.label, fmt(r.amplitude_deg, 1), (r.rate_s_per_day >= 0 ? '+' : '') + fmt(r.rate_s_per_day, 2), fmt(r.beat_error_ms, 3), fmt(r.power_uW, 4)]), true);
    const dec = D.error_decomposition;
    const keys = [['amplitude_deg', 'Amplitude °'], ['gravity_unpoise', 'Out-of-poise (gravity)'], ['beat_offset_asymmetry', 'Beat-offset asymmetry'],
      ['escapement_and_damping', 'Escapement + damping'], ['regulator_offset', 'Regulator offset'], ['total', 'Total rate']];
    table($('decomp-table'), ['Contribution [s/day]'].concat(Object.keys(dec).map(p => POS_LABEL[p])),
      keys.map(([k, l]) => [l].concat(Object.keys(dec).map(p => fmt(dec[p][k], 2)))), true);
    const ac = $('auto-charts'); ac.replaceChildren();
    const ar = D.autowinding.rates;
    Plotly.newPlot(card(ac, 'auto-rates'), [{ type: 'bar', x: ar.map(r => r.profile), y: ar.map(r => r.turns_per_hour), marker: { color: s[0] }, width: 0.5, name: 'winding', hovertemplate: '%{y:.2f} turns/h<extra>%{x}</extra>' },
      { x: ar.map(r => r.profile), y: ar.map(r => r.consumption_turns_per_hour), mode: 'lines', name: 'consumption', line: { color: s[1], width: 2 }, hovertemplate: '%{y:.3f} turns/h' }],
      baseLayout({ title: { text: 'Self-winding rate by activity (model)' }, yaxis: { title: { text: 'barrel turns / h' } }, hovermode: 'closest' }), CFG);
    const sc = D.autowinding.scenario;
    Plotly.newPlot(card(ac, 'auto-day'), [{ x: sc.t_h, y: sc.n.map(v => 100 * v / sc.n_dev), line: { color: s[0], width: 2 }, hovertemplate: '%{y:.0f} %<extra></extra>' }],
      baseLayout({ title: { text: 'Wind state over three days of wear' }, xaxis: { title: { text: 'hours' } }, yaxis: { title: { text: '% of full wind' }, range: [0, 105] }, showlegend: false }), CFG);
  }
  function besselJ1(x) {   // Numerical Recipes rational approximation
    const ax = Math.abs(x);
    if (ax < 8) {
      const y = x * x;
      const a1 = x * (72362614232.0 + y * (-7895059235.0 + y * (242396853.1 + y * (-2972611.439 + y * (15704.48260 + y * (-30.16036606))))));
      const a2 = 144725228442.0 + y * (2300535178.0 + y * (18583304.74 + y * (99447.43394 + y * (376.9991397 + y))));
      return a1 / a2;
    }
    const z = 8 / ax, y = z * z, xx = ax - 2.356194491;
    const a1 = 1 + y * (0.183105e-2 + y * (-0.3516396496e-4 + y * (0.2457520174e-5 + y * (-0.240337019e-6))));
    const a2 = 0.04687499995 + y * (-0.2002690873e-3 + y * (0.8449199096e-5 + y * (-0.88228987e-6 + y * 0.105787412e-6)));
    const ans = Math.sqrt(0.636619772 / ax) * (Math.cos(xx) * a1 - z * Math.sin(xx) * a2);
    return x < 0 ? -ans : ans;
  }

  /* ---------------------------------------------------------------- experiments tab */
  const OUT_LABEL = { rate_DU: 'Rate, dial up [s/day]', amplitude_DU: 'Amplitude, dial up [°]', power_reserve_h: 'Power reserve [h]',
    eta_total: 'Total efficiency [-]', posture_difference: 'Posture difference [s/day]', rate_CD: 'Rate, crown down [s/day]',
    amplitude_CD: 'Amplitude, crown down [°]', eta_escapement: 'Escapement efficiency [-]', beat_error_DU: 'Beat error [ms]', rate_mean_4pos: 'Mean rate, 4 positions [s/day]' };
  function expTab() {
    if (!D.oat) { $('tornado').textContent = 'Sensitivity results not computed (run: python -m twin sensitivity).'; return; }
    const so = $('exp-out'), sp = $('exp-par');
    if (!so.options.length) {
      for (const [k, l] of Object.entries(OUT_LABEL)) { const o = document.createElement('option'); o.value = k; o.textContent = l; so.appendChild(o); }
      for (const r of D.oat.table) { const o = document.createElement('option'); o.value = r.param; o.textContent = r.param; sp.appendChild(o); }
      so.addEventListener('change', expTab); sp.addEventListener('change', expTab);
    }
    const out = so.value, s = S();
    const base = D.oat.base[out];
    const rows = D.oat.table.filter(r => Number.isFinite(r[out + '_lo']) && Number.isFinite(r[out + '_hi']))
      .map(r => ({ p: r.param, lo: r[out + '_lo'] - base, hi: r[out + '_hi'] - base }))
      .sort((a, b) => Math.max(Math.abs(a.lo), Math.abs(a.hi)) - Math.max(Math.abs(b.lo), Math.abs(b.hi))).slice(-14);
    ro.observe($('tornado'));
    Plotly.newPlot($('tornado'), [
      { type: 'bar', orientation: 'h', y: rows.map(r => r.p), x: rows.map(r => r.lo), name: 'parameter −10 %', marker: { color: s[0] }, hovertemplate: '%{x:+.4g}<extra>−10 %</extra>' },
      { type: 'bar', orientation: 'h', y: rows.map(r => r.p), x: rows.map(r => r.hi), name: 'parameter +10 %', marker: { color: s[1] }, hovertemplate: '%{x:+.4g}<extra>+10 %</extra>' },
    ], baseLayout({ title: { text: `Change in ${OUT_LABEL[out]} (baseline ${Number(base).toPrecision(4)})` }, barmode: 'overlay', hovermode: 'closest',
      margin: { l: 220, r: 14, t: 34, b: 104 }, yaxis: { automargin: true, tickfont: { size: 10 } } }), CFG);
    const r = D.oat.table.find(x => x.param === sp.value) || D.oat.table[0];
    table($('exp-table'), [`${r.param}`, `${fmtNum(r.x_lo)} (−)`, `${fmtNum(r.x0)} (base)`, `${fmtNum(r.x_hi)} (+)`, 'Elasticity'],
      Object.entries(OUT_LABEL).map(([k, l]) => [l, fmtNum(r[k + '_lo']), fmtNum(D.oat.base[k]), fmtNum(r[k + '_hi']), Number.isFinite(r['S_' + k]) ? fmtNum(r['S_' + k]) : '–']), true);
    const mc = D.mc;
    const box = $('mc-charts'); box.replaceChildren();
    if (!mc) return;
    const specs = { power_reserve_h: [40, 42], rate_DU: [-20, 40], posture_difference: [50] };
    for (const k of ['power_reserve_h', 'amplitude_DU', 'rate_DU', 'posture_difference']) {
      const v = mc.outputs[k].filter(Number.isFinite);
      const sh = (specs[k] || []).map(x => ({ type: 'line', x0: x, x1: x, yref: 'paper', y0: 0, y1: 1, line: { color: tok('--s8'), width: 2 } }));
      Plotly.newPlot(card(box, 'mc-' + k), [{ type: 'histogram', x: v, marker: { color: s[0], line: { color: tok('--surface'), width: 2 } }, nbinsx: 16, hovertemplate: '%{y} runs<extra>%{x}</extra>' }],
        baseLayout({ title: { text: OUT_LABEL[k] }, xaxis: { title: { text: 'red = verified specification' } }, yaxis: { title: { text: 'runs' } }, showlegend: false, hovermode: 'closest', bargap: 0.02, shapes: sh }), CFG);
    }
  }
  function fmtNum(v) { if (v === null || v === undefined || !Number.isFinite(v)) return '–'; const a = Math.abs(v); return a !== 0 && (a < 1e-3 || a >= 1e5) ? v.toExponential(3) : Number(v.toPrecision(5)).toString(); }

  /* ---------------------------------------------------------------- validation tab */
  function valTab() {
    const v = D.validation || [];
    const n = st => v.filter(r => r.status === st).length;
    $('val-summary').replaceChildren();
    const p = document.createElement('p'); p.className = 'lede';
    p.textContent = `${n('PASS')} checks pass, ${n('FAIL')} fail, ${n('INFO')} are informational (the reference was calibrated or is itself an estimate). Each check compares the model with a specification, a closed-form relationship, or an independent computation.`;
    $('val-summary').appendChild(p);
    table($('val-table'), ['ID', 'Check', 'Governing relation', 'Expected', 'Model', 'Status', 'Basis'],
      v.map(r => [r.id, r.check, code(r.equation), r.expected, r.value, statusEl(r.status), r.basis]));
    const c = $('val-charts'); c.replaceChildren();
    if (D.bruteforce && D.bruteforce.qs_t) {
      const s = S(), bf = D.bruteforce;
      Plotly.newPlot(card(c, 'bf'), [
        { x: bf.qs_t, y: bf.qs_A, name: 'multi-time-scale model', line: { color: s[0], width: 2 }, hovertemplate: '%{y:.2f}°' },
        { x: bf.t_h, y: bf.amplitude, name: 'full event-by-event integration', line: { color: s[1], width: 1.5 }, hovertemplate: '%{y:.2f}°' },
      ], baseLayout({ title: { text: 'Amplitude: multi-time-scale model vs brute force (≈10⁶ events, blind mainspring)' }, xaxis: { title: { text: 'hours since full wind' } }, yaxis: { title: { text: 'degrees' } } }), CFG);
    }
  }

  /* ---------------------------------------------------------------- parameters tab */
  let provFilter = null;
  function parTab() {
    const f = $('prov-filter');
    if (!f.childElementCount) {
      for (const p of ['all', 'measured', 'sourced', 'inferred', 'assumed']) {
        const b = document.createElement('button'); b.className = 'chip' + (p === 'all' ? ' on' : ''); b.textContent = p;
        b.addEventListener('click', () => { provFilter = p === 'all' ? null : p; [...f.children].forEach(x => x.classList.toggle('on', x === b)); parTab(); });
        f.appendChild(b);
      }
      $('par-search').addEventListener('input', parTab);
    }
    const q = $('par-search').value.toLowerCase();
    const rows = D.parameters.filter(r => (!provFilter || r.prov === provFilter) && (!q || (r.key + r.src + r.unit).toLowerCase().includes(q)));
    table($('par-table'), ['Parameter', 'Value', 'Unit', 'Provenance', 'Range', 'Source / rationale'],
      rows.map(r => [r.key, fmtNum(r.value), r.unit, badge(r.prov), r.range ? `${fmtNum(r.range[0])} – ${fmtNum(r.range[1])}` : '', r.src]), false, { numCols: [1] });
    const dq = D.derived.filter(r => !q || (r.name + r.equation).toLowerCase().includes(q));
    table($('der-table'), ['Quantity', 'Value', 'Unit', 'Equation'], dq.map(r => [r.name, fmtNum(r.value), r.unit, code(r.equation)]), false, { numCols: [1] });
  }

  /* ---------------------------------------------------------------- about tab */
  function aboutTab() {
    const a = $('about'); a.className = 'about'; a.replaceChildren();
    const id = D.identification;
    const sec = (title, paras) => {
      const h = document.createElement('h3'); h.textContent = title; a.appendChild(h);
      for (const t of paras) { const p = document.createElement('p'); p.textContent = t; a.appendChild(p); }
    };
    sec('What this is', [
      `A physics-based digital twin of a ${id.watch_reference}, whose movement is the ${id.caliber}. The geometry, gear train, escapement and oscillator are reconstructed parametrically from photographs of the owner's watch, Miyota's published specifications and engineering estimates. Every parameter is tagged measured, sourced, inferred or assumed (see Parameters).`,
      'The 3-D model on the left is extruded in your browser from the same parametric outlines that produce the STEP CAD files. Every wheel is driven by the simulated escape-wheel motion through exact tooth-count ratios; the balance, pallet fork and escape wheel follow the event-resolved hybrid limit cycle at the current state of wind.',
    ]);
    sec('How the physics works', [
      'Balance: I θ″ + c θ′ + T_c sgn θ′ + k θ + gravity(θ) = escapement torque, with viscous (air, oil, hairspring) and Coulomb (pivot) damping and an out-of-poise gravity torque in vertical positions.',
      'Escapement: a hybrid automaton with free, unlocking, impulse (with unilateral wheel contact and catch-up impacts), drop and lock phases; every transition is located to 1 ns and impacts conserve generalised momentum. The energy ledger closes to machine precision.',
      'Mainspring: Euler-Bernoulli spiral with strength-limited full-wind torque, inter-coil friction and run-down roll-off. The train transmits torque with mesh and pivot efficiencies. Because the amplitude settles in about 30 s while the mainspring changes over hours, the power reserve is computed with a multi-time-scale method and validated against a full brute-force integration.',
    ]);
    sec('Controls', [
      'Drag to orbit, scroll to zoom, click a part to inspect it. Use Event ◀ ▶ to step through unlocking, impulse, drop and lock; Slow-mo plays the escapement at 1/500 speed; 3600× runs an hour per second so the power reserve visibly drains. Pull the crown to hack the balance; wind it or switch on wrist motion to rewind.',
    ]);
    sec('Run it yourself', ['pip install -r requirements.txt, then python -m twin run-all. Experiments: python -m twin experiment --set KEY=VALUE.']);
  }

  /* ---------------------------------------------------------------- wiring */
  function tabs() {
    const btns = document.querySelectorAll('.tabs button');
    const built = {};
    const builders = { live: liveCharts, esc: escTab, time: timeTab, exp: expTab, val: valTab, par: parTab, about: aboutTab };
    const show = name => {
      btns.forEach(b => b.setAttribute('aria-selected', String(b.dataset.tab === name)));
      document.querySelectorAll('.tabpanel').forEach(p => (p.hidden = p.id !== 'panel-' + name));
      if (!built[name]) { builders[name](); built[name] = true; }
      else window.dispatchEvent(new Event('resize'));
      try { localStorage.setItem('twin-tab', name); } catch (e) { /* storage unavailable */ }
    };
    btns.forEach(b => b.addEventListener('click', () => show(b.dataset.tab)));
    document.querySelectorAll('[data-pt]').forEach(b => b.addEventListener('click', () => {
      ptState = b.dataset.pt; document.querySelectorAll('[data-pt]').forEach(x => x.classList.toggle('on', x === b)); timeTab();
    }));
    let start = 'live';
    try { start = localStorage.getItem('twin-tab') || 'live'; } catch (e) { /* ignore */ }
    if (location.hash && builders[location.hash.slice(1)]) start = location.hash.slice(1);
    show(start);
    return { rebuild() { for (const k of Object.keys(built)) delete built[k]; show(document.querySelector('.tabs [aria-selected="true"]').dataset.tab); } };
  }

  return { buildKpis, updateKpis, updateLive, liveCharts, tabs, lossBars };
})();
