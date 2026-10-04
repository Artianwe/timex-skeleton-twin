/* ============================================================================================
   main.js - wires the controls to the twin state, the 3-D viewer and the dashboard.
   ============================================================================================ */
(function main() {
  const $ = id => document.getElementById(id);
  const speedFromSlider = v => Math.pow(10, (v - 50) / 50 * 3.6);     // 10^-3.6 … 10^3.6 (≈0.00025× … 3981×)
  const sliderFromSpeed = s => 50 + 50 * Math.log10(s) / 3.6;
  const fmtSpeed = s => s >= 10 ? s.toFixed(0) + '×' : s >= 1 ? s.toFixed(1) + '×' : s >= 0.01 ? s.toFixed(3) + '×' : s.toExponential(1) + '×';

  Viewer.init();
  Dash.buildKpis();
  const T = Dash.tabs();

  function setSpeed(s) { Twin.speed = s; $('speed').value = sliderFromSpeed(s); $('speed-out').textContent = fmtSpeed(s); }
  setSpeed(1);
  $('speed').addEventListener('input', e => setSpeed(speedFromSlider(+e.target.value)));
  document.querySelectorAll('[data-speed]').forEach(b => b.addEventListener('click', () => { setSpeed(+b.dataset.speed); Twin.playing = true; syncPlay(); }));
  function syncPlay() { $('btn-play').textContent = Twin.playing ? 'Pause' : 'Play'; $('btn-play').setAttribute('aria-pressed', String(Twin.playing)); }
  $('btn-play').addEventListener('click', () => { Twin.playing = !Twin.playing; Twin.eventMsg = ''; syncPlay(); });
  $('btn-next').addEventListener('click', () => { Twin.jumpEvent(1); syncPlay(); });
  $('btn-prev').addEventListener('click', () => { Twin.jumpEvent(-1); syncPlay(); });
  $('position').addEventListener('change', e => { Twin.pos = e.target.value; Dash.liveCharts(); });
  $('btn-wind').addEventListener('click', () => Twin.wind(10 * V.keyless_ratio));
  $('btn-full').addEventListener('click', () => Twin.wind(V.n_dev));
  $('btn-hack').addEventListener('click', e => { Twin.hacked = !Twin.hacked; e.target.setAttribute('aria-pressed', String(Twin.hacked)); e.target.textContent = Twin.hacked ? 'Push crown in' : 'Pull crown (hack)'; });
  $('btn-wrist').addEventListener('click', e => { Twin.wrist = !Twin.wrist; e.target.setAttribute('aria-pressed', String(Twin.wrist)); });
  const vs = Viewer.S;
  $('t-skeleton').addEventListener('change', e => { vs.skeleton = e.target.checked; Viewer.applyVisibility(); });
  $('t-bridges').addEventListener('change', e => { vs.show.structure = e.target.checked; Viewer.applyVisibility(); });
  $('t-dial').addEventListener('change', e => { vs.show.dialcase = e.target.checked; Viewer.applyVisibility(); });
  $('t-rotor').addEventListener('change', e => { vs.show.rotor = e.target.checked; Viewer.applyVisibility(); });
  $('t-labels').addEventListener('change', e => { vs.labels = e.target.checked; Viewer.applyVisibility(); });
  $('t-arrows').addEventListener('change', e => { vs.arrows = e.target.checked; Viewer.applyVisibility(); });
  $('t-flow').addEventListener('change', e => { vs.flow = e.target.checked; Viewer.applyVisibility(); });
  $('explode').addEventListener('input', e => { vs.explode = (+e.target.value) / 100; });
  $('btn-view-dial').addEventListener('click', () => Viewer.view('dial'));
  $('btn-view-back').addEventListener('click', () => Viewer.view('back'));
  $('btn-view-iso').addEventListener('click', () => Viewer.view('iso'));
  $('btn-isolate').addEventListener('click', () => Viewer.toggleIsolate());
  // sensible opening view for a skeleton watch: dial and case hidden, labels on
  $('t-dial').checked = false; vs.show.dialcase = false;
  $('t-skeleton').checked = true; vs.skeleton = true;
  Viewer.applyVisibility();

  const pad = n => String(Math.floor(n)).padStart(2, '0');
  let lastUi = 0, lastCharts = 0;
  function ui(now) {
    if (now - lastUi > 120) {
      lastUi = now;
      const w = ((Twin.watchTimeSeconds() % 86400) + 86400) % 86400;
      $('clock').textContent = `${pad(w / 3600)}:${pad((w % 3600) / 60)}:${pad(w % 60)}`;
      $('hud-state').textContent = Twin.hacked ? 'Hacked (balance stopped)' : Twin.running() ? `Running · ${Twin.at('A').toFixed(0)}° amplitude` : 'Stopped: mainspring let down';
      $('hud-pos').textContent = POS_LABEL[Twin.pos];
      $('hud-event').textContent = Twin.playing ? '' : Twin.eventMsg;
      Dash.updateKpis();
      const rows = D.stage_torques;
      Viewer.updateFlowLabels(rows, Twin.at('T') / V.M_full);
    }
    if (now - lastCharts > 700 && !document.getElementById('panel-live').hidden) { lastCharts = now; Dash.updateLive(); }
    requestAnimationFrame(ui);
  }
  Viewer.start();
  requestAnimationFrame(ui);
  // re-theme charts when the viewer's theme changes
  const mq = window.matchMedia('(prefers-color-scheme: dark)');
  const rebuild = () => setTimeout(() => T.rebuild(), 30);
  mq.addEventListener?.('change', rebuild);
  new MutationObserver(rebuild).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
})();
