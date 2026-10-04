/* ============================================================================================
   viewer.js - three.js rendering of the parametric CAD (outlines extruded in the browser).
   Units: millimetres. +z points toward the dial (viewer), rotations are CCW-positive seen from
   the dial, exactly as in the Python model, so ratios can be applied without sign juggling.
   ============================================================================================ */
const Viewer = (() => {
  const el = document.getElementById('viewport');
  const css = () => getComputedStyle(document.documentElement);
  const parts = [];            // {def, group, meshes, baseZ, material}
  const byName = {};
  let scene, camera, renderer, labelRenderer, controls, raycaster, pointer;
  let hsLine, msLine, msN = -1, arrows = [], labels = [], flowLabels = [];
  let selected = null, isolated = null;
  const S = { explode: 0, skeleton: true, show: { structure: true, dialcase: true, rotor: true }, labels: false, arrows: false, flow: false };

  const MAT = {
    steel: { metalness: 0.45, roughness: 0.32 }, brass: { metalness: 0.45, roughness: 0.38 }, CuBe: { metalness: 0.5, roughness: 0.3 },
    ruby: { metalness: 0.1, roughness: 0.15 }, 'brass-rhodium': { metalness: 0.25, roughness: 0.5 }, 'Fe-Ni': { metalness: 0.5, roughness: 0.25 },
    '316L': { metalness: 0.5, roughness: 0.3 }, sapphire: { metalness: 0.0, roughness: 0.05 }, 'mineral glass': { metalness: 0, roughness: 0.05 },
    'brass-lacquer': { metalness: 0.3, roughness: 0.6 },
  };
  const STRUCT_GROUPS = new Set(['structure', 'jewels']);
  const DIALCASE = new Set(['dial', 'hands', 'case']);

  function ringToPts(arr, ShapeOrPath) {
    const s = new ShapeOrPath();
    for (let i = 0; i < arr.length; i += 2) {
      const x = arr[i] / 1000, y = arr[i + 1] / 1000;
      if (i === 0) s.moveTo(x, y); else s.lineTo(x, y);
    }
    return s;
  }

  function buildPart(def) {
    const group = new THREE.Group();
    group.position.set(def.c[0] / 1000, def.c[1] / 1000, 0);
    group.name = def.name;
    const z0 = def.z[0] / 1000, z1 = def.z[1] / 1000;
    const m = MAT[def.material] || MAT.steel;
    const material = new THREE.MeshStandardMaterial({
      color: new THREE.Color(def.color), metalness: m.metalness, roughness: m.roughness,
      transparent: def.opacity < 1, opacity: def.opacity, side: THREE.DoubleSide,
      emissive: new THREE.Color(def.material === 'ruby' ? '#3a0010' : '#000000'),
    });
    const meshes = [];
    for (const sh of def.shapes) {
      const shape = ringToPts(sh.outer, THREE.Shape);
      for (const h of sh.holes) shape.holes.push(ringToPts(h, THREE.Path));
      const geom = new THREE.ExtrudeGeometry(shape, { depth: z1 - z0, bevelEnabled: false, curveSegments: 1 });
      geom.translate(0, 0, z0);
      const mesh = new THREE.Mesh(geom, material);
      mesh.userData.part = def.name;
      group.add(mesh);
      meshes.push(mesh);
    }
    const p = { def, group, meshes, material, baseOpacity: def.opacity, zmid: (z0 + z1) / 2 };
    parts.push(p);
    byName[def.name] = p;
    scene.add(group);
    return p;
  }

  function spiralPoints(rIn, rOut, turns, a0, twist, n = 900) {
    const pts = [];
    for (let i = 0; i <= n; i++) {
      const s = i / n;
      const r = rIn + (rOut - rIn) * s;
      const a = a0 + TAU * turns * s + twist * (1 - s);
      pts.push(new THREE.Vector3(r * Math.cos(a), r * Math.sin(a), 0));
    }
    return pts;
  }

  function mainspringPoints(n) {
    const sp = V.spring, mm = 1000;
    const x = Math.max(0, Math.min(1, n / sp.n_dev));
    const L = sp.L, e = sp.e;
    const Larb = (0.08 + 0.84 * x) * L, Lwall = L - Larb;
    const pts = [];
    let s = 0, ph = 0;
    while (s < Larb) { const r = sp.r + e * ph / TAU; pts.push([r, ph]); const d = TAU / 60; s += r * d; ph += d; }
    const wall = [];
    let s2 = 0, ph2 = ph + 0.6;
    while (s2 < Lwall) { const r = sp.R - e * (s2 / (TAU * sp.R)); wall.push([r, ph2]); const d = TAU / 60; s2 += r * d; ph2 += d; }
    for (let i = wall.length - 1; i >= 0; i--) pts.push(wall[i]);
    return pts.map(([r, a]) => new THREE.Vector3(r * mm * Math.cos(a), r * mm * Math.sin(a), 0));
  }

  function makeLabel(text, cls = 'label3d') {
    const div = document.createElement('div');
    div.className = cls;
    div.textContent = text;
    return new THREE.CSS2DObject(div);
  }

  function arrowFor(p, speedLog) {
    const r = Math.max(0.9, Math.min(4.2, 0.6 + 0.45 * speedLog));
    const dir = Math.sign(V.omega[p.def.arbor] || 0);
    const g = new THREE.Group();
    const curve = new THREE.EllipseCurve(0, 0, r, r, 0.3, 0.3 + Math.PI * 1.25, false, 0);
    const pts = curve.getPoints(48).map(v => new THREE.Vector3(v.x, v.y, 0));
    const col = new THREE.Color(css().getPropertyValue('--accent').trim() || '#b4123f');
    g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: col })));
    const end = dir > 0 ? pts[pts.length - 1] : pts[0];
    const prev = dir > 0 ? pts[pts.length - 2] : pts[1];
    const cone = new THREE.Mesh(new THREE.ConeGeometry(0.16, 0.45, 12), new THREE.MeshBasicMaterial({ color: col }));
    cone.position.copy(end);
    const t = new THREE.Vector3().subVectors(end, prev).normalize();
    cone.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), t);
    g.add(cone);
    g.position.set(p.def.c[0] / 1000, p.def.c[1] / 1000, (p.def.z[1] / 1000) + 0.25);
    g.userData.p = p;
    g.userData.z0 = g.position.z;
    g.visible = false;
    scene.add(g);
    arrows.push(g);
  }

  function init() {
    scene = new THREE.Scene();
    const w = el.clientWidth, h = el.clientHeight;
    camera = new THREE.PerspectiveCamera(32, w / h, 0.5, 400);
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setSize(w, h);
    renderer.outputEncoding = THREE.sRGBEncoding;
    el.appendChild(renderer.domElement);
    labelRenderer = new THREE.CSS2DRenderer();
    labelRenderer.setSize(w, h);
    labelRenderer.domElement.style.position = 'absolute';
    labelRenderer.domElement.style.inset = '0';
    labelRenderer.domElement.style.pointerEvents = 'none';
    el.appendChild(labelRenderer.domElement);
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.minDistance = 8; controls.maxDistance = 200;
    scene.add(new THREE.HemisphereLight(0xffffff, 0x445566, 0.85));
    const d1 = new THREE.DirectionalLight(0xffffff, 0.9); d1.position.set(30, 40, 60); scene.add(d1);
    const d2 = new THREE.DirectionalLight(0xffffff, 0.5); d2.position.set(-40, -20, -50); scene.add(d2);
    const d3 = new THREE.PointLight(0xfff2dd, 0.5, 200); d3.position.set(0, 0, 30); scene.add(d3);

    for (const def of G.parts) buildPart(def);
    // dynamic hairspring and mainspring replace their static CAD bodies
    const hs = byName.hairspring;
    if (hs) {
      hs.meshes.forEach(m => (m.visible = false));
      const a0 = hs.def.meta.a0 || 0;
      const geo = new THREE.BufferGeometry().setFromPoints(spiralPoints(V.hairspring.r_in * 1000, V.hairspring.r_out * 1000, V.hairspring.turns, a0, 0));
      hsLine = new THREE.Line(geo, new THREE.LineBasicMaterial({ color: new THREE.Color('#5b8fd6') }));
      hsLine.position.set(hs.def.c[0] / 1000, hs.def.c[1] / 1000, (hs.def.z[0] + hs.def.z[1]) / 2000);
      hsLine.userData.a0 = a0;
      scene.add(hsLine);
    }
    const ms = byName.mainspring;
    if (ms) {
      ms.meshes.forEach(m => (m.visible = false));
      msLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: new THREE.Color('#8fa3b8') }));
      msLine.position.set(ms.def.c[0] / 1000, ms.def.c[1] / 1000, (ms.def.z[0] + ms.def.z[1]) / 2000);
      scene.add(msLine);
    }
    // labels for the principal assemblies
    const LABELS = { barrel_teeth: 'Barrel', centre_wheel: 'Centre wheel', third_wheel: 'Third wheel', fourth_wheel: 'Fourth wheel',
      escape_wheel: 'Escape wheel', pallet_fork: 'Pallet fork', balance_rim: 'Balance', ratchet_wheel: 'Ratchet', rotor: 'Rotor',
      reduction_wheel: 'Reduction wheel', minute_wheel: 'Minute wheel', train_bridge: 'Train bridge', balance_cock: 'Balance cock' };
    for (const [name, text] of Object.entries(LABELS)) {
      const p = byName[name];
      if (!p) continue;
      const lab = makeLabel(text);
      lab.position.set(p.def.c[0] / 1000, p.def.c[1] / 1000, p.def.z[1] / 1000 + 0.3);
      if (name === 'rotor') lab.position.set(4, 6, p.def.z[0] / 1000 - 0.3);
      lab.visible = false;
      lab.userData.part = name;
      lab.userData.z0 = lab.position.z;
      scene.add(lab);
      labels.push(lab);
    }
    for (const name of ['barrel_teeth', 'centre_wheel', 'third_wheel', 'fourth_wheel', 'escape_wheel', 'seconds_pinion']) {
      const p = byName[name];
      if (!p) continue;
      const w = Math.abs(V.omega[p.def.arbor] || 1e-6);
      arrowFor(p, Math.log10(w * 1e5));
    }
    for (const [name, stage] of [['barrel_teeth', 0], ['centre_wheel', 2], ['third_wheel', 4], ['fourth_wheel', 6], ['escape_wheel', 8]]) {
      const p = byName[name];
      if (!p) continue;
      const lab = makeLabel('', 'label3d flow');
      lab.position.set(p.def.c[0] / 1000 + 0.6, p.def.c[1] / 1000 - 0.6, p.def.z[1] / 1000 + 0.4);
      lab.userData.stage = stage;
      lab.userData.part = name;
      lab.userData.z0 = lab.position.z;
      lab.visible = false;
      scene.add(lab);
      flowLabels.push(lab);
    }
    raycaster = new THREE.Raycaster();
    pointer = new THREE.Vector2();
    renderer.domElement.addEventListener('pointerup', onPick);
    window.addEventListener('resize', resize);
    new ResizeObserver(resize).observe(el);
    view('iso');
    document.getElementById('loading').hidden = true;
  }

  let downAt = 0;
  el.addEventListener('pointerdown', () => { downAt = performance.now(); });
  function onPick(ev) {
    if (performance.now() - downAt > 300) return;  // a drag, not a click
    const r = renderer.domElement.getBoundingClientRect();
    pointer.x = ((ev.clientX - r.left) / r.width) * 2 - 1;
    pointer.y = -((ev.clientY - r.top) / r.height) * 2 + 1;
    raycaster.setFromCamera(pointer, camera);
    const vis = [];
    for (const p of parts) if (p.group.visible) for (const m of p.meshes) if (m.visible) vis.push(m);
    const hit = raycaster.intersectObjects(vis, false)[0];
    select(hit ? byName[hit.object.userData.part] : null);
  }

  function select(p) {
    if (selected) selected.material.emissive.set(selected.def.material === 'ruby' ? '#3a0010' : '#000000');
    selected = p;
    const box = document.getElementById('selinfo');
    document.getElementById('btn-isolate').disabled = !p && !isolated;
    if (!p) { box.hidden = true; return; }
    p.material.emissive.set('#2a3550');
    box.hidden = false;
    box.replaceChildren();
    const h = document.createElement('h4'); h.textContent = p.def.label; box.appendChild(h);
    const dl = document.createElement('dl');
    const add = (k, v) => { const dt = document.createElement('dt'); dt.textContent = k; const dd = document.createElement('dd'); dd.textContent = v; dl.append(dt, dd); };
    add('Group', p.def.group);
    if (p.def.part_no) add('Miyota part no.', p.def.part_no);
    add('Material', p.def.material);
    add('Axial band', `${(p.def.z[0] / 1000).toFixed(2)} … ${(p.def.z[1] / 1000).toFixed(2)} mm`);
    if (p.def.motion === 'train' && p.def.arbor) {
      const w = V.omega[p.def.arbor];
      add('Speed', `${fmtRpm(w * 60 / TAU)}  ${w < 0 ? '↻ CW' : '↺ CCW'} (dial view)`);
      add('Period', fmtPeriod(TAU / Math.abs(w)));
    } else if (p.def.motion === 'balance') add('Motion', '3 Hz oscillation (balance)');
    else if (p.def.motion === 'fork') add('Motion', '±' + (V.phi_B * 180 / Math.PI).toFixed(2) + '° between bankings');
    else add('Motion', p.def.motion);
    box.appendChild(dl);
  }

  function fmtRpm(r) { const a = Math.abs(r); return (a >= 1 ? a.toFixed(2) : a.toExponential(3)) + ' rpm'; }
  function fmtPeriod(s) { return s < 120 ? s.toFixed(2) + ' s' : s < 7200 ? (s / 60).toFixed(2) + ' min' : (s / 3600).toFixed(2) + ' h'; }

  function resize() {
    const w = el.clientWidth, h = el.clientHeight;
    if (!w || !h) return;
    camera.aspect = w / h; camera.updateProjectionMatrix();
    renderer.setSize(w, h); labelRenderer.setSize(w, h);
  }

  function view(kind) {
    controls.target.set(0, 0, -1.5);
    const k = Math.max(1, 1.15 / Math.max(camera.aspect, 0.3));     // back off on narrow screens
    if (kind === 'dial') camera.position.set(0, -0.01, 62 * k);
    else if (kind === 'back') camera.position.set(0, 0.01, -62 * k);
    else camera.position.set(-28 * k, -42 * k, 34 * k);
    camera.up.set(0, 0, 1);
    if (kind === 'dial' || kind === 'back') camera.up.set(0, 1, 0);
    controls.update();
  }

  function isVisibleByToggles(p) {
    const g = p.def.group;
    if (STRUCT_GROUPS.has(g) && !S.show.structure) return false;
    if (DIALCASE.has(g) && !S.show.dialcase) return false;
    if (p.def.name === 'rotor' && !S.show.rotor) return false;
    if (isolated) return isolated.has(p.def.name);
    return true;
  }

  function applyVisibility() {
    for (const p of parts) {
      p.group.visible = isVisibleByToggles(p);
      const ghost = S.skeleton && (STRUCT_GROUPS.has(p.def.group) || DIALCASE.has(p.def.group) || p.def.name === 'rotor');
      p.material.transparent = ghost || p.baseOpacity < 1;
      p.material.opacity = ghost ? Math.min(0.14, p.baseOpacity) : p.baseOpacity;
      p.material.depthWrite = !ghost && p.baseOpacity >= 0.5;
    }
    if (hsLine) hsLine.visible = !isolated || isolated.has('hairspring');
    if (msLine) msLine.visible = !isolated || isolated.has('mainspring');
    labels.forEach(l => (l.visible = S.labels && byName[l.userData.part].group.visible));
    arrows.forEach(a => (a.visible = S.arrows));
    flowLabels.forEach(l => (l.visible = S.flow));
  }

  function toggleIsolate() {
    if (isolated) { isolated = null; }
    else if (selected) {
      const d = selected.def;
      isolated = new Set(parts.filter(p => (d.arbor && p.def.arbor === d.arbor && p.def.motion === d.motion) ||
        (d.motion !== 'train' && d.motion !== 'static' && p.def.motion === d.motion) || p.def.name === d.name).map(p => p.def.name));
      if (d.motion === 'balance') isolated.add('hairspring');
    }
    document.getElementById('btn-isolate').textContent = isolated ? 'Show all' : 'Isolate selection';
    document.getElementById('btn-isolate').disabled = !isolated && !selected;
    applyVisibility();
  }

  let lastReal = performance.now();
  function frame(now) {
    const dt = (now - lastReal) / 1000;
    lastReal = now;
    Twin.step(dt);
    const k = Twin.kinematics();
    const psi = k.psi;
    const theta = Twin.hacked ? (Twin._frozen ?? (Twin._frozen = k.theta)) : (Twin._frozen = undefined, k.theta);
    const wristAngle = Twin.wrist ? 1.3 * Math.sin(now / 1000 * TAU * 0.9) : 0;
    const ex = S.explode;
    for (const p of parts) {
      const d = p.def;
      let rot = 0;
      if (d.motion === 'train') rot = (RATIO[d.arbor] ?? 0) * psi;
      else if (d.motion === 'balance') rot = theta;
      else if (d.motion === 'fork') rot = -k.phi - V.phi_B;
      else if (d.motion === 'rotor') rot = wristAngle;
      else if (d.motion === 'auto') rot = d.ratio * wristAngle;
      else if (d.motion === 'crown') rot = d.name === 'ratchet_wheel' ? Twin.ratchet : -Twin.ratchet * 56 / 24;
      if (d.group === 'hands') {
        const h0 = Twin.clock0;
        const base = d.name === 'hour_hand' ? (h0 % 43200) / 43200 : d.name === 'minute_hand' ? (h0 % 3600) / 3600 : (h0 % 60) / 60;
        rot += -base * TAU;
      }
      p.group.rotation.z = rot;
      // explode: stretch the axial stack; dial side up, caseback side down
      const extra = DIALCASE.has(d.group) ? Math.sign(p.zmid + 0.001) * 6 : d.group === 'automatic winding' ? -5 : 0;
      p.group.position.z = ex * (p.zmid * 3.2 + extra);
      if (['stem', 'clutch', 'winding_pinion', 'crown'].includes(d.name)) p.group.position.x = Twin.hacked ? 0.9 : 0;
    }
    if (hsLine) {
      const pts = spiralPoints(V.hairspring.r_in * 1000, V.hairspring.r_out * 1000, V.hairspring.turns, hsLine.userData.a0, theta);
      hsLine.geometry.setFromPoints(pts);
      hsLine.position.z = (byName.hairspring.zmid) * (1 + 3.2 * ex);
    }
    if (msLine && Math.abs(Twin.n - msN) > 0.01) {
      msN = Twin.n;
      msLine.geometry.dispose();
      msLine.geometry = new THREE.BufferGeometry().setFromPoints(mainspringPoints(Twin.n));
    }
    if (msLine) {
      msLine.rotation.z = (RATIO.barrel ?? 0) * psi;
      msLine.position.z = byName.mainspring.zmid * (1 + 3.2 * ex);
    }
    for (const a of arrows) a.position.z = a.userData.z0 + a.userData.p.group.position.z;
    for (const l of labels.concat(flowLabels)) l.position.z = l.userData.z0 + (byName[l.userData.part]?.group.position.z || 0);
    controls.update();
    renderer.render(scene, camera);
    labelRenderer.render(scene, camera);
    requestAnimationFrame(frame);
  }

  function updateFlowLabels(rows, scale) {
    for (const l of flowLabels) {
      const r = rows[l.userData.stage];
      if (!r) continue;
      const T = r.T_out * scale, P = r.P_out * scale;
      l.element.textContent = `${fmtTorque(T)} · ${(P * 1e6).toFixed(3)} µW`;
    }
  }
  function fmtTorque(T) {
    const a = Math.abs(T);
    return a >= 1e-4 ? (a * 1e3).toFixed(3) + ' N·mm' : (a * 1e6).toFixed(3) + ' µN·m';
  }

  return {
    init, view, select, toggleIsolate, applyVisibility, updateFlowLabels, S,
    start() { requestAnimationFrame(t => { lastReal = t; frame(t); }); },
  };
})();
