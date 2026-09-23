// Sheet Chassis Studio - application shell.
import { Viewer, THREE } from './render/scene.js';
import { ChassisView } from './render/chassisView.js';
import { SuspensionView } from './render/suspensionView.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { PRESET_LIST, makePreset } from './vehicle/presets.js';
import { buildChassis, GROUPS, GROUP_LABELS } from './chassis/mesh.js';
import { massModel } from './fea/model.js';
import { MATERIALS, TUBE_MATERIALS } from './fea/materials.js';
import { makeCorner, analyseAxle, HARDPOINTS } from './suspension/kinematics.js';
import { massProperties, rideRoll, balanceWeight } from './vehicle/dynamics.js';
import { optimiseCorner, STUDIES, SUSP_METRICS } from './optim/suspension.js';
import { STANDARD_GAUGES } from './optim/structure.js';
import { Jobs } from './app/jobs.js';
import { cutList, hardpointsCSV, nastranBDF } from './app/exporters.js';
import { h, $, section, num, check, select, kpi, fmt, fmtK, statusDot, download, safeStorage } from './ui/dom.js';
import { lineChart, barChart } from './ui/charts.js';
import { GROUP_COLORS } from './render/chassisView.js';

const STORE_KEY = 'sheet-chassis-cfg-v1';
const store = safeStorage();

const ANALYSIS_DEFAULTS = { torque: 1000, bendLoad: 5000, solver: 'cpu', modalWithMasses: false, nModes: 6, targetK: 30000 };

const state = {
  cfg: null,
  mesh: null,
  kin: null,
  mp: null,
  ride: null,
  fea: null,
  feaStale: false,
  leftTab: 'chassis',
  rightTab: 'fea',
  suspAxle: 'front',
  kinAxle: 'front',
  loadcase: 'torsion',
  field: 'material',
  deformOn: true,
  smooth: true,
  deformScale: null,
  modeIdx: null,
  selHp: null,
  optim: { susp: null, gauges: null, sweep: null, balance: null },
  gpu: { available: null, label: null },
};

// ------------------------------------------------------------------ boot
const viewer = await new Viewer($('#viewport')).init();
const chassisView = new ChassisView(viewer);
const suspView = new SuspensionView(viewer);
const jobs = new Jobs();

const badge = $('#gpuBadge');
badge.textContent = viewer.isWebGPU ? 'WebGPU renderer' : 'WebGL2 fallback';
badge.classList.add(viewer.isWebGPU ? 'gpu' : 'gl');
badge.title = viewer.isWebGPU ? 'Rendering with three.js WebGPURenderer (WebGPU backend)' : 'WebGPU unavailable - three.js WebGPURenderer is using its WebGL2 backend';
jobs.run('probe').then((r) => {
  state.gpu = { available: r.gpu, label: r.label };
  if (r.gpu) badge.title += ` · compute: ${r.label}`;
  if (state.rightTab === 'fea') renderRight();
}).catch(() => {});

function loadCfg() {
  const saved = store?.getItem(STORE_KEY);
  if (saved) {
    try {
      const c = JSON.parse(saved);
      if (c?.chassis && c?.suspension) return normalise(c);
    } catch { /* ignore */ }
  }
  return normalise(makePreset('supercar'));
}
function normalise(c) {
  const base = makePreset(c.type || 'supercar');
  const out = { ...base, ...c };
  out.vehicle = { ...base.vehicle, ...c.vehicle };
  out.chassis = { ...base.chassis, ...c.chassis };
  out.gauges = { ...base.gauges, ...c.gauges };
  out.tubes = { ...base.tubes, ...c.tubes };
  out.ride = { ...base.ride, ...c.ride };
  out.analysis = { ...ANALYSIS_DEFAULTS, ...(c.analysis || {}) };
  return out;
}
let saveTimer = null;
function persist() {
  clearTimeout(saveTimer);
  saveTimer = setTimeout(() => { try { store?.setItem(STORE_KEY, JSON.stringify(state.cfg)); } catch { /* quota */ } }, 300);
}

const cfg = () => state.cfg;

// ------------------------------------------------------------------ model updates
function chassisMassCG(mesh) {
  const mm = massModel(mesh.model);
  const n = mesh.model.nodes;
  let M = 0; const c = [0, 0, 0];
  for (let i = 0; i < n.length / 3; i++) { const w = mm.m[6 * i]; M += w; for (let k = 0; k < 3; k++) c[k] += w * n[3 * i + k]; }
  return { mass: mm.total, cg: c.map((v) => v / M), groups: mm.groupMass };
}

function computeKinematics() {
  const c = state.cfg;
  const cgH = state.mp ? state.mp.cg[2] : 400;
  const veh = (isFront) => ({ wheelbase: c.vehicle.wheelbase, cgH, brakeFront: c.vehicle.brakeFront, driveFront: c.vehicle.driveFront, isFront });
  state.kin = {
    front: analyseAxle(makeCorner(c.suspension.front), { travel: [-50, 50], steps: 21, rollMax: 3, rackMax: c.suspension.front.rackMax, veh: veh(true) }),
    rear: analyseAxle(makeCorner(c.suspension.rear), { travel: [-50, 50], steps: 21, rollMax: 3, rackMax: c.suspension.rear.rackMax || 0, veh: veh(false) }),
  };
}

function computeDynamics() {
  if (!state.mesh || !state.kin?.front?.ok || !state.kin?.rear?.ok) { state.ride = null; return; }
  state.chassisMass = chassisMassCG(state.mesh);
  state.mp = massProperties(state.cfg, state.chassisMass, state.kin);
  const K = state.fea && !state.feaStale && state.fea.torsion ? state.fea.torsion.K : null;
  state.ride = rideRoll(state.cfg, state.mp, state.kin, K);
}

function rebuildChassis() {
  try {
    const t0 = performance.now();
    state.mesh = buildChassis(state.cfg);
    chassisView.setMesh(state.mesh);
    const st = state.mesh.stats;
    $('#meshStats').textContent = `${st.nodes.toLocaleString()} nodes · ${st.shells.toLocaleString()} shells · ${st.beams} beams · ${st.dofs.toLocaleString()} dof · mesh ${(performance.now() - t0).toFixed(0)} ms`;
    const b = chassisView.bounds();
    b.expandByPoint(new THREE.Vector3(0, 900, 0)).expandByPoint(new THREE.Vector3(-state.cfg.vehicle.wheelbase, -900, 0));
    viewer.setBounds(b);
    if (state.fea) state.feaStale = true;
    chassisView.setDeformation(null);
    chassisView.animateMode(null);
    applyField();
  } catch (e) {
    setStatus(`Chassis build failed: ${e.message}`, 'bad');
  }
}

let chassisTimer = null;
function onChassisChange(immediate = false) {
  persist();
  clearTimeout(chassisTimer);
  const run = () => { rebuildChassis(); computeDynamics(); refreshPanels(); };
  if (immediate) run(); else chassisTimer = setTimeout(run, 140);
}

function onSuspChange(final = true) {
  persist();
  computeKinematics();
  suspView.setConfig(state.cfg);
  if (final) onChassisChange();
  else if (state.rightTab === 'kin') renderRightThrottled();
}

function onVehicleChange() {
  persist();
  computeDynamics();
  computeKinematics();
  refreshPanels();
}

function fullReload() {
  suspOptState.targets = null;
  gaugeState.groups = null;
  gaugeState.hist = [];
  sweepState.lo = null;
  state.fea = null;
  state.feaStale = false;
  state.optim = { susp: null, gauges: null, sweep: null, balance: null };
  state.selHp = null;
  chassisView.select(null);
  detachGizmo();
  computeKinematics();
  rebuildChassis();
  computeDynamics();
  computeKinematics();
  suspView.setConfig(state.cfg);
  $('#cfgName').textContent = state.cfg.name;
  $('#presetSelect').value = state.cfg.type;
  renderLeft();
  renderRight();
  viewer.view('iso');
}

function refreshPanels() {
  if (state.leftTab === 'optimise') renderLeft();
  renderRight();
}

// ------------------------------------------------------------------ status
function setStatus(msg, level = '') {
  const s = $('#statusText');
  s.textContent = msg;
  s.style.color = level === 'bad' ? 'var(--bad)' : level === 'good' ? 'var(--good)' : '';
}
function setProgress(p) {
  const bar = $('#progress');
  if (p === null) { bar.classList.add('hidden'); return; }
  bar.classList.remove('hidden');
  bar.classList.toggle('indet', p < 0);
  $('#progressBar').style.width = p < 0 ? '' : `${Math.round(p * 100)}%`;
}

// ------------------------------------------------------------------ toolbar
function buildToolbar() {
  const tb = $('#viewToolbar');
  tb.innerHTML = '';
  const tog = (label, get, set, title) => {
    const b = h('button', { class: get() ? 'on' : '', title }, label);
    b.addEventListener('click', () => { set(!get()); b.classList.toggle('on', get()); });
    return b;
  };
  const cv = chassisView.opts, sv = suspView.opts;
  const layout = h('div', { class: 'grp' }, h('span', {}, 'Layout'),
    ...['single', 'quad'].map((m) => {
      const b = h('button', { class: viewer.mode === m ? 'on' : '' }, m === 'single' ? '3D' : '4-view');
      b.addEventListener('click', () => { viewer.setMode(m); tb.querySelectorAll('[data-layout]').forEach((x) => x.classList.toggle('on', x.dataset.layout === m)); drawViewLabels(); });
      b.dataset.layout = m;
      return b;
    }));
  const views = h('div', { class: 'grp' }, h('span', {}, 'View'), ...['iso', 'front', 'side', 'top', 'rear', 'under'].map((v) => {
    const b = h('button', {}, v[0].toUpperCase() + v.slice(1));
    b.addEventListener('click', () => viewer.view(v));
    return b;
  }));
  const show = h('div', { class: 'grp' }, h('span', {}, 'Show'),
    tog('Panels', () => cv.panels, (v) => chassisView.setOption('panels', v)),
    tog('Edges', () => cv.edges, (v) => chassisView.setOption('edges', v)),
    tog('Mesh', () => cv.wire, (v) => chassisView.setOption('wire', v), 'Finite-element mesh lines'),
    tog('Tubes', () => cv.tubes, (v) => chassisView.setOption('tubes', v)),
    tog('Pick-ups', () => cv.hardpoints, (v) => chassisView.setOption('hardpoints', v), 'Suspension pick-ups and their bracket links'),
    tog('Suspension', () => sv.suspension, (v) => suspView.setOption('suspension', v)),
    tog('Wheels', () => sv.wheels, (v) => suspView.setOption('wheels', v)),
    tog('RC / IC', () => sv.construction, (v) => suspView.setOption('construction', v), 'Front-view instant centres, roll centres and roll axis'),
    tog('X-ray', () => cv.xray, (v) => chassisView.setOption('xray', v)));
  const fieldSel = h('select', { id: 'fieldSel' },
    [['material', 'Aluminium'], ['groups', 'Panel groups'], ['thickness', 'Thickness'], ['disp', 'Displacement'], ['vm', 'von Mises stress'], ['sed', 'Strain energy density']].map(([v, t]) => h('option', { value: v }, t)));
  fieldSel.value = state.field;
  fieldSel.addEventListener('change', () => { state.field = fieldSel.value; applyField(); if (state.rightTab === 'fea') renderRight(); });
  const colour = h('div', { class: 'grp' }, h('span', {}, 'Colour'), fieldSel);
  tb.append(layout, views, colour, show);
}

function drawViewLabels() {
  const el = $('#viewLabels');
  el.innerHTML = '';
  if (viewer.mode !== 'quad') return;
  el.append(h('div', { class: 'div-h' }), h('div', { class: 'div-v' }),
    h('div', { class: 'vl', style: { left: '0', top: '0' } }, 'Front'),
    h('div', { class: 'vl', style: { left: '50%', top: '0' } }, 'Side (left)'),
    h('div', { class: 'vl', style: { left: '0', top: '50%' } }, 'Plan'),
    h('div', { class: 'vl', style: { left: '50%', top: '50%' } }, 'Perspective'));
}

function updatePoseHud() {
  const p = suspView.pose;
  $('#poseHud').textContent = `heave ${p.heave.toFixed(0)} mm · roll ${p.roll.toFixed(1)}° · rack ${p.rack.toFixed(0)} mm`;
}

// ------------------------------------------------------------------ result fields
function applyField() {
  const f = state.field;
  const res = currentResult();
  chassisView.opts.display = f;
  if (['disp', 'vm', 'sed'].includes(f)) {
    if (!res || state.feaStale) {
      chassisView.setField(null);
      chassisView.setOption('display', 'material');
      if (!res) setStatus('Run the FEA to show result contours.');
      return;
    }
    const m = state.mesh.model;
    let field;
    if (f === 'disp') {
      const u = res.u;
      const nn = m.nodes.length / 3;
      const v = new Float32Array(nn);
      for (let i = 0; i < nn; i++) v[i] = Math.hypot(u[6 * i], u[6 * i + 1], u[6 * i + 2]);
      field = { type: 'disp', values: v, perNode: true, label: `Displacement - ${res.name}`, unit: 'mm', digits: 3 };
    } else if (f === 'vm') {
      field = { type: 'vm', values: res.post.shellVM, perNode: false, label: `von Mises - ${res.name}`, unit: 'MPa', digits: 1 };
    } else {
      const X = res.post.shellU;
      const v = new Float32Array(X.length);
      const areas = elementAreas();
      for (let e = 0; e < X.length; e++) v[e] = (X[e] / (areas[e] * m.shellT[e])) * 1e3; // N mm / mm^3 = MJ/m^3 -> kJ/m^3
      field = { type: 'sed', values: v, perNode: false, label: `Strain energy density - ${res.name}`, unit: 'kJ/m³', digits: 2 };
    }
    if (!field.perNode && state.smooth) field = { ...field, values: nodalAverage(field.values), perNode: true, smoothed: true };
    chassisView.setField(field);
  }
  chassisView.setOption('display', f);
  updateDeformation();
}

/** Average element values to nodes (area-weighted) for smooth contour plots. */
function nodalAverage(vals) {
  const m = state.mesh.model;
  const nn = m.nodes.length / 3;
  const sum = new Float64Array(nn), w = new Float64Array(nn);
  const areas = elementAreas();
  for (let e = 0; e < vals.length; e++) for (let k = 0; k < 4; k++) { const n = m.shells[4 * e + k]; sum[n] += vals[e] * areas[e]; w[n] += areas[e]; }
  const out = new Float32Array(nn);
  for (let i = 0; i < nn; i++) out[i] = w[i] ? sum[i] / w[i] : 0;
  return out;
}

let areaCache = null;
function elementAreas() {
  if (areaCache && areaCache.mesh === state.mesh) return areaCache.a;
  const m = state.mesh.model;
  const a = new Float64Array(m.shells.length / 4);
  for (let e = 0; e < a.length; e++) {
    const p = [0, 1, 2, 3].map((k) => { const n = m.shells[4 * e + k]; return [m.nodes[3 * n], m.nodes[3 * n + 1], m.nodes[3 * n + 2]]; });
    const d1 = p[2].map((v, i) => v - p[0][i]), d2 = p[3].map((v, i) => v - p[1][i]);
    a[e] = 0.5 * Math.hypot(d1[1] * d2[2] - d1[2] * d2[1], d1[2] * d2[0] - d1[0] * d2[2], d1[0] * d2[1] - d1[1] * d2[0]);
  }
  areaCache = { mesh: state.mesh, a };
  return a;
}

function currentResult() {
  const F = state.fea;
  if (!F || state.feaStale) return null;
  if (state.loadcase === 'bending' && F.bending) return { ...F.bending, name: 'bending' };
  if (F.torsion) return { ...F.torsion, name: 'torsion' };
  return null;
}

function autoScale(u) {
  const m = state.mesh.model;
  let umax = 0;
  for (let i = 0; i < m.nodes.length / 3; i++) umax = Math.max(umax, Math.hypot(u[6 * i], u[6 * i + 1], u[6 * i + 2]));
  return umax > 0 ? 120 / umax : 1;
}

function updateDeformation() {
  if (state.modeIdx !== null && state.fea?.modal && !state.feaStale) {
    const mode = state.fea.modal[state.modeIdx];
    chassisView.animateMode(mode.shape, autoScale(mode.shape) * 0.8);
    return;
  }
  chassisView.animateMode(null);
  const res = currentResult();
  if (!res || !state.deformOn) { chassisView.setDeformation(null); return; }
  const s = state.deformScale ?? autoScale(res.u);
  chassisView.setDeformation(res.u, s);
}

// ------------------------------------------------------------------ FEA jobs
async function runFEA(which) {
  if (jobs.busy) return;
  const c = state.cfg;
  setStatus('Meshing and solving…');
  setProgress(0);
  state.modeIdx = null;
  try {
    const t0 = performance.now();
    const r = await jobs.run('analyse', { cfg: c, which, solver: c.analysis.solver }, (p) => {
      if (p.msg) setStatus(p.msg);
      if (p.p !== undefined) setProgress(p.p);
    });
    const prev = state.fea && !state.feaStale ? state.fea : {};
    state.fea = { ...prev, ...r.result };
    state.feaStale = false;
    state.deformScale = null;
    computeDynamics();
    setStatus(`Analysis complete in ${((performance.now() - t0) / 1000).toFixed(1)} s`, 'good');
    if (which.includes('modal') && !which.includes('torsion')) state.modeIdx = null;
    applyField();
    renderRight();
    if (state.leftTab === 'optimise') renderLeft();
  } catch (e) {
    setStatus(e.message === 'Cancelled' ? 'Analysis cancelled' : `Analysis failed: ${e.message}`, e.message === 'Cancelled' ? '' : 'bad');
  } finally {
    setProgress(null);
    if (state.rightTab === 'fea') renderRight();
  }
}

// ------------------------------------------------------------------ left panel
const LEFT_TABS = [['vehicle', 'Vehicle'], ['chassis', 'Chassis'], ['suspension', 'Suspension'], ['optimise', 'Optimisers']];
const RIGHT_TABS = [['fea', 'Structure (FEA)'], ['kin', 'Kinematics'], ['dyn', 'Vehicle dynamics']];

function renderTabs(el, tabs, active, onPick) {
  el.innerHTML = '';
  for (const [k, t] of tabs) {
    const b = h('button', { class: k === active ? 'active' : '' }, t);
    b.addEventListener('click', () => onPick(k));
    el.append(b);
  }
}

function renderLeft() {
  renderTabs($('#leftTabs'), LEFT_TABS, state.leftTab, (k) => { state.leftTab = k; renderLeft(); });
  const body = $('#leftBody');
  const scroll = body.scrollTop;
  body.innerHTML = '';
  const tab = state.leftTab;
  if (tab === 'vehicle') body.append(...vehiclePanel());
  if (tab === 'chassis') body.append(...chassisPanel());
  if (tab === 'suspension') body.append(...suspensionPanel());
  if (tab === 'optimise') body.append(...optimisePanel());
  body.scrollTop = scroll;
}

function vehiclePanel() {
  const V = () => state.cfg.vehicle, R = () => state.cfg.ride, A = () => state.cfg.analysis;
  const ch = () => onVehicleChange();
  const masses = h('table', { class: 'grid' },
    h('tr', {}, h('th', {}, 'Component'), h('th', {}, 'kg'), h('th', {}, 'x'), h('th', {}, 'z')),
    state.cfg.masses.map((m, i) => h('tr', {},
      h('td', { title: m.name }, m.name),
      ...['m', 'x', 'z'].map((k) => {
        const inp = h('input', { type: 'number', step: k === 'm' ? 1 : 10, value: m[k] });
        inp.addEventListener('change', () => { state.cfg.masses[i][k] = parseFloat(inp.value) || 0; onVehicleChange(); });
        return h('td', {}, inp);
      }))));
  return [
    section('Layout', [
      num('Wheelbase', V, 'wheelbase', { step: 10, min: 1800, max: 3600, unit: 'mm', onChange: (v, f) => { if (f) { onSuspChange(true); onVehicleChange(); } } }),
      num('Brake bias (front)', V, 'brakeFront', { step: 0.01, min: 0.3, max: 0.85, onChange: ch }),
      num('Drive split (front)', V, 'driveFront', { step: 0.05, min: 0, max: 1, onChange: ch, title: '0 = RWD, 1 = FWD, EV AWD typically 0.3-0.5' }),
      h('div', { class: 'hint' }, 'x is measured forward from the front axle (rear axle at -wheelbase), y to the left, z up from the ground.'),
    ]),
    section('Mass items (sprung)', [masses, h('div', { class: 'hint' }, 'Chassis structure mass and CG come from the FE model. Unsprung masses are placed at the wheel centres.'),
      num('Unsprung per corner - front', () => state.cfg.unsprung, 'front', { step: 1, unit: 'kg', onChange: ch }),
      num('Unsprung per corner - rear', () => state.cfg.unsprung, 'rear', { step: 1, unit: 'kg', onChange: ch })]),
    section('Ride & roll targets', [
      num('Ride frequency front', R, 'fF', { step: 0.05, unit: 'Hz', onChange: ch }),
      num('Ride frequency rear', R, 'fR', { step: 0.05, unit: 'Hz', onChange: ch }),
      num('Roll gradient', R, 'rollGrad', { step: 0.05, unit: '°/g', onChange: ch }),
      num('LLTD front', R, 'lltd', { step: 0.5, unit: '%', onChange: ch }),
      num('Tyre vertical rate front', R, 'tyreRateF', { step: 5, unit: 'N/mm', onChange: ch }),
      num('Tyre vertical rate rear', R, 'tyreRateR', { step: 5, unit: 'N/mm', onChange: ch }),
      num('Damping ratio', R, 'dampRatio', { step: 0.05, onChange: ch }),
    ], { collapsed: true }),
    section('Analysis settings', [
      select('Solver', A, 'solver', [['cpu', 'CPU - envelope Cholesky (f64)'], ['gpu', 'WebGPU compute - PCG']], { onChange: () => renderRight() }),
      num('Torsion test torque', A, 'torque', { step: 100, unit: 'Nm' }),
      num('Bending load (sills)', A, 'bendLoad', { step: 500, unit: 'N' }),
      num('Modes to extract', A, 'nModes', { step: 1, min: 1, max: 12 }),
      check('Include component masses in modal', A, 'modalWithMasses'),
      num('Torsional stiffness target', A, 'targetK', { step: 1000, unit: 'Nm/°', onChange: () => renderRight() }),
    ]),
  ];
}

function chassisPanel() {
  const C = () => state.cfg.chassis;
  const chg = (v, final = true) => { if (final) onChassisChange(); };
  const n = (label, key, o = {}) => num(label, C, key, { step: 10, unit: 'mm', onChange: chg, ...o });
  const tube = (key, label) => {
    const T = () => state.cfg.tubes[key];
    return [h('div', { class: 'hint', style: { marginTop: '6px', color: 'var(--text-secondary)' } }, label),
      num('OD', T, 'od', { step: 1, unit: 'mm', onChange: chg }), num('Wall', T, 'wall', { step: 0.1, unit: 'mm', onChange: chg }),
      select('Material', T, 'mat', Object.entries(TUBE_MATERIALS).map(([k, v]) => [k, v.name]), { onChange: chg })];
  };
  const gauges = GROUPS.map((g, i) => {
    const row = num(GROUP_LABELS[g], () => state.cfg.gauges, g, { step: 0.1, min: 0.5, max: 8, unit: 'mm', slider: true, onChange: chg });
    row.querySelector('label').prepend(h('i', { style: { display: 'inline-block', width: '9px', height: '9px', borderRadius: '2px', background: GROUP_COLORS[i], marginRight: '6px' } }));
    return row;
  });
  return [
    section('Stations (x from front axle)', [
      n('Nose / crash-rail tip', 'xNose'), n('Front bulkhead', 'xFront'), n('Dash / scuttle', 'xDash'), n('Seat-back firewall', 'xSeat'), n('Rear rail end / gearbox', 'xRear'),
    ]),
    section('Tub section', [
      n('Floor height (ground clearance)', 'floorZ', { step: 5 }), n('Half-width (outer)', 'W'), n('Sill / side-box width', 'sillW', { step: 5 }), n('Sill height', 'sillH'),
      n('Firewall height', 'firewallH'),
      num('Plan taper at front bulkhead', C, 'taperW', { step: 0.02, min: 0.3, max: 1, onChange: chg }),
      num('Height taper at front bulkhead', C, 'taperH', { step: 0.02, min: 0.4, max: 1, onChange: chg }),
      check('Centre tunnel', C, 'tunnel', { onChange: chg }), n('Tunnel half-width', 'tunnelHalf', { step: 5 }), n('Tunnel height', 'tunnelH'),
      check('Closed footwell deck / scuttle', C, 'deck', { onChange: chg }), n('Deck / scuttle height', 'deckH'),
      check('Structural battery enclosure', C, 'battery', { onChange: chg }), n('Battery height', 'batteryH', { step: 5 }),
      num('Battery bays (cross-members + 1)', C, 'batteryBays', { step: 1, min: 1, max: 12, onChange: chg }),
    ]),
    section('Rails, subframes & tubes', [
      check('Front rails / suspension towers', C, 'frontRails', { onChange: chg }),
      check('Rear rails', C, 'rearRails', { onChange: chg }),
      check('Main roll hoop / B-pillar ring', C, 'rollHoop', { onChange: chg }), n('Hoop height above floor', 'hoopH'),
      check('A-pillars + roof (cage)', C, 'cage', { onChange: chg }), n('Roof height above floor', 'roofH'),
      check('Front hoop (single seater)', C, 'frontHoop', { onChange: chg }),
      check('Engine-bay X-brace', C, 'rearBrace', { onChange: chg }),
      check('Crash beams', C, 'bumpers', { onChange: chg }),
      check('Stressed engine + gearbox', C, 'engine', { onChange: chg }), n('Engine rear face x', 'engineEnd'),
      num('Pick-up bracket footprint', C, 'hpSpread', { step: 10, min: 0, max: 300, unit: 'mm', onChange: chg, title: 'Radius over which each suspension pick-up spreads load into the sheet (local doubler / bracket).' }),
    ], { collapsed: true }),
    section('Sheet gauges', [
      select('Sheet material', cfg, 'material', Object.entries(MATERIALS).map(([k, v]) => [k, `${v.name} (E ${v.E / 1000} GPa)`]), { onChange: chg }),
      ...gauges,
      h('div', { class: 'hint' }, `Standard gauges: ${STANDARD_GAUGES.join(', ')} mm. The gauge optimiser (Optimisers tab) sizes these for minimum mass.`),
    ]),
    section('Tubes', [...tube('hoop', 'Roll hoops'), ...tube('cage', 'Cage / pillars'), ...tube('brace', 'Braces & crash beams')], { collapsed: true }),
    section('Mesh', [
      num('Target element size', C, 'mesh', { step: 5, min: 30, max: 200, unit: 'mm', onChange: chg, title: 'Smaller = more accurate but slower solves' }),
      h('div', { class: 'hint' }, 'Conforming quad mesh; every panel shares grid lines with its neighbours so joints are continuous. 60-90 mm gives converged stiffness trends in a few seconds.'),
    ], { collapsed: true }),
  ];
}

function suspensionPanel() {
  const ax = state.suspAxle;
  const D = () => state.cfg.suspension[ax];
  const axSel = h('div', { class: 'btnrow' }, ...['front', 'rear'].map((a) => {
    const b = h('button', { class: a === ax ? 'primary' : '' }, a === 'front' ? 'Front axle' : 'Rear axle');
    b.addEventListener('click', () => { state.suspAxle = a; renderLeft(); });
    return b;
  }));
  const pose = suspView.pose;
  const P = () => pose;
  const poseCh = () => { suspView.update(); updatePoseHud(); };
  const def = D();
  const hpNames = HARDPOINTS.filter(([k]) => def.actuation !== 'direct' || !k.startsWith('rock'));
  const table = h('table', { class: 'grid' }, h('tr', {}, h('th', {}, 'Point (left)'), h('th', {}, 'x'), h('th', {}, 'y'), h('th', {}, 'z')),
    hpNames.map(([k, label]) => {
      const isSel = state.selHp && state.selHp.axle === (ax === 'front' ? 'F' : 'R') && state.selHp.name === k;
      return h('tr', { class: isSel ? 'sel' : '' }, h('td', { title: label }, label),
        ...[0, 1, 2].map((i) => {
          const inp = h('input', { type: 'number', step: 1, value: def.hp[k][i].toFixed(1) });
          inp.addEventListener('change', () => { def.hp[k][i] = parseFloat(inp.value); onSuspChange(true); });
          return h('td', {}, inp);
        }));
    }));
  const ch = (v, f = true) => onSuspChange(f);
  return [
    section('Live pose (3D view)', [
      num('Heave (wheel travel)', P, 'heave', { step: 1, min: -60, max: 60, unit: 'mm', slider: true, onChange: poseCh }),
      num('Body roll', P, 'roll', { step: 0.1, min: -4, max: 4, unit: '°', slider: true, onChange: poseCh }),
      num('Rack travel', P, 'rack', { step: 1, min: -80, max: 80, unit: 'mm', slider: true, onChange: poseCh }),
      h('div', { class: 'btnrow' }, h('button', { onclick: animatePose }, '▶ Animate travel'), h('button', { onclick: () => { Object.assign(pose, { heave: 0, roll: 0, rack: 0 }); poseCh(); renderLeft(); } }, 'Reset')),
    ]),
    axSel,
    section('Corner set-up', [
      select('Actuation', D, 'actuation', [['direct', 'Coil-over on wishbone'], ['pushrod', 'Push/pull-rod + rocker']], { onChange: () => { onSuspChange(true); renderLeft(); } }),
      select('Pushrod / damper mounts to', D, 'pushOn', [['lca', 'Lower wishbone'], ['uca', 'Upper wishbone (pull-rod)'], ['upright', 'Upright']], { onChange: ch }),
      num('Static camber', D, 'camber', { step: 0.1, unit: '°', onChange: ch }),
      num('Static toe (in +)', D, 'toe', { step: 0.05, unit: '°', onChange: ch }),
      num('Tyre loaded radius', D, 'tyreR', { step: 1, unit: 'mm', onChange: ch }),
      num('Tyre width', D, 'tyreW', { step: 5, unit: 'mm', onChange: ch }),
      num('Spring rate', D, 'springRate', { step: 1, unit: 'N/mm', onChange: () => onVehicleChange() }),
      num('ARB (wheel-rate equiv.)', D, 'arbRate', { step: 1, unit: 'N/mm', onChange: () => onVehicleChange() }),
      ...(ax === 'front' ? [num('Rack travel (max)', D, 'rackMax', { step: 1, unit: 'mm', onChange: ch }), num('Rack c-factor', D, 'cFactor', { step: 1, unit: 'mm/rev', onChange: () => renderRight() })] : []),
    ]),
    section('Hardpoints (left side, x axle-local)', [
      h('div', { class: 'hint' }, 'Click a pick-up sphere in the 3D view to select and drag it. Right side is mirrored.'),
      table,
    ]),
  ];
}

let animReq = null;
function animatePose() {
  if (animReq) { cancelAnimationFrame(animReq); animReq = null; return; }
  const t0 = performance.now();
  const step = () => {
    const t = (performance.now() - t0) / 1000;
    suspView.setPose({ heave: 40 * Math.sin(t * 1.3), roll: 2.5 * Math.sin(t * 0.7), rack: (state.cfg.suspension.front.rackMax || 0) * 0.8 * Math.sin(t * 0.45) });
    updatePoseHud();
    if (t < 16) animReq = requestAnimationFrame(step); else animReq = null;
  };
  step();
}

// ------------------------------------------------------------------ optimisers panel
function optimisePanel() {
  return [suspOptSection(), gaugeOptSection(), rideOptSection(), balanceSection(), sweepSection()];
}

const suspOptState = { axle: 'front', study: 'bumpsteer', targets: null, vars: null, running: false, progress: null };
function initStudy() {
  const s = STUDIES[suspOptState.study];
  suspOptState.targets = Object.fromEntries(Object.entries(SUSP_METRICS).map(([k]) => [k, { on: k in s.targets && s.targets[k] !== null, v: s.targets[k] ?? defaultTarget(k) }]));
  suspOptState.vars = s.vars.map(([hp, a, r]) => ({ hp, a, r, on: true }));
}
function defaultTarget(k) {
  const m = state.kin?.[suspOptState.axle]?.metrics;
  const v = m?.[k];
  return Number.isFinite(v) ? +v.toFixed(2) : 0;
}

function suspOptSection() {
  const S = suspOptState;
  if (!S.targets) initStudy();
  const axSel = select('Axle', () => S, 'axle', [['front', 'Front'], ['rear', 'Rear']], { onChange: () => renderLeft() });
  const studySel = select('Study', () => S, 'study', Object.entries(STUDIES).map(([k, v]) => [k, v.label]), { onChange: () => { initStudy(); renderLeft(); } });
  const m = state.kin?.[S.axle]?.metrics || {};
  const tgt = h('table', { class: 'grid' }, h('tr', {}, h('th', {}, 'Target'), h('th', {}, 'now'), h('th', {}, 'goal')),
    Object.entries(SUSP_METRICS).filter(([k]) => !(S.axle === 'rear' && ['antiDive', 'ackermann'].includes(k)) && !(S.axle === 'front' && k === 'antiSquat')).map(([k, meta]) => {
      const t = S.targets[k];
      const cb = h('input', { type: 'checkbox' }); cb.checked = t.on;
      cb.addEventListener('change', () => { t.on = cb.checked; });
      const inp = h('input', { type: 'number', step: 0.1, value: t.v });
      inp.addEventListener('change', () => { t.v = parseFloat(inp.value); });
      return h('tr', {}, h('td', {}, h('label', { style: { display: 'flex', gap: '5px', alignItems: 'center' } }, cb, `${meta.label}${meta.unit ? ` (${meta.unit})` : ''}`)), h('td', {}, fmt(m[k], 2)), h('td', {}, k === 'toeRange' ? '→ 0' : inp));
    }));
  const vars = h('table', { class: 'grid' }, h('tr', {}, h('th', {}, 'Free coordinate'), h('th', {}, '± range')),
    S.vars.map((v) => {
      const cb = h('input', { type: 'checkbox' }); cb.checked = v.on;
      cb.addEventListener('change', () => { v.on = cb.checked; });
      const inp = h('input', { type: 'number', step: 5, value: v.r });
      inp.addEventListener('change', () => { v.r = parseFloat(inp.value); });
      return h('tr', {}, h('td', {}, h('label', { style: { display: 'flex', gap: '5px', alignItems: 'center' } }, cb, `${v.hp}.${'xyz'[v.a]}`)), h('td', {}, inp));
    }));
  const addVar = h('select', {}, h('option', { value: '' }, '+ add coordinate…'), HARDPOINTS.flatMap(([k]) => [0, 1, 2].map((a) => h('option', { value: `${k}:${a}` }, `${k}.${'xyz'[a]}`))));
  addVar.addEventListener('change', () => { const [hp, a] = addVar.value.split(':'); if (hp) { S.vars.push({ hp, a: +a, r: 30, on: true }); renderLeft(); } });
  const run = h('button', { class: 'primary', disabled: S.running }, S.running ? 'Optimising…' : 'Optimise geometry');
  run.addEventListener('click', runSuspOpt);
  const out = [];
  if (S.progress) out.push(h('div', { class: 'hint mono' }, S.progress));
  const R = state.optim.susp;
  if (R && R.axle === S.axle) {
    const keys = Object.entries(R.targets).map(([k]) => k);
    out.push(h('table', { class: 'grid' }, h('tr', {}, h('th', {}, 'Metric'), h('th', {}, 'before'), h('th', {}, 'after'), h('th', {}, 'goal')),
      keys.map((k) => h('tr', {}, h('td', {}, SUSP_METRICS[k].label), h('td', {}, fmt(R.before[k], 2)), h('td', {}, fmt(R.after?.[k], 2)), h('td', {}, k === 'toeRange' ? '0' : fmt(R.targets[k], 2))))));
    out.push(h('table', { class: 'grid' }, h('tr', {}, h('th', {}, 'Hardpoint'), h('th', {}, 'from'), h('th', {}, 'to'), h('th', {}, 'Δ')),
      R.changes.map((c) => h('tr', {}, h('td', {}, `${c.hp}.${c.axis}`), h('td', {}, fmt(c.from, 1)), h('td', {}, fmt(c.to, 1)), h('td', { class: Math.abs(c.to - c.from) > 0.05 ? 'up' : '' }, fmt(c.to - c.from, 1))))));
    out.push(h('div', { class: 'hint' }, `Objective ${fmt(R.f0, 3)} → ${fmt(R.f, 4)} in ${R.evals} kinematic evaluations.`));
    const apply = h('button', { class: 'primary' }, 'Apply to design');
    apply.addEventListener('click', () => {
      state.cfg.suspension[R.axle].hp = R.def.hp;
      state.optim.susp = null;
      onSuspChange(true);
      renderLeft();
      setStatus('Optimised hardpoints applied.', 'good');
    });
    const discard = h('button', {}, 'Discard');
    discard.addEventListener('click', () => { state.optim.susp = null; renderLeft(); });
    out.push(h('div', { class: 'btnrow' }, apply, discard));
  }
  return section('Suspension geometry optimiser', [
    h('div', { class: 'hint' }, 'Bounded Nelder-Mead over chosen hardpoint coordinates, evaluating full 3-D kinematics (heave, roll and steer sweeps) at every step.'),
    axSel, studySel, tgt, vars, addVar, h('div', { class: 'btnrow' }, run), ...out,
  ]);
}

async function runSuspOpt() {
  const S = suspOptState;
  const c = state.cfg;
  const def = c.suspension[S.axle];
  const targets = Object.fromEntries(Object.entries(S.targets).filter(([, t]) => t.on).map(([k, t]) => [k, t.v]));
  const vars = S.vars.filter((v) => v.on).map((v) => [v.hp, v.a, v.r]);
  if (!vars.length || !Object.keys(targets).length) { setStatus('Select at least one target and one free coordinate.', 'bad'); return; }
  S.running = true; S.progress = 'starting…';
  renderLeft();
  setProgress(-1);
  try {
    const veh = { wheelbase: c.vehicle.wheelbase, cgH: state.mp?.cg[2] ?? 400, brakeFront: c.vehicle.brakeFront, driveFront: c.vehicle.driveFront, isFront: S.axle === 'front' };
    let lastUi = 0;
    const r = await optimiseCorner(def, {
      vars, targets, veh, rackMax: def.rackMax || 0, maxEval: 700,
      onIter: ({ evals, best }) => {
        if (performance.now() - lastUi > 150) { lastUi = performance.now(); S.progress = `${evals} evaluations · objective ${best.toFixed(4)}`; setStatus(`Suspension optimiser: ${S.progress}`); }
      },
    });
    state.optim.susp = { ...r, axle: S.axle, targets };
    setStatus('Suspension optimisation finished - review and apply.', 'good');
  } catch (e) {
    setStatus(`Optimiser failed: ${e.message}`, 'bad');
  } finally {
    S.running = false; S.progress = null;
    setProgress(null);
    renderLeft();
  }
}

const gaugeState = { groups: null, tMin: 0.8, tMax: 6, snap: true, iters: 12, hist: [] };
function gaugeOptSection() {
  const G = gaugeState;
  const present = new Set(state.mesh ? [...state.mesh.model.shellGroup].map((g) => GROUPS[g]) : GROUPS);
  if (!G.groups) G.groups = Object.fromEntries(GROUPS.map((g) => [g, true]));
  const A = () => state.cfg.analysis;
  const rows = GROUPS.filter((g) => present.has(g)).map((g) => {
    const cb = h('input', { type: 'checkbox' }); cb.checked = G.groups[g];
    cb.addEventListener('change', () => { G.groups[g] = cb.checked; });
    const R = state.optim.gauges;
    return h('tr', {}, h('td', {}, h('label', { style: { display: 'flex', gap: '5px', alignItems: 'center' } }, cb, GROUP_LABELS[g])),
      h('td', {}, fmt(state.cfg.gauges[g], 2)), h('td', { class: R ? (R.gauges[g] > state.cfg.gauges[g] ? 'up' : R.gauges[g] < state.cfg.gauges[g] ? 'down' : '') : '' }, R ? fmt(R.gauges[g], 2) : '–'));
  });
  const run = h('button', { class: 'primary', disabled: jobs.busy }, 'Optimise gauges (FEA)');
  run.addEventListener('click', runGaugeOpt);
  const R = state.optim.gauges;
  const out = [];
  if (G.hist.length > 1) {
    out.push(lineChart({ title: 'Optimisation history', xLabel: 'iteration', series: [{ name: 'Mass (kg)', x: G.hist.map((_, i) => i), y: G.hist.map((p) => p.mass), markers: true }], height: 110, yFmt: (v) => `${v.toFixed(1)} kg` }));
    out.push(lineChart({ series: [{ name: 'K (Nm/°)', x: G.hist.map((_, i) => i), y: G.hist.map((p) => p.K), color: 'var(--series-2)', markers: true }], hlines: [{ y: state.cfg.analysis.targetK, label: 'target' }], height: 110, xLabel: 'iteration', yFmt: (v) => `${fmtK(v)} Nm/°` }));
  }
  if (R) {
    out.push(h('div', { class: 'kpis' },
      kpi('Mass', fmt(R.final.mass, 1), 'kg', `was ${fmt(R.initial.mass, 1)} kg`),
      kpi('Torsional K', fmtK(R.final.K), 'Nm/°', `was ${fmtK(R.initial.K)}`)));
    const apply = h('button', { class: 'primary' }, 'Apply gauges');
    apply.addEventListener('click', () => { Object.assign(state.cfg.gauges, R.gauges); state.optim.gauges = null; gaugeState.hist = []; onChassisChange(true); renderLeft(); setStatus('Optimised gauges applied - re-run FEA to refresh results.', 'good'); });
    const discard = h('button', {}, 'Discard');
    discard.addEventListener('click', () => { state.optim.gauges = null; gaugeState.hist = []; renderLeft(); });
    out.push(h('div', { class: 'btnrow' }, apply, discard));
  }
  return section('Sheet gauge optimiser (min mass @ stiffness)', [
    h('div', { class: 'hint' }, 'Optimality-criteria sizing: each iteration solves the torsion case, computes membrane/bending strain-energy sensitivities per panel group, and resizes gauges to hit the stiffness target at minimum mass. Final gauges snap to standard sheet sizes.'),
    num('Target torsional stiffness', A, 'targetK', { step: 1000, unit: 'Nm/°' }),
    num('Min gauge', () => G, 'tMin', { step: 0.1, unit: 'mm' }), num('Max gauge', () => G, 'tMax', { step: 0.5, unit: 'mm' }),
    num('Iterations', () => G, 'iters', { step: 1, min: 2, max: 30 }),
    check('Snap to standard gauges', () => G, 'snap'),
    h('table', { class: 'grid' }, h('tr', {}, h('th', {}, 'Group'), h('th', {}, 'now'), h('th', {}, 'optimum')), rows),
    h('div', { class: 'btnrow' }, run, jobs.busy ? h('button', { onclick: () => jobs.cancel() }, 'Cancel') : null), ...out,
  ]);
}

async function runGaugeOpt() {
  if (jobs.busy) return;
  const G = gaugeState;
  G.hist = [];
  setProgress(0);
  setStatus('Gauge optimiser running…');
  renderLeft();
  try {
    const free = Object.entries(G.groups).filter(([, v]) => v).map(([k]) => k);
    const r = await jobs.run('gauges', { cfg: state.cfg, solver: state.cfg.analysis.solver, opts: { target: state.cfg.analysis.targetK, free, tMin: G.tMin, tMax: G.tMax, iters: G.iters, snap: G.snap } }, (p) => {
      if (p.opt) {
        G.hist.push(p.opt);
        setStatus(`Gauge optimiser: iteration ${p.opt.it} · K ${fmtK(p.opt.K)} Nm/° · mass ${p.opt.mass.toFixed(1)} kg`);
        setProgress(typeof p.opt.it === 'number' ? (p.opt.it + 1) / G.iters : -1);
        if (state.leftTab === 'optimise') renderLeft();
      } else if (p.msg) setStatus(p.msg);
    });
    state.optim.gauges = r.result;
    setStatus(`Gauges optimised: ${r.result.final.mass.toFixed(1)} kg at ${fmtK(r.result.final.K)} Nm/°`, 'good');
  } catch (e) {
    setStatus(`Gauge optimiser: ${e.message}`, e.message === 'Cancelled' ? '' : 'bad');
  } finally {
    setProgress(null);
    renderLeft();
  }
}

function rideOptSection() {
  const R = state.ride;
  if (!R) return section('Ride & roll set-up optimiser', [h('div', { class: 'hint' }, 'Kinematics must solve first.')]);
  const r = R.rec;
  const c = state.cfg;
  const rows = [
    ['Spring rate front', c.suspension.front.springRate, r.springF, 'N/mm'],
    ['Spring rate rear', c.suspension.rear.springRate, r.springR, 'N/mm'],
    ['ARB front (wheel equiv.)', c.suspension.front.arbRate, Math.max(0, r.arbF), 'N/mm'],
    ['ARB rear (wheel equiv.)', c.suspension.rear.arbRate, Math.max(0, r.arbR), 'N/mm'],
    ['Damper (low-speed) front', null, r.damperF, 'N·s/mm'],
    ['Damper (low-speed) rear', null, r.damperR, 'N·s/mm'],
  ];
  const apply = h('button', { class: 'primary' }, 'Apply springs & ARBs');
  apply.addEventListener('click', () => {
    c.suspension.front.springRate = +r.springF.toFixed(1); c.suspension.rear.springRate = +r.springR.toFixed(1);
    c.suspension.front.arbRate = +Math.max(0, r.arbF).toFixed(1); c.suspension.rear.arbRate = +Math.max(0, r.arbR).toFixed(1);
    onVehicleChange(); renderLeft(); setStatus('Spring and ARB rates applied.', 'good');
  });
  return section('Ride & roll set-up optimiser', [
    h('div', { class: 'hint' }, `Solves spring rates from target ride frequencies (tyre in series, motion ratios from kinematics), then splits the roll stiffness needed for ${c.ride.rollGrad}°/g between axles to hit ${c.ride.lltd}% front LLTD, accounting for roll-centre (geometric) and unsprung transfer.`),
    h('table', { class: 'grid' }, h('tr', {}, h('th', {}, ''), h('th', {}, 'now'), h('th', {}, 'recommended')),
      rows.map(([l, now, rec, u]) => h('tr', {}, h('td', {}, `${l} (${u})`), h('td', {}, now === null ? '–' : fmt(now, 1)), h('td', {}, fmt(rec, 1))))),
    !r.feasible ? h('div', { class: 'callout warn' }, 'Targets need a negative ARB on one axle: springs alone already give more roll stiffness than the LLTD target allows there. Relax the roll-gradient or LLTD target, or soften that axle.') : null,
    h('div', { class: 'btnrow' }, apply),
  ]);
}

const balState = { comp: 0, target: 42 };
function balanceSection() {
  const c = state.cfg;
  const mp = state.mp;
  if (!mp) return section('Weight distribution optimiser', [h('div', { class: 'hint' }, 'Waiting for mass properties.')]);
  if (balState.target === 42 && c.type === 'ev') balState.target = 50;
  const opts = c.masses.map((m, i) => [String(i), `${m.name} (${m.m} kg)`]);
  const res = state.optim.balance;
  const run = h('button', { class: 'primary' }, 'Solve position');
  run.addEventListener('click', () => {
    const lim = [c.chassis.xRear, c.chassis.xNose];
    state.optim.balance = balanceWeight(c, mp, +balState.comp, balState.target / 100, lim);
    renderLeft();
  });
  const out = [];
  if (res) {
    const comp = c.masses[+balState.comp];
    out.push(h('div', { class: 'callout' }, `Move "${comp.name}" to x = ${res.x.toFixed(0)} mm (from ${comp.x.toFixed(0)}) → front weight ${(res.achieved * 100).toFixed(1)}%${res.clamped ? ' (clamped to vehicle length)' : ''}.`));
    const apply = h('button', { class: 'primary' }, 'Apply');
    apply.addEventListener('click', () => { comp.x = +res.x.toFixed(0); state.optim.balance = null; onVehicleChange(); renderLeft(); });
    out.push(h('div', { class: 'btnrow' }, apply));
  }
  return section('Weight distribution optimiser', [
    h('div', { class: 'hint' }, `Current front weight: ${(mp.frontFrac * 100).toFixed(1)}%. Relocates one component (battery, ballast, fuel cell…) along x to reach the target.`),
    select('Component', () => balState, 'comp', opts),
    num('Target front weight', () => balState, 'target', { step: 0.5, unit: '%' }),
    h('div', { class: 'btnrow' }, run), ...out,
  ], { collapsed: true });
}

const SWEEP_PARAMS = [
  ['chassis.sillH', 'Sill height', 'mm'], ['chassis.sillW', 'Sill width', 'mm'], ['chassis.W', 'Tub half-width', 'mm'],
  ['chassis.tunnelH', 'Tunnel height', 'mm'], ['chassis.deckH', 'Deck / scuttle height', 'mm'], ['chassis.firewallH', 'Firewall height', 'mm'],
  ['chassis.batteryH', 'Battery height', 'mm'], ['chassis.batteryBays', 'Battery bays', ''], ['gauges.floor', 'Floor gauge', 'mm'],
  ['gauges.sills', 'Sill gauge', 'mm'], ['gauges.bulkheads', 'Bulkhead gauge', 'mm'], ['chassis.hpSpread', 'Pick-up bracket footprint', 'mm'],
];
const sweepState = { path: 'chassis.sillH', lo: null, hi: null, n: 5 };
function sweepSection() {
  const S = sweepState;
  const cur = S.path.split('.').reduce((o, k) => o?.[k], state.cfg) ?? 0;
  if (S.lo === null) { S.lo = +(cur * 0.7).toFixed(1); S.hi = +(cur * 1.3).toFixed(1); }
  const run = h('button', { class: 'primary', disabled: jobs.busy }, 'Run sweep');
  run.addEventListener('click', runSweep);
  const R = state.optim.sweep;
  const out = [];
  if (R && R.path === S.path) {
    const ok = R.data.filter((d) => !d.error);
    const meta = SWEEP_PARAMS.find((p) => p[0] === R.path);
    out.push(lineChart({ title: 'Torsional stiffness', xLabel: `${meta[1]} (${meta[2]})`, series: [{ name: 'K', x: ok.map((d) => d.value), y: ok.map((d) => d.K), markers: true }], vlines: [{ x: cur, label: 'current' }], height: 120, yFmt: (v) => `${fmtK(v)} Nm/°` }));
    out.push(lineChart({ title: 'Specific stiffness (per kg of structure)', xLabel: `${meta[1]} (${meta[2]})`, series: [{ name: 'K / mass', x: ok.map((d) => d.value), y: ok.map((d) => d.spec), color: 'var(--series-3)', markers: true }], vlines: [{ x: cur, label: 'current' }], height: 120, yFmt: (v) => `${v.toFixed(0)} Nm/°/kg` }));
    const best = ok.reduce((b, d) => (d.spec > (b?.spec ?? -1) ? d : b), null);
    if (best) {
      const apply = h('button', {}, `Use best specific stiffness (${best.value})`);
      apply.addEventListener('click', () => { const ks = R.path.split('.'); state.cfg[ks[0]][ks[1]] = best.value; onChassisChange(true); renderLeft(); });
      out.push(h('div', { class: 'btnrow' }, apply));
    }
  }
  return section('Design sweep (geometry sensitivity)', [
    h('div', { class: 'hint' }, 'Re-meshes and solves torsion across a range of one parameter to show stiffness and stiffness-per-kg trends.'),
    select('Parameter', () => S, 'path', SWEEP_PARAMS.map(([p, l]) => [p, l]), { onChange: () => { S.lo = null; renderLeft(); } }),
    num('From', () => S, 'lo', { step: 1 }), num('To', () => S, 'hi', { step: 1 }), num('Steps', () => S, 'n', { step: 1, min: 2, max: 12 }),
    h('div', { class: 'btnrow' }, run), ...out,
  ], { collapsed: true });
}

async function runSweep() {
  if (jobs.busy) return;
  const S = sweepState;
  const int = ['chassis.batteryBays'].includes(S.path);
  const values = Array.from({ length: S.n }, (_, i) => { const v = S.lo + ((S.hi - S.lo) * i) / (S.n - 1); return int ? Math.round(v) : +v.toFixed(2); });
  setProgress(0);
  try {
    const r = await jobs.run('sweep', { cfg: state.cfg, path: S.path, values, solver: state.cfg.analysis.solver }, (p) => {
      if (p.sweep) { setProgress((p.sweep.i + 1) / p.sweep.n); setStatus(`Sweep ${p.sweep.i + 1}/${p.sweep.n}: K = ${fmtK(p.sweep.last.K)} Nm/°`); }
    });
    state.optim.sweep = { path: S.path, data: r.result };
    setStatus('Sweep complete.', 'good');
  } catch (e) {
    setStatus(`Sweep: ${e.message}`, 'bad');
  } finally { setProgress(null); renderLeft(); }
}

// ------------------------------------------------------------------ right panel
let rightTimer = null;
function renderRightThrottled() { if (rightTimer) return; rightTimer = setTimeout(() => { rightTimer = null; renderRight(); }, 120); }

function renderRight() {
  renderTabs($('#rightTabs'), RIGHT_TABS, state.rightTab, (k) => { state.rightTab = k; renderRight(); });
  const body = $('#rightBody');
  const scroll = body.scrollTop;
  body.innerHTML = '';
  if (state.rightTab === 'fea') body.append(...feaPanel());
  if (state.rightTab === 'kin') body.append(...kinPanel());
  if (state.rightTab === 'dyn') body.append(...dynPanel());
  body.scrollTop = scroll;
}

function feaPanel() {
  const c = state.cfg;
  const F = state.fea;
  const busy = jobs.busy;
  const b1 = h('button', { class: 'primary', disabled: busy }, 'Torsion + bending');
  b1.addEventListener('click', () => runFEA(['torsion', 'bending']));
  const b2 = h('button', { disabled: busy }, 'Modal');
  b2.addEventListener('click', () => runFEA(['modal']));
  const b3 = h('button', { disabled: busy }, 'Run all');
  b3.addEventListener('click', () => runFEA(['torsion', 'bending', 'modal']));
  const cancel = busy ? h('button', { onclick: () => { jobs.cancel(); setProgress(null); renderRight(); } }, 'Cancel') : null;
  const solverNote = c.analysis.solver === 'gpu'
    ? (state.gpu.available === false ? h('div', { class: 'callout warn' }, 'WebGPU compute is unavailable here; switch the solver to CPU in Vehicle → Analysis settings.') : h('div', { class: 'hint' }, `Solver: WebGPU block-Jacobi PCG with f64 iterative refinement${state.gpu.label ? ` on ${state.gpu.label}` : ''}.`))
    : h('div', { class: 'hint' }, 'Solver: RCM-ordered envelope Cholesky (f64) in a Web Worker; bending re-uses the torsion factorisation.');
  const out = [h('div', { class: 'btnrow' }, b1, b2, b3, cancel), solverNote];
  if (!F) {
    out.push(h('div', { class: 'callout' }, 'Run the torsion + bending cases to get stiffness, stress and strain-energy maps. Rear pick-ups are pinned; a pure torque is applied through the front pick-ups (suspension locked, chassis-only stiffness).'));
    out.push(massSummary());
    return out;
  }
  if (state.feaStale) out.push(h('div', { class: 'callout warn' }, 'Design changed since the last run - results below are out of date.'));
  const T = F.torsion, B = F.bending;
  const mass = F.mass?.structure;
  const target = c.analysis.targetK;
  if (T) {
    const lvl = T.K >= target ? 'good' : T.K >= 0.85 * target ? 'warn' : 'bad';
    const area = (c.vehicle.wheelbase / 1000) * ((state.kin?.front?.trackStatic ?? 1600) / 1000);
    const Lw = (mass / (T.K * area)) * 1000;
    out.push(h('div', { class: 'kpis' },
      kpi('Torsional stiffness', fmtK(T.K), 'Nm/°', [statusDot(lvl), `${T.K >= target ? 'meets' : 'below'} target ${fmtK(target)}`], 'hero'),
      kpi('Bending stiffness', B ? fmtK(B.K) : '–', 'N/mm', B ? `${fmt(B.loadN / 1000, 1)} kN on sills` : ''),
      kpi('Structure mass', fmt(mass, 1), 'kg', `${fmt(F.mass.shells, 1)} sheet + ${fmt(F.mass.beams, 1)} tube`),
      kpi('Specific stiffness', fmt(T.K / mass, 0), 'Nm/°/kg'),
      kpi('Lightweight index', fmt(Lw, 2), '', 'm/(K·track·wb) ×10³ - lower is better'),
      kpi('Twist @ ' + fmtK(T.torqueNm) + ' Nm', fmt(T.thetaDeg, 4), '°'),
      kpi('Peak von Mises', fmt(percentile(T.post.shellVM, 0.99), 1), 'MPa', `torsion, 99th pct · yield ${MATERIALS[c.material].yield}`),
    ));
  }
  // display controls
  const lc = h('select', {}, [['torsion', 'Torsion'], ['bending', 'Bending']].map(([v, t]) => h('option', { value: v }, t)));
  lc.value = state.loadcase;
  lc.addEventListener('change', () => { state.loadcase = lc.value; state.modeIdx = null; state.deformScale = null; applyField(); renderRight(); });
  const res = currentResult();
  const autoS = res ? autoScale(res.u) : 1;
  const sc = h('input', { type: 'range', min: 0, max: 3, step: 0.01, value: Math.log10(state.deformScale ?? autoS) });
  const scLbl = h('span', { class: 'mono muted' }, `×${Math.round(state.deformScale ?? autoS)}`);
  sc.addEventListener('input', () => { state.deformScale = 10 ** parseFloat(sc.value); scLbl.textContent = `×${Math.round(state.deformScale)}`; updateDeformation(); });
  const defCb = h('input', { type: 'checkbox' }); defCb.checked = state.deformOn;
  defCb.addEventListener('change', () => { state.deformOn = defCb.checked; updateDeformation(); });
  const smCb = h('input', { type: 'checkbox' }); smCb.checked = state.smooth;
  smCb.addEventListener('change', () => { state.smooth = smCb.checked; applyField(); });
  const fieldSel = h('select', {}, [['material', 'Aluminium'], ['groups', 'Panel groups'], ['thickness', 'Thickness'], ['disp', 'Displacement'], ['vm', 'von Mises'], ['sed', 'Strain energy density']].map(([v, t]) => h('option', { value: v }, t)));
  fieldSel.value = state.field;
  fieldSel.addEventListener('change', () => { state.field = fieldSel.value; $('#fieldSel').value = state.field; applyField(); });
  out.push(section('Display', [
    h('div', { class: 'row' }, h('label', {}, 'Load case'), lc),
    h('div', { class: 'row' }, h('label', {}, 'Contour'), fieldSel),
    h('label', { class: 'row check' }, defCb, h('span', {}, 'Deformed shape')),
    h('label', { class: 'row check' }, smCb, h('span', {}, 'Smooth contours (nodal average)')),
    h('div', { class: 'row' }, h('label', {}, 'Deformation scale'), h('div', { class: 'ctl', style: { width: '160px' } }, sc, scLbl)),
  ]));
  if (T) {
    const ch = c.chassis;
    out.push(lineChart({
      title: 'Twist distribution along the tub (torsion case)', xLabel: 'x (mm from front axle)', yLabel: 'twist °',
      series: [{ name: 'Sill-top twist', x: T.twist.map((t) => t.x), y: T.twist.map((t) => t.deg) }],
      vlines: [{ x: ch.xFront, label: 'front bh' }, { x: ch.xDash, label: 'dash' }, { x: ch.xSeat, label: 'firewall' }], height: 150, yFmt: (v) => `${v.toFixed(4)}°`, xFmt: (v) => `${v.toFixed(0)} mm`,
    }));
    const ge = T.groupEnergy;
    const tot = Object.values(ge).reduce((s, g) => s + g.U, 0) || 1;
    const items = Object.entries(ge).filter(([, g]) => g.U > 0).sort((a, b) => b[1].U - a[1].U).map(([k, g]) => ({
      label: GROUP_LABELS[k] || 'Tubes & frames', value: (g.U / tot) * 100, color: k === 'tubes' ? '#8d8c84' : GROUP_COLORS[GROUPS.indexOf(k)],
      note: `membrane ${((g.Um / (g.U || 1)) * 100).toFixed(0)}% · bending ${((g.Ub / (g.U || 1)) * 100).toFixed(0)}%`,
    }));
    out.push(barChart({ title: 'Where the twist goes - strain energy share (torsion)', items, unit: '%', fmt: (v) => v.toFixed(1) }));
    out.push(h('div', { class: 'hint' }, 'Groups holding most strain energy are where added material buys the most stiffness; bending-dominated panels benefit from beads, flanges or closing into boxes.'));
    out.push(h('div', { class: 'hint mono' }, `${T.solver} · ${(T.ms / 1000).toFixed(2)} s${T.info?.equations ? ` · ${T.info.equations.toLocaleString()} eq · envelope ${(T.info.envelope / 1e6).toFixed(1)} M` : ''}${T.info?.iterations ? ` · ${T.info.iterations} PCG its · rel. residual ${T.info.relResidual.toExponential(1)}` : ''}`));
  }
  if (F.modal) {
    const list = h('div', { class: 'modes' }, F.modal.map((m, i) => {
      const b = h('button', { class: state.modeIdx === i ? 'on' : '' }, h('span', {}, `Mode ${i + 1} · ${m.label}`), h('span', { class: 'mono' }, `${m.freq.toFixed(1)} Hz`));
      b.addEventListener('click', () => { state.modeIdx = state.modeIdx === i ? null : i; updateDeformation(); renderRight(); });
      return b;
    }));
    out.push(section('Free-free modes', [h('div', { class: 'hint' }, `Shift-invert Lanczos, ${c.analysis.modalWithMasses ? 'with' : 'without'} component masses. Click a mode to animate it.`), list]));
  }
  out.push(massSummary());
  return out;
}

function massSummary() {
  if (!state.chassisMass) return h('div');
  const g = state.chassisMass.groups;
  const items = Object.entries(g).map(([k, v]) => ({ label: GROUP_LABELS[k], value: v, color: GROUP_COLORS[GROUPS.indexOf(k)] }));
  const tubes = state.chassisMass.mass - Object.values(g).reduce((s, v) => s + v, 0);
  if (tubes > 0.05) items.push({ label: 'Tubes', value: tubes, color: '#8d8c84' });
  return section('Structure mass breakdown', [barChart({ items: items.sort((a, b) => b.value - a.value), unit: ' kg', fmt: (v) => v.toFixed(1) }),
    h('div', { class: 'hint' }, `${MATERIALS[state.cfg.material].name}, total ${state.chassisMass.mass.toFixed(1)} kg.`)], { collapsed: false });
}

function percentile(arr, p) {
  if (!arr || !arr.length) return NaN;
  const s = Array.from(arr).sort((a, b) => a - b);
  return s[Math.min(s.length - 1, Math.floor(s.length * p))];
}

function kinPanel() {
  const ax = state.kinAxle;
  const K = state.kin?.[ax];
  const axSel = h('div', { class: 'btnrow' }, ...['front', 'rear'].map((a) => {
    const b = h('button', { class: a === ax ? 'primary' : '' }, a === 'front' ? 'Front' : 'Rear');
    b.addEventListener('click', () => { state.kinAxle = a; renderRight(); });
    return b;
  }));
  if (!K || !K.ok) return [axSel, h('div', { class: 'callout bad' }, K?.error || 'Kinematics not solved.')];
  const m = K.metrics;
  const def = state.cfg.suspension[ax];
  const lvl = (v, good, warn) => (Math.abs(v) <= good ? 'good' : Math.abs(v) <= warn ? 'warn' : 'bad');
  const tiles = [
    kpi('Static camber', fmt(m.camber, 2), '°'), kpi('Static toe (in +)', fmt(m.toe, 2), '°'),
    kpi('Kingpin inclination', fmt(m.kpi, 1), '°'), kpi('Caster', fmt(m.caster, 1), '°'),
    kpi('Scrub radius', fmt(m.scrub, 0), 'mm'), kpi('Mechanical trail', fmt(m.trail, 0), 'mm'),
    kpi('Roll-centre height', fmt(m.rcHeight, 0), 'mm'), kpi('RC lateral migration', fmt(m.rcMigration, 1), 'mm/°', [statusDot(lvl(m.rcMigration, 10, 30))]),
    kpi('Camber gain (bump)', fmt(m.camberGain, 3), '°/10mm'), kpi('Roll camber compensation', fmt(m.camberComp, 0), '%'),
    kpi('Bump steer', fmt(m.bumpSteer, 2), '°/m', [statusDot(lvl(m.bumpSteer, 1, 5)), `toe range ${fmt(m.toeRange, 3)}° over ±50 mm`]),
    kpi('Motion ratio (damper/wheel)', fmt(m.mr, 3), '', `ΔMR ${fmt(m.mrProg, 3)} / 100 mm`),
    ...(ax === 'front' ? [kpi('Anti-dive', fmt(m.antiDive, 1), '%'), kpi('Ackermann', fmt(m.ackermann, 0), '%')] : [kpi('Anti-squat', fmt(m.antiSquat, 1), '%'), kpi('Anti-lift (braking)', fmt(m.antiLift, 1), '%')]),
    kpi('Track change', fmt(m.trackChange, 3), 'mm/mm'), kpi('Static track', fmt(K.trackStatic, 0), 'mm'),
  ];
  const cv = K.curves;
  const out = [axSel, h('div', { class: 'kpis' }, tiles)];
  out.push(lineChart({ title: 'Camber vs wheel travel', xLabel: 'wheel travel (mm, + bump)', series: [{ name: 'Camber', x: cv.travel, y: cv.camber }], yFmt: (v) => `${v.toFixed(3)}°`, height: 130, minSpan: 0.5 }));
  out.push(lineChart({ title: 'Toe vs wheel travel (bump steer)', xLabel: 'wheel travel (mm, + bump)', series: [{ name: 'Toe-in', x: cv.travel, y: cv.toe, color: 'var(--series-2)' }], yFmt: (v) => `${v.toFixed(4)}°`, height: 130, minSpan: 0.2 }));
  out.push(lineChart({ title: 'Motion ratio vs wheel travel', xLabel: 'wheel travel (mm)', series: [{ name: 'MR', x: cv.travel, y: cv.mr, color: 'var(--series-3)' }], yFmt: (v) => v.toFixed(3), height: 120, minSpan: 0.1 }));
  out.push(lineChart({ title: 'Track & wheelbase change', xLabel: 'wheel travel (mm)', series: [{ name: 'Track Δ', x: cv.travel, y: cv.track }, { name: 'Contact patch x Δ', x: cv.travel, y: cv.wbase, color: 'var(--series-2)', dash: true }], yFmt: (v) => `${v.toFixed(2)} mm`, height: 120, yZero: true }));
  const rc = K.rollCurves;
  out.push(lineChart({ title: 'Roll centre in roll', xLabel: 'body roll (°)', series: [{ name: 'RC height', x: rc.roll, y: rc.rcH }, { name: 'RC lateral', x: rc.roll, y: rc.rcY, color: 'var(--series-2)' }], yFmt: (v) => `${v.toFixed(1)} mm`, height: 140, yZero: true }));
  out.push(lineChart({ title: 'Camber to ground in roll', xLabel: 'body roll (°, + left side down)', series: [{ name: 'Left wheel', x: rc.roll, y: rc.camberL }, { name: 'Right wheel', x: rc.roll, y: rc.camberR, color: 'var(--series-2)' }], yFmt: (v) => `${v.toFixed(2)}°`, height: 130 }));
  if (K.steer) {
    const st = K.steer;
    const cf = def.cFactor || 55;
    out.push(lineChart({ title: 'Steering - inner / outer / ideal Ackermann', xLabel: 'rack travel (mm)', series: [{ name: 'Inner', x: st.rack, y: st.inner }, { name: 'Outer', x: st.rack, y: st.outer, color: 'var(--series-2)' }, { name: 'Ideal outer', x: st.rack, y: st.ideal, color: 'var(--series-3)', dash: true }], yFmt: (v) => `${v.toFixed(2)}°`, height: 150 }));
    const lock = st.inner[st.inner.length - 1];
    const sw = (st.rack[st.rack.length - 1] / cf) * 360;
    const Rturn = state.cfg.vehicle.wheelbase / Math.tan(((lock + st.outer[st.outer.length - 1]) / 2) * Math.PI / 180) / 1000;
    out.push(h('div', { class: 'kpis three' }, kpi('Lock (inner)', fmt(lock, 1), '°'), kpi('Overall ratio', fmt(sw / ((lock + st.outer[st.outer.length - 1]) / 2), 1), ':1', `${fmt(sw, 0)}° hand-wheel`), kpi('Turning radius', fmt(Rturn, 2), 'm', 'kerb-to-kerb approx.')));
  }
  if (m.svic) out.push(h('div', { class: 'hint' }, `Side-view IC at x=${fmt(m.svic[0], 0)} mm, z=${fmt(m.svic[1], 0)} mm (axle-local). Front-view IC (left): ${m.icFront ? `y=${fmt(m.icFront[0], 0)}, z=${fmt(m.icFront[1], 0)} mm` : 'at infinity (parallel arms)'}.`));
  return out;
}

function dynPanel() {
  const mp = state.mp, R = state.ride, c = state.cfg;
  if (!mp || !R) return [h('div', { class: 'callout' }, 'Mass properties need a solved chassis and suspension.')];
  const cur = R.current;
  const out = [];
  out.push(section('Mass & inertia', [h('div', { class: 'kpis' },
    kpi('Total mass', fmt(mp.M, 0), 'kg', `sprung ${fmt(mp.Ms, 0)} kg`),
    kpi('Front weight', fmt(mp.frontFrac * 100, 1), '%'),
    kpi('CG height', fmt(mp.cg[2], 0), 'mm'),
    kpi('CG x from front axle', fmt(-mp.cg[0], 0), 'mm'),
    kpi('Yaw inertia Izz', fmt(mp.Izz, 0), 'kg·m²'),
    kpi('Roll inertia Ixx', fmt(mp.Ixx, 0), 'kg·m²'))]));
  const d = (a, b) => (Math.abs(a - b) / b < 0.05 ? 'good' : Math.abs(a - b) / b < 0.15 ? 'warn' : 'bad');
  out.push(section('Ride & roll (current set-up)', [h('div', { class: 'kpis' },
    kpi('Ride frequency front', fmt(cur.fF, 2), 'Hz', [statusDot(d(cur.fF, c.ride.fF)), `target ${c.ride.fF}`]),
    kpi('Ride frequency rear', fmt(cur.fR, 2), 'Hz', [statusDot(d(cur.fR, c.ride.fR)), `target ${c.ride.fR}`]),
    kpi('Wheel rate F / R', `${fmt(cur.wheelRateF, 0)} / ${fmt(cur.wheelRateR, 0)}`, 'N/mm'),
    kpi('Roll gradient', fmt(cur.rollGrad, 2), '°/g', [statusDot(d(cur.rollGrad, c.ride.rollGrad)), `target ${c.ride.rollGrad}`]),
    kpi('LLTD front', fmt(cur.lltd, 1), '%', [statusDot(Math.abs(cur.lltd - c.ride.lltd) < 2 ? 'good' : Math.abs(cur.lltd - c.ride.lltd) < 5 ? 'warn' : 'bad'), `target ${c.ride.lltd}%`]),
    kpi('Roll moment arm', fmt(cur.hArm, 0), 'mm', `RC at CG ${fmt(cur.zrc, 0)} mm`))]));
  if (R.chassis) {
    const C = R.chassis;
    const lvl = C.ratio >= 5 ? 'good' : C.ratio >= 3 ? 'warn' : 'bad';
    out.push(section('Chassis stiffness vs suspension roll stiffness', [
      h('div', { class: 'kpis' },
        kpi('K chassis / K roll (F+R)', fmt(C.ratio, 1), '×', [statusDot(lvl), 'aim ≥ 3-5×']),
        kpi('LLTD rigid → flexible', `${fmt(C.lltdRigid, 1)} → ${fmt(C.lltdFlex, 1)}`, '%')),
      h('div', { class: `callout ${lvl}` }, C.advice),
      lineChart({ title: 'Front LLTD vs chassis torsional stiffness', xLabel: 'chassis K / total roll stiffness (log)', series: [{ name: 'LLTD front', x: C.curve.map((p) => Math.log10(p.ratio)), y: C.curve.map((p) => p.lltd) }], markers: [{ x: Math.log10(C.ratio), y: C.lltdFlex, label: 'this chassis' }], xFmt: (v) => `${(10 ** v).toFixed(2)}×`, yFmt: (v) => `${v.toFixed(1)}%`, height: 150 }),
      h('div', { class: 'hint' }, 'Twin-spring model: front and rear axle roll springs linked by the chassis torsion spring. A flexible chassis pulls LLTD toward the static weight split and blunts ARB tuning.'),
    ]));
  } else {
    out.push(h('div', { class: 'callout' }, 'Run the torsion FEA to see how chassis stiffness compares with the suspension roll stiffness and what it does to LLTD.'));
  }
  out.push(section('Mass items', [h('table', { class: 'grid' }, h('tr', {}, h('th', {}, 'Item'), h('th', {}, 'kg'), h('th', {}, 'x'), h('th', {}, 'z')),
    mp.items.map((i) => h('tr', {}, h('td', {}, i.name), h('td', {}, fmt(i.m, 1)), h('td', {}, fmt(i.x, 0)), h('td', {}, fmt(i.z, 0)))))], { collapsed: true }));
  return out;
}

// ------------------------------------------------------------------ picking + gizmo
const raycaster = new THREE.Raycaster();
let gizmo = null, gizmoProxy = null, dragging = false;
try {
  gizmo = new TransformControls(viewer.persp, viewer.renderer.domElement);
  gizmo.setSize(0.8);
  gizmoProxy = new THREE.Object3D();
  viewer.scene.add(gizmoProxy);
  const helper = gizmo.getHelper ? gizmo.getHelper() : gizmo;
  viewer.scene.add(helper);
  gizmo.addEventListener('dragging-changed', (e) => {
    viewer.controls.enabled = !e.value;
    dragging = e.value;
    if (!e.value && state.selHp) { onSuspChange(true); renderLeft(); }
  });
  gizmo.addEventListener('objectChange', () => {
    if (!state.selHp) return;
    const { axle, side, name } = state.selHp;
    const def = axle === 'F' ? state.cfg.suspension.front : state.cfg.suspension.rear;
    const x0 = axle === 'F' ? 0 : -state.cfg.vehicle.wheelbase;
    const p = gizmoProxy.position;
    def.hp[name] = [p.x - x0, side * p.y, p.z];
    chassisView.selMarker.position.copy(p);
    onSuspChange(false);
    showHpHud();
  });
} catch (e) {
  gizmo = null;
}
function detachGizmo() { if (gizmo) gizmo.detach(); $('#hpHud').classList.add('hidden'); }

viewer.renderer.domElement.addEventListener('pointerdown', (ev) => { downAt = [ev.clientX, ev.clientY]; });
let downAt = null;
viewer.renderer.domElement.addEventListener('pointerup', (ev) => {
  if (!downAt || dragging) return;
  if (Math.hypot(ev.clientX - downAt[0], ev.clientY - downAt[1]) > 4) return;
  const { cam, ndc } = viewer.pickCamera(ev.clientX, ev.clientY);
  raycaster.setFromCamera(ndc, cam);
  if (!chassisView.hpMesh?.visible) return;
  const hit = raycaster.intersectObject(chassisView.hpMesh, false)[0];
  if (hit && hit.instanceId !== undefined) {
    const hp = state.mesh.hardpoints[hit.instanceId];
    state.selHp = { axle: hp.axle, side: hp.side, name: hp.name, index: hit.instanceId };
    chassisView.select(hit.instanceId);
    if (gizmo && cam === viewer.persp) { gizmoProxy.position.set(...hp.pos); gizmo.attach(gizmoProxy); }
    state.suspAxle = hp.axle === 'F' ? 'front' : 'rear';
    if (state.leftTab === 'suspension') renderLeft();
    showHpHud();
  } else if (!gizmo || !gizmo.dragging) {
    state.selHp = null;
    chassisView.select(null);
    detachGizmo();
    if (state.leftTab === 'suspension') renderLeft();
  }
});

function showHpHud() {
  const s = state.selHp;
  const hud = $('#hpHud');
  if (!s) { hud.classList.add('hidden'); return; }
  const def = s.axle === 'F' ? state.cfg.suspension.front : state.cfg.suspension.rear;
  const label = HARDPOINTS.find(([k]) => k === s.name)?.[1] || s.name;
  const p = def.hp[s.name];
  const inputs = [0, 1, 2].map((i) => {
    const inp = h('input', { type: 'number', step: 1, value: p[i].toFixed(1) });
    inp.addEventListener('change', () => { def.hp[s.name][i] = parseFloat(inp.value); onSuspChange(true); refreshSelection(); });
    return inp;
  });
  hud.innerHTML = '';
  hud.append(h('div', { class: 't' }, `${s.axle === 'F' ? 'Front' : 'Rear'} ${s.side > 0 ? 'left' : 'right'} · ${label}`),
    h('div', { class: 'muted' }, 'x (axle-local), y (left side), z'),
    h('div', { class: 'xyz' }, inputs),
    h('div', { class: 'muted' }, gizmo ? 'Drag the gizmo in the 3D view; mirrored to the other side.' : ''));
  hud.classList.remove('hidden');
}
function refreshSelection() {
  const s = state.selHp;
  if (!s || !state.mesh) return;
  const idx = state.mesh.hardpoints.findIndex((q) => q.axle === s.axle && q.side === s.side && q.name === s.name);
  if (idx < 0) return;
  s.index = idx;
  chassisView.select(idx);
  if (gizmo && gizmo.object) gizmoProxy.position.set(...state.mesh.hardpoints[idx].pos);
}

// ------------------------------------------------------------------ top bar + files
const presetSel = $('#presetSelect');
for (const [k, t] of PRESET_LIST) presetSel.append(h('option', { value: k }, t));
presetSel.addEventListener('change', () => { state.cfg = normalise(makePreset(presetSel.value)); persist(); fullReload(); });
const menu = $('#fileMenu');
$('#btnExport').addEventListener('click', (e) => { e.stopPropagation(); menu.classList.toggle('hidden'); });
document.addEventListener('click', () => menu.classList.add('hidden'));
menu.addEventListener('click', async (e) => {
  const act = e.target.dataset.act;
  if (!act) return;
  const slug = state.cfg.type;
  if (act === 'save-json') download(`chassis-${slug}.json`, JSON.stringify(state.cfg, null, 2), 'application/json');
  if (act === 'load-json') $('#fileInput').click();
  if (act === 'cutlist') download(`cutlist-${slug}.csv`, cutList(state.cfg, state.mesh), 'text/csv');
  if (act === 'hardpoints') download(`hardpoints-${slug}.csv`, hardpointsCSV(state.cfg), 'text/csv');
  if (act === 'bdf') download(`chassis-${slug}.bdf`, nastranBDF(state.cfg, state.mesh));
  if (act === 'shot') download(`chassis-${slug}.png`, await viewer.screenshot());
  if (act === 'reset') { state.cfg = normalise(makePreset(state.cfg.type)); persist(); fullReload(); }
});
$('#fileInput').addEventListener('change', async (e) => {
  const f = e.target.files[0];
  if (!f) return;
  try {
    const c = JSON.parse(await f.text());
    if (!c.chassis || !c.suspension) throw new Error('not a chassis design file');
    state.cfg = normalise(c);
    persist();
    fullReload();
    setStatus(`Loaded ${f.name}`, 'good');
  } catch (err) {
    setStatus(`Could not load file: ${err.message}`, 'bad');
  }
  e.target.value = '';
});

// ------------------------------------------------------------------ start
state.cfg = loadCfg();
buildToolbar();
suspView.onPose = updatePoseHud;
updatePoseHud();
fullReload();
setStatus(`Ready - ${viewer.isWebGPU ? 'WebGPU' : 'WebGL2'} renderer. Run the FEA from the right panel.`);
window.__app = { state, viewer, chassisView, suspView, runFEA };
