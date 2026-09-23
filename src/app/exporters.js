// File exports: sheet cut list, hardpoints, Nastran bulk data.
import { shellFrame } from '../fea/elements.js';
import { GROUP_LABELS } from '../chassis/mesh.js';
import { MATERIALS, TUBE_MATERIALS } from '../fea/materials.js';

const csv = (rows) => rows.map((r) => r.map((c) => (typeof c === 'string' && /[",\n]/.test(c) ? `"${c.replace(/"/g, '""')}"` : c)).join(',')).join('\n');

export function cutList(cfg, mesh) {
  const m = mesh.model;
  const mat = MATERIALS[cfg.material];
  const X = new Float64Array(12);
  const area = new Float64Array(mesh.panels.length);
  for (let e = 0; e < m.shells.length / 4; e++) {
    for (let k = 0; k < 4; k++) for (let c = 0; c < 3; c++) X[3 * k + c] = m.nodes[3 * m.shells[4 * e + k] + c];
    area[m.shellPanel[e]] += shellFrame(X).area;
  }
  const rows = [['Item', 'Group', 'Material', 'Thickness (mm)', 'Blank A (mm)', 'Blank B (mm)', 'Area (m2)', 'Mass (kg)']];
  let total = 0;
  mesh.panels.forEach((p, i) => {
    const t = cfg.gauges[p.group];
    const kg = area[i] * t * mat.rho;
    total += kg;
    rows.push([p.name, GROUP_LABELS[p.group], mat.name, t.toFixed(2), p.size[0].toFixed(0), p.size[1].toFixed(0), (area[i] / 1e6).toFixed(4), kg.toFixed(2)]);
  });
  rows.push([]);
  rows.push(['Tube', 'Class', 'Material', 'Wall (mm)', 'OD (mm)', 'Length (mm)', '', 'Mass (kg)']);
  for (const b of mesh.beamInfo) {
    if (b.frame) continue;
    let L = 0;
    for (let k = 1; k < b.nodes.length; k++) {
      const a = b.nodes[k - 1], c = b.nodes[k];
      L += Math.hypot(m.nodes[3 * c] - m.nodes[3 * a], m.nodes[3 * c + 1] - m.nodes[3 * a + 1], m.nodes[3 * c + 2] - m.nodes[3 * a + 2]);
    }
    const sec = m.sections[b.sec];
    const kg = sec.A * L * sec.rho;
    total += kg;
    const key = sec.name;
    const t = cfg.tubes[key];
    rows.push([b.name, key, TUBE_MATERIALS[t.mat].name, t.wall, t.od, L.toFixed(0), '', kg.toFixed(2)]);
  }
  rows.push([]);
  rows.push(['TOTAL', '', '', '', '', '', '', total.toFixed(2)]);
  return csv(rows);
}

export function hardpointsCSV(cfg) {
  const rows = [['Axle', 'Side', 'Point', 'X (mm, global)', 'Y (mm)', 'Z (mm)']];
  for (const [ax, def, x0] of [['Front', cfg.suspension.front, 0], ['Rear', cfg.suspension.rear, -cfg.vehicle.wheelbase]]) {
    for (const [k, p] of Object.entries(def.hp)) {
      if (def.actuation === 'direct' && k.startsWith('rock')) continue;
      rows.push([ax, 'L', k, (x0 + p[0]).toFixed(1), p[1].toFixed(1), p[2].toFixed(1)]);
      rows.push([ax, 'R', k, (x0 + p[0]).toFixed(1), (-p[1]).toFixed(1), p[2].toFixed(1)]);
    }
  }
  return csv(rows);
}

/** Nastran bulk data (free field). Units: mm, N, tonne (rho converted), MPa. */
export function nastranBDF(cfg, mesh) {
  const m = mesh.model;
  const L = [];
  const f = (v) => (Number.isInteger(v) ? String(v) : Number(v).toPrecision(8).replace(/e\+?/, 'E'));
  L.push('$ Sheet Chassis Studio - shell/beam chassis model');
  L.push(`$ ${cfg.name}`);
  L.push('$ units: mm, N, tonne, s, MPa');
  L.push('BEGIN BULK');
  L.push(`MAT1,1,${f(m.mat.E)},,${f(m.mat.nu)},${f(m.mat.rho * 1e-3)}`);
  const pidOf = new Map();
  let pid = 1;
  for (let e = 0; e < m.shellT.length; e++) {
    const key = `${m.shellGroup[e]}_${m.shellT[e]}`;
    if (!pidOf.has(key)) { pidOf.set(key, pid); L.push(`$ ${m.groups[m.shellGroup[e]]}`); L.push(`PSHELL,${pid},1,${f(m.shellT[e])},1,,1`); pid++; }
  }
  const beamPid = new Map();
  let mid = 2;
  m.sections.forEach((s, i) => {
    if (s.rigid) return;
    L.push(`MAT1,${mid},${f(s.E)},${f(s.G)},,${f(s.rho * 1e-3)}`);
    L.push(`PBAR,${pid},${mid},${f(s.A)},${f(s.Iy)},${f(s.Iz)},${f(s.J)}`);
    beamPid.set(i, pid); pid++; mid++;
  });
  for (let i = 0; i < m.nodes.length / 3; i++) L.push(`GRID,${i + 1},,${f(+m.nodes[3 * i].toFixed(3))},${f(+m.nodes[3 * i + 1].toFixed(3))},${f(+m.nodes[3 * i + 2].toFixed(3))}`);
  let eid = 1;
  for (let e = 0; e < m.shellT.length; e++) {
    const q = m.shells.subarray(4 * e, 4 * e + 4);
    L.push(`CQUAD4,${eid++},${pidOf.get(`${m.shellGroup[e]}_${m.shellT[e]}`)},${q[0] + 1},${q[1] + 1},${q[2] + 1},${q[3] + 1}`);
  }
  const rigid = new Map();
  for (let e = 0; e < m.beams.length / 2; e++) {
    const a = m.beams[2 * e], b = m.beams[2 * e + 1];
    const s = m.sections[m.beamSec[e]];
    if (s.rigid) { if (!rigid.has(a)) rigid.set(a, []); rigid.get(a).push(b); continue; }
    const dz = Math.abs(m.nodes[3 * b + 2] - m.nodes[3 * a + 2]);
    const dl = Math.hypot(m.nodes[3 * b] - m.nodes[3 * a], m.nodes[3 * b + 1] - m.nodes[3 * a + 1], m.nodes[3 * b + 2] - m.nodes[3 * a + 2]);
    const v = dz > 0.95 * dl ? '1.,0.,0.' : '0.,0.,1.';
    L.push(`CBAR,${eid++},${beamPid.get(m.beamSec[e])},${a + 1},${b + 1},${v}`);
  }
  for (const [a, deps] of rigid) {
    const ids = deps.map((d) => d + 1);
    let line = `RBE2,${eid++},${a + 1},123456`;
    const parts = [line, ...ids.map(String)];
    // continuation every 8 fields (free-field allows long lines, but keep it tidy)
    L.push(parts.join(','));
  }
  L.push('$ suspension pick-up grids:');
  for (const hp of mesh.hardpoints) L.push(`$   ${hp.axle}${hp.side > 0 ? 'L' : 'R'} ${hp.name} -> GRID ${hp.node + 1}`);
  L.push('ENDDATA');
  return L.join('\n');
}
