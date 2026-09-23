// Chassis load cases: torsional stiffness, bending stiffness, free-free modal.
import { prepare, solveStatic, postprocess, massModel, modal, solveWithConstraints } from './model.js';
import { lumpComponents } from '../chassis/mesh.js';

const RAD2DEG = 180 / Math.PI;

function hpNodes(mesh, axle) {
  return mesh.hardpoints.filter((h) => h.axle === axle);
}

/** Torsion: rear pick-ups pinned, equal and opposite vertical loads at the front pick-ups. */
export function torsionBC(mesh, torqueNm = 1000) {
  const nn = mesh.model.nodes.length / 3;
  const fixed = new Uint8Array(6 * nn);
  for (const h of hpNodes(mesh, 'R')) for (let d = 0; d < 3; d++) fixed[6 * h.node + d] = 1;
  const front = hpNodes(mesh, 'F');
  const L = front.filter((h) => h.side > 0), R = front.filter((h) => h.side < 0);
  const yL = L.reduce((s, h) => s + h.pos[1], 0) / L.length;
  const yR = R.reduce((s, h) => s + h.pos[1], 0) / R.length;
  const Fz = (torqueNm * 1000) / (yL - yR); // N
  const f = new Float64Array(6 * nn);
  for (const h of L) f[6 * h.node + 2] += Fz / L.length;
  for (const h of R) f[6 * h.node + 2] -= Fz / R.length;
  return { fixed, loads: [f], meta: { L, R, yL, yR, torqueNm, Fz } };
}

/** Bending: rear pick-ups pinned, front pick-ups vertically supported, cockpit sills loaded. */
export function bendingBC(mesh, loadN = 5000) {
  const nn = mesh.model.nodes.length / 3;
  const fixed = new Uint8Array(6 * nn);
  for (const h of hpNodes(mesh, 'R')) for (let d = 0; d < 3; d++) fixed[6 * h.node + d] = 1;
  for (const h of hpNodes(mesh, 'F')) fixed[6 * h.node + 2] = 1;
  const f = new Float64Array(6 * nn);
  const fn = mesh.floorNodes.length ? mesh.floorNodes : [0];
  for (const i of fn) f[6 * i + 2] -= loadN / fn.length;
  return { fixed, loads: [f], meta: { loadN, nodes: fn } };
}

export async function runAnalyses(mesh, cfg, which, opts = {}) {
  const t0 = performance.now();
  const progress = opts.onProgress || (() => {});
  const model = mesh.model;
  progress('Assembling element matrices', 0.02);
  const prep = prepare(model);
  const mass = massModel(model);
  const out = { stats: mesh.stats, mass: { structure: mass.total, shells: mass.shellMass, beams: mass.beamMass, groups: mass.groupMass } };
  const solverOpts = { iterative: opts.iterative, onProgress: (p) => progress('Factorising stiffness matrix', 0.05 + 0.4 * p) };

  let torsionFac = null;
  if (which.includes('torsion') || which.includes('bending')) {
    progress('Torsion load case', 0.05);
    const bc = torsionBC(mesh, cfg.analysis?.torque ?? 1000);
    const r = await solveStatic(prep, bc, solverOpts);
    torsionFac = r.fac || null;
    const u = r.u[0];
    const { L, R, yL, yR, torqueNm } = bc.meta;
    const wL = L.reduce((s, h) => s + u[6 * h.node + 2], 0) / L.length;
    const wR = R.reduce((s, h) => s + u[6 * h.node + 2], 0) / R.length;
    const theta = (wL - wR) / (yL - yR); // rad
    const K = torqueNm / (theta * RAD2DEG);
    const post = postprocess(model, u);
    const twist = mesh.twistStations.map((s) => {
      const yl = model.nodes[3 * s.nL + 1], yr = model.nodes[3 * s.nR + 1];
      return { x: s.x, deg: ((u[6 * s.nL + 2] - u[6 * s.nR + 2]) / (yl - yr)) * RAD2DEG };
    });
    const hpDisp = mesh.hardpoints.map((h) => u[6 * h.node + 2]);
    out.torsion = {
      K, thetaDeg: theta * RAD2DEG, torqueNm, u, post, twist, hpDisp,
      solver: r.method, ms: r.ms, info: r.info,
      groupEnergy: groupEnergy(model, post),
    };
    progress('Torsion solved', 0.5);
  }
  if (which.includes('bending')) {
    progress('Bending load case', 0.5);
    const bc = bendingBC(mesh, cfg.analysis?.bendLoad ?? 5000);
    let u, r;
    const t1 = performance.now();
    if (torsionFac && !opts.iterative) {
      // same rear constraints as torsion: re-use that factorisation, add front z supports by Lagrange multipliers
      const cdofs = hpNodes(mesh, 'F').map((h) => 6 * h.node + 2);
      u = solveWithConstraints(torsionFac, bc.loads[0], cdofs);
      r = { method: 'Envelope Cholesky (re-used) + Lagrange supports', ms: performance.now() - t1 };
    } else {
      r = await solveStatic(prep, bc, { ...solverOpts, onProgress: (p) => progress('Factorising (bending)', 0.5 + 0.3 * p) });
      u = r.u[0];
    }
    const dz = bc.meta.nodes.reduce((s, i) => s + u[6 * i + 2], 0) / bc.meta.nodes.length;
    let dmax = 0;
    for (const i of bc.meta.nodes) dmax = Math.min(dmax, u[6 * i + 2]);
    const post = postprocess(model, u);
    out.bending = { K: bc.meta.loadN / Math.abs(dz), dMean: dz, dMax: dmax, loadN: bc.meta.loadN, u, post, solver: r.method, ms: r.ms };
    progress('Bending solved', 0.8);
  }
  if (which.includes('modal')) {
    progress('Modal analysis (shift-invert Lanczos)', 0.8);
    const mm = { ...model, lumped: cfg.analysis?.modalWithMasses ? lumpComponents(mesh, cfg.masses) : [] };
    const modes = modal(prep, mm, cfg.analysis?.nModes ?? 6, (p) => progress('Modal analysis', 0.8 + 0.19 * p));
    out.modal = modes;
  }
  out.ms = performance.now() - t0;
  progress('Done', 1);
  return out;
}

export function groupEnergy(model, post) {
  const g = {};
  for (let e = 0; e < post.shellU.length; e++) {
    const name = model.groups[model.shellGroup[e]];
    g[name] = g[name] || { U: 0, Um: 0, Ub: 0 };
    g[name].U += post.shellU[e]; g[name].Um += post.shellUm[e]; g[name].Ub += post.shellUb[e];
  }
  let beams = 0;
  for (let e = 0; e < post.beamU.length; e++) if (!model.sections[model.beamSec[e]].rigid) beams += post.beamU[e];
  g.tubes = { U: beams, Um: beams, Ub: 0 };
  return g;
}
