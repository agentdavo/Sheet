// FE model assembly, static load cases, post-processing and modal analysis.
import { shellGlobal, shellRecover, beamGlobal, beamRecover, shellFrame } from './elements.js';
import { rcm, EnvelopeMatrix } from './sparse.js';
import { symEig } from './linalg.js';

const nodeP = (nodes, i) => [nodes[3 * i], nodes[3 * i + 1], nodes[3 * i + 2]];

/**
 * Pre-compute element matrices, graph ordering and element dof lists.
 * model = { nodes, shells, shellT, shellGroup, mat, beams, beamSec, beamUp, sections, lumped }
 */
export function prepare(model) {
  const nn = model.nodes.length / 3;
  const ns = model.shells.length / 4;
  const nb = model.beams.length / 2;
  const { E, nu } = model.mat;
  const elems = [];
  const X = new Float64Array(12);
  for (let e = 0; e < ns; e++) {
    const conn = model.shells.subarray(4 * e, 4 * e + 4);
    for (let k = 0; k < 4; k++) for (let c = 0; c < 3; c++) X[3 * k + c] = model.nodes[3 * conn[k] + c];
    const { K } = shellGlobal(X, model.shellT[e], E, nu);
    elems.push({ type: 0, idx: e, conn: Array.from(conn), K });
  }
  for (let e = 0; e < nb; e++) {
    const a = model.beams[2 * e], b = model.beams[2 * e + 1];
    const sec = model.sections[model.beamSec[e]];
    const up = [model.beamUp[3 * e], model.beamUp[3 * e + 1], model.beamUp[3 * e + 2]];
    const { K } = beamGlobal(nodeP(model.nodes, a), nodeP(model.nodes, b), up, sec);
    elems.push({ type: 1, idx: e, conn: [a, b], K });
  }
  // node graph
  const sets = Array.from({ length: nn }, () => new Set());
  for (const el of elems)
    for (const a of el.conn) for (const b of el.conn) if (a !== b) sets[a].add(b);
  const adj = sets.map((s) => Int32Array.from(s));
  const perm = rcm(adj);
  const rank = new Int32Array(nn);
  for (let i = 0; i < nn; i++) rank[perm[i]] = i;
  return { nn, elems, perm, rank, adj };
}

/** Equation numbering given constrained dofs (Uint8Array(6nn), 1 = fixed). */
function numbering(prep, fixed) {
  const { nn, perm } = prep;
  const eq = new Int32Array(6 * nn).fill(-1);
  let n = 0;
  for (let p = 0; p < nn; p++) {
    const node = perm[p];
    for (let d = 0; d < 6; d++) if (!fixed[6 * node + d]) eq[6 * node + d] = n++;
  }
  return { eq, n };
}

function assemble(prep, eq, n, shift, mass) {
  const first = new Int32Array(n);
  for (let i = 0; i < n; i++) first[i] = i;
  for (const el of prep.elems) {
    let m = Infinity;
    for (const a of el.conn) for (let d = 0; d < 6; d++) { const q = eq[6 * a + d]; if (q >= 0 && q < m) m = q; }
    for (const a of el.conn) for (let d = 0; d < 6; d++) { const q = eq[6 * a + d]; if (q >= 0 && m < first[q]) first[q] = m; }
  }
  const K = new EnvelopeMatrix(n, first);
  const g = new Int32Array(24);
  for (const el of prep.elems) {
    const nd = el.conn.length * 6;
    for (let k = 0; k < el.conn.length; k++) for (let d = 0; d < 6; d++) g[6 * k + d] = eq[6 * el.conn[k] + d];
    const Ke = el.K;
    for (let a = 0; a < nd; a++) {
      const i = g[a];
      if (i < 0) continue;
      for (let b = 0; b < nd; b++) {
        const j = g[b];
        if (j < 0 || j > i) continue;
        K.add(i, j, Ke[a * nd + b]);
      }
    }
  }
  if (shift && mass) {
    for (let dof = 0; dof < eq.length; dof++) { const q = eq[dof]; if (q >= 0) K.add(q, q, shift * mass[dof]); }
  }
  return K;
}

/** Build a CSR matrix (reduced system) for iterative / GPU solvers. */
export function buildCSR(prep, eq, n) {
  const rows = Array.from({ length: n }, () => new Map());
  for (const el of prep.elems) {
    const nd = el.conn.length * 6;
    const g = [];
    for (const a of el.conn) for (let d = 0; d < 6; d++) g.push(eq[6 * a + d]);
    for (let a = 0; a < nd; a++) {
      const i = g[a];
      if (i < 0) continue;
      const row = rows[i];
      for (let b = 0; b < nd; b++) {
        const j = g[b];
        if (j < 0) continue;
        row.set(j, (row.get(j) || 0) + el.K[a * nd + b]);
      }
    }
  }
  const rowPtr = new Uint32Array(n + 1);
  let nnz = 0;
  for (let i = 0; i < n; i++) { rowPtr[i] = nnz; nnz += rows[i].size; }
  rowPtr[n] = nnz;
  const col = new Uint32Array(nnz);
  const val = new Float64Array(nnz);
  for (let i = 0; i < n; i++) {
    const ent = [...rows[i].entries()].sort((a, b) => a[0] - b[0]);
    let p = rowPtr[i];
    for (const [j, v] of ent) { col[p] = j; val[p] = v; p++; }
  }
  return { n, rowPtr, col, val };
}

export function csrMul(A, x, y) {
  const { n, rowPtr, col, val } = A;
  for (let i = 0; i < n; i++) {
    let s = 0;
    for (let p = rowPtr[i]; p < rowPtr[i + 1]; p++) s += val[p] * x[col[p]];
    y[i] = s;
  }
  return y;
}

/** Factorise K for a set of fixed dofs; returns a solver working on full-length vectors. */
export function factorize(prep, fixed, onProgress) {
  const { eq, n } = numbering(prep, fixed);
  const K = assemble(prep, eq, n);
  K.factor(onProgress);
  const solveFull = (f) => {
    const rhs = new Float64Array(n);
    for (let d = 0; d < eq.length; d++) if (eq[d] >= 0) rhs[eq[d]] = f[d];
    return expand(K.solve(rhs), eq);
  };
  return { solveFull, eq, n, info: { envelope: K.size, equations: n } };
}

/**
 * Solve with extra homogeneous point constraints (u[dof] = 0 for dof in `cdofs`) re-using an
 * existing factorisation, via Lagrange multipliers: u = u0 + X lambda, (C X) lambda = -C u0.
 */
export function solveWithConstraints(fac, f, cdofs) {
  const u0 = fac.solveFull(f);
  const m = cdofs.length;
  if (!m) return u0;
  const X = cdofs.map((d) => { const e = new Float64Array(f.length); e[d] = 1; return fac.solveFull(e); });
  // dense m x m system S lambda = -u0[c]
  const S = new Float64Array(m * m);
  for (let i = 0; i < m; i++) for (let j = 0; j < m; j++) S[i * m + j] = X[j][cdofs[i]];
  const b = cdofs.map((d) => -u0[d]);
  const lam = gaussSolve(S, b, m);
  const u = Float64Array.from(u0);
  for (let j = 0; j < m; j++) { const xj = X[j]; const l = lam[j]; for (let i = 0; i < u.length; i++) u[i] += l * xj[i]; }
  return u;
}

function gaussSolve(A0, b0, n) {
  const A = Float64Array.from(A0), b = Float64Array.from(b0);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(A[r * n + c]) > Math.abs(A[p * n + c])) p = r;
    if (p !== c) { for (let k = 0; k < n; k++) [A[c * n + k], A[p * n + k]] = [A[p * n + k], A[c * n + k]]; [b[c], b[p]] = [b[p], b[c]]; }
    const d = A[c * n + c] || 1e-300;
    for (let r = c + 1; r < n; r++) {
      const f = A[r * n + c] / d;
      if (!f) continue;
      for (let k = c; k < n; k++) A[r * n + k] -= f * A[c * n + k];
      b[r] -= f * b[c];
    }
  }
  const x = new Float64Array(n);
  for (let r = n - 1; r >= 0; r--) {
    let s = b[r];
    for (let k = r + 1; k < n; k++) s -= A[r * n + k] * x[k];
    x[r] = s / (A[r * n + r] || 1e-300);
  }
  return x;
}

/**
 * Static analysis for one set of boundary conditions and one or more load vectors.
 * bc: { fixed: Uint8Array(6nn), loads: [Float64Array(6nn), ...] }
 * opts.iterative: optional async function(csr, rhs, prep, eq) -> {x, method, info} (e.g. WebGPU PCG).
 */
export async function solveStatic(prep, bc, opts = {}) {
  const t0 = performance.now();
  const results = [];
  let method = 'Envelope Cholesky (CPU, f64)';
  let info = {};
  if (opts.iterative) {
    const { eq, n } = numbering(prep, bc.fixed);
    const csr = buildCSR(prep, eq, n);
    for (const f of bc.loads) {
      const rhs = new Float64Array(n);
      for (let d = 0; d < eq.length; d++) if (eq[d] >= 0) rhs[eq[d]] = f[d];
      const r = await opts.iterative(csr, rhs, prep, eq);
      info = r.info || {};
      method = r.method;
      results.push(expand(r.x, eq));
    }
  } else {
    const fac = factorize(prep, bc.fixed, opts.onProgress);
    info = fac.info;
    for (const f of bc.loads) results.push(fac.solveFull(f));
    return { u: results, ms: performance.now() - t0, method, info, fac };
  }
  return { u: results, ms: performance.now() - t0, method, info };
}

function expand(x, eq) {
  const u = new Float64Array(eq.length);
  for (let d = 0; d < eq.length; d++) if (eq[d] >= 0) u[d] = x[eq[d]];
  return u;
}

/** Element results: von Mises, strain energy (membrane/bending split) per shell; per beam. */
export function postprocess(model, u) {
  const ns = model.shells.length / 4;
  const nb = model.beams.length / 2;
  const { E, nu } = model.mat;
  const shellVM = new Float32Array(ns);
  const shellU = new Float64Array(ns);
  const shellUm = new Float64Array(ns);
  const shellUb = new Float64Array(ns);
  const X = new Float64Array(12);
  const ug = new Float64Array(24);
  for (let e = 0; e < ns; e++) {
    for (let k = 0; k < 4; k++) {
      const nd = model.shells[4 * e + k];
      for (let c = 0; c < 3; c++) X[3 * k + c] = model.nodes[3 * nd + c];
      for (let d = 0; d < 6; d++) ug[6 * k + d] = u[6 * nd + d];
    }
    const r = shellRecover(X, model.shellT[e], E, nu, ug);
    shellVM[e] = r.vm;
    shellUm[e] = r.Um; shellUb[e] = r.Ub; shellU[e] = r.Um + r.Ub;
  }
  const beamVM = new Float32Array(nb);
  const beamU = new Float64Array(nb);
  const ub = new Float64Array(12);
  for (let e = 0; e < nb; e++) {
    const a = model.beams[2 * e], b = model.beams[2 * e + 1];
    for (let d = 0; d < 6; d++) { ub[d] = u[6 * a + d]; ub[6 + d] = u[6 * b + d]; }
    const sec = model.sections[model.beamSec[e]];
    const up = [model.beamUp[3 * e], model.beamUp[3 * e + 1], model.beamUp[3 * e + 2]];
    const r = beamRecover(nodeP(model.nodes, a), nodeP(model.nodes, b), up, sec, ub);
    beamVM[e] = sec.rigid ? 0 : r.vm;
    beamU[e] = r.U;
  }
  return { shellVM, shellU, shellUm, shellUb, beamVM, beamU };
}

/** Structural mass and lumped mass vector (6 dof/node). */
export function massModel(model) {
  const nn = model.nodes.length / 3;
  const ns = model.shells.length / 4;
  const nb = model.beams.length / 2;
  const m = new Float64Array(6 * nn);
  let total = 0;
  const groupMass = {};
  const X = new Float64Array(12);
  for (let e = 0; e < ns; e++) {
    for (let k = 0; k < 4; k++) for (let c = 0; c < 3; c++) X[3 * k + c] = model.nodes[3 * model.shells[4 * e + k] + c];
    const A = shellFrame(X).area;
    const me = A * model.shellT[e] * model.mat.rho;
    total += me;
    const g = model.groups[model.shellGroup[e]];
    groupMass[g] = (groupMass[g] || 0) + me;
    const rot = (me / 4) * (A / 12);
    for (let k = 0; k < 4; k++) {
      const nd = model.shells[4 * e + k];
      for (let d = 0; d < 3; d++) { m[6 * nd + d] += me / 4; m[6 * nd + 3 + d] += rot; }
    }
  }
  let beamMass = 0;
  for (let e = 0; e < nb; e++) {
    const a = model.beams[2 * e], b = model.beams[2 * e + 1];
    const sec = model.sections[model.beamSec[e]];
    const pa = nodeP(model.nodes, a), pb = nodeP(model.nodes, b);
    const L = Math.hypot(pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]);
    const me = sec.rigid ? 0 : sec.A * L * sec.rho;
    beamMass += me;
    const rot = (me / 2) * (L * L / 12) + 1e-3;
    for (const nd of [a, b]) for (let d = 0; d < 3; d++) { m[6 * nd + d] += me / 2 + 1e-4; m[6 * nd + 3 + d] += rot; }
  }
  for (const l of model.lumped || []) for (let d = 0; d < 3; d++) m[6 * l.node + d] += l.mass;
  // floor on nodal mass keeps the Lanczos M-inner product well conditioned (massless frames / spiders)
  for (let i = 0; i < m.length; i++) if (m[i] < (i % 6 < 3 ? 5e-3 : 5)) m[i] = i % 6 < 3 ? 5e-3 : 5;
  return { m, shellMass: total, beamMass, total: total + beamMass, groupMass };
}

/**
 * Free-free modal analysis via shift-invert Lanczos with full re-orthogonalisation.
 * Returns elastic modes (rigid body modes skipped).
 */
export function modal(prep, model, nModes = 6, onProgress) {
  const nn = prep.nn;
  const { m } = massModel(model);
  const fixed = new Uint8Array(6 * nn);
  const { eq, n } = numbering(prep, fixed);
  const mr = new Float64Array(n);
  for (let d = 0; d < eq.length; d++) if (eq[d] >= 0) mr[eq[d]] = m[d];
  // K [N/mm], M [kg] -> omega^2 = 1000 * lambda  (1 N = 1000 kg mm / s^2)
  const sigma = (2 * Math.PI * 5) ** 2 / 1000; // shift (5 Hz) makes K + sigma M positive definite
  const K = assemble(prep, eq, n, sigma, m);
  K.factor((p) => onProgress && onProgress(0.4 * p));
  const steps = Math.min(n, Math.max(60, 6 * (nModes + 6)));
  const Q = [];
  const alpha = [], beta = [];
  let r = new Float64Array(n);
  let seed = 12345;
  for (let i = 0; i < n; i++) { seed = (seed * 1103515245 + 12345) & 0x7fffffff; r[i] = seed / 0x7fffffff - 0.5; }
  const mdot = (a, b) => { let s = 0; for (let i = 0; i < n; i++) s += a[i] * mr[i] * b[i]; return s; };
  let b = Math.sqrt(mdot(r, r));
  let qPrev = null;
  for (let j = 0; j < steps; j++) {
    const q = new Float64Array(n);
    for (let i = 0; i < n; i++) q[i] = r[i] / b;
    Q.push(q);
    const p = new Float64Array(n);
    for (let i = 0; i < n; i++) p[i] = mr[i] * q[i];
    r = K.solve(p);
    if (qPrev) for (let i = 0; i < n; i++) r[i] -= b * qPrev[i];
    const a = mdot(q, r);
    for (let i = 0; i < n; i++) r[i] -= a * q[i];
    for (let pass = 0; pass < 2; pass++)
      for (const qi of Q) { const c = mdot(qi, r); if (c) for (let i = 0; i < n; i++) r[i] -= c * qi[i]; }
    alpha.push(a);
    b = Math.sqrt(mdot(r, r));
    beta.push(b);
    qPrev = q;
    if (onProgress) onProgress(0.4 + (0.55 * (j + 1)) / steps);
    if (b < 1e-12) break;
  }
  const k = alpha.length;
  const T = new Float64Array(k * k);
  for (let i = 0; i < k; i++) {
    T[i * k + i] = alpha[i];
    if (i + 1 < k) { T[i * k + i + 1] = beta[i]; T[(i + 1) * k + i] = beta[i]; }
  }
  const { values, vectors } = symEig(T, k);
  // theta = 1/(lambda + sigma)  -> lambda = 1/theta - sigma ; sort by lambda ascending
  const idx = [...values.keys()].filter((i) => values[i] > 1e-14).sort((a, c) => values[c] - values[a]);
  const modes = [];
  for (const i of idx) {
    const lam = 1 / values[i] - sigma;
    const f = lam > 0 ? Math.sqrt(1000 * lam) / (2 * Math.PI) : 0;
    if (f < 1.0) continue; // rigid-body
    const x = new Float64Array(n);
    for (let c = 0; c < k; c++) { const s = vectors[c * k + i]; if (s) { const qc = Q[c]; for (let t = 0; t < n; t++) x[t] += s * qc[t]; } }
    const u = expand(x, eq);
    modes.push({ freq: f, shape: u, label: classifyMode(model, u) });
    if (modes.length >= nModes) break;
  }
  return modes;
}

function classifyMode(model, u) {
  const nn = model.nodes.length / 3;
  let xmin = Infinity, xmax = -Infinity;
  for (let i = 0; i < nn; i++) { xmin = Math.min(xmin, model.nodes[3 * i]); xmax = Math.max(xmax, model.nodes[3 * i]); }
  const NB = 12;
  const L = new Float64Array(NB), R = new Float64Array(NB), cl = new Float64Array(NB), cr = new Float64Array(NB), Y = new Float64Array(NB), cy = new Float64Array(NB);
  let ax = 0;
  for (let i = 0; i < nn; i++) {
    const b = Math.min(NB - 1, Math.floor(((model.nodes[3 * i] - xmin) / (xmax - xmin + 1e-9)) * NB));
    const y = model.nodes[3 * i + 1];
    const uz = u[6 * i + 2];
    if (y > 30) { L[b] += uz; cl[b]++; } else if (y < -30) { R[b] += uz; cr[b]++; }
    Y[b] += u[6 * i + 1]; cy[b]++;
    ax += u[6 * i] * u[6 * i];
  }
  let sym = 0, anti = 0, lat = 0;
  for (let b = 0; b < NB; b++) {
    const l = cl[b] ? L[b] / cl[b] : 0, r = cr[b] ? R[b] / cr[b] : 0;
    sym += ((l + r) / 2) ** 2; anti += ((l - r) / 2) ** 2;
    if (cy[b]) lat += (Y[b] / cy[b]) ** 2;
  }
  const axial = ax / nn;
  const best = Math.max(sym, anti, lat, axial * 0.5);
  if (best === anti) return 'Torsion';
  if (best === sym) return 'Vertical bending';
  if (best === lat) return 'Lateral bending';
  return 'Local / axial';
}

export { numbering };
