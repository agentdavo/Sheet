// Structural optimisers driven by the shell FE model.
//  * optimiseGauges  - minimum-mass sheet gauges meeting a torsional-stiffness target
//                      (optimality-criteria sizing with analytic sensitivities)
//  * sweepParameter  - design-space exploration of one geometric parameter
import { buildChassis, GROUPS } from '../chassis/mesh.js';
import { prepare, solveStatic, postprocess, massModel } from '../fea/model.js';
import { torsionBC } from '../fea/analysis.js';

export const STANDARD_GAUGES = [0.7, 0.8, 1.0, 1.2, 1.5, 1.6, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0, 8.0];
const RAD2DEG = 180 / Math.PI;

async function torsionOnly(mesh, cfg, iterative) {
  const prep = prepare(mesh.model);
  const bc = torsionBC(mesh, cfg.analysis?.torque ?? 1000);
  const r = await solveStatic(prep, bc, { iterative });
  const u = r.u[0];
  const { L, R, yL, yR, torqueNm } = bc.meta;
  const wL = L.reduce((s, h) => s + u[6 * h.node + 2], 0) / L.length;
  const wR = R.reduce((s, h) => s + u[6 * h.node + 2], 0) / R.length;
  const theta = (wL - wR) / (yL - yR);
  const C = bc.loads[0].reduce((s, f, i) => s + f * u[i], 0); // compliance f.u (N mm)
  return { K: torqueNm / (theta * RAD2DEG), theta, u, C, torqueNm };
}

function groupData(model, post) {
  const ng = GROUPS.length;
  const Um = new Float64Array(ng), Ub = new Float64Array(ng), area = new Float64Array(ng), t = new Float64Array(ng);
  const cnt = new Float64Array(ng);
  const mm = massModel(model);
  for (let e = 0; e < model.shellT.length; e++) {
    const g = model.shellGroup[e];
    Um[g] += post.shellUm[e]; Ub[g] += post.shellUb[e];
    t[g] += model.shellT[e]; cnt[g]++;
  }
  for (let g = 0; g < ng; g++) {
    t[g] = cnt[g] ? t[g] / cnt[g] : 0;
    area[g] = cnt[g] ? (mm.groupMass[GROUPS[g]] || 0) / (model.mat.rho * t[g]) : 0;
  }
  return { Um, Ub, area, t, cnt, mass: mm.total };
}

/**
 * Minimum-mass gauges s.t. K_torsion >= target.
 * opts: { target (Nm/deg), free: [group names], tMin, tMax, iters, snap, onProgress, iterative }
 */
export async function optimiseGauges(cfg, opts) {
  const { target, free = GROUPS, tMin = 0.8, tMax = 6, iters = 12, snap = true, onProgress, iterative, move = 0.3 } = opts;
  const mesh = buildChassis(cfg);
  const model = mesh.model;
  const rho = model.mat.rho;
  const gauges = { ...cfg.gauges };
  const history = [];
  const apply = () => { for (let e = 0; e < model.shellT.length; e++) model.shellT[e] = gauges[GROUPS[model.shellGroup[e]]]; };
  const active = GROUPS.map((g, i) => (free.includes(g) ? i : -1)).filter((i) => i >= 0);
  let last = null;
  for (let it = 0; it < iters; it++) {
    apply();
    const tr = await torsionOnly(mesh, cfg, iterative);
    const post = postprocess(model, tr.u);
    const gd = groupData(model, post);
    last = { tr, gd };
    history.push({ it, K: tr.K, mass: gd.mass, gauges: { ...gauges } });
    onProgress && onProgress({ it, iters, K: tr.K, mass: gd.mass, gauges: { ...gauges } });
    // sensitivities: C = f.u ; dC/dt_g = -(2 Um + 6 Ub)/t  (membrane ~ t, bending ~ t^3)
    const dC = GROUPS.map((_, g) => (gd.cnt[g] && gd.t[g] ? -(2 * gd.Um[g] + 6 * gd.Ub[g]) / gd.t[g] : 0));
    const dM = GROUPS.map((_, g) => rho * gd.area[g]);
    const Ctarget = tr.C * (tr.K / target); // compliance scales as 1/K for fixed load
    const conv = Math.abs(tr.K - target) / target < 0.005 && it > 2 && Math.abs(history[it - 1].mass - gd.mass) / gd.mass < 0.002;
    if (conv) break;
    const trial = (lam) => {
      const tn = {};
      let Cp = tr.C;
      for (const g of active) {
        const name = GROUPS[g];
        const t0 = gauges[name];
        if (!gd.cnt[g] || dM[g] <= 0) { tn[name] = t0; continue; }
        const B = Math.max(1e-12, (lam * -dC[g]) / dM[g]);
        let t1 = t0 * Math.sqrt(B);
        t1 = Math.min(t0 * (1 + move), Math.max(t0 * (1 - move), t1));
        t1 = Math.min(tMax, Math.max(tMin, t1));
        tn[name] = t1;
        Cp += dC[g] * (t1 - t0);
      }
      return { tn, Cp };
    };
    let lo = 1e-12, hi = 1e12;
    for (let k = 0; k < 80; k++) {
      const mid = Math.sqrt(lo * hi);
      if (trial(mid).Cp > Ctarget) lo = mid; else hi = mid;
    }
    Object.assign(gauges, trial(hi).tn);
  }
  let snapped = null;
  if (snap) {
    const up = (t) => STANDARD_GAUGES.find((s) => s >= t - 1e-9) ?? STANDARD_GAUGES[STANDARD_GAUGES.length - 1];
    const down = (t) => [...STANDARD_GAUGES].reverse().find((s) => s <= t + 1e-9) ?? STANDARD_GAUGES[0];
    for (const g of active) {
      const name = GROUPS[g];
      const t = gauges[name];
      const u = up(t), d = down(t);
      gauges[name] = Math.abs(u - t) < Math.abs(t - d) ? u : d;
    }
    for (let k = 0; k < 10; k++) {
      apply();
      const tr = await torsionOnly(mesh, cfg, iterative);
      const post = postprocess(model, tr.u);
      const gd = groupData(model, post);
      snapped = { K: tr.K, mass: gd.mass };
      onProgress && onProgress({ it: 'snap', iters, K: tr.K, mass: gd.mass, gauges: { ...gauges } });
      if (tr.K >= target) break;
      // bump the group with the best stiffness-per-kg to the next standard gauge
      let best = -1, bestScore = -Infinity;
      for (const g of active) {
        const name = GROUPS[g];
        if (!gd.cnt[g]) continue;
        const next = STANDARD_GAUGES.find((s) => s > gauges[name] + 1e-9);
        if (!next || next > tMax + 1e-9) continue;
        const dCg = -(2 * gd.Um[g] + 6 * gd.Ub[g]) / gauges[name];
        const score = (-dCg * (next - gauges[name])) / (rho * gd.area[g] * (next - gauges[name]));
        if (score > bestScore) { bestScore = score; best = g; }
      }
      if (best < 0) break;
      gauges[GROUPS[best]] = STANDARD_GAUGES.find((s) => s > gauges[GROUPS[best]] + 1e-9);
    }
  }
  apply();
  const final = snapped || { K: last.tr.K, mass: last.gd.mass };
  return { gauges, history, final, initial: history[0] };
}

/** Sweep one chassis parameter (path like 'chassis.sillH') and report stiffness & mass. */
export async function sweepParameter(cfg, path, values, { onProgress, iterative } = {}) {
  const out = [];
  for (let i = 0; i < values.length; i++) {
    const c = JSON.parse(JSON.stringify(cfg));
    const keys = path.split('.');
    let o = c;
    for (const k of keys.slice(0, -1)) o = o[k];
    o[keys[keys.length - 1]] = values[i];
    try {
      const mesh = buildChassis(c);
      const tr = await torsionOnly(mesh, c, iterative);
      const mass = massModel(mesh.model).total;
      out.push({ value: values[i], K: tr.K, mass, spec: tr.K / mass });
    } catch (e) {
      out.push({ value: values[i], error: e.message });
    }
    onProgress && onProgress({ i, n: values.length, last: out[out.length - 1] });
  }
  return out;
}
