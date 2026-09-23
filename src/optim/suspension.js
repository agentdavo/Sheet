// Suspension geometry optimisers built on the kinematic solver.
import { makeCorner, analyseAxle } from '../suspension/kinematics.js';
import { nelderMead } from './nelderMead.js';

/** Metrics that can be targeted. scale = the deviation that costs 1 unit. */
export const SUSP_METRICS = {
  bumpSteer: { label: 'Bump steer', unit: '°/m', scale: 0.5 },
  toeRange: { label: 'Toe variation over travel', unit: '°', scale: 0.05 },
  rcHeight: { label: 'Roll-centre height', unit: 'mm', scale: 5 },
  rcMigration: { label: 'RC lateral migration', unit: 'mm/°', scale: 5 },
  camberComp: { label: 'Roll camber compensation', unit: '%', scale: 5 },
  camberGain: { label: 'Camber gain', unit: '°/10mm', scale: 0.02 },
  antiDive: { label: 'Anti-dive', unit: '%', scale: 3 },
  antiSquat: { label: 'Anti-squat', unit: '%', scale: 3 },
  antiLift: { label: 'Anti-lift', unit: '%', scale: 3 },
  ackermann: { label: 'Ackermann', unit: '%', scale: 3 },
  mr: { label: 'Motion ratio (damper/wheel)', unit: '', scale: 0.02 },
  mrProg: { label: 'MR progression', unit: 'ΔMR/100mm', scale: 0.04 },
  trackChange: { label: 'Track change', unit: 'mm/mm', scale: 0.02 },
};

/** Canned studies: which hardpoint coordinates move and which targets apply. */
export const STUDIES = {
  bumpsteer: {
    label: 'Bump steer (tie-rod height & length)',
    vars: [['tieI', 1, 40], ['tieI', 2, 50], ['tieO', 2, 30]],
    targets: { bumpSteer: 0, toeRange: 0 },
  },
  rollcentre: {
    label: 'Roll centre & camber (inner pick-up heights)',
    vars: [['ucaF', 2, 60], ['ucaR', 2, 60], ['lcaF', 2, 40], ['lcaR', 2, 40], ['ucaF', 1, 50], ['ucaR', 1, 50]],
    targets: { rcHeight: 40, rcMigration: 0, camberComp: 55, bumpSteer: 0 },
    alsoBumpSteer: true,
  },
  anti: {
    label: 'Anti-dive / anti-squat (side-view pick-up inclination)',
    vars: [['ucaF', 2, 50], ['ucaR', 2, 50], ['lcaF', 2, 35], ['lcaR', 2, 35]],
    targets: { antiDive: 25, antiSquat: 30, rcHeight: null },
    alsoBumpSteer: true,
  },
  ackermann: {
    label: 'Ackermann (steering arm & rack position)',
    vars: [['tieO', 0, 40], ['tieO', 1, 25], ['tieI', 0, 50]],
    targets: { ackermann: 60, bumpSteer: 0 },
    frontOnly: true,
  },
  motion: {
    label: 'Motion ratio (damper / rocker mounts)',
    vars: [['damperC', 1, 60], ['damperC', 2, 60], ['pushO', 1, 40]],
    targets: { mr: 0.9, mrProg: 0.03 },
  },
};

export function cloneDef(def) {
  return JSON.parse(JSON.stringify(def));
}

/** Weighted objective of an axle analysis. */
export function objective(res, targets, weights = {}) {
  if (!res.ok) return 1e9;
  let f = 0;
  for (const [k, t] of Object.entries(targets)) {
    if (t === null || t === undefined) continue;
    const v = res.metrics[k];
    if (v === null || v === undefined || !Number.isFinite(v)) continue;
    const s = SUSP_METRICS[k]?.scale || 1;
    const w = weights[k] ?? 1;
    const d = k === 'toeRange' ? v / s : (v - t) / s;
    f += w * d * d;
  }
  return f;
}

/**
 * Optimise a corner. vars: [[hpName, axis, range], ...]; targets {metric: value}.
 * Returns the best definition and the before/after metrics.
 */
export async function optimiseCorner(def, { vars, targets, weights, veh, rackMax, onIter, maxEval = 500, shouldStop }) {
  const base = cloneDef(def);
  const x0 = vars.map(([h, a]) => base.hp[h][a]);
  const lo = vars.map(([, , r], i) => x0[i] - r);
  const hi = vars.map(([, , r], i) => x0[i] + r);
  const aOpts = {
    travel: [-40, 40], steps: 9, rollMax: 3, rollSteps: 7,
    rackMax: targets.ackermann !== undefined && targets.ackermann !== null ? rackMax : 0,
    steerSteps: 9, veh,
  };
  const evalDef = (x) => {
    const d = cloneDef(base);
    vars.forEach(([h, a], i) => { d.hp[h][a] = x[i]; });
    return d;
  };
  const f = (x) => {
    const d = evalDef(x);
    const res = analyseAxle(makeCorner(d), aOpts);
    return objective(res, targets, weights);
  };
  const before = analyseAxle(makeCorner(base), { ...aOpts, rackMax });
  const f0 = objective(before, targets, weights);
  let best = await nelderMead(f, x0, lo, hi, { maxEval: Math.round(maxEval * 0.6), onIter, shouldStop });
  // restart from best with a smaller simplex to escape early collapse
  const second = await nelderMead(f, best.x, lo, hi, { maxEval: Math.round(maxEval * 0.4), step: 0.05, onIter, shouldStop });
  if (second.f < best.f) best = { ...second, evals: best.evals + second.evals };
  const out = evalDef(best.x);
  const after = analyseAxle(makeCorner(out), { ...aOpts, rackMax });
  return {
    def: out, f0, f: best.f, evals: best.evals,
    before: before.metrics, after: after.ok ? after.metrics : null,
    changes: vars.map(([h, a], i) => ({ hp: h, axis: 'xyz'[a], from: x0[i], to: best.x[i] })),
  };
}
