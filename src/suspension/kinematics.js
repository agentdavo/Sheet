// Double-wishbone corner kinematics.
//
// Coordinates: x forward, y left, z up (mm). Hardpoints of a corner are stored for the
// LEFT side in axle-local x (x = 0 at the axle centreline) with y, z absolute. The right
// side is obtained by symmetry: right(s, rack) = mirrorY(left(s, -rack)).
//
// The corner is solved as a rigid upright constrained by:
//   lower ball joint on a circle about the lower inner axis (driver: lower arm angle)
//   upper ball joint on a circle about the upper inner axis at fixed kingpin length
//   tie-rod outer at fixed length from the (rack-displaced) tie-rod inner point.
import {
  add, sub, scale, dot, cross, len, dist, norm, deg, rad, rotAbout, rotVec, alignRot, matVec, solve1D, intersect2D,
} from './vec.js';

export const HARDPOINTS = [
  ['lcaF', 'Lower wishbone - front inner'],
  ['lcaR', 'Lower wishbone - rear inner'],
  ['lcaO', 'Lower ball joint'],
  ['ucaF', 'Upper wishbone - front inner'],
  ['ucaR', 'Upper wishbone - rear inner'],
  ['ucaO', 'Upper ball joint'],
  ['tieI', 'Tie rod / toe link - inner'],
  ['tieO', 'Tie rod / toe link - outer'],
  ['wc', 'Wheel centre'],
  ['pushO', 'Pushrod / damper - outer'],
  ['rockP', 'Rocker pivot'],
  ['rockA', 'Rocker axis point'],
  ['rockPush', 'Rocker - pushrod pick-up'],
  ['rockDamp', 'Rocker - damper pick-up'],
  ['damperC', 'Damper - chassis'],
];

/** Chassis-side hardpoints (the ones that load the structure). */
export function chassisPoints(def) {
  const pts = ['lcaF', 'lcaR', 'ucaF', 'ucaR', 'tieI', 'damperC'];
  if (def.actuation !== 'direct') pts.push('rockP');
  return pts;
}

const reset = (c) => { c.last = { tl: 0, tu: 0, psi: 0, phi: 0 }; c.lastS = 0; };

/** Pre-compute rigid-body reference data of a corner definition. */
export function makeCorner(def) {
  const hp = def.hp;
  const k0 = sub(hp.ucaO, hp.lcaO);
  // wheel spin axis from static camber / toe (left wheel, axis pointing outboard)
  const cam = rad(def.camber), toe = rad(def.toe);
  let axis = [0, 1, 0];
  axis = rotVec(axis, [1, 0, 0], -cam); // negative camber -> axis z > 0
  axis = rotVec(axis, [0, 0, 1], -toe); // toe-in on the left wheel = steer right
  const c = {
    def,
    hp,
    Lk: len(k0),
    Lt: dist(hp.tieO, hp.tieI),
    Lp: def.actuation === 'direct' ? 0 : dist(hp.pushO, hp.rockPush),
    axisL: norm(sub(hp.lcaR, hp.lcaF)),
    axisU: norm(sub(hp.ucaR, hp.ucaF)),
    axisRock: norm(sub(hp.rockA, hp.rockP)),
    spin0: axis,
  };
  reset(c);
  return c;
}

function uprightPose(c, tl, rack) {
  const hp = c.hp;
  const lbj = rotAbout(hp.lcaO, hp.lcaF, c.axisL, tl);
  const fu = (tu) => dist(rotAbout(hp.ucaO, hp.ucaF, c.axisU, tu), lbj) - c.Lk;
  const tu = solve1D(fu, c.last.tu, c.last.tu + 1e-3);
  if (!Number.isFinite(tu) || Math.abs(tu) > 1.5) return null;
  const ubj = rotAbout(hp.ucaO, hp.ucaF, c.axisU, tu);
  const k0 = norm(sub(hp.ucaO, hp.lcaO));
  const k = norm(sub(ubj, lbj));
  const Ra = alignRot(k0, k);
  const tieI = add(hp.tieI, [0, rack, 0]);
  const tr = (p, psi) => add(lbj, rotVec(matVec(Ra, sub(p, hp.lcaO)), k, psi));
  const ft = (psi) => dist(tr(hp.tieO, psi), tieI) - c.Lt;
  const psi = solve1D(ft, c.last.psi, c.last.psi + 1e-3);
  if (!Number.isFinite(psi) || Math.abs(psi) > 1.2) return null;
  const trp = (p) => tr(p, psi);
  const trv = (v) => rotVec(matVec(Ra, v), k, psi);
  return { tl, tu, psi, lbj, ubj, tieI, tieO: trp(hp.tieO), wc: trp(hp.wc), spin: trv(c.spin0), tr: trp };
}

/**
 * Solve the left corner for wheel-centre vertical travel s (mm, + = bump) and rack travel (mm, + = left).
 */
export function solveCorner(c, s = 0, rack = 0) {
  const hp = c.hp;
  const target = hp.wc[2] + s;
  let pose = null;
  const f = (tl) => {
    const p = uprightPose(c, tl, rack);
    if (!p) return NaN;
    pose = p;
    c.last.tu = p.tu; c.last.psi = p.psi;
    return p.wc[2] - target;
  };
  const lever = Math.max(50, Math.abs(hp.lcaO[1] - hp.lcaF[1]));
  const g0 = c.last.tl;
  let tl = solve1D(f, g0, g0 + (s - c.lastS) / lever + 1e-4, 1e-9);
  if (!Number.isFinite(tl)) {
    reset(c);
    tl = solve1D(f, 0, s / lever + 1e-4, 1e-9);
    if (!Number.isFinite(tl)) return null;
  }
  const p = uprightPose(c, tl, rack);
  if (!p) return null;
  c.last.tl = p.tl; c.last.tu = p.tu; c.last.psi = p.psi;
  c.lastS = s;
  void pose;
  return decorate(c, p);
}

function decorate(c, p) {
  const def = c.def;
  const hp = c.hp;
  const a = norm(p.spin);
  // contact patch: lowest point of the tyre circle
  const down = norm(sub([0, 0, 1], scale(a, a[2])));
  const cp = sub(p.wc, scale(down, def.tyreR));
  const camber = -deg(Math.asin(Math.max(-1, Math.min(1, a[2]))));
  const steer = deg(Math.atan2(-a[0], a[1])); // + = left
  const toe = -steer; // toe-in positive (left wheel)
  const k = sub(p.ubj, p.lbj);
  const kpi = deg(Math.atan2(-k[1], k[2]));
  const caster = deg(Math.atan2(-k[0], k[2]));
  const tg = (cp[2] - p.lbj[2]) / k[2];
  const gnd = add(p.lbj, scale(k, tg));
  const scrub = cp[1] - gnd[1];
  const trail = gnd[0] - cp[0];
  const lcaT = (q) => rotAbout(q, hp.lcaF, c.axisL, p.tl);
  const ucaT = (q) => rotAbout(q, hp.ucaF, c.axisU, p.tu);
  const mount = def.pushOn === 'uca' ? ucaT : def.pushOn === 'upright' ? p.tr : lcaT;
  const pushO = mount(hp.pushO);
  let damperLen, rock = null;
  if (def.actuation === 'direct') {
    damperLen = dist(pushO, hp.damperC);
  } else {
    const fr = (phi) => dist(rotAbout(hp.rockPush, hp.rockP, c.axisRock, phi), pushO) - c.Lp;
    let phi = solve1D(fr, c.last.phi || 0, (c.last.phi || 0) + 1e-3);
    if (!Number.isFinite(phi)) phi = 0;
    c.last.phi = phi;
    const rp = rotAbout(hp.rockPush, hp.rockP, c.axisRock, phi);
    const rd = rotAbout(hp.rockDamp, hp.rockP, c.axisRock, phi);
    damperLen = dist(rd, hp.damperC);
    rock = { phi, push: rp, damp: rd };
  }
  return {
    ...p,
    cp, camber, toe, steer, kpi, caster, scrub, trail, gnd, pushO, damperLen, rock,
    ucaInner: [hp.ucaF, hp.ucaR], lcaInner: [hp.lcaF, hp.lcaR],
  };
}

// ---------------------------------------------------------------- geometry helpers

/** Intersect the plane through 3 points with the plane coord[axis] = v; returns a 2-D line in the other two coords. */
function slicePlane(p0, p1, p2, axis, v) {
  const n = cross(sub(p1, p0), sub(p2, p0));
  const c = dot(n, p0);
  const [i, j] = axis === 0 ? [1, 2] : [0, 2];
  const rhs = c - n[axis] * v;
  const dir = [n[j], -n[i]];
  const pt = Math.abs(n[j]) > Math.abs(n[i]) ? [0, rhs / n[j]] : [rhs / n[i], 0];
  return { pt, dir };
}

/** Front-view instant centre of a left-corner solution: {pt:[y,z]} or {dir} when the arms are parallel. */
export function frontViewIC(sol) {
  const x = sol.cp[0];
  const U = slicePlane(sol.ucaInner[0], sol.ucaInner[1], sol.ubj, 0, x);
  const L = slicePlane(sol.lcaInner[0], sol.lcaInner[1], sol.lbj, 0, x);
  const p = intersect2D(U.pt, U.dir, L.pt, L.dir);
  if (!p || Math.abs(p[0]) > 1e6) return { pt: null, dir: L.dir };
  return { pt: p, dir: null };
}

/** Side-view instant centre (x, z) in the plane y = contact patch y. */
export function sideViewIC(sol) {
  const y = sol.cp[1];
  const U = slicePlane(sol.ucaInner[0], sol.ucaInner[1], sol.ubj, 1, y);
  const L = slicePlane(sol.lcaInner[0], sol.lcaInner[1], sol.lbj, 1, y);
  const p = intersect2D(U.pt, U.dir, L.pt, L.dir);
  return p && Math.abs(p[0]) < 1e6 ? p : null;
}

/** Roll centre from a left solution and a (mirrored-into-left) right solution. */
export function rollCentre(left, rightM) {
  const icL = frontViewIC(left);
  const icRm = frontViewIC(rightM);
  const cpL = [left.cp[1], left.cp[2]];
  const cpR = [-rightM.cp[1], rightM.cp[2]];
  const dirL = icL.pt ? [icL.pt[0] - cpL[0], icL.pt[1] - cpL[1]] : icL.dir;
  const icR = icRm.pt ? [-icRm.pt[0], icRm.pt[1]] : null;
  const dirR = icR ? [icR[0] - cpR[0], icR[1] - cpR[1]] : [-icRm.dir[0], icRm.dir[1]];
  const rc = intersect2D(cpL, dirL, cpR, dirR);
  const ground = (cpL[1] + cpR[1]) / 2;
  if (!rc) return { y: 0, z: ground, h: 0, icL: icL.pt, icR };
  return { y: rc[0], z: rc[1], h: rc[1] - ground, icL: icL.pt, icR };
}

/**
 * Full kinematic sweep of one axle.
 * opts: { travel: [min,max], steps, rollMax (deg), rackMax (mm), veh: {wheelbase, cgH, brakeFront, driveFront, isFront} }
 */
export function analyseAxle(corner, opts) {
  const { travel = [-50, 50], steps = 21, rollMax = 3, rackMax = 0, veh } = opts;
  const c = corner;
  reset(c);
  const st = solveCorner(c, 0, 0);
  if (!st) return { ok: false, error: 'Static geometry does not assemble - check link lengths.' };
  const trackStatic = 2 * st.cp[1];
  const heave = [];
  for (let i = 0; i < steps; i++) heave.push(travel[0] + ((travel[1] - travel[0]) * i) / (steps - 1));
  const sols = new Array(steps);
  const i0 = heave.reduce((b, s, i) => (Math.abs(s) < Math.abs(heave[b]) ? i : b), 0);
  reset(c);
  for (let i = i0; i < steps; i++) sols[i] = solveCorner(c, heave[i], 0);
  reset(c);
  for (let i = i0 - 1; i >= 0; i--) sols[i] = solveCorner(c, heave[i], 0);
  if (!sols.every(Boolean)) return { ok: false, error: 'Geometry locks up within the travel range.' };

  const curves = {
    travel: heave,
    camber: sols.map((s) => s.camber),
    toe: sols.map((s) => s.toe),
    track: sols.map((s) => 2 * s.cp[1] - trackStatic),
    wbase: sols.map((s) => s.cp[0] - st.cp[0]),
    caster: sols.map((s) => s.caster),
    kpi: sols.map((s) => s.kpi),
    damper: sols.map((s) => s.damperLen),
    rcHeave: sols.map((s) => rollCentre(s, s).h),
  };
  // motion ratio = d(damper compression)/d(wheel travel)
  curves.mr = heave.map((_, i) => {
    const a = Math.max(0, i - 1), b = Math.min(steps - 1, i + 1);
    return -(curves.damper[b] - curves.damper[a]) / (heave[b] - heave[a]);
  });
  const slope = (ys) => {
    const a = Math.max(0, i0 - 1), b = Math.min(steps - 1, i0 + 1);
    return (ys[b] - ys[a]) / (heave[b] - heave[a]);
  };
  // roll sweep: body rolls phi (+ = left side down): left wheel bumps, right droops
  const halfT = trackStatic / 2;
  const roll = [], rcY = [], rcH = [], camberL = [], camberR = [];
  const nR = opts.rollSteps || 13;
  for (let i = 0; i < nR; i++) {
    const phi = -rollMax + (2 * rollMax * i) / (nR - 1);
    const s = halfT * Math.tan(rad(phi));
    reset(c);
    const L = solveCorner(c, s, 0);
    reset(c);
    const R = solveCorner(c, -s, 0);
    if (!L || !R) continue;
    const rc = rollCentre(L, R);
    roll.push(phi); rcY.push(rc.y); rcH.push(rc.h);
    camberL.push(L.camber + phi); // camber relative to the ground
    camberR.push(R.camber - phi);
  }
  // steering sweep: rack direction that turns the car left; left wheel = inner wheel
  let steerCurve = null;
  if (rackMax > 0) {
    reset(c);
    const probe = solveCorner(c, 0, rackMax * 0.1);
    const sgn = probe && probe.steer < st.steer ? -1 : 1;
    const rk = [], inner = [], outer = [], ideal = [];
    const n = opts.steerSteps || 21;
    const cot = (a) => 1 / Math.tan(rad(a));
    for (let i = 0; i < n; i++) {
      const r = sgn * (rackMax * i) / (n - 1);
      reset(c);
      const L = solveCorner(c, 0, r);
      reset(c);
      const Rm = solveCorner(c, 0, -r);
      if (!L || !Rm) break;
      const di = L.steer - st.steer; // inner (left) wheel
      const dout = -(Rm.steer - st.steer); // outer (right) wheel, mirrored
      rk.push(Math.abs(r)); inner.push(di); outer.push(dout);
      ideal.push(di > 0.05 ? deg(Math.atan(1 / (cot(di) + (2 * halfT) / veh.wheelbase))) : di);
    }
    const last = rk.length - 1;
    const ack = last > 0 && inner[last] - ideal[last] > 1e-6 ? ((inner[last] - outer[last]) / (inner[last] - ideal[last])) * 100 : 0;
    steerCurve = { rack: rk, inner, outer, ideal, ackermann: ack, sign: sgn };
  }
  reset(c);
  // side view: anti features
  const svic = sideViewIC(st);
  const h = veh.cgH, L = veh.wheelbase;
  const anti = { antiDive: null, antiLift: null, antiSquat: null, svic, svaDeg: null };
  if (svic) {
    const dxCP = veh.isFront ? st.cp[0] - svic[0] : svic[0] - st.cp[0];
    const dxWC = veh.isFront ? st.wc[0] - svic[0] : svic[0] - st.wc[0];
    const tanCP = (svic[1] - st.cp[2]) / dxCP;
    const tanWC = (svic[1] - st.wc[2]) / dxWC;
    if (veh.isFront) {
      anti.antiDive = tanCP * (L / h) * veh.brakeFront * 100;
      anti.antiLift = veh.driveFront > 0 ? tanWC * (L / h) * 100 : null;
    } else {
      anti.antiLift = tanCP * (L / h) * (1 - veh.brakeFront) * 100;
      anti.antiSquat = veh.driveFront < 1 ? tanWC * (L / h) * 100 : null;
    }
    anti.svaDeg = deg(Math.atan(tanCP));
  }
  const rcStatic = rollCentre(st, st);
  const iP = roll.findIndex((r) => r > 0.99);
  const iZ = roll.findIndex((r) => Math.abs(r) < 1e-9);
  const migPerDeg = iP >= 0 && iZ >= 0 ? (rcY[iP] - rcY[iZ]) / roll[iP] : 0;
  // roll camber compensation: 100% = wheel stays upright relative to the ground in roll
  const dGround = iP >= 0 && iZ >= 0 ? (camberL[iP] - camberL[iZ]) / roll[iP] : 1;
  return {
    ok: true,
    static: st,
    trackStatic,
    curves,
    rollCurves: { roll, rcY, rcH, camberL, camberR },
    steer: steerCurve,
    metrics: {
      camber: st.camber,
      toe: st.toe,
      kpi: st.kpi,
      caster: st.caster,
      scrub: st.scrub,
      trail: st.trail,
      rcHeight: rcStatic.h,
      rcMigration: migPerDeg,
      camberGain: slope(curves.camber) * 10, // deg per 10 mm bump
      camberComp: (1 - dGround) * 100,
      bumpSteer: slope(curves.toe) * 1000, // deg / m
      toeRange: Math.max(...curves.toe) - Math.min(...curves.toe),
      trackChange: slope(curves.track),
      mr: curves.mr[i0],
      mrProg: ((curves.mr[steps - 1] - curves.mr[0]) / (heave[steps - 1] - heave[0])) * 100,
      ackermann: steerCurve ? steerCurve.ackermann : null,
      ...anti,
      icFront: frontViewIC(st).pt,
    },
  };
}
