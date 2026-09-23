// Vehicle-level characteristics: mass properties, ride & roll, LLTD, chassis compliance effects.
// Inputs in mm / kg / N/mm (UI units); internals in SI.

const G = 9.81;

/**
 * Mass properties. chassis: {mass, cg:[x,y,z]} from the FE model (structure only).
 * Returns totals, CG, weight distribution, inertias (kg m^2), sprung/unsprung split.
 */
export function massProperties(cfg, chassis, kin) {
  const L = cfg.vehicle.wheelbase;
  const items = cfg.masses.filter((m) => m.m > 0).map((m) => ({ ...m, sprung: true }));
  items.push({ name: 'Chassis structure (FE)', m: chassis.mass, x: chassis.cg[0], y: chassis.cg[1], z: chassis.cg[2], sprung: true });
  const wcF = kin?.front?.static?.wc || [0, 800, 330];
  const wcR = kin?.rear?.static?.wc || [0, 800, 340];
  for (const s of [1, -1]) {
    items.push({ name: `Unsprung F${s > 0 ? 'L' : 'R'}`, m: cfg.unsprung.front, x: wcF[0], y: s * wcF[1], z: wcF[2], sprung: false });
    items.push({ name: `Unsprung R${s > 0 ? 'L' : 'R'}`, m: cfg.unsprung.rear, x: -L + wcR[0], y: s * wcR[1], z: wcR[2], sprung: false });
  }
  const sum = (arr, f) => arr.reduce((s, it) => s + f(it), 0);
  const M = sum(items, (i) => i.m);
  const cg = ['x', 'y', 'z'].map((k) => sum(items, (i) => i.m * i[k]) / M);
  const sprung = items.filter((i) => i.sprung);
  const Ms = sum(sprung, (i) => i.m);
  const cgs = ['x', 'y', 'z'].map((k) => sum(sprung, (i) => i.m * i[k]) / Ms);
  const front = -cg[0] / L; // x from front axle (0) to rear (-L): front fraction = distance to rear / L
  const frontFrac = (L + cg[0]) / L;
  void front;
  const I = (a, b) => sum(items, (i) => (i.m * ((i[a] - cg['xyz'.indexOf(a)]) ** 2 + (i[b] - cg['xyz'.indexOf(b)]) ** 2)) / 1e6);
  return {
    items, M, cg, frontFrac, Ms, cgs,
    sprungFront: (L + cgs[0]) / L,
    Izz: I('x', 'y'), Iyy: I('x', 'z'), Ixx: I('y', 'z'),
    unsprungF: cfg.unsprung.front, unsprungR: cfg.unsprung.rear,
  };
}

function axleRoll(kWheel, kArb, kTyre, track) {
  // all N/mm per wheel; returns Nm/rad for the axle (suspension, tyre, combined)
  const t = track / 1000;
  const ks = (kWheel + kArb) * 1000 * (t * t) / 2;
  const kt = kTyre * 1000 * (t * t) / 2;
  return { susp: ks, tyre: kt, total: 1 / (1 / ks + 1 / kt) };
}

/**
 * Ride & roll analysis of the current set-up plus a recommendation that meets the
 * targets (ride frequencies, roll gradient, LLTD).
 */
export function rideRoll(cfg, mp, kin, chassisK = null) {
  const L = cfg.vehicle.wheelbase / 1000;
  const r = cfg.ride;
  const sF = cfg.suspension.front, sR = cfg.suspension.rear;
  const mrF = Math.abs(kin.front.metrics.mr) || 1, mrR = Math.abs(kin.rear.metrics.mr) || 1;
  const tF = kin.front.trackStatic, tR = kin.rear.trackStatic;
  const msF = (mp.Ms * mp.sprungFront) / 2, msR = (mp.Ms * (1 - mp.sprungFront)) / 2; // per corner
  const a = (-mp.cgs[0]) / 1000; // front axle -> sprung CG (m)
  const b = L - a;
  const hcg = mp.cgs[2] / 1000;
  const zrcF = kin.front.metrics.rcHeight / 1000, zrcR = kin.rear.metrics.rcHeight / 1000;
  const zrc = zrcF + ((zrcR - zrcF) * a) / L;
  const hArm = hcg - zrc;
  const ktF = r.tyreRateF, ktR = r.tyreRateR;

  const corner = (ks, mr, kt, ms) => {
    const kw = ks * mr * mr; // N/mm
    const kr = (kw * kt) / (kw + kt);
    const f = Math.sqrt((kr * 1000) / ms) / (2 * Math.PI);
    return { kw, kr, f };
  };
  const cF = corner(sF.springRate, mrF, ktF, msF);
  const cR = corner(sR.springRate, mrR, ktR, msR);
  const rollF = axleRoll(cF.kw, sF.arbRate, ktF, tF);
  const rollR = axleRoll(cR.kw, sR.arbRate, ktR, tR);
  const usF = mp.unsprungF * 2, usR = mp.unsprungR * 2;
  const rF = sF.tyreR / 1000, rR = sR.tyreR / 1000;
  const lltd = (kf, kr) => {
    const x = kf / (kf + kr);
    const Af = x * mp.Ms * hArm + mp.Ms * (b / L) * zrcF + usF * rF;
    const Ar = (1 - x) * mp.Ms * hArm + mp.Ms * (a / L) * zrcR + usR * rR;
    return (Af / (tF / 1000)) / (Af / (tF / 1000) + Ar / (tR / 1000));
  };
  const Kphi = rollF.total + rollR.total;
  const Mroll = mp.Ms * G * hArm; // Nm per g
  const rollGrad = (Mroll / Math.max(1, Kphi - Mroll)) * (180 / Math.PI);
  const current = {
    fF: cF.f, fR: cR.f, wheelRateF: cF.kw, wheelRateR: cR.kw, rollGrad,
    lltd: lltd(rollF.total, rollR.total) * 100, KphiF: rollF.total, KphiR: rollR.total, hArm: hArm * 1000,
    zrc: zrc * 1000, mrF, mrR,
  };

  // ---------------- recommendation
  const need = (f, ms, kt, mr) => {
    const kr = ((2 * Math.PI * f) ** 2 * ms) / 1000; // N/mm ride rate
    const kw = kt > kr ? (kr * kt) / (kt - kr) : NaN;
    return { kr, kw, spring: kw / (mr * mr) };
  };
  const nF = need(r.fF, msF, ktF, mrF), nR = need(r.fR, msR, ktR, mrR);
  const KphiReq = (Mroll * (180 / Math.PI)) / r.rollGrad + Mroll; // Nm/rad
  // find front share x of total roll stiffness giving target LLTD
  let lo = 0, hi = 1;
  for (let i = 0; i < 60; i++) {
    const m = (lo + hi) / 2;
    if (lltd(m * KphiReq, (1 - m) * KphiReq) * 100 < r.lltd) lo = m; else hi = m;
  }
  const x = (lo + hi) / 2;
  const t2 = (t) => ((t / 1000) ** 2) / 2 * 1000; // Nm/rad per N/mm of wheel rate
  const suspFrom = (Kaxle, kt, t) => { const Kt = kt * t2(t); return Kaxle < Kt ? 1 / (1 / Kaxle - 1 / Kt) : Infinity; };
  const KsF = suspFrom(x * KphiReq, ktF, tF), KsR = suspFrom((1 - x) * KphiReq, ktR, tR);
  const arbF = KsF / t2(tF) - nF.kw, arbR = KsR / t2(tR) - nR.kw;
  const damper = (ms, kw, kt, mr) => {
    const kr = (kw * kt) / (kw + kt);
    const cc = 2 * Math.sqrt(kr * 1000 * ms); // N s/m at wheel
    return (r.dampRatio * cc) / (mr * mr) / 1000; // N s/mm at damper
  };
  const rec = {
    springF: nF.spring, springR: nR.spring, arbF, arbR, wheelRateF: nF.kw, wheelRateR: nR.kw,
    KphiReq, frontShare: x, damperF: damper(msF, nF.kw, ktF, mrF), damperR: damper(msR, nR.kw, ktR, mrR),
    feasible: arbF >= -1e-6 && arbR >= -1e-6 && Number.isFinite(nF.kw) && Number.isFinite(nR.kw),
  };

  // ---------------- chassis compliance (twin-spring model)
  const lltdWithChassis = (Kc) => {
    const Kf = rollF.total, Kr = rollR.total;
    const am = b / L; // share of the sprung roll moment reacted at the front station
    // [Kf+Kc, -Kc; -Kc, Kr+Kc] [p1; p2] = [am; 1-am] (per unit roll moment)
    const det = (Kf + Kc) * (Kr + Kc) - Kc * Kc;
    const p1 = (am * (Kr + Kc) + (1 - am) * Kc) / det;
    const p2 = ((1 - am) * (Kf + Kc) + am * Kc) / det;
    const fShare = (Kf * p1) / (Kf * p1 + Kr * p2);
    return lltd(fShare, 1 - fShare) * 100;
  };
  let chassis = null;
  if (chassisK) {
    const Kc = chassisK * (180 / Math.PI); // Nm/deg -> Nm/rad
    const ratio = Kc / (rollF.total + rollR.total);
    const curve = [];
    for (let k = 0; k <= 40; k++) {
      const f = 0.05 * Math.pow(10, (k / 40) * 2.5); // 0.05 .. 15.8 x (Kf+Kr)
      const kc = f * (rollF.total + rollR.total);
      curve.push({ ratio: f, Kc: kc / (180 / Math.PI), lltd: lltdWithChassis(kc) });
    }
    chassis = {
      ratio, lltdRigid: current.lltd, lltdFlex: lltdWithChassis(Kc), curve,
      // stiffness needed so chassis flex costs < 1% LLTD control authority
      advice: ratio >= 5 ? 'Chassis is stiff relative to suspension roll stiffness (>= 5x) - LLTD responds to set-up changes.'
        : ratio >= 3 ? 'Acceptable (3-5x roll stiffness). Some loss of LLTD sensitivity.' : 'Chassis too flexible relative to roll stiffness (< 3x) - ARB changes will be partly absorbed by chassis twist.',
    };
  }
  return { current, rec, chassis, msF, msR, a, b, hcg, Mroll };
}

/** Move one component along x to hit a target front weight fraction (clamped to limits). */
export function balanceWeight(cfg, mp, compIndex, targetFront, xLimits) {
  const L = cfg.vehicle.wheelbase;
  const c = cfg.masses[compIndex];
  if (!c || c.m <= 0) return null;
  const xcgTarget = targetFront * L - L; // frontFrac = (L + x)/L
  const dx = ((xcgTarget - mp.cg[0]) * mp.M) / c.m;
  const x = Math.min(xLimits[1], Math.max(xLimits[0], c.x + dx));
  const achieved = (L + mp.cg[0] + ((x - c.x) * c.m) / mp.M) / L;
  return { x, achieved, clamped: x !== c.x + dx };
}
