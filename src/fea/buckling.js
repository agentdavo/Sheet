// Panel buckling check for thin sheet (shear + compression interaction).
//
// Each bay (sheet area between lines where other panels are welded on) is treated as a
// flat rectangular plate. Bay-average membrane stress resultants from the linear FE solve
// are compared with classical elastic buckling loads:
//   shear       N_xy,cr = k_s * pi^2 * (D_a * D_b^3)^(1/4) / b^2   (orthotropic, Seydel form)
//               k_s = 5.35 + 4/alpha^2 (simply supported) or 8.98 + 5.6/alpha^2 (clamped),
//               alpha = (a/b) * (D_b/D_a)^(1/4) >= 1
//   compression N_cr = k_c * pi^2 * sqrt(D_a * D_b) / w^2,  k_c = min_m (m/phi + phi/m)^2
// capped at first yield (plastic buckling not modelled), combined with the standard
// interaction R_c + R_s^2 = 1. Beads (if any) run across the short span, so they stiffen
// D_b = bending across the short span.
import { MATERIALS } from './materials.js';
import { beadSection, beadsFor } from '../chassis/stiffening.js';

const PI2 = Math.PI * Math.PI;

function kCompression(phi) {
  let best = Infinity;
  for (let m = 1; m <= 12; m++) best = Math.min(best, (m / phi + phi / m) ** 2);
  return best;
}

/**
 * @param mesh      chassis mesh (with bays)
 * @param cfg       design config
 * @param post      post-processed results of a load case (shellNx/Ny/Nxy)
 * @param appliedNm torque that produced `post`
 * @param designNm  design twist torque to check against
 */
export function bucklingCheck(mesh, cfg, post, appliedNm, designNm, { edges = 'ss' } = {}) {
  const m = mesh.model;
  const mat = MATERIALS[cfg.material];
  const { E, nu } = mat;
  const tauY = mat.yield / Math.sqrt(3);
  const areas = shellAreas(m);
  const util = new Float32Array(m.shellT.length);
  const out = [];
  for (let k = 0; k < mesh.bays.length; k++) {
    const bay = mesh.bays[k];
    let A = 0, nx = 0, ny = 0, nxy = 0, tSum = 0, doubler = 0;
    for (const e of bay.elems) {
      const a = areas[e];
      A += a; nx += post.shellNx[e] * a; ny += post.shellNy[e] * a; nxy += post.shellNxy[e] * a;
      if (m.groups[m.shellGroup[e]] === 'doublers') doubler += a; else tSum += m.shellT[e] * a;
    }
    if (!A) continue;
    nx /= A; ny /= A; nxy /= A;
    const t = tSum > 0 ? tSum / (A - doubler) : m.shellT[bay.elems[0]];
    const Dflat = (E * t ** 3) / (12 * (1 - nu * nu));
    const bead = beadsFor(cfg, bay.group);
    const Dbead = bead ? Dflat * beadSection(t, bead).Dratio : Dflat;
    const longA = bay.LA >= bay.LB;
    const a = Math.max(bay.LA, bay.LB), b = Math.min(bay.LA, bay.LB);
    const Da = Dflat, Db = Dbead; // beads across the short span
    const alpha = Math.max(1, (a / b) * (Db / Da) ** 0.25);
    const ks = edges === 'clamped' ? 8.98 + 5.6 / alpha ** 2 : 5.35 + 4 / alpha ** 2;
    const NsEl = (ks * PI2 * (Da * Db ** 3) ** 0.25) / (b * b);
    const Nscr = Math.min(NsEl, tauY * t);
    const Dc = Math.sqrt(Da * Db) * (edges === 'clamped' ? 1.75 : 1);
    // compression along A (loaded width LB) and along B (loaded width LA)
    const NaEl = (kCompression(bay.LA / bay.LB) * PI2 * Dc) / bay.LB ** 2;
    const NbEl = (kCompression(bay.LB / bay.LA) * PI2 * Dc) / bay.LA ** 2;
    const NcrA = Math.min(NaEl, mat.yield * t);
    const NcrB = Math.min(NbEl, mat.yield * t);
    const Rs = Math.abs(nxy) / Nscr;
    const Rc = Math.max(0, -nx) / NcrA + Math.max(0, -ny) / NcrB;
    let lambda;
    if (Rs < 1e-12 && Rc < 1e-12) lambda = Infinity;
    else if (Rs < 1e-12) lambda = 1 / Rc;
    else lambda = (-Rc + Math.sqrt(Rc * Rc + 4 * Rs * Rs)) / (2 * Rs * Rs);
    const Tcr = lambda * appliedNm;
    const u = designNm / Tcr;
    const shearGoverns = Rs * Rs >= Rc;
    const capped = shearGoverns ? NsEl > Nscr : (-nx > 0 && NaEl > NcrA) || (-ny > 0 && NbEl > NcrB);
    for (const e of bay.elems) util[e] = u;
    out.push({
      bay: k, panel: bay.name, group: bay.group, a, b, t, beaded: !!bead, longA,
      tau: Math.abs(nxy) / t, tauCr: Nscr / t, sigma: Math.min(nx, ny) / t,
      lambda, Tcr, util: u, mode: capped ? 'local yield' : shearGoverns ? 'shear buckling' : 'compression buckling', yieldCapped: capped,
    });
  }
  out.sort((p, q) => p.Tcr - q.Tcr);
  const crit = out[0] || null;
  const critBuckle = out.find((b) => !b.yieldCapped) || null;
  const failing = out.filter((b) => b.util >= 1);
  return {
    bays: out, util, crit, critBuckle, TcrBuckle: critBuckle ? critBuckle.Tcr : Infinity, Tcr: crit ? crit.Tcr : Infinity, designNm, reserve: crit ? crit.Tcr / designNm : Infinity,
    failingCount: failing.length, failingArea: failing.reduce((s, b) => s + b.a * b.b, 0) / 1e6,
  };
}

function shellAreas(m) {
  const n = m.shellT.length;
  const a = new Float64Array(n);
  for (let e = 0; e < n; e++) {
    const p = [0, 1, 2, 3].map((k) => { const i = m.shells[4 * e + k]; return [m.nodes[3 * i], m.nodes[3 * i + 1], m.nodes[3 * i + 2]]; });
    const d1 = p[2].map((v, i) => v - p[0][i]), d2 = p[3].map((v, i) => v - p[1][i]);
    a[e] = 0.5 * Math.hypot(d1[1] * d2[2] - d1[2] * d2[1], d1[2] * d2[0] - d1[0] * d2[2], d1[0] * d2[1] - d1[1] * d2[0]);
  }
  return a;
}

/** Design twist torque: dynamic factor x front-axle static wheel load x front track (Nm). */
export function designTwistTorque(mp, trackF, factor) {
  const wheel = (mp.M * mp.frontFrac * 9.81) / 2;
  return factor * wheel * (trackF / 1000);
}
