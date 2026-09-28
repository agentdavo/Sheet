// Sheet stiffening: swage beads, weld-on doublers and equivalent shell properties.
//
// Beads are modelled as a half-sine corrugation of depth d and width w repeated at pitch p.
// Along the bead axis the bending stiffness rises with the offset of the sheet from its
// neutral axis; across the beads it stays ~flat-plate. The isotropic shell element gets the
// geometric mean of the two (an equivalent bending thickness); the buckling check uses the
// orthotropic pair directly.

/** Bead cross-section properties per unit width for sheet thickness t. */
export function beadSection(t, bead) {
  const { pitch: p, depth: d, width: w } = bead;
  const Iflat = (t * t * t) / 12;
  if (!(p > 0 && d > 0 && w > 0) || w > p) return { Dratio: 1, memF: 1, massF: 1, Iflat, Ialong: Iflat };
  const ybar = (2 * d * w) / (Math.PI * p);
  const Ialong = Iflat + (t * ((d * d * w) / 2 - p * ybar * ybar)) / p;
  // developed length of a half-sine of width w, depth d (Ramanujan-free series, accurate for d < w)
  const k = (Math.PI * d) / w;
  const dev = w * (1 + (k * k) / 4 - (3 * k ** 4) / 64);
  const devRatio = (p - w + dev) / p;
  return {
    Dratio: Ialong / Iflat, // bending stiffness along beads / flat plate
    memF: 1 / devRatio, // shear flow follows the longer developed path
    massF: devRatio, // extra material in the formed beads
    Iflat, Ialong,
  };
}

export const DEFAULT_BEADS = { on: false, pitch: 125, depth: 6, width: 40, groups: ['floor', 'sills', 'tunnel', 'bulkheads', 'deck', 'battery'] };

export function beadsFor(cfg, group) {
  const b = { ...DEFAULT_BEADS, ...(cfg.chassis.beads || {}) };
  return b.on && b.groups.includes(group) ? b : null;
}

/**
 * Write per-element sheet thickness and equivalent stiffness thicknesses into mesh.model.
 * gauges defaults to cfg.gauges (the gauge optimiser passes trial values).
 */
export function applyGauges(mesh, cfg, gauges = cfg.gauges) {
  const m = mesh.model;
  const ne = m.shellGroup.length;
  m.shellT = m.shellT && m.shellT.length === ne ? m.shellT : new Float64Array(ne);
  m.shellTm = new Float64Array(ne);
  m.shellTb = new Float64Array(ne);
  m.shellMassF = new Float64Array(ne);
  m.shellBexp = new Float64Array(ne);
  const groups = m.groups;
  const cache = new Map();
  for (let e = 0; e < ne; e++) {
    const base = groups[m.shellBaseGroup[e]];
    const doubler = groups[m.shellGroup[e]] === 'doublers';
    const t = (gauges[base] ?? 2) + (doubler ? gauges.doublers ?? 0 : 0);
    const bead = doubler ? null : beadsFor(cfg, base);
    let tm = t, tb = t, mf = 1, bexp = 3;
    if (bead) {
      const key = `${t}|${bead.pitch}|${bead.depth}|${bead.width}`;
      let s = cache.get(key);
      if (!s) { s = beadSection(t, bead); cache.set(key, s); }
      tb = t * Math.cbrt(Math.sqrt(s.Dratio));
      tm = t * s.memF;
      mf = s.massF;
      bexp = 2; // D_eq ~ sqrt(D_along * D_flat) ~ t^2 when the bead term dominates
    }
    m.shellT[e] = t;
    m.shellTm[e] = tm;
    m.shellTb[e] = tb;
    m.shellMassF[e] = mf;
    m.shellBexp[e] = bexp;
  }
  return m;
}
