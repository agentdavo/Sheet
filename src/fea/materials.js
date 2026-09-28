// Units used throughout the FEA core: mm, N, MPa (N/mm^2), kg, s.
// Density is therefore in kg/mm^3.
export const MATERIALS = {
  al5754: { name: 'Aluminium 5754-H22', E: 70000, nu: 0.33, rho: 2.67e-6, yield: 130, cost: 4.5, alpha: 2.38e-05, k: 130, weldNote: 'Weldable (MIG/TIG, 5356 filler); HAZ keeps most of its O-temper strength.' },
  al6082: { name: 'Aluminium 6082-T6', E: 70000, nu: 0.33, rho: 2.70e-6, yield: 250, cost: 5.0, alpha: 2.34e-05, k: 170, weldNote: 'Welding the T6 temper roughly halves strength in the HAZ - design welds into low-stress zones or bond/rivet instead.' },
  al7075: { name: 'Aluminium 7075-T6', E: 71700, nu: 0.33, rho: 2.81e-6, yield: 460, cost: 9.0, alpha: 2.36e-05, k: 130, weldNote: 'Not fusion-weldable (hot cracking) - bond, rivet or friction-stir only.' },
  dc04: { name: 'Mild steel DC04', E: 210000, nu: 0.3, rho: 7.85e-6, yield: 210, cost: 1.2, alpha: 1.2e-05, k: 50, weldNote: 'Readily spot/MIG welded.' },
  dp800: { name: 'Dual-phase DP800', E: 210000, nu: 0.3, rho: 7.85e-6, yield: 500, cost: 1.8, alpha: 1.2e-05, k: 45, weldNote: 'Spot/laser weldable; HAZ softening - keep welds out of peak-stress areas.' },
  ss304: { name: 'Stainless 304 (annealed)', E: 193000, nu: 0.29, rho: 7.93e-6, yield: 215, cost: 4.0, alpha: 1.73e-05, k: 16, weldNote: 'Standard-carbon 304 sensitises in the HAZ (chromium carbides) - intergranular corrosion along welds. Use 304L or 316L.' },
  ss304L: { name: 'Stainless 304L (low carbon)', E: 193000, nu: 0.29, rho: 7.93e-6, yield: 170, cost: 4.3, alpha: 1.73e-05, k: 16, weldNote: 'Low carbon - no HAZ sensitisation. High expansion and low conductivity: fixture, stitch-weld or laser-weld thin gauge.' },
  ss316: { name: 'Stainless 316L', E: 193000, nu: 0.29, rho: 7.98e-6, yield: 170, cost: 5.5, alpha: 1.6e-05, k: 16, weldNote: 'Low carbon - no HAZ sensitisation. High expansion and low conductivity: fixture, stitch-weld or laser-weld thin gauge.' },
  ss301: { name: 'Stainless 301 1/2-hard', E: 193000, nu: 0.29, rho: 7.88e-6, yield: 760, cost: 5.0, alpha: 1.7e-05, k: 16, weldNote: 'Welding anneals the cold-worked temper locally (HAZ strength falls towards ~250 MPa); prefer spot/laser welds.' },
  ti64: { name: 'Titanium Ti-6Al-4V', E: 114000, nu: 0.34, rho: 4.43e-6, yield: 880, cost: 40, alpha: 8.6e-06, k: 7, weldNote: 'Weldable only with full inert-gas shielding (trailing shield / chamber).' },
  cfrp: { name: 'CFRP quasi-isotropic', E: 55000, nu: 0.3, rho: 1.55e-6, yield: 450, cost: 60, alpha: 1e-06, k: 5, weldNote: 'Not weldable - bonded and bolted construction.' },
};

// Tube materials for beams (roll hoops, subframes, wishbones).
export const TUBE_MATERIALS = {
  s355: { name: 'Steel S355 / T45', E: 210000, G: 80800, rho: 7.85e-6, yield: 355 },
  crmo: { name: '4130 Chromoly', E: 205000, G: 80000, rho: 7.85e-6, yield: 460 },
  al6082: { name: 'Aluminium 6082-T6', E: 70000, G: 26300, rho: 2.70e-6, yield: 250 },
  ti: { name: 'Titanium Grade 9', E: 105000, G: 40000, rho: 4.48e-6, yield: 500 },
};

export function tubeSection(od, wall) {
  const ro = od / 2;
  const ri = Math.max(0, ro - wall);
  const A = Math.PI * (ro * ro - ri * ri);
  const I = (Math.PI / 4) * (ro ** 4 - ri ** 4);
  return { A, Iy: I, Iz: I, J: 2 * I };
}

export function boxSection(w, h, wall) {
  const wi = Math.max(0, w - 2 * wall);
  const hi = Math.max(0, h - 2 * wall);
  const A = w * h - wi * hi;
  const Iy = (w * h ** 3 - wi * hi ** 3) / 12;
  const Iz = (h * w ** 3 - hi * wi ** 3) / 12;
  // Bredt-Batho thin-walled torsion constant
  const am = (w - wall) * (h - wall);
  const p = 2 * ((w - wall) + (h - wall));
  const J = (4 * am * am * wall) / p;
  return { A, Iy, Iz, J };
}
