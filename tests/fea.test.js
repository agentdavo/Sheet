import { test } from 'node:test';
import assert from 'node:assert/strict';
import { prepare, solveStatic, modal } from '../src/fea/model.js';
import { tubeSection } from '../src/fea/materials.js';

const E = 70000, nu = 0.3;

function plateModel(L, b, t, nx, ny, plane = 'xy') {
  const nodes = [];
  for (let j = 0; j <= ny; j++)
    for (let i = 0; i <= nx; i++) {
      const x = (L * i) / nx, y = (b * j) / ny;
      nodes.push(...(plane === 'xy' ? [x, y, 0] : [x, 0, y]));
    }
  const shells = [];
  for (let j = 0; j < ny; j++)
    for (let i = 0; i < nx; i++) {
      const a = j * (nx + 1) + i;
      shells.push(a, a + 1, a + nx + 2, a + nx + 1);
    }
  return {
    nodes: Float64Array.from(nodes),
    shells: Int32Array.from(shells),
    shellT: new Float64Array(shells.length / 4).fill(t),
    shellGroup: new Int32Array(shells.length / 4),
    groups: ['plate'],
    mat: { E, nu, rho: 2.7e-6 },
    beams: new Int32Array(0), beamSec: new Int32Array(0), beamUp: new Float64Array(0), sections: [],
  };
}

function cantileverBC(model, nx, ny, dof, P) {
  const nn = model.nodes.length / 3;
  const fixed = new Uint8Array(6 * nn);
  const f = new Float64Array(6 * nn);
  for (let j = 0; j <= ny; j++) {
    const root = j * (nx + 1);
    for (let d = 0; d < 6; d++) fixed[6 * root + d] = 1;
    const tip = j * (nx + 1) + nx;
    f[6 * tip + dof] = P / (ny + 1);
  }
  return { fixed, loads: [f] };
}

test('plate bending cantilever (MITC4) matches beam theory', async () => {
  const L = 1000, b = 100, t = 5, P = 10;
  const m = plateModel(L, b, t, 20, 2);
  const prep = prepare(m);
  const r = await solveStatic(prep, cantileverBC(m, 20, 2, 2, P));
  const tip = 20;
  const w = r.u[0][6 * tip + 2];
  const I = (b * t ** 3) / 12;
  const exact = (P * L ** 3) / (3 * E * I); // narrow strip: anticlastic curvature free -> beam modulus
  assert.ok(Math.abs(w / exact - 1) < 0.03, `w=${w} exact=${exact}`);
});

test('in-plane cantilever (incompatible modes) matches beam theory', async () => {
  const L = 1000, b = 100, t = 5, P = 100;
  const m = plateModel(L, b, t, 20, 2);
  const prep = prepare(m);
  const r = await solveStatic(prep, cantileverBC(m, 20, 2, 1, P));
  const v = r.u[0][6 * 20 + 1];
  const I = (t * b ** 3) / 12;
  const exact = (P * L ** 3) / (3 * E * I) + (P * L) / ((E / (2 * (1 + nu))) * b * t * (5 / 6));
  assert.ok(Math.abs(v / exact - 1) < 0.05, `v=${v} exact=${exact}`);
});

test('closed square tube torsion matches Bredt-Batho', async () => {
  // square tube a x a, length L along x, built from 4 shell walls
  const a = 200, L = 1200, t = 2, n = 8, nx = 24;
  const ring = [];
  for (let k = 0; k < 4 * n; k++) {
    const side = Math.floor(k / n), s = (k % n) / n;
    const pts = [[-a / 2 + a * s, -a / 2], [a / 2, -a / 2 + a * s], [a / 2 - a * s, a / 2], [-a / 2, a / 2 - a * s]];
    ring.push(pts[side]);
  }
  const nr = ring.length;
  const nodes = [];
  for (let i = 0; i <= nx; i++) for (const [y, z] of ring) nodes.push((L * i) / nx, y, z);
  const shells = [];
  for (let i = 0; i < nx; i++) for (let k = 0; k < nr; k++) {
    const a0 = i * nr + k, a1 = i * nr + ((k + 1) % nr);
    shells.push(a0, a0 + nr, a1 + nr, a1);
  }
  const m = {
    nodes: Float64Array.from(nodes), shells: Int32Array.from(shells),
    shellT: new Float64Array(shells.length / 4).fill(t), shellGroup: new Int32Array(shells.length / 4), groups: ['tube'],
    mat: { E, nu, rho: 2.7e-6 }, beams: new Int32Array(0), beamSec: new Int32Array(0), beamUp: new Float64Array(0), sections: [],
  };
  const nn = nodes.length / 3;
  const fixed = new Uint8Array(6 * nn);
  for (let k = 0; k < nr; k++) for (let d = 0; d < 6; d++) fixed[6 * k + d] = 1;
  const f = new Float64Array(6 * nn);
  // torque as tangential shear flow at the tip ring
  const T = 1e6;
  const q = T / (2 * a * a); // shear flow N/mm
  for (let k = 0; k < nr; k++) {
    const node = nx * nr + k;
    const [y, z] = ring[k];
    const [y1, z1] = ring[(k + 1) % nr];
    const fx = q * (y1 - y), fz = q * (z1 - z); // edge force along perimeter (ccw)
    f[6 * node + 1] += fx / 2; f[6 * node + 2] += fz / 2;
    const nb = nx * nr + ((k + 1) % nr);
    f[6 * nb + 1] += fx / 2; f[6 * nb + 2] += fz / 2;
  }
  const prep = prepare(m);
  const r = await solveStatic(prep, { fixed, loads: [f] });
  // twist from corner displacement
  const cn = nx * nr + 0; // corner (-a/2,-a/2)
  const uy = r.u[0][6 * cn + 1], uz = r.u[0][6 * cn + 2];
  const [y0, z0] = ring[0];
  const theta = (y0 * uz - z0 * uy) / (y0 * y0 + z0 * z0);
  const G = E / (2 * (1 + nu));
  const J = (4 * (a * a) ** 2 * t) / (4 * a);
  const exact = (T * L) / (G * J);
  assert.ok(Math.abs(theta / exact - 1) < 0.05, `theta=${theta} exact=${exact}`);
});

test('beam cantilever and modal frequency', async () => {
  const L = 1000, nEl = 10;
  const sec = { ...tubeSection(40, 2), E: 210000, G: 80000, rho: 7.85e-6, c: 20 };
  const nodes = [], beams = [];
  for (let i = 0; i <= nEl; i++) nodes.push((L * i) / nEl, 0, 0);
  for (let i = 0; i < nEl; i++) beams.push(i, i + 1);
  const m = {
    nodes: Float64Array.from(nodes), shells: new Int32Array(0), shellT: new Float64Array(0), shellGroup: new Int32Array(0), groups: [],
    mat: { E, nu, rho: 2.7e-6 }, beams: Int32Array.from(beams), beamSec: new Int32Array(nEl), beamUp: new Float64Array(3 * nEl).fill(0).map((_, i) => (i % 3 === 2 ? 1 : 0)), sections: [sec],
  };
  const nn = nEl + 1;
  const fixed = new Uint8Array(6 * nn);
  for (let d = 0; d < 6; d++) fixed[d] = 1;
  const f = new Float64Array(6 * nn);
  f[6 * nEl + 2] = 100;
  const prep = prepare(m);
  const r = await solveStatic(prep, { fixed, loads: [f] });
  const exact = (100 * L ** 3) / (3 * sec.E * sec.Iy);
  assert.ok(Math.abs(r.u[0][6 * nEl + 2] / exact - 1) < 1e-6);
  // free-free beam first bending: f = (4.730^2 / 2pi) sqrt(EI / (m L^4))
  const modes = modal(prepare(m), m, 2);
  const mu = sec.A * sec.rho; // kg/mm
  const fEx = (4.730 ** 2 / (2 * Math.PI)) * Math.sqrt((sec.E * 1000 * sec.Iy) / (mu * L ** 4)) ;
  // E in N/mm^2 with kg/mm and mm -> need unit factor 1000 (N = kg m/s^2 = 1000 kg mm/s^2)
  assert.ok(Math.abs(modes[0].freq / fEx - 1) < 0.08, `f=${modes[0].freq} exact=${fEx}`);
});
