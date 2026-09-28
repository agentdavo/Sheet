import { test } from 'node:test';
import assert from 'node:assert/strict';
import { prepare, solveStatic, postprocess } from '../src/fea/model.js';
import { bucklingCheck } from '../src/fea/buckling.js';
import { beadSection } from '../src/chassis/stiffening.js';
import { buildChassis } from '../src/chassis/mesh.js';
import { runAnalyses } from '../src/fea/analysis.js';
import { makePreset } from '../src/vehicle/presets.js';
import { MATERIALS } from '../src/fea/materials.js';

test('bead section: flat limit and depth scaling', () => {
  const flat = beadSection(1, { pitch: 125, depth: 0, width: 40 });
  assert.equal(flat.Dratio, 1);
  const b6 = beadSection(1, { pitch: 125, depth: 6, width: 40 });
  const b3 = beadSection(1, { pitch: 125, depth: 3, width: 40 });
  assert.ok(b6.Dratio > 50 && b6.Dratio < 200, `ratio ${b6.Dratio}`);
  // bead term ~ d^2 dominates
  assert.ok(Math.abs((b6.Dratio - 1) / (b3.Dratio - 1) - 4) < 0.05);
  assert.ok(b6.massF > 1 && b6.massF < 1.05);
});

test('FE shear panel: buckling load factor matches plate theory', async () => {
  const La = 600, Lb = 300, t = 1, nx = 24, ny = 12, q = 5; // q = shear flow N/mm
  const mat = MATERIALS.ss304L;
  const nodes = [];
  for (let j = 0; j <= ny; j++) for (let i = 0; i <= nx; i++) nodes.push((La * i) / nx, (Lb * j) / ny, 0);
  const shells = [];
  for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) { const a = j * (nx + 1) + i; shells.push(a, a + 1, a + nx + 2, a + nx + 1); }
  const ns = shells.length / 4;
  const model = {
    nodes: Float64Array.from(nodes), shells: Int32Array.from(shells), shellT: new Float64Array(ns).fill(t),
    shellGroup: new Int32Array(ns), groups: ['floor'], mat: { E: mat.E, nu: mat.nu, rho: mat.rho },
    beams: new Int32Array(0), beamSec: new Int32Array(0), beamUp: new Float64Array(0), sections: [],
  };
  const nn = nodes.length / 3;
  const id = (i, j) => j * (nx + 1) + i;
  const fixed = new Uint8Array(6 * nn);
  for (let n = 0; n < nn; n++) for (const d of [2, 3, 4]) fixed[6 * n + d] = 1; // membrane problem
  fixed[6 * id(0, 0)] = 1; fixed[6 * id(0, 0) + 1] = 1; fixed[6 * id(nx, 0) + 1] = 1;
  const f = new Float64Array(6 * nn);
  const edge = (pts, dof, sign, len) => pts.forEach((n, k) => { f[6 * n + dof] += sign * q * len * (k === 0 || k === pts.length - 1 ? 0.5 : 1); });
  edge([...Array(nx + 1).keys()].map((i) => id(i, 0)), 0, -1, La / nx);
  edge([...Array(nx + 1).keys()].map((i) => id(i, ny)), 0, 1, La / nx);
  edge([...Array(ny + 1).keys()].map((j) => id(0, j)), 1, -1, Lb / ny);
  edge([...Array(ny + 1).keys()].map((j) => id(nx, j)), 1, 1, Lb / ny);
  const r = await solveStatic(prepare(model), { fixed, loads: [f] });
  const post = postprocess(model, r.u[0]);
  const mesh = { model, bays: [{ name: 'panel', group: 'floor', LA: La, LB: Lb, elems: Int32Array.from([...Array(ns).keys()]) }] };
  const cfg = { material: 'ss304L', chassis: { beads: { on: false } } };
  const res = bucklingCheck(mesh, cfg, post, 1000, 1000);
  const D = (mat.E * t ** 3) / (12 * (1 - mat.nu ** 2));
  const ks = 5.35 + 4 * (Lb / La) ** 2;
  const tauCr = (ks * Math.PI ** 2 * D) / (t * Lb * Lb);
  const expected = tauCr / (q / t);
  assert.ok(Math.abs(res.crit.lambda / expected - 1) < 0.02, `lambda ${res.crit.lambda} expected ${expected}`);
  assert.ok(Math.abs(res.crit.tau - q / t) / (q / t) < 0.02, 'recovered shear stress');
});

test('diaphragms split sill bays; stiffened stainless tub out-performs plain 1 mm in buckling', async () => {
  const plain = makePreset('supercar_ss');
  plain.chassis.mesh = 120;
  plain.chassis.beads.on = false;
  plain.chassis.diaphragmPitch = 0;
  plain.gauges = { ...plain.gauges, floor: 1, sills: 1, frontRails: 1, rearRails: 1, doublers: 0 };
  const stiff = makePreset('supercar_ss');
  stiff.chassis.mesh = 120;
  const mP = buildChassis(plain), mS = buildChassis(stiff);
  assert.ok(mS.stats.bays > mP.stats.bays, `${mS.stats.bays} vs ${mP.stats.bays} bays`);
  const sillBays = mS.bays.filter((b) => b.name === 'Sill outer LH');
  assert.ok(sillBays.length >= 3 && Math.max(...sillBays.map((b) => b.LA)) < 700);
  const rP = await runAnalyses(mP, plain, ['torsion']);
  const rS = await runAnalyses(mS, stiff, ['torsion']);
  const bP = bucklingCheck(mP, plain, rP.torsion.post, rP.torsion.torqueNm, 5000);
  const bS = bucklingCheck(mS, stiff, rS.torsion.post, rS.torsion.torqueNm, 5000);
  assert.ok(bS.Tcr > 2 * bP.Tcr, `stiffened ${bS.Tcr} vs plain ${bP.Tcr}`);
  assert.ok(rS.torsion.hpPeak < rP.torsion.hpPeak, 'doublers lower pick-up stress');
});
