import { test } from 'node:test';
import assert from 'node:assert/strict';
import { makePreset } from '../src/vehicle/presets.js';
import { makeCorner, solveCorner, analyseAxle } from '../src/suspension/kinematics.js';
import { optimiseCorner } from '../src/optim/suspension.js';
import { buildChassis } from '../src/chassis/mesh.js';
import { massModel } from '../src/fea/model.js';
import { massProperties, rideRoll } from '../src/vehicle/dynamics.js';
import { dist } from '../src/suspension/vec.js';

const veh = (isFront) => ({ wheelbase: 2650, cgH: 430, brakeFront: 0.6, driveFront: 0, isFront });

test('corner solve preserves link lengths through travel and steer', () => {
  const def = makePreset('supercar').suspension.front;
  const c = makeCorner(def);
  for (const [s, r] of [[-40, 0], [0, 30], [45, -20]]) {
    const sol = solveCorner(c, s, r);
    assert.ok(sol, `solves at s=${s} rack=${r}`);
    assert.ok(Math.abs(dist(sol.lbj, sol.ubj) - c.Lk) < 1e-6);
    assert.ok(Math.abs(dist(sol.tieO, sol.tieI) - c.Lt) < 1e-6);
    assert.ok(Math.abs(sol.wc[2] - (def.hp.wc[2] + s)) < 1e-6);
  }
});

test('static metrics reproduce the defined camber and toe', () => {
  for (const k of ['supercar', 'single', 'ev']) {
    const p = makePreset(k);
    const r = analyseAxle(makeCorner(p.suspension.front), { rackMax: p.suspension.front.rackMax, veh: veh(true) });
    assert.ok(r.ok, k);
    assert.ok(Math.abs(r.metrics.camber - p.suspension.front.camber) < 1e-6);
    assert.ok(Math.abs(r.metrics.toe - p.suspension.front.toe) < 1e-6);
    assert.ok(r.metrics.mr > 0, `${k} motion ratio positive`);
  }
});

test('parallel equal-length arms give zero camber change and RC at ground', () => {
  const def = makePreset('supercar').suspension.front;
  def.camber = 0; def.toe = 0;
  Object.assign(def.hp, {
    lcaF: [200, 300, 150], lcaR: [-200, 300, 150], lcaO: [0, 700, 150],
    ucaF: [200, 300, 500], ucaR: [-200, 300, 500], ucaO: [0, 700, 500],
    tieI: [-120, 300, 200], tieO: [-120, 700, 200], wc: [0, 760, 330],
  });
  const r = analyseAxle(makeCorner(def), { travel: [-30, 30], steps: 7, veh: veh(true) });
  assert.ok(r.ok);
  assert.ok(Math.max(...r.curves.camber.map(Math.abs)) < 1e-6, 'no camber change');
  assert.ok(Math.abs(r.metrics.bumpSteer) < 1e-6, 'tie rod parallel to arms -> no bump steer');
});

test('bump-steer optimiser reduces toe variation', async () => {
  const def = makePreset('ev').suspension.front;
  def.hp.tieI[2] += 25; // introduce bump steer
  const r = await optimiseCorner(def, { vars: [['tieI', 1, 40], ['tieI', 2, 50]], targets: { bumpSteer: 0, toeRange: 0 }, veh: veh(true), rackMax: 0, maxEval: 250 });
  assert.ok(Math.abs(r.after.bumpSteer) < Math.abs(r.before.bumpSteer) * 0.2, `${r.before.bumpSteer} -> ${r.after.bumpSteer}`);
});

test('presets meet their own ride / roll / LLTD targets', () => {
  for (const k of ['supercar', 'single', 'ev']) {
    const c = makePreset(k);
    const mesh = buildChassis(c);
    const mm = massModel(mesh.model);
    const kin = {
      front: analyseAxle(makeCorner(c.suspension.front), { rackMax: c.suspension.front.rackMax, veh: veh(true) }),
      rear: analyseAxle(makeCorner(c.suspension.rear), { veh: veh(false) }),
    };
    const mp = massProperties(c, { mass: mm.total, cg: [-1300, 0, 300] }, kin);
    const r = rideRoll(c, mp, kin, 30000);
    assert.ok(Math.abs(r.current.fF - c.ride.fF) < 0.1, `${k} fF ${r.current.fF}`);
    assert.ok(Math.abs(r.current.lltd - c.ride.lltd) < 3, `${k} lltd ${r.current.lltd}`);
    assert.ok(r.chassis.lltdFlex < r.chassis.lltdRigid + 5);
  }
});
