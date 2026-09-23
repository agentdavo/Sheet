// Renders the four double-wishbone corners (arms, uprights, steering, pushrods / dampers,
// wheels) plus front-view geometry construction (instant centres, roll centres, roll axis).
import { THREE } from './scene.js';
import { makeCorner, solveCorner, rollCentre } from '../suspension/kinematics.js';
import { rad } from '../suspension/vec.js';

const Y = new THREE.Vector3(0, 1, 0);

function cyl(r, color, metal = 0.5) {
  const m = new THREE.Mesh(new THREE.CylinderGeometry(1, 1, 1, 12), new THREE.MeshStandardMaterial({ color, metalness: metal, roughness: 0.35 }));
  m.userData.r = r;
  return m;
}
function place(mesh, a, b, r = mesh.userData.r) {
  const A = new THREE.Vector3(...a), B = new THREE.Vector3(...b);
  const d = B.clone().sub(A);
  const L = d.length();
  mesh.position.copy(A).add(B).multiplyScalar(0.5);
  mesh.quaternion.setFromUnitVectors(Y, d.normalize());
  mesh.scale.set(r, Math.max(L, 1e-3), r);
}

class CornerView {
  constructor(parent, axle) {
    this.g = new THREE.Group();
    parent.add(this.g);
    const arm = 0x8d97a3, alu = 0xd0d4da;
    const col = axle === 'F' ? 0x3987e5 : 0xd95926;
    this.parts = {
      ucaF: cyl(9, arm), ucaR: cyl(9, arm), lcaF: cyl(10, arm), lcaR: cyl(10, arm),
      tie: cyl(7, 0x8b9199), push: cyl(7, 0x8b9199),
      kingpin: cyl(16, alu, 0.7), arm1: cyl(12, alu, 0.7), arm2: cyl(12, alu, 0.7), stub: cyl(18, alu, 0.7),
      dampBody: cyl(22, 0x2c2f33, 0.3), dampRod: cyl(9, 0xd8dce0, 0.9), spring: cyl(34, col, 0.2),
    };
    this.parts.spring.material.transparent = true;
    this.parts.spring.material.opacity = 0.35;
    for (const p of Object.values(this.parts)) this.g.add(p);
    this.rocker = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshStandardMaterial({ color: 0xaab0b8, metalness: 0.6, roughness: 0.4, side: THREE.DoubleSide }));
    this.rocker.geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(9), 3));
    this.g.add(this.rocker);
    // wheel
    this.wheel = new THREE.Group();
    const tyreMat = new THREE.MeshStandardMaterial({ color: 0x17181a, roughness: 0.9, metalness: 0 });
    this.tyre = new THREE.Mesh(new THREE.CylinderGeometry(1, 1, 1, 48, 1, false), tyreMat);
    this.rim = new THREE.Mesh(new THREE.CylinderGeometry(1, 1, 1, 40, 1, false), new THREE.MeshStandardMaterial({ color: 0x8e949b, metalness: 0.85, roughness: 0.3 }));
    this.disc = new THREE.Mesh(new THREE.CylinderGeometry(1, 1, 1, 40, 1, false), new THREE.MeshStandardMaterial({ color: 0x55595e, metalness: 0.8, roughness: 0.5 }));
    this.wheel.add(this.tyre, this.rim, this.disc);
    this.g.add(this.wheel);
    this.cp = new THREE.Mesh(new THREE.CircleGeometry(1, 24), new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: 0.8, side: THREE.DoubleSide }));
    this.g.add(this.cp);
  }

  update(sol, def, x0, side) {
    const G = (p) => [x0 + p[0], side * p[1], p[2]];
    const hp = def.hp;
    const P = this.parts;
    place(P.ucaF, G(hp.ucaF), G(sol.ubj)); place(P.ucaR, G(hp.ucaR), G(sol.ubj));
    place(P.lcaF, G(hp.lcaF), G(sol.lbj)); place(P.lcaR, G(hp.lcaR), G(sol.lbj));
    place(P.tie, G(sol.tieI), G(sol.tieO));
    place(P.kingpin, G(sol.lbj), G(sol.ubj));
    const mid = sol.lbj.map((v, i) => (v + sol.ubj[i]) / 2);
    place(P.arm1, G(mid), G(sol.tieO));
    place(P.arm2, G(mid), G(sol.wc), 14);
    // actuation
    let dTop, dBot;
    if (def.actuation === 'direct') {
      P.push.visible = false; this.rocker.visible = false;
      dTop = hp.damperC; dBot = sol.pushO;
    } else {
      P.push.visible = true; this.rocker.visible = true;
      place(P.push, G(sol.pushO), G(sol.rock.push));
      const pa = this.rocker.geometry.attributes.position;
      [hp.rockP, sol.rock.push, sol.rock.damp].forEach((p, i) => pa.setXYZ(i, ...G(p)));
      pa.needsUpdate = true;
      this.rocker.geometry.computeVertexNormals();
      this.rocker.geometry.computeBoundingSphere();
      dTop = hp.damperC; dBot = sol.rock.damp;
    }
    const L = Math.hypot(dTop[0] - dBot[0], dTop[1] - dBot[1], dTop[2] - dBot[2]);
    const t = Math.min(0.62, 150 / Math.max(L, 1));
    const split = dTop.map((v, i) => v + (dBot[i] - v) * Math.max(0.45, t));
    place(P.dampBody, G(dTop), G(split));
    place(P.dampRod, G(split), G(dBot));
    const sA = dTop.map((v, i) => v + (dBot[i] - v) * 0.12), sB = dTop.map((v, i) => v + (dBot[i] - v) * 0.8);
    place(P.spring, G(sA), G(sB));
    // wheel: cylinder axis along spin axis
    const a = new THREE.Vector3(sol.spin[0], side * sol.spin[1], sol.spin[2]).normalize();
    const w = def.tyreW, R = def.tyreR;
    this.wheel.position.set(...G(sol.wc));
    this.wheel.quaternion.setFromUnitVectors(Y, a);
    this.tyre.scale.set(R, w, R);
    this.rim.scale.set(R * 0.72, w * 1.02, R * 0.72);
    this.disc.scale.set(R * 0.58, 26, R * 0.58);
    this.disc.position.set(0, -w * 0.18, 0);
    this.cp.position.set(...G(sol.cp));
    this.cp.position.z += 1;
    this.cp.scale.setScalar(w * 0.35);
  }

  setVisible(susp, wheels) {
    for (const p of Object.values(this.parts)) p.visible = susp && p.visible !== false;
    this.g.visible = true;
    Object.values(this.parts).forEach((p) => { p.visible = susp; });
    this.rocker.visible = susp && this.rocker.visible;
    this.wheel.visible = wheels;
    this.cp.visible = wheels;
  }
}

export class SuspensionView {
  constructor(viewer) {
    this.viewer = viewer;
    this.root = new THREE.Group();
    viewer.scene.add(this.root);
    this.corners = { FL: new CornerView(this.root, 'F'), FR: new CornerView(this.root, 'F'), RL: new CornerView(this.root, 'R'), RR: new CornerView(this.root, 'R') };
    this.construction = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xf1c46a, transparent: true, opacity: 0.8 }));
    this.construction.geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(3 * 64), 3));
    this.root.add(this.construction);
    this.rcMarkers = new THREE.InstancedMesh(new THREE.OctahedronGeometry(1), new THREE.MeshBasicMaterial({ color: 0xf1c46a }), 2);
    this.root.add(this.rcMarkers);
    this.opts = { suspension: true, wheels: true, construction: true };
    this.pose = { heave: 0, roll: 0, rack: 0 };
  }

  setConfig(cfg) {
    this.cfg = cfg;
    this.solvers = {
      F: { L: makeCorner(cfg.suspension.front), R: makeCorner(cfg.suspension.front) },
      R: { L: makeCorner(cfg.suspension.rear), R: makeCorner(cfg.suspension.rear) },
    };
    this.update();
  }

  setPose(p) {
    Object.assign(this.pose, p);
    this.update();
  }

  /** Solve all four corners for the current pose; returns solutions (null if a corner locks). */
  solveAll() {
    const cfg = this.cfg;
    const out = {};
    for (const [ax, def, x0] of [['F', cfg.suspension.front, 0], ['R', cfg.suspension.rear, -cfg.vehicle.wheelbase]]) {
      const half = def.hp.wc[1];
      const sRoll = half * Math.tan(rad(this.pose.roll));
      const rack = ax === 'F' ? this.pose.rack : 0;
      const L = solveCorner(this.solvers[ax].L, this.pose.heave + sRoll, rack);
      const R = solveCorner(this.solvers[ax].R, this.pose.heave - sRoll, -rack);
      out[ax] = { L, R, def, x0 };
    }
    return out;
  }

  update() {
    if (!this.cfg) return;
    const sols = this.solveAll();
    this.last = sols;
    const segs = [];
    const m4 = new THREE.Matrix4();
    let ri = 0;
    for (const ax of ['F', 'R']) {
      const { L, R, def, x0 } = sols[ax];
      if (L) this.corners[`${ax}L`].update(L, def, x0, 1);
      if (R) this.corners[`${ax}R`].update(R, def, x0, -1);
      if (L && R) {
        const rc = rollCentre(L, R);
        const x = x0 + L.cp[0];
        const zg = (L.cp[2] + R.cp[2]) / 2;
        if (rc.icL) segs.push([x, L.cp[1], L.cp[2]], [x, rc.icL[0], rc.icL[1]]);
        if (rc.icR) segs.push([x, -R.cp[1], R.cp[2]], [x, rc.icR[0], rc.icR[1]]);
        // extend to the roll centre
        segs.push([x, L.cp[1], L.cp[2]], [x, rc.y, rc.z], [x, -R.cp[1], R.cp[2]], [x, rc.y, rc.z]);
        m4.compose(new THREE.Vector3(x, rc.y, rc.z), new THREE.Quaternion(), new THREE.Vector3(18, 18, 18));
        this.rcMarkers.setMatrixAt(ri++, m4);
        this[`rc${ax}`] = [x, rc.y, rc.z, zg];
      }
    }
    if (this.rcF && this.rcR) segs.push(this.rcF.slice(0, 3), this.rcR.slice(0, 3));
    const pa = this.construction.geometry.attributes.position;
    pa.array.fill(0);
    segs.slice(0, 64).forEach((p, i) => pa.setXYZ(i, ...p));
    this.construction.geometry.setDrawRange(0, Math.min(64, segs.length));
    pa.needsUpdate = true;
    this.construction.geometry.computeBoundingSphere();
    this.rcMarkers.count = ri;
    this.rcMarkers.instanceMatrix.needsUpdate = true;
    this.applyVisibility();
  }

  setOption(k, v) { this.opts[k] = v; this.applyVisibility(); }
  applyVisibility() {
    for (const c of Object.values(this.corners)) c.setVisible(this.opts.suspension, this.opts.wheels);
    const act = this.cfg;
    if (act) {
      for (const [k, c] of Object.entries(this.corners)) {
        const def = k[0] === 'F' ? act.suspension.front : act.suspension.rear;
        if (def.actuation === 'direct') { c.parts.push.visible = false; c.rocker.visible = false; }
      }
    }
    this.construction.visible = this.opts.construction;
    this.rcMarkers.visible = this.opts.construction;
  }
}
