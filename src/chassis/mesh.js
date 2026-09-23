// Parametric sheet-metal chassis -> conforming shell + beam finite-element model.
//
// Every panel is an axis-aligned rectangle in a "logical" space whose grid lines are
// shared globally (all key coordinates + uniform fill to the target element size). This
// guarantees conforming meshes wherever panels meet (T-joints, folds, bulkheads). The
// logical mesh is then mapped to the physical shape (plan-view taper toward the front
// bulkhead, height taper of the nose) - a smooth map, so conformity is preserved.
import { MATERIALS, TUBE_MATERIALS, tubeSection } from '../fea/materials.js';
import { chassisPoints } from '../suspension/kinematics.js';

export const GROUPS = ['floor', 'sills', 'tunnel', 'bulkheads', 'deck', 'frontRails', 'rearRails', 'battery'];
export const GROUP_LABELS = {
  floor: 'Floor pan', sills: 'Sills / side boxes', tunnel: 'Centre tunnel', bulkheads: 'Bulkheads',
  deck: 'Scuttle / footwell deck', frontRails: 'Front rails / towers', rearRails: 'Rear rails', battery: 'Battery enclosure',
};

const RIGID = { A: 5000, Iy: 5e6, Iz: 5e6, J: 1e7, E: 2.1e6, G: 8e5, rho: 0, c: 40, rigid: true, name: 'rigid' };

export function buildChassis(cfg) {
  const ch = cfg.chassis;
  const h = Math.max(25, ch.mesh);
  const F = ch.floorZ;
  const W = ch.W, Wi = ch.W - ch.sillW;
  const deck = ch.deck;
  const bulkH = deck ? ch.deckH : Math.max(ch.sillH, ch.deckH * 0.8);
  const hasFR = ch.frontRails && ch.xNose > ch.xFront + 1;
  const hasRR = ch.rearRails && ch.xRear < ch.xSeat - 1;
  const sW = (x) => (x >= ch.xFront ? ch.taperW : x <= ch.xDash ? 1 : 1 + ((ch.taperW - 1) * (x - ch.xDash)) / (ch.xFront - ch.xDash));
  const sH = (x) => (x >= ch.xFront ? ch.taperH : x <= ch.xDash ? 1 : 1 + ((ch.taperH - 1) * (x - ch.xDash)) / (ch.xFront - ch.xDash));
  const toPhys = (x, y, z) => [x, y * sW(x), F + (z - F) * sH(x)];
  // rails are specified physically; convert to logical at the bulkhead taper
  const frY = ch.frRailY.map((v) => v / ch.taperW);
  const frZ = ch.frRailZ.map((v) => F + (v - F) / ch.taperH);
  const rrY = ch.rrRailY, rrZ = ch.rrRailZ;

  // ------------------------------------------------------------ key lines
  const keysX = [ch.xFront, ch.xDash, ch.xSeat];
  if (hasFR) keysX.push(ch.xNose);
  if (hasRR) keysX.push(ch.xRear);
  const bayX = [];
  if (ch.battery) for (let i = 1; i < ch.batteryBays; i++) bayX.push(ch.xFront + ((ch.xSeat - ch.xFront) * i) / ch.batteryBays);
  keysX.push(...bayX);
  const keysY = [0, Wi, W];
  if (ch.tunnel) keysY.push(ch.tunnelHalf);
  if (hasFR) keysY.push(...frY);
  if (hasRR) keysY.push(...rrY);
  const keysZ = [F, F + ch.sillH, F + ch.firewallH, F + bulkH];
  if (ch.tunnel) keysZ.push(F + ch.tunnelH);
  if (deck) keysZ.push(F + ch.deckH);
  if (ch.battery) keysZ.push(F + ch.batteryH);
  if (hasFR) keysZ.push(...frZ);
  if (hasRR) keysZ.push(...rrZ);
  const mirror = (ks) => [...new Set(ks.flatMap((v) => [v, -v]))];

  const snapList = (vals, tol) => {
    const s = [...vals].sort((a, b) => a - b);
    const out = [];
    for (const v of s) if (!out.length || v - out[out.length - 1] > tol) out.push(v);
    return out;
  };
  const kx = snapList(keysX, h * 0.3);
  const ky = snapList(mirror(keysY), h * 0.3);
  const kz = snapList(keysZ, h * 0.25);
  const snapTo = (list) => (v) => list.reduce((b, k) => (Math.abs(k - v) < Math.abs(b - v) ? k : b), list[0]);
  const SX = snapTo(kx), SY = snapTo(ky), SZ = snapTo(kz);
  const fill = (keys) => {
    const g = [keys[0]];
    for (let i = 1; i < keys.length; i++) {
      const a = keys[i - 1], b = keys[i];
      const n = Math.max(1, Math.round((b - a) / h));
      for (let k = 1; k <= n; k++) g.push(a + ((b - a) * k) / n);
    }
    return g;
  };
  const GX = fill(kx), GY = fill(ky), GZ = fill(kz);

  // ------------------------------------------------------------ nodes / shells
  const nodeMap = new Map();
  const nodes = [];
  const nodeKey = (x, y, z) => `${Math.round(x * 10)},${Math.round(y * 10)},${Math.round(z * 10)}`;
  const logical = [];
  const getNode = (x, y, z) => {
    const k = nodeKey(x, y, z);
    let id = nodeMap.get(k);
    if (id === undefined) {
      id = nodes.length / 3;
      nodeMap.set(k, id);
      nodes.push(...toPhys(x, y, z));
      logical.push([x, y, z]);
    }
    return id;
  };
  const shells = [], shellGroup = [], shellPanel = [];
  const quadSeen = new Set();
  const panels = [];
  const between = (G, a, b) => {
    const lo = Math.min(a, b) - 1e-6, hi = Math.max(a, b) + 1e-6;
    return G.filter((v) => v >= lo && v <= hi);
  };
  const addPanel = (name, group, axis, at, r1, r2) => {
    let pts;
    if (axis === 'z') { at = SZ(at); r1 = r1.map(SX); r2 = r2.map(SY); pts = [between(GX, ...r1), between(GY, ...r2)]; }
    if (axis === 'y') { at = SY(at); r1 = r1.map(SX); r2 = r2.map(SZ); pts = [between(GX, ...r1), between(GZ, ...r2)]; }
    if (axis === 'x') { at = SX(at); r1 = r1.map(SY); r2 = r2.map(SZ); pts = [between(GY, ...r1), between(GZ, ...r2)]; }
    const [A, B] = pts;
    if (A.length < 2 || B.length < 2) return;
    const P = (a, b) => (axis === 'z' ? [a, b, at] : axis === 'y' ? [a, at, b] : [at, a, b]);
    const gi = GROUPS.indexOf(group);
    const pi = panels.length;
    let count = 0;
    const start = shells.length / 4;
    for (let i = 0; i < A.length - 1; i++)
      for (let j = 0; j < B.length - 1; j++) {
        const q = [P(A[i], B[j]), P(A[i + 1], B[j]), P(A[i + 1], B[j + 1]), P(A[i], B[j + 1])].map((p) => getNode(...p));
        const key = [...q].sort((a, b) => a - b).join(',');
        if (quadSeen.has(key)) continue;
        quadSeen.add(key);
        shells.push(...q);
        shellGroup.push(gi);
        shellPanel.push(pi);
        count++;
      }
    if (!count) return;
    // physical outline (sampled along the grid so tapers are followed)
    const edge = [];
    for (const a of A) edge.push(toPhys(...P(a, B[0])));
    for (const b of B.slice(1)) edge.push(toPhys(...P(A[A.length - 1], b)));
    for (const a of [...A].reverse().slice(1)) edge.push(toPhys(...P(a, B[B.length - 1])));
    for (const b of [...B].reverse().slice(1)) edge.push(toPhys(...P(A[0], b)));
    panels.push({ name, group, axis, start, count, outline: edge, size: [Math.abs(A[A.length - 1] - A[0]), Math.abs(B[B.length - 1] - B[0])] });
  };

  // ---- tub
  const xs = [ch.xSeat, ch.xFront];
  addPanel('Floor pan', 'floor', 'z', F, xs, [-W, W]);
  for (const s of [1, -1]) {
    const side = s > 0 ? 'LH' : 'RH';
    addPanel(`Sill outer ${side}`, 'sills', 'y', s * W, xs, [F, F + ch.sillH]);
    addPanel(`Sill inner ${side}`, 'sills', 'y', s * Wi, xs, [F, F + ch.sillH]);
    addPanel(`Sill top ${side}`, 'sills', 'z', F + ch.sillH, xs, [s * Wi, s * W]);
  }
  if (ch.tunnel) {
    for (const s of [1, -1]) addPanel(`Tunnel wall ${s > 0 ? 'LH' : 'RH'}`, 'tunnel', 'y', s * ch.tunnelHalf, xs, [F, F + ch.tunnelH]);
    addPanel('Tunnel top', 'tunnel', 'z', F + ch.tunnelH, xs, [-ch.tunnelHalf, ch.tunnelHalf]);
  }
  addPanel('Front bulkhead', 'bulkheads', 'x', ch.xFront, [-W, W], [F, F + bulkH]);
  addPanel('Firewall / seat-back bulkhead', 'bulkheads', 'x', ch.xSeat, [-W, W], [F, F + ch.firewallH]);
  if (deck) {
    addPanel('Scuttle / footwell deck', 'deck', 'z', F + ch.deckH, [ch.xDash, ch.xFront], [-W, W]);
    for (const s of [1, -1]) addPanel(`Footwell side ${s > 0 ? 'LH' : 'RH'}`, 'deck', 'y', s * W, [ch.xDash, ch.xFront], [F + ch.sillH, F + ch.deckH]);
    addPanel('Dash bulkhead', 'bulkheads', 'x', ch.xDash, [-W, W], [F + ch.sillH, F + ch.deckH]);
  }
  if (ch.battery) {
    addPanel('Battery cover', 'battery', 'z', F + ch.batteryH, xs, [-Wi, Wi]);
    bayX.forEach((x, i) => addPanel(`Battery cross-member ${i + 1}`, 'battery', 'x', x, [-Wi, Wi], [F, F + ch.batteryH]));
    if (!ch.tunnel) addPanel('Battery spine', 'battery', 'y', 0, xs, [F, F + ch.batteryH]);
  }
  const rail = (label, group, xr, Y, Z) => {
    for (const s of [1, -1]) {
      const side = s > 0 ? 'LH' : 'RH';
      addPanel(`${label} inner ${side}`, group, 'y', s * Y[0], xr, Z);
      addPanel(`${label} outer ${side}`, group, 'y', s * Y[1], xr, Z);
      addPanel(`${label} lower ${side}`, group, 'z', Z[0], xr, [s * Y[0], s * Y[1]]);
      addPanel(`${label} upper ${side}`, group, 'z', Z[1], xr, [s * Y[0], s * Y[1]]);
      addPanel(`${label} end plate ${side}`, group, 'x', xr[0] === ch.xFront ? xr[1] : xr[0], [s * Y[0], s * Y[1]], Z);
    }
  };
  if (hasFR) rail('Front rail', 'frontRails', [ch.xFront, ch.xNose], frY, frZ);
  if (hasRR) rail('Rear rail', 'rearRails', [ch.xRear, ch.xSeat], rrY, rrZ);

  const nShellNodes = nodes.length / 3;
  const nodeXYZ = (i) => [nodes[3 * i], nodes[3 * i + 1], nodes[3 * i + 2]];
  const nearest = (p, pool = null, k = 1, exclude = null) => {
    const best = [];
    const n = nodes.length / 3;
    const test = (i) => {
      if (exclude && exclude.has(i)) return;
      const d = (nodes[3 * i] - p[0]) ** 2 + (nodes[3 * i + 1] - p[1]) ** 2 + (nodes[3 * i + 2] - p[2]) ** 2;
      if (best.length < k || d < best[best.length - 1][0]) {
        best.push([d, i]);
        best.sort((a, b) => a[0] - b[0]);
        if (best.length > k) best.pop();
      }
    };
    if (pool) for (const i of pool) test(i); else for (let i = 0; i < n; i++) test(i);
    return best.map(([d, i]) => ({ i, d: Math.sqrt(d) }));
  };

  // ------------------------------------------------------------ beams
  const sections = [RIGID];
  const secIndex = {};
  const section = (key) => {
    if (secIndex[key] !== undefined) return secIndex[key];
    const t = cfg.tubes[key];
    const m = TUBE_MATERIALS[t.mat];
    const s = { ...tubeSection(t.od, t.wall), E: m.E, G: m.G, rho: m.rho, c: t.od / 2, name: key, od: t.od };
    sections.push(s);
    secIndex[key] = sections.length - 1;
    return secIndex[key];
  };
  const beams = [], beamSec = [], beamUp = [];
  const beamInfo = [];
  const hardNodes = new Set();
  const snapTol = h * 0.75;
  const pointNode = (p, attach = true) => {
    if (attach) {
      const [n] = nearest(p, null, 1, hardNodes);
      if (n && n.d < snapTol) return n.i;
    }
    nodes.push(...p);
    return nodes.length / 3 - 1;
  };
  const addBeamLine = (name, pts, secKey, attachEnds = true) => {
    const sec = section(secKey);
    const ids = [];
    for (let k = 0; k < pts.length - 1; k++) {
      const a = pts[k], b = pts[k + 1];
      const L = Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
      const n = Math.max(1, Math.round(L / (h * 1.5)));
      const i0 = k === 0 ? pointNode(a, attachEnds) : ids[ids.length - 1];
      if (k === 0) ids.push(i0);
      for (let j = 1; j <= n; j++) {
        const t = j / n;
        const p = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
        const last = j === n && k === pts.length - 2;
        const id = j === n ? pointNode(p, last ? attachEnds : false) : pointNode(p, false);
        const prev = ids[ids.length - 1];
        if (id !== prev) { beams.push(prev, id); beamSec.push(sec); beamUp.push(0, 0, 1); }
        ids.push(id);
      }
    }
    beamInfo.push({ name, nodes: ids, sec, od: cfg.tubes[secKey].od });
    return ids;
  };
  const P = (x, y, z) => toPhys(x, y, z);
  if (ch.rollHoop) {
    const zb = F + Math.min(ch.firewallH, ch.sillH), zt = F + ch.hoopH;
    const wt = Math.max(120, W - 140);
    for (const s of [1, -1]) addBeamLine(`Main hoop leg ${s > 0 ? 'LH' : 'RH'}`, [P(ch.xSeat, s * W, zb), [ch.xSeat - 40, s * wt, zt]], 'hoop');
    addBeamLine('Main hoop top', [[ch.xSeat - 40, wt, zt], [ch.xSeat - 40, -wt, zt]], 'hoop');
  }
  if (ch.cage && ch.rollHoop) {
    const zBase = deck ? F + ch.deckH : F + ch.sillH;
    const xa = ch.xDash - 0.55 * (ch.xDash - ch.xSeat);
    const wa = W - 150;
    const wt = Math.max(120, W - 140);
    for (const s of [1, -1]) {
      const side = s > 0 ? 'LH' : 'RH';
      addBeamLine(`A-pillar ${side}`, [P(ch.xDash, s * W, zBase), [xa, s * wa, F + ch.roofH]], 'cage');
      addBeamLine(`Roof rail ${side}`, [[xa, s * wa, F + ch.roofH], [ch.xSeat - 40, s * wt, F + ch.hoopH]], 'cage');
    }
    addBeamLine('Windscreen header', [[xa, wa, F + ch.roofH], [xa, -wa, F + ch.roofH]], 'cage');
  }
  if (ch.frontHoop) {
    const zb = deck ? F + ch.deckH : F + ch.sillH;
    const pL = P(ch.xDash, W, zb), pR = P(ch.xDash, -W, zb);
    const top = zb + 150;
    addBeamLine('Front hoop', [pL, [ch.xDash, W * 0.45, top], [ch.xDash, -W * 0.45, top], pR], 'hoop');
  }
  if (hasRR && ch.rearBrace) {
    const zt = rrZ[1];
    addBeamLine('Engine-bay X-brace A', [[ch.xSeat, rrY[0], zt], [ch.xRear, -rrY[0], zt]], 'brace');
    addBeamLine('Engine-bay X-brace B', [[ch.xSeat, -rrY[0], zt], [ch.xRear, rrY[0], zt]], 'brace');
    const xa = -cfg.vehicle.wheelbase;
    if (xa < ch.xSeat && xa > ch.xRear) addBeamLine('Rear strut brace', [[xa, rrY[0], zt], [xa, -rrY[0], zt]], 'brace');
  }
  if (ch.bumpers) {
    if (hasFR) { const zm = (ch.frRailZ[0] + ch.frRailZ[1]) / 2 - 150; addBeamLine('Front crash beam', [[ch.xNose, ch.frRailY[1], zm], [ch.xNose, -ch.frRailY[1], zm]], 'brace'); }
    if (hasRR) { const zm = (rrZ[0] + rrZ[1]) / 2 - 150; addBeamLine('Rear crash beam', [[ch.xRear, rrY[1], zm], [ch.xRear, -rrY[1], zm]], 'brace'); }
  }
  if (ch.engine) {
    // stressed engine + gearbox as a braced box frame
    const x0 = ch.xSeat, x1 = ch.engineEnd ?? ch.xSeat - 800, x2 = ch.xRear;
    const ey = 150, ez0 = F + 90, ez1 = F + Math.min(ch.firewallH - 80, 420);
    const gy = 110, gz0 = F + 85, gz1 = F + 380;
    const ring = (x, y, z0, z1) => [[x, y, z0], [x, -y, z0], [x, -y, z1], [x, y, z1]];
    const secs = [];
    const nSeg = Math.max(2, Math.round((x0 - x1) / 300));
    for (let i = 0; i <= nSeg; i++) secs.push(ring(x0 + ((x1 - x0) * i) / nSeg, ey, ez0, ez1));
    const nG = Math.max(2, Math.round((x1 - x2) / 300));
    for (let i = 1; i <= nG; i++) {
      const t = i / nG;
      secs.push(ring(x1 + (x2 - x1) * t, ey + (gy - ey) * t, ez0 + (gz0 - ez0) * t, ez1 + (gz1 - ez1) * t));
    }
    // engine block modelled as a braced frame; mass is carried by the component list (rho = 0)
    const ids = secs.map((r) => r.map((p) => pointNode(p, false)));
    const sec = section('engine');
    sections[sec].rho = 0;
    // engine mounts: each front-ring node bolted to the firewall through a stiff spider
    const fw = [];
    for (let i = 0; i < nShellNodes; i++) if (Math.abs(nodes[3 * i] - ch.xSeat) < 1) fw.push(i);
    for (const id of ids[0]) for (const { i } of nearest(nodeXYZ(id), fw, 6)) { beams.push(id, i); beamSec.push(0); beamUp.push(0, 0, 1); }
    const link = (a, b) => { if (a !== b) { beams.push(a, b); beamSec.push(sec); beamUp.push(0, 0, 1); } };
    for (let i = 0; i < ids.length; i++) {
      for (let k = 0; k < 4; k++) link(ids[i][k], ids[i][(k + 1) % 4]);
      if (i > 0) for (let k = 0; k < 4; k++) { link(ids[i - 1][k], ids[i][k]); link(ids[i - 1][k], ids[i][(k + 1) % 4]); }
      if (i > 0) link(ids[i][0], ids[i][2]);
    }
    beamInfo.push({ name: 'Engine + gearbox (stressed member)', nodes: ids.flat(), sec, od: cfg.tubes.engine.od, frame: ids });
    if (ch.rollHoop) {
      // roll-hoop back-stays onto the engine cam covers
      const ring = ids[Math.min(2, ids.length - 1)];
      const wt = Math.max(120, W - 140);
      for (const [s, k] of [[1, 3], [-1, 2]]) addBeamLine(`Hoop back-stay ${s > 0 ? 'LH' : 'RH'}`, [[ch.xSeat - 40, s * wt, F + ch.hoopH], nodeXYZ(ring[k])], 'brace');
    }
  }

  // ------------------------------------------------------------ suspension hardpoints & spiders
  const structural = [];
  for (let i = 0; i < nodes.length / 3; i++) structural.push(i);
  const hardpoints = [];
  const axles = [
    ['F', cfg.suspension.front, 0],
    ['R', cfg.suspension.rear, -cfg.vehicle.wheelbase],
  ];
  for (const [ax, def, x0] of axles) {
    for (const name of chassisPoints(def)) {
      for (const s of [1, -1]) {
        const p = def.hp[name];
        const g = [x0 + p[0], s * p[1], p[2]];
        const id = nodes.length / 3;
        nodes.push(...g);
        hardNodes.add(id);
        // bracket footprint: all structural nodes within `hpSpread` of the closest one (4..16 links)
        const cand = nearest(g, structural, 24);
        const d0 = cand[0]?.d ?? 0;
        const nb = cand.filter((q, k) => k < 4 || (q.d <= d0 + (ch.hpSpread ?? 110) && k < 16));
        for (const { i } of nb) { beams.push(id, i); beamSec.push(0); beamUp.push(0, 0, 1); }
        hardpoints.push({ axle: ax, side: s, name, node: id, pos: g, dist: nb[0]?.d ?? 0, attach: nb.map((q) => q.i) });
      }
    }
  }

  // ------------------------------------------------------------ bookkeeping
  const twistStations = [];
  for (const x of GX) {
    if (x > ch.xFront + 1e-6 || x < ch.xSeat - 1e-6) continue;
    const kl = nodeMap.get(nodeKey(x, SY(W), SZ(F + ch.sillH)));
    const kr = nodeMap.get(nodeKey(x, SY(-W), SZ(F + ch.sillH)));
    if (kl !== undefined && kr !== undefined) twistStations.push({ x, nL: kl, nR: kr });
  }
  // bending load introduction: sill tops along the cockpit (seat / occupant mounts)
  const floorNodes = [];
  const zs = SZ(F + ch.sillH);
  for (let i = 0; i < nShellNodes; i++) {
    const [x, y, z] = logical[i];
    if (Math.abs(z - zs) < 1e-6 && x < ch.xDash - 1 && x > ch.xSeat + 1 && Math.abs(y) >= SY(Wi) - 1e-6) floorNodes.push(i);
  }
  const mat = MATERIALS[cfg.material];
  const shellT = shellGroup.map((g) => cfg.gauges[GROUPS[g]] ?? 2);
  const model = {
    nodes: Float64Array.from(nodes),
    shells: Int32Array.from(shells),
    shellT: Float64Array.from(shellT),
    shellGroup: Int32Array.from(shellGroup),
    shellPanel: Int32Array.from(shellPanel),
    groups: GROUPS,
    mat: { E: mat.E, nu: mat.nu, rho: mat.rho },
    beams: Int32Array.from(beams),
    beamSec: Int32Array.from(beamSec),
    beamUp: Float64Array.from(beamUp),
    sections,
  };
  return {
    model, panels, beamInfo, hardpoints, twistStations, floorNodes,
    stats: { nodes: nodes.length / 3, shells: shells.length / 4, beams: beams.length / 2, dofs: (nodes.length / 3) * 6 },
    toPhys,
  };
}

/** Distribute component masses (for modal analysis) to their nearest structural nodes. */
export function lumpComponents(mesh, masses) {
  const nodes = mesh.model.nodes;
  const n = nodes.length / 3;
  const hard = new Set(mesh.hardpoints.map((h) => h.node));
  const out = [];
  for (const c of masses) {
    if (!c.m) continue;
    const best = [];
    for (let i = 0; i < n; i++) {
      if (hard.has(i)) continue;
      const d = (nodes[3 * i] - c.x) ** 2 + (nodes[3 * i + 1] - c.y) ** 2 + (nodes[3 * i + 2] - c.z) ** 2;
      best.push([d, i]);
    }
    best.sort((a, b) => a[0] - b[0]);
    const k = Math.min(6, best.length);
    for (let j = 0; j < k; j++) out.push({ node: best[j][1], mass: c.m / k });
  }
  return out;
}
