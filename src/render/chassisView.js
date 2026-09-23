// Renders the chassis FE model: sheet panels, feature edges, tubes, suspension pick-ups,
// result contours, deformed shapes and animated mode shapes.
import { THREE } from './scene.js';
import { GROUPS } from '../chassis/mesh.js';

// categorical (identity) - fixed order, validated dark-mode steps
export const GROUP_COLORS = ['#3987e5', '#d95926', '#199e70', '#c98500', '#d55181', '#008300', '#9085e9', '#e66767'];
// sequential single-hue ramp (magnitude): dark -> light
const RAMP = ['#0d366b', '#104281', '#184f95', '#1c5cab', '#256abf', '#2a78d6', '#3987e5', '#5598e7', '#6da7ec', '#86b6ef', '#9ec5f4', '#b7d3f6', '#cde2fb'];
const rampRGB = RAMP.map((c) => new THREE.Color(c));

export function rampColor(t, out = new THREE.Color()) {
  const x = (Number.isFinite(t) ? Math.min(1, Math.max(0, t)) : 0) * (rampRGB.length - 1);
  const i = Math.min(rampRGB.length - 2, Math.floor(x));
  return out.copy(rampRGB[i]).lerp(rampRGB[i + 1], x - i);
}
export const RAMP_CSS = `linear-gradient(90deg, ${RAMP.join(',')})`;

const TUBE_COLORS = { hoop: 0xc9ccd1, cage: 0xc9ccd1, brace: 0xa7adb5, engine: 0x6f757d };

export class ChassisView {
  constructor(viewer) {
    this.viewer = viewer;
    this.root = new THREE.Group();
    viewer.scene.add(this.root);
    this.opts = { panels: true, edges: true, wire: false, tubes: true, hardpoints: true, xray: false, display: 'material' };
    this.deform = null; // {u, scale}
    this.anim = null; // {shape, scale, t0}
    this.matMetal = new THREE.MeshStandardMaterial({ color: 0xb4bac2, metalness: 0.55, roughness: 0.42, side: THREE.DoubleSide, polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1 });
    this.matLit = new THREE.MeshStandardMaterial({ vertexColors: true, metalness: 0.2, roughness: 0.6, side: THREE.DoubleSide, polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1 });
    this.matFlat = new THREE.MeshBasicMaterial({ vertexColors: true, side: THREE.DoubleSide, polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1 });
    this.edgeMat = new THREE.LineBasicMaterial({ color: 0x1b1d20, transparent: true, opacity: 0.9 });
    this.wireMat = new THREE.LineBasicMaterial({ color: 0x0e1012, transparent: true, opacity: 0.35 });
    this.linkMat = new THREE.LineBasicMaterial({ color: 0x9aa0a6, transparent: true, opacity: 0.55 });
    viewer.onFrame.push((t) => this.tick(t));
  }

  setMesh(mesh) {
    this.mesh = mesh;
    // results belong to the previous mesh
    this.field = null;
    this.deform = null;
    this.anim = null;
    const m = mesh.model;
    this.root.clear();
    const ns = m.shells.length / 4;
    this.nn = m.nodes.length / 3;
    this.base = m.nodes;
    this.cur = Float64Array.from(m.nodes);
    // ---- shells (non-indexed, 2 triangles per quad)
    const vn = new Int32Array(ns * 6);
    for (let e = 0; e < ns; e++) {
      const q = m.shells.subarray(4 * e, 4 * e + 4);
      vn.set([q[0], q[1], q[2], q[0], q[2], q[3]], 6 * e);
    }
    this.vertNode = vn;
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(vn.length * 3), 3));
    geo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(vn.length * 3), 3));
    this.shellMesh = new THREE.Mesh(geo, this.matMetal);
    this.shellMesh.name = 'shells';
    this.root.add(this.shellMesh);
    // ---- feature edges: panel boundaries (edges used once within a panel)
    const edgePairs = [];
    const wirePairs = new Set();
    const byPanel = new Map();
    for (let e = 0; e < ns; e++) {
      const p = m.shellPanel[e];
      if (!byPanel.has(p)) byPanel.set(p, new Map());
      const map = byPanel.get(p);
      for (let k = 0; k < 4; k++) {
        const a = m.shells[4 * e + k], b = m.shells[4 * e + ((k + 1) % 4)];
        const key = a < b ? `${a}_${b}` : `${b}_${a}`;
        map.set(key, (map.get(key) || 0) + 1);
        wirePairs.add(key);
      }
    }
    for (const map of byPanel.values()) for (const [k, c] of map) if (c === 1) edgePairs.push(k.split('_').map(Number));
    this.edgeNodes = Int32Array.from(edgePairs.flat());
    this.wireNodes = Int32Array.from([...wirePairs].flatMap((k) => k.split('_').map(Number)));
    const lineGeo = (nodesArr) => {
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(nodesArr.length * 3), 3));
      return g;
    };
    this.edges = new THREE.LineSegments(lineGeo(this.edgeNodes), this.edgeMat);
    this.wire = new THREE.LineSegments(lineGeo(this.wireNodes), this.wireMat);
    this.root.add(this.edges, this.wire);
    // ---- tubes (instanced cylinders) and rigid links
    const tubes = [], links = [];
    for (let e = 0; e < m.beams.length / 2; e++) {
      const sec = m.sections[m.beamSec[e]];
      if (sec.rigid) links.push(m.beams[2 * e], m.beams[2 * e + 1]);
      else tubes.push({ a: m.beams[2 * e], b: m.beams[2 * e + 1], r: (sec.od || 40) / 2, kind: sec.name });
    }
    this.tubeList = tubes;
    const cyl = new THREE.CylinderGeometry(1, 1, 1, 14, 1, false);
    this.tubeMesh = new THREE.InstancedMesh(cyl, new THREE.MeshStandardMaterial({ color: 0xffffff, metalness: 0.6, roughness: 0.35 }), Math.max(1, tubes.length));
    this.tubeMesh.count = tubes.length;
    tubes.forEach((t, i) => this.tubeMesh.setColorAt(i, new THREE.Color(TUBE_COLORS[t.kind] ?? 0xb0b5bb)));
    this.root.add(this.tubeMesh);
    // stressed engine / gearbox: draw its braced frame as a closed casing
    const eng = mesh.beamInfo.find((b) => b.frame);
    const hullQuads = [];
    if (eng) {
      const F = eng.frame;
      for (let i = 1; i < F.length; i++) for (let k = 0; k < 4; k++) hullQuads.push([F[i - 1][k], F[i][k], F[i][(k + 1) % 4], F[i - 1][(k + 1) % 4]]);
      hullQuads.push(F[0], [...F[F.length - 1]].reverse());
      this.tubeList = tubes.filter((t) => t.kind !== 'engine');
      this.tubeMesh.count = this.tubeList.length;
      this.tubeList.forEach((t, i) => this.tubeMesh.setColorAt(i, new THREE.Color(TUBE_COLORS[t.kind] ?? 0xb0b5bb)));
    }
    this.hullNode = Int32Array.from(hullQuads.flatMap((q) => [q[0], q[1], q[2], q[0], q[2], q[3]]));
    const hg = new THREE.BufferGeometry();
    hg.setAttribute('position', new THREE.BufferAttribute(new Float32Array(this.hullNode.length * 3), 3));
    this.hull = new THREE.Mesh(hg, new THREE.MeshStandardMaterial({ color: 0x4b5058, metalness: 0.5, roughness: 0.5, side: THREE.DoubleSide }));
    this.root.add(this.hull);
    this.linkNodes = Int32Array.from(links);
    this.links = new THREE.LineSegments(lineGeo(this.linkNodes), this.linkMat);
    this.root.add(this.links);
    // ---- pick-ups
    const hp = mesh.hardpoints;
    const sph = new THREE.SphereGeometry(1, 16, 12);
    this.hpMesh = new THREE.InstancedMesh(sph, new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.4, metalness: 0.1 }), Math.max(1, hp.length));
    this.hpMesh.count = hp.length;
    const cF = new THREE.Color('#3987e5'), cR = new THREE.Color('#d95926');
    hp.forEach((p, i) => this.hpMesh.setColorAt(i, p.axle === 'F' ? cF : cR));
    this.hpMesh.name = 'hardpoints';
    this.root.add(this.hpMesh);
    this.selMarker = new THREE.Mesh(new THREE.SphereGeometry(1, 20, 14), new THREE.MeshBasicMaterial({ color: 0xffffff, wireframe: true }));
    this.selMarker.visible = false;
    this.root.add(this.selMarker);
    this.updateGeometry();
    this.applyDisplay();
    this.applyVisibility();
  }

  // ------------------------------------------------------------------ geometry
  displaced() {
    const n = this.nn;
    const cur = this.cur;
    const base = this.base;
    let u = null, s = 0;
    if (this.anim) { u = this.anim.shape; s = this.anim.scale * Math.sin(this.anim.phase || 0); }
    else if (this.deform) { u = this.deform.u; s = this.deform.scale; }
    for (let i = 0; i < n; i++) {
      cur[3 * i] = base[3 * i] + (u ? u[6 * i] * s : 0);
      cur[3 * i + 1] = base[3 * i + 1] + (u ? u[6 * i + 1] * s : 0);
      cur[3 * i + 2] = base[3 * i + 2] + (u ? u[6 * i + 2] * s : 0);
    }
    return cur;
  }

  updateGeometry() {
    if (!this.mesh) return;
    const cur = this.displaced();
    const fill = (attr, idx) => {
      const a = attr.array;
      for (let k = 0; k < idx.length; k++) { const i = idx[k]; a[3 * k] = cur[3 * i]; a[3 * k + 1] = cur[3 * i + 1]; a[3 * k + 2] = cur[3 * i + 2]; }
      attr.needsUpdate = true;
    };
    fill(this.shellMesh.geometry.attributes.position, this.vertNode);
    this.shellMesh.geometry.computeVertexNormals();
    this.shellMesh.geometry.computeBoundingSphere();
    fill(this.edges.geometry.attributes.position, this.edgeNodes);
    fill(this.wire.geometry.attributes.position, this.wireNodes);
    fill(this.links.geometry.attributes.position, this.linkNodes);
    fill(this.hull.geometry.attributes.position, this.hullNode);
    this.hull.geometry.computeVertexNormals();
    this.hull.geometry.computeBoundingSphere();
    for (const g of [this.edges.geometry, this.wire.geometry, this.links.geometry]) g.computeBoundingSphere();
    const m4 = new THREE.Matrix4(), q = new THREE.Quaternion(), up = new THREE.Vector3(0, 1, 0);
    const A = new THREE.Vector3(), B = new THREE.Vector3(), D = new THREE.Vector3(), S = new THREE.Vector3();
    this.tubeList.forEach((t, i) => {
      A.set(cur[3 * t.a], cur[3 * t.a + 1], cur[3 * t.a + 2]);
      B.set(cur[3 * t.b], cur[3 * t.b + 1], cur[3 * t.b + 2]);
      D.subVectors(B, A);
      const L = D.length();
      q.setFromUnitVectors(up, D.normalize());
      S.set(t.r, L, t.r);
      m4.compose(A.clone().add(B).multiplyScalar(0.5), q, S);
      this.tubeMesh.setMatrixAt(i, m4);
    });
    this.tubeMesh.instanceMatrix.needsUpdate = true;
    this.tubeMesh.computeBoundingSphere?.();
    const one = new THREE.Quaternion();
    this.mesh.hardpoints.forEach((h, i) => {
      const n = h.node;
      m4.compose(new THREE.Vector3(cur[3 * n], cur[3 * n + 1], cur[3 * n + 2]), one, new THREE.Vector3(14, 14, 14));
      this.hpMesh.setMatrixAt(i, m4);
    });
    this.hpMesh.instanceMatrix.needsUpdate = true;
    this.hpMesh.computeBoundingSphere?.();
  }

  // ------------------------------------------------------------------ display
  setOption(k, v) {
    this.opts[k] = v;
    if (k === 'display') this.applyDisplay();
    this.applyVisibility();
  }

  applyVisibility() {
    if (!this.mesh) return;
    const o = this.opts;
    this.shellMesh.visible = o.panels;
    this.edges.visible = o.edges;
    this.wire.visible = o.wire;
    this.tubeMesh.visible = o.tubes;
    this.hull.visible = o.tubes && this.hullNode.length > 0;
    this.hpMesh.visible = o.hardpoints;
    this.links.visible = o.hardpoints;
    for (const mat of [this.matMetal, this.matLit, this.matFlat]) {
      mat.transparent = o.xray;
      mat.opacity = o.xray ? 0.28 : 1;
      mat.depthWrite = !o.xray;
      mat.needsUpdate = true;
    }
  }

  /** field: {type:'groups'|'thickness'|'nodal'|'element', values, label, unit} */
  setField(field) {
    this.field = field;
    this.applyDisplay();
  }

  applyDisplay() {
    if (!this.mesh) return;
    const legend = document.getElementById('legend');
    const d = this.opts.display;
    const m = this.mesh.model;
    const col = this.shellMesh.geometry.attributes.color;
    const c = new THREE.Color();
    const ns = m.shells.length / 4;
    const setQuad = (e, color) => { for (let k = 0; k < 6; k++) col.setXYZ(6 * e + k, color.r, color.g, color.b); };
    legend.innerHTML = '';
    if (d === 'material') {
      this.shellMesh.material = this.matMetal;
      legend.classList.add('hidden');
      return;
    }
    if (d === 'groups') {
      const cols = GROUP_COLORS.map((x) => new THREE.Color(x));
      for (let e = 0; e < ns; e++) setQuad(e, cols[m.shellGroup[e]]);
      col.needsUpdate = true;
      this.shellMesh.material = this.matLit;
      const present = [...new Set(m.shellGroup)].sort((a, b) => a - b);
      legend.append(el('div', 'lt', 'Panel groups'));
      for (const g of present) legend.append(elHTML('div', 'cat', `<i style="background:${GROUP_COLORS[g]}"></i>${GROUP_NAMES[GROUPS[g]]} · ${m.shellT[m.shellGroup.indexOf(g)].toFixed(1)} mm`));
      legend.classList.remove('hidden');
      return;
    }
    // magnitude fields
    let vals, perNode = false, label, unit, digits = 1;
    if (d === 'thickness') { vals = m.shellT; label = 'Sheet thickness'; unit = 'mm'; }
    else if (this.field && this.field.type === d) { vals = this.field.values; perNode = this.field.perNode; label = this.field.label; unit = this.field.unit; digits = this.field.digits ?? 2; }
    else { this.shellMesh.material = this.matMetal; legend.classList.add('hidden'); return; }
    let lo = Infinity, hi = -Infinity;
    const sample = perNode ? Array.from(vals) : Array.from(vals).slice(0, ns);
    for (const v of sample) { if (v < lo) lo = v; if (v > hi) hi = v; }
    // clip contour at the 98th percentile so single-point singularities do not wash out the plot
    const clip = d === 'vm' || d === 'sed';
    if (clip) {
      const sorted = [...sample].sort((a, b) => a - b);
      hi = sorted[Math.floor(sorted.length * 0.98)] || hi;
    }
    const span = hi - lo || 1;
    if (perNode) {
      for (let k = 0; k < this.vertNode.length; k++) { rampColor((vals[this.vertNode[k]] - lo) / span, c); col.setXYZ(k, c.r, c.g, c.b); }
    } else {
      for (let e = 0; e < ns; e++) { rampColor((vals[e] - lo) / span, c); setQuad(e, c); }
    }
    col.needsUpdate = true;
    this.shellMesh.material = this.matFlat;
    legend.append(el('div', 'lt', `${label} (${unit})`));
    const bar = el('div', 'bar'); bar.style.background = RAMP_CSS;
    const ticks = el('div', 'ticks');
    for (let i = 0; i <= 3; i++) ticks.append(el('span', '', (lo + (span * i) / 3).toFixed(digits)));
    legend.append(bar, ticks);
    if (clip) legend.append(el('div', 'muted', 'upper limit = 98th percentile'));
    legend.classList.remove('hidden');
  }

  setDeformation(u, scale) {
    this.deform = u ? { u, scale } : null;
    this.updateGeometry();
  }

  animateMode(shape, scale) {
    this.anim = shape ? { shape, scale, phase: 0 } : null;
    if (!shape) this.updateGeometry();
  }

  tick(t) {
    if (this.anim) {
      this.anim.phase = (t / 1000) * Math.PI * 2 * 0.9;
      this.updateGeometry();
    }
  }

  select(index) {
    if (!this.selMarker) return;
    if (index === null || index === undefined || !this.mesh) { this.selMarker.visible = false; return; }
    const h = this.mesh.hardpoints[index];
    this.selMarker.position.set(...h.pos);
    this.selMarker.scale.setScalar(26);
    this.selMarker.visible = true;
  }

  bounds() {
    const b = new THREE.Box3();
    const n = this.base;
    for (let i = 0; i < n.length; i += 3) b.expandByPoint(new THREE.Vector3(n[i], n[i + 1], n[i + 2]));
    return b;
  }
}

const GROUP_NAMES = {
  floor: 'Floor', sills: 'Sills', tunnel: 'Tunnel', bulkheads: 'Bulkheads', deck: 'Deck', frontRails: 'Front rails', rearRails: 'Rear rails', battery: 'Battery',
};
function el(tag, cls, text) { const e = document.createElement(tag); if (cls) e.className = cls; if (text !== undefined) e.textContent = text; return e; }
function elHTML(tag, cls, html) { const e = el(tag, cls); e.innerHTML = html; return e; }
