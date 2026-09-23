// Three.js WebGPU renderer (automatic WebGL2 fallback), cameras, controls, quad view.
import * as THREE from 'three/webgpu';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// three r186 always passes the identity swizzle ('rgba') to createView(); Chromium builds that
// predate the string form of GPUTextureViewDescriptor.swizzle reject it. Identity == default,
// so drop it for compatibility with those browsers.
if (typeof GPUTexture !== 'undefined' && !GPUTexture.prototype.__swizzleShim) {
  const createView = GPUTexture.prototype.createView;
  GPUTexture.prototype.createView = function (desc) {
    if (desc && desc.swizzle === 'rgba') {
      const { swizzle, ...rest } = desc;
      void swizzle;
      return createView.call(this, rest);
    }
    return createView.call(this, desc);
  };
  GPUTexture.prototype.__swizzleShim = true;
}

export class Viewer {
  constructor(container) {
    this.container = container;
    this.scene = new THREE.Scene();
    this.mode = 'single'; // 'single' | 'quad'
    this.onFrame = [];
    this.bounds = new THREE.Box3(new THREE.Vector3(-4000, -1000, 0), new THREE.Vector3(1000, 1000, 1300));
  }

  async init() {
    const forceWebGL = new URLSearchParams(location.search).has('webgl');
    this.renderer = new THREE.WebGPURenderer({ antialias: true, forceWebGL, alpha: true });
    this.renderer.setPixelRatio(Math.min(2, window.devicePixelRatio));
    await this.renderer.init();
    this.isWebGPU = !!this.renderer.backend?.isWebGPUBackend;
    this.renderer.setClearColor(0x000000, 0);
    this.container.prepend(this.renderer.domElement);

    // Z-up vehicle coordinates
    THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
    this.persp = new THREE.PerspectiveCamera(35, 1, 10, 60000);
    this.persp.up.set(0, 0, 1);
    this.persp.position.set(3200, 4200, 2600);
    this.controls = new OrbitControls(this.persp, this.renderer.domElement);
    this.controls.target.set(-1300, 0, 350);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.12;
    this.controls.update();

    const mk = () => { const c = new THREE.OrthographicCamera(-1, 1, 1, -1, -20000, 20000); c.up.set(0, 0, 1); return c; };
    this.ortho = { front: mk(), side: mk(), top: mk() };
    this.ortho.front.position.set(1, 0, 0); this.ortho.front.lookAt(0, 0, 0);
    this.ortho.side.position.set(0, 1, 0); this.ortho.side.lookAt(0, 0, 0);
    this.ortho.top.up.set(1, 0, 0); this.ortho.top.position.set(0, 0, 1); this.ortho.top.lookAt(0, 0, 0);

    // lights
    this.scene.add(new THREE.HemisphereLight(0xe8eef8, 0x2a2622, 1.6));
    const key = new THREE.DirectionalLight(0xffffff, 2.2);
    key.position.set(3000, 2500, 5000);
    this.scene.add(key);
    const rim = new THREE.DirectionalLight(0x9fc4ff, 0.9);
    rim.position.set(-5000, -3000, 2000);
    this.scene.add(rim);

    // ground grid in the XY plane
    const grid = new THREE.GridHelper(12000, 60, 0x3b3d40, 0x26282a);
    grid.rotation.x = Math.PI / 2;
    grid.position.set(-1400, 0, 0);
    grid.material.transparent = true;
    grid.material.opacity = 0.55;
    this.grid = grid;
    this.scene.add(grid);
    const axes = new THREE.AxesHelper(300);
    axes.position.set(0, 0, 1);
    this.scene.add(axes);

    this.resize();
    new ResizeObserver(() => this.resize()).observe(this.container);
    this.renderer.setAnimationLoop((t) => this.frame(t));
    return this;
  }

  resize() {
    const w = this.container.clientWidth, h = this.container.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.w = w; this.h = h;
    this.fitOrtho();
  }

  setBounds(box) {
    this.bounds.copy(box);
    this.fitOrtho();
  }

  fitOrtho() {
    if (!this.w) return;
    const b = this.bounds;
    const c = b.getCenter(new THREE.Vector3());
    const s = b.getSize(new THREE.Vector3());
    const vw = this.mode === 'quad' ? this.w / 2 : this.w;
    const vh = this.mode === 'quad' ? this.h / 2 : this.h;
    const asp = vw / vh;
    const set = (cam, halfW, halfH, pos) => {
      const k = Math.max(halfW / asp, halfH) * 1.12;
      cam.left = -k * asp; cam.right = k * asp; cam.top = k; cam.bottom = -k;
      cam.position.copy(pos);
      cam.updateProjectionMatrix();
    };
    set(this.ortho.front, s.y / 2, s.z / 2, new THREE.Vector3(c.x + 10000, c.y, c.z));
    this.ortho.front.lookAt(c.x, c.y, c.z);
    set(this.ortho.side, s.x / 2, s.z / 2, new THREE.Vector3(c.x, c.y + 10000, c.z));
    this.ortho.side.lookAt(c.x, c.y, c.z);
    set(this.ortho.top, s.y / 2, s.x / 2, new THREE.Vector3(c.x, c.y, c.z + 10000));
    this.ortho.top.lookAt(c.x, c.y, c.z);
    this.persp.aspect = vw / vh;
    this.persp.updateProjectionMatrix();
  }

  setMode(mode) {
    this.mode = mode;
    this.fitOrtho();
  }

  /** Animate the perspective camera to a named view. */
  view(name) {
    const b = this.bounds;
    const c = b.getCenter(new THREE.Vector3());
    const r = b.getSize(new THREE.Vector3()).length() * 0.95;
    const dirs = {
      iso: new THREE.Vector3(0.62, 0.72, 0.45), front: new THREE.Vector3(1, 0, 0.08), side: new THREE.Vector3(0, 1, 0.05),
      top: new THREE.Vector3(0.0001, 0, 1), rear: new THREE.Vector3(-0.8, -0.5, 0.35), under: new THREE.Vector3(0.3, 0.5, -0.6),
    };
    const d = (dirs[name] || dirs.iso).clone().normalize();
    const from = this.persp.position.clone(), to = c.clone().addScaledVector(d, r * 1.25);
    const t0 = this.controls.target.clone();
    const start = performance.now();
    const step = () => {
      const t = Math.min(1, (performance.now() - start) / 450);
      const e = t * t * (3 - 2 * t);
      this.persp.position.lerpVectors(from, to, e);
      this.controls.target.lerpVectors(t0, c, e);
      this.controls.update();
      if (t < 1) requestAnimationFrame(step);
    };
    step();
  }

  frame(t) {
    this.controls.update();
    for (const f of this.onFrame) f(t);
    const r = this.renderer;
    if (this.mode === 'single') {
      r.setScissorTest(false);
      r.setViewport(0, 0, this.w, this.h);
      r.render(this.scene, this.persp);
      return;
    }
    const hw = Math.floor(this.w / 2), hh = Math.floor(this.h / 2);
    const views = [
      [this.ortho.front, 0, 0],
      [this.ortho.side, hw, 0],
      [this.ortho.top, 0, hh],
      [this.persp, hw, hh],
    ];
    r.setScissorTest(true);
    for (const [cam, x, y] of views) {
      // WebGPURenderer uses a top-left viewport origin on both of its backends
      r.setViewport(x, y, hw, hh);
      r.setScissor(x, y, hw, hh);
      r.render(this.scene, cam);
    }
    r.setScissorTest(false);
  }

  /** Which camera is under client point (for picking in quad mode) + NDC. */
  pickCamera(clientX, clientY) {
    const rect = this.renderer.domElement.getBoundingClientRect();
    const px = clientX - rect.left, py = clientY - rect.top;
    if (this.mode === 'single') return { cam: this.persp, ndc: new THREE.Vector2((px / rect.width) * 2 - 1, -(py / rect.height) * 2 + 1) };
    const hw = rect.width / 2, hh = rect.height / 2;
    const col = px < hw ? 0 : 1, row = py < hh ? 0 : 1;
    const cam = [[this.ortho.front, this.ortho.side], [this.ortho.top, this.persp]][row][col];
    const lx = px - col * hw, ly = py - row * hh;
    return { cam, ndc: new THREE.Vector2((lx / hw) * 2 - 1, -(ly / hh) * 2 + 1) };
  }

  async screenshot() {
    this.frame(performance.now());
    return new Promise((res) => this.renderer.domElement.toBlob(res, 'image/png'));
  }
}

export { THREE };
