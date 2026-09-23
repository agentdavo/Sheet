// WebGPU compute solver: node-block-Jacobi preconditioned conjugate gradient in f32,
// wrapped in f64 iterative refinement on the CPU (mixed precision). Works in the main
// thread or in a dedicated worker (navigator.gpu is available in both in Chromium).
import { csrMul } from './model.js';

const WG = 256;

const SPMV = /* wgsl */ `
struct P { n: u32, nwg: u32, o0: u32, o1: u32 };
@group(0) @binding(0) var<storage, read> rowPtr: array<u32>;
@group(0) @binding(1) var<storage, read> colIdx: array<u32>;
@group(0) @binding(2) var<storage, read> vals: array<f32>;
@group(0) @binding(3) var<storage, read> p: array<f32>;
@group(0) @binding(4) var<storage, read_write> Ap: array<f32>;
@group(0) @binding(5) var<uniform> prm: P;
@compute @workgroup_size(${WG}) fn main(@builtin(global_invocation_id) g: vec3<u32>) {
  let i = g.x;
  if (i >= prm.n) { return; }
  var s = 0.0;
  for (var k = rowPtr[i]; k < rowPtr[i + 1u]; k = k + 1u) { s = s + vals[k] * p[colIdx[k]]; }
  Ap[i] = s;
}`;

const DOT2 = /* wgsl */ `
struct P { n: u32, nwg: u32, o0: u32, o1: u32 };
@group(0) @binding(0) var<storage, read> a: array<f32>;
@group(0) @binding(1) var<storage, read> b: array<f32>;
@group(0) @binding(2) var<storage, read> c: array<f32>;
@group(0) @binding(3) var<storage, read> d: array<f32>;
@group(0) @binding(4) var<storage, read_write> part: array<f32>;
@group(0) @binding(5) var<uniform> prm: P;
var<workgroup> sa: array<f32, ${WG}>;
var<workgroup> sb: array<f32, ${WG}>;
@compute @workgroup_size(${WG}) fn main(@builtin(global_invocation_id) g: vec3<u32>, @builtin(local_invocation_id) l: vec3<u32>, @builtin(workgroup_id) w: vec3<u32>) {
  var s1 = 0.0; var s2 = 0.0;
  var i = g.x;
  loop {
    if (i >= prm.n) { break; }
    s1 = s1 + a[i] * b[i];
    s2 = s2 + c[i] * d[i];
    i = i + prm.nwg * ${WG}u;
  }
  sa[l.x] = s1; sb[l.x] = s2;
  workgroupBarrier();
  var st = ${WG / 2}u;
  loop {
    if (st == 0u) { break; }
    if (l.x < st) { sa[l.x] = sa[l.x] + sa[l.x + st]; sb[l.x] = sb[l.x] + sb[l.x + st]; }
    workgroupBarrier();
    st = st >> 1u;
  }
  if (l.x == 0u) { part[2u * w.x] = sa[0]; part[2u * w.x + 1u] = sb[0]; }
}`;

const REDUCE = /* wgsl */ `
struct P { n: u32, nwg: u32, o0: u32, o1: u32 };
@group(0) @binding(0) var<storage, read> part: array<f32>;
@group(0) @binding(1) var<storage, read_write> scal: array<f32>;
@group(0) @binding(2) var<uniform> prm: P;
var<workgroup> sa: array<f32, ${WG}>;
var<workgroup> sb: array<f32, ${WG}>;
@compute @workgroup_size(${WG}) fn main(@builtin(local_invocation_id) l: vec3<u32>) {
  var s1 = 0.0; var s2 = 0.0;
  for (var i = l.x; i < prm.nwg; i = i + ${WG}u) { s1 = s1 + part[2u * i]; s2 = s2 + part[2u * i + 1u]; }
  sa[l.x] = s1; sb[l.x] = s2;
  workgroupBarrier();
  var st = ${WG / 2}u;
  loop {
    if (st == 0u) { break; }
    if (l.x < st) { sa[l.x] = sa[l.x] + sa[l.x + st]; sb[l.x] = sb[l.x] + sb[l.x + st]; }
    workgroupBarrier();
    st = st >> 1u;
  }
  if (l.x == 0u) { scal[prm.o0] = sa[0]; scal[prm.o1] = sb[0]; }
}`;

// scal: [0] rz, [1] pAp, [2] rzNew, [3] rr
const UPDATE1 = /* wgsl */ `
struct P { n: u32, nwg: u32, o0: u32, o1: u32 };
@group(0) @binding(0) var<storage, read_write> x: array<f32>;
@group(0) @binding(1) var<storage, read_write> r: array<f32>;
@group(0) @binding(2) var<storage, read> p: array<f32>;
@group(0) @binding(3) var<storage, read> Ap: array<f32>;
@group(0) @binding(4) var<storage, read> scal: array<f32>;
@group(0) @binding(5) var<uniform> prm: P;
@compute @workgroup_size(${WG}) fn main(@builtin(global_invocation_id) g: vec3<u32>) {
  let i = g.x;
  if (i >= prm.n) { return; }
  let alpha = select(0.0, scal[0] / scal[1], scal[1] != 0.0);
  x[i] = x[i] + alpha * p[i];
  r[i] = r[i] - alpha * Ap[i];
}`;

const PRECOND = /* wgsl */ `
struct P { n: u32, nwg: u32, o0: u32, o1: u32 };
@group(0) @binding(0) var<storage, read> r: array<f32>;
@group(0) @binding(1) var<storage, read_write> z: array<f32>;
@group(0) @binding(2) var<storage, read> blk: array<f32>;
@group(0) @binding(3) var<storage, read> info: array<u32>;
@group(0) @binding(4) var<uniform> prm: P;
@compute @workgroup_size(${WG}) fn main(@builtin(global_invocation_id) g: vec3<u32>) {
  let i = g.x;
  if (i >= prm.n) { return; }
  let off = info[3u * i]; let st = info[3u * i + 1u]; let sz = info[3u * i + 2u];
  let lr = i - st;
  var s = 0.0;
  for (var j = 0u; j < sz; j = j + 1u) { s = s + blk[off + lr * sz + j] * r[st + j]; }
  z[i] = s;
}`;

const UPDATE2 = /* wgsl */ `
struct P { n: u32, nwg: u32, o0: u32, o1: u32 };
@group(0) @binding(0) var<storage, read> z: array<f32>;
@group(0) @binding(1) var<storage, read_write> p: array<f32>;
@group(0) @binding(2) var<storage, read> scal: array<f32>;
@group(0) @binding(3) var<uniform> prm: P;
@compute @workgroup_size(${WG}) fn main(@builtin(global_invocation_id) g: vec3<u32>) {
  let i = g.x;
  if (i >= prm.n) { return; }
  let beta = select(0.0, scal[2] / scal[0], scal[0] != 0.0);
  p[i] = z[i] + beta * p[i];
}`;

const SHIFT = /* wgsl */ `
@group(0) @binding(0) var<storage, read_write> scal: array<f32>;
@compute @workgroup_size(1) fn main() { scal[0] = scal[2]; }`;

let devicePromise = null;
export async function getDevice() {
  if (devicePromise) return devicePromise;
  devicePromise = (async () => {
    if (typeof navigator === 'undefined' || !navigator.gpu) return null;
    try {
      const adapter = await navigator.gpu.requestAdapter({ powerPreference: 'high-performance' });
      if (!adapter) return null;
      const device = await adapter.requestDevice({
        requiredLimits: {
          maxStorageBufferBindingSize: Math.min(adapter.limits.maxStorageBufferBindingSize, 1 << 30),
          maxBufferSize: Math.min(adapter.limits.maxBufferSize, 1 << 30),
        },
      });
      const info = adapter.info || {};
      device.__label = [info.vendor, info.architecture, info.description].filter(Boolean).join(' ') || 'WebGPU adapter';
      return device;
    } catch (e) {
      return null;
    }
  })();
  return devicePromise;
}

/** Node-block Jacobi preconditioner data from CSR (f64) and the node -> equation layout. */
function blockJacobi(csr, prep, eq) {
  const n = csr.n;
  const info = new Uint32Array(3 * n);
  const blocks = [];
  let off = 0;
  const blkVals = [];
  for (let p = 0; p < prep.nn; p++) {
    const node = prep.perm[p];
    const ids = [];
    for (let d = 0; d < 6; d++) { const q = eq[6 * node + d]; if (q >= 0) ids.push(q); }
    if (!ids.length) continue;
    const sz = ids.length, st = ids[0];
    const A = new Float64Array(sz * sz);
    for (let a = 0; a < sz; a++) {
      const i = ids[a];
      for (let k = csr.rowPtr[i]; k < csr.rowPtr[i + 1]; k++) {
        const j = csr.col[k];
        if (j >= st && j < st + sz) A[a * sz + (j - st)] = csr.val[k];
      }
    }
    const Ainv = invertSmall(A, sz);
    for (let a = 0; a < sz; a++) { info[3 * (st + a)] = off; info[3 * (st + a) + 1] = st; info[3 * (st + a) + 2] = sz; }
    blkVals.push(Ainv);
    off += sz * sz;
    blocks.push(sz);
  }
  const blk = new Float32Array(off);
  let o = 0;
  for (const b of blkVals) { blk.set(b, o); o += b.length; }
  return { blk, info };
}

function invertSmall(A0, n) {
  const a = Float64Array.from(A0);
  const b = new Float64Array(n * n);
  for (let i = 0; i < n; i++) b[i * n + i] = 1;
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(a[r * n + c]) > Math.abs(a[p * n + c])) p = r;
    if (p !== c) for (let k = 0; k < n; k++) { [a[c * n + k], a[p * n + k]] = [a[p * n + k], a[c * n + k]]; [b[c * n + k], b[p * n + k]] = [b[p * n + k], b[c * n + k]]; }
    const d = a[c * n + c] || 1e-30;
    for (let k = 0; k < n; k++) { a[c * n + k] /= d; b[c * n + k] /= d; }
    for (let r = 0; r < n; r++) if (r !== c) {
      const f = a[r * n + c];
      if (f) for (let k = 0; k < n; k++) { a[r * n + k] -= f * a[c * n + k]; b[r * n + k] -= f * b[c * n + k]; }
    }
  }
  return b;
}

/**
 * Returns an async solver usable as `opts.iterative` in solveStatic, or null when WebGPU
 * compute is unavailable.
 */
export async function makeGpuSolver({ tol = 1e-8, innerTol = 1e-4, maxIter = 40000, maxInner = 4000, chunk = 64, onProgress } = {}) {
  const device = await getDevice();
  if (!device) return null;
  const mod = (code) => device.createShaderModule({ code });
  const pipe = (code) => device.createComputePipeline({ layout: 'auto', compute: { module: mod(code), entryPoint: 'main' } });
  const P = { spmv: pipe(SPMV), dot2: pipe(DOT2), reduce: pipe(REDUCE), up1: pipe(UPDATE1), pre: pipe(PRECOND), up2: pipe(UPDATE2), shift: pipe(SHIFT) };
  const S = GPUBufferUsage.STORAGE | GPUBufferUsage.COPY_DST | GPUBufferUsage.COPY_SRC;

  return async function gpuSolve(csr, rhs, prep, eq) {
    const t0 = performance.now();
    const n = csr.n;
    const nwg = Math.min(256, Math.ceil(n / WG));
    const buf = (data, usage = S) => {
      const b = device.createBuffer({ size: Math.max(16, data.byteLength), usage, mappedAtCreation: true });
      new data.constructor(b.getMappedRange()).set(data);
      b.unmap();
      return b;
    };
    const zeros = (len) => device.createBuffer({ size: Math.max(16, len * 4), usage: S });
    const { blk, info } = blockJacobi(csr, prep, eq);
    const B = {
      rowPtr: buf(csr.rowPtr), col: buf(csr.col), val: buf(Float32Array.from(csr.val)),
      x: zeros(n), r: zeros(n), z: zeros(n), p: zeros(n), Ap: zeros(n),
      scal: zeros(8), part: zeros(2 * nwg), blk: buf(blk), info: buf(info),
    };
    const uni = (o0, o1) => buf(new Uint32Array([n, nwg, o0, o1]), GPUBufferUsage.UNIFORM | GPUBufferUsage.COPY_DST);
    const U = { base: uni(0, 0), pap: uni(1, 1), rz: uni(2, 3), init: uni(0, 3) };
    const bg = (p, list) => device.createBindGroup({ layout: p.getBindGroupLayout(0), entries: list.map((b, i) => ({ binding: i, resource: { buffer: b } })) });
    const G = {
      spmv: bg(P.spmv, [B.rowPtr, B.col, B.val, B.p, B.Ap, U.base]),
      dotPAp: bg(P.dot2, [B.p, B.Ap, B.p, B.Ap, B.part, U.base]),
      redPAp: bg(P.reduce, [B.part, B.scal, U.pap]),
      up1: bg(P.up1, [B.x, B.r, B.p, B.Ap, B.scal, U.base]),
      pre: bg(P.pre, [B.r, B.z, B.blk, B.info, U.base]),
      dotRZ: bg(P.dot2, [B.r, B.z, B.r, B.r, B.part, U.base]),
      redRZ: bg(P.reduce, [B.part, B.scal, U.rz]),
      redInit: bg(P.reduce, [B.part, B.scal, U.init]),
      up2: bg(P.up2, [B.z, B.p, B.scal, U.base]),
      shift: bg(P.shift, [B.scal]),
    };
    const nG = Math.ceil(n / WG);
    const read = device.createBuffer({ size: 32, usage: GPUBufferUsage.MAP_READ | GPUBufferUsage.COPY_DST });
    const readX = device.createBuffer({ size: n * 4, usage: GPUBufferUsage.MAP_READ | GPUBufferUsage.COPY_DST });
    const run = (pass, p, g, x) => { pass.setPipeline(p); pass.setBindGroup(0, g); pass.dispatchWorkgroups(x); };

    const bnorm = Math.sqrt(rhs.reduce((s, v) => s + v * v, 0)) || 1;
    const x64 = new Float64Array(n);
    const res = Float64Array.from(rhs);
    const Ax = new Float64Array(n);
    let totalIt = 0, outer = 0, relRes = 1;
    for (outer = 0; outer < 16; outer++) {
      // inner solve A d = res (scaled to unit norm for f32 range)
      const rn = Math.sqrt(res.reduce((s, v) => s + v * v, 0));
      relRes = rn / bnorm;
      if (relRes < tol) break;
      const r32 = new Float32Array(n);
      for (let i = 0; i < n; i++) r32[i] = res[i] / rn;
      device.queue.writeBuffer(B.r, 0, r32);
      device.queue.writeBuffer(B.x, 0, new Float32Array(n));
      let enc = device.createCommandEncoder();
      let pass = enc.beginComputePass();
      run(pass, P.pre, G.pre, nG);
      pass.end();
      enc.copyBufferToBuffer(B.z, 0, B.p, 0, n * 4);
      pass = enc.beginComputePass();
      run(pass, P.dot2, G.dotRZ, nwg);
      run(pass, P.reduce, G.redInit, 1); // scal[0] = r.z, scal[3] = r.r
      pass.end();
      device.queue.submit([enc.finish()]);
      let it = 0;
      for (;;) {
        enc = device.createCommandEncoder();
        pass = enc.beginComputePass();
        for (let k = 0; k < chunk; k++) {
          run(pass, P.spmv, G.spmv, nG);
          run(pass, P.dot2, G.dotPAp, nwg);
          run(pass, P.reduce, G.redPAp, 1);
          run(pass, P.up1, G.up1, nG);
          run(pass, P.pre, G.pre, nG);
          run(pass, P.dot2, G.dotRZ, nwg);
          run(pass, P.reduce, G.redRZ, 1);
          run(pass, P.up2, G.up2, nG);
          run(pass, P.shift, G.shift, 1);
        }
        pass.end();
        enc.copyBufferToBuffer(B.scal, 0, read, 0, 32);
        device.queue.submit([enc.finish()]);
        await read.mapAsync(GPUMapMode.READ);
        const sc = new Float32Array(read.getMappedRange().slice(0));
        read.unmap();
        it += chunk;
        const rr = Math.sqrt(Math.abs(sc[3]));
        if (onProgress) onProgress({ outer, it: totalIt + it, rel: rr * relRes });
        if (!(rr > innerTol) || it >= maxInner || totalIt + it >= maxIter || !Number.isFinite(rr)) break;
      }
      totalIt += it;
      enc = device.createCommandEncoder();
      enc.copyBufferToBuffer(B.x, 0, readX, 0, n * 4);
      device.queue.submit([enc.finish()]);
      await readX.mapAsync(GPUMapMode.READ);
      const d = new Float32Array(readX.getMappedRange().slice(0));
      readX.unmap();
      for (let i = 0; i < n; i++) x64[i] += d[i] * rn;
      // f64 residual on the CPU
      csrMul(csr, x64, Ax);
      for (let i = 0; i < n; i++) res[i] = rhs[i] - Ax[i];
      if (totalIt >= maxIter) break;
    }
    const rn = Math.sqrt(res.reduce((s, v) => s + v * v, 0));
    relRes = rn / bnorm;
    for (const b of [...Object.values(B), ...Object.values(U), read, readX]) b.destroy();
    return {
      x: x64,
      method: `WebGPU block-Jacobi PCG (f32) + f64 refinement - ${device.__label}`,
      info: { iterations: totalIt, refinements: outer, relResidual: relRes, ms: performance.now() - t0, nnz: csr.col.length },
    };
  };
}
