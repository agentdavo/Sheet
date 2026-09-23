// Analysis worker: meshing, FE solves (CPU direct or WebGPU PCG), structural optimisers.
import { buildChassis } from '../chassis/mesh.js';
import { runAnalyses } from './analysis.js';
import { makeGpuSolver, getDevice } from './gpu.js';
import { optimiseGauges, sweepParameter } from '../optim/structure.js';

let gpuSolver = null;
async function iterativeFor(solver, id) {
  if (solver !== 'gpu') return undefined;
  if (!gpuSolver) gpuSolver = await makeGpuSolver({ onProgress: (p) => post(id, 'progress', { msg: `WebGPU PCG: refinement ${p.outer + 1}, ${p.it} iterations, residual ${p.rel.toExponential(1)}`, p: -1 }) });
  if (!gpuSolver) throw new Error('WebGPU compute is not available in this browser - use the CPU solver.');
  return gpuSolver;
}

function post(id, type, data, transfer) {
  self.postMessage({ id, type, ...data }, transfer || []);
}

self.onmessage = async (ev) => {
  const { id, type, cfg } = ev.data;
  try {
    if (type === 'probe') {
      const dev = await getDevice();
      post(id, 'result', { gpu: !!dev, label: dev?.__label || null });
      return;
    }
    if (type === 'analyse') {
      const mesh = buildChassis(cfg);
      const iterative = await iterativeFor(ev.data.solver, id);
      const res = await runAnalyses(mesh, cfg, ev.data.which, {
        iterative,
        onProgress: (msg, p) => post(id, 'progress', { msg, p }),
      });
      const transfer = [];
      for (const k of ['torsion', 'bending']) {
        if (!res[k]) continue;
        transfer.push(res[k].u.buffer);
      }
      if (res.modal) for (const m of res.modal) transfer.push(m.shape.buffer);
      post(id, 'result', { result: res }, transfer);
      return;
    }
    if (type === 'gauges') {
      const iterative = await iterativeFor(ev.data.solver, id);
      const res = await optimiseGauges(cfg, { ...ev.data.opts, iterative, onProgress: (p) => post(id, 'progress', { opt: p }) });
      post(id, 'result', { result: res });
      return;
    }
    if (type === 'sweep') {
      const iterative = await iterativeFor(ev.data.solver, id);
      const res = await sweepParameter(cfg, ev.data.path, ev.data.values, { iterative, onProgress: (p) => post(id, 'progress', { sweep: p }) });
      post(id, 'result', { result: res });
      return;
    }
    throw new Error(`Unknown job ${type}`);
  } catch (e) {
    post(id, 'error', { error: e.message || String(e) });
  }
};
