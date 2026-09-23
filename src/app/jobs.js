// Promise-based client for the analysis worker (terminates + respawns on cancel).
export class Jobs {
  constructor() {
    this.seq = 0;
    this.pending = new Map();
    this.spawn();
  }
  spawn() {
    this.w = new Worker(new URL('../fea/worker.js', import.meta.url), { type: 'module' });
    this.w.onmessage = (ev) => {
      const { id, type } = ev.data;
      const p = this.pending.get(id);
      if (!p) return;
      if (type === 'progress') p.onProgress && p.onProgress(ev.data);
      else if (type === 'result') { this.pending.delete(id); p.res(ev.data); }
      else if (type === 'error') { this.pending.delete(id); p.rej(new Error(ev.data.error)); }
    };
    this.w.onerror = (e) => {
      for (const p of this.pending.values()) p.rej(new Error(e.message || 'Worker error'));
      this.pending.clear();
    };
  }
  get busy() { return this.pending.size > 0; }
  run(type, data = {}, onProgress) {
    const id = ++this.seq;
    return new Promise((res, rej) => {
      this.pending.set(id, { res, rej, onProgress });
      this.w.postMessage({ id, type, ...data });
    });
  }
  cancel() {
    this.w.terminate();
    for (const p of this.pending.values()) p.rej(new Error('Cancelled'));
    this.pending.clear();
    this.spawn();
  }
}
