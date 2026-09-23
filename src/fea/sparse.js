// Sparse direct solver: reverse Cuthill-McKee node ordering + envelope (profile)
// Cholesky factorisation (Jennings row-oriented scheme). Robust for the
// ill-conditioned shell/beam systems we assemble, and reusable for many
// right-hand sides (load cases, subspace iteration).

/** Reverse Cuthill-McKee ordering. adj: array of Int32Array neighbour lists. Returns perm[newIndex] = oldIndex. */
export function rcm(adj) {
  const n = adj.length;
  const deg = new Int32Array(n);
  for (let i = 0; i < n; i++) deg[i] = adj[i].length;
  const visited = new Uint8Array(n);
  const order = [];
  const bfsLevels = (start, mark) => {
    // returns last level
    const seen = new Uint8Array(n);
    let level = [start];
    seen[start] = 1;
    let last = level;
    while (level.length) {
      last = level;
      const next = [];
      for (const v of level) for (const w of adj[v]) if (!seen[w] && !mark[w]) { seen[w] = 1; next.push(w); }
      level = next;
    }
    return last;
  };
  for (;;) {
    // pick unvisited min-degree node
    let s = -1;
    for (let i = 0; i < n; i++) if (!visited[i] && (s < 0 || deg[i] < deg[s])) s = i;
    if (s < 0) break;
    // pseudo-peripheral node (two sweeps)
    for (let it = 0; it < 2; it++) {
      const last = bfsLevels(s, visited);
      let best = last[0];
      for (const v of last) if (deg[v] < deg[best]) best = v;
      s = best;
    }
    // Cuthill-McKee BFS
    visited[s] = 1;
    const queue = [s];
    for (let q = 0; q < queue.length; q++) {
      const v = queue[q];
      order.push(v);
      const nb = [];
      for (const w of adj[v]) if (!visited[w]) { visited[w] = 1; nb.push(w); }
      nb.sort((a, b) => deg[a] - deg[b]);
      for (const w of nb) queue.push(w);
    }
  }
  order.reverse();
  return Int32Array.from(order);
}

export class EnvelopeMatrix {
  /**
   * @param {number} n  number of equations
   * @param {Int32Array} first first[i] = lowest column index in row i (<= i)
   */
  constructor(n, first) {
    this.n = n;
    this.first = first;
    const ptr = new Float64Array(n + 1); // use float64 to allow > 2^31 safety
    let s = 0;
    for (let i = 0; i < n; i++) { ptr[i] = s; s += i - first[i] + 1; }
    ptr[n] = s;
    this.size = s;
    // base[i] such that index(i,j) = base[i] + j
    this.base = new Int32Array(n); // envelope sizes stay well below 2^31
    for (let i = 0; i < n; i++) this.base[i] = ptr[i] - first[i];
    this.a = new Float64Array(s);
    this.factored = false;
  }
  add(i, j, v) {
    // lower triangle only (caller guarantees j <= i)
    this.a[this.base[i] + j] += v;
  }
  diag(i) { return this.a[this.base[i] + i]; }

  /** In-place Cholesky L L^T. Throws {singular: eq} on loss of positive-definiteness. */
  factor(onProgress) {
    const { n, first, base, a } = this;
    const d0 = new Float64Array(n);
    for (let i = 0; i < n; i++) d0[i] = a[base[i] + i];
    for (let i = 0; i < n; i++) {
      const fi = first[i];
      const bi = base[i];
      for (let j = fi; j < i; j++) {
        const fj = first[j];
        const bj = base[j];
        const k0 = fi > fj ? fi : fj;
        // 4-way unrolled dot product (the hot loop of the whole solver)
        let s0 = 0, s1 = 0, s2 = 0, s3 = 0;
        let k = k0;
        const pi = bi, pj = bj;
        for (const kEnd = j - 3; k < kEnd; k += 4) {
          s0 += a[pi + k] * a[pj + k];
          s1 += a[pi + k + 1] * a[pj + k + 1];
          s2 += a[pi + k + 2] * a[pj + k + 2];
          s3 += a[pi + k + 3] * a[pj + k + 3];
        }
        for (; k < j; k++) s0 += a[pi + k] * a[pj + k];
        a[bi + j] = (a[bi + j] - (s0 + s1 + s2 + s3)) / a[bj + j];
      }
      let s = a[bi + i];
      for (let k = fi; k < i; k++) { const v = a[bi + k]; s -= v * v; }
      if (!(s > 1e-11 * Math.abs(d0[i]))) {
        const err = new Error(`Stiffness matrix singular at equation ${i} (unconstrained mechanism)`);
        err.singular = i;
        throw err;
      }
      a[bi + i] = Math.sqrt(s);
      if (onProgress && (i & 1023) === 0) onProgress(i / n);
    }
    this.factored = true;
  }

  /** Solve (L L^T) x = b. Returns new Float64Array. */
  solve(b) {
    const { n, first, base, a } = this;
    const y = Float64Array.from(b);
    for (let i = 0; i < n; i++) {
      const bi = base[i];
      let s = y[i];
      for (let k = first[i]; k < i; k++) s -= a[bi + k] * y[k];
      y[i] = s / a[bi + i];
    }
    for (let i = n - 1; i >= 0; i--) {
      const bi = base[i];
      const xi = (y[i] /= a[bi + i]);
      for (let k = first[i]; k < i; k++) y[k] -= a[bi + k] * xi;
    }
    return y;
  }
}
