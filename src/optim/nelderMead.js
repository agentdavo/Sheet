/**
 * Bounded Nelder-Mead simplex minimiser working in a normalised [0,1]^n box.
 * f(x) receives physical variables; lo/hi bound each one.
 * Supports cooperative async execution (yields every `yieldEvery` evaluations).
 */
export async function nelderMead(f, x0, lo, hi, opts = {}) {
  const n = x0.length;
  const { maxEval = 400, tol = 1e-7, step = 0.15, onIter, yieldEvery = 25, shouldStop } = opts;
  const toPhys = (u) => u.map((v, i) => lo[i] + Math.min(1, Math.max(0, v)) * (hi[i] - lo[i]));
  const toUnit = (x) => x.map((v, i) => (hi[i] > lo[i] ? (v - lo[i]) / (hi[i] - lo[i]) : 0.5));
  let evals = 0;
  const F = async (u) => {
    evals++;
    if (yieldEvery && evals % yieldEvery === 0) await new Promise((r) => setTimeout(r, 0));
    const v = f(toPhys(u));
    return Number.isFinite(v) ? v : 1e12;
  };
  const u0 = toUnit(x0);
  let simplex = [u0];
  for (let i = 0; i < n; i++) {
    const u = u0.slice();
    u[i] = u[i] + step <= 1 ? u[i] + step : u[i] - step;
    simplex.push(u);
  }
  let fv = [];
  for (const u of simplex) fv.push(await F(u));
  const history = [];
  while (evals < maxEval) {
    const ord = fv.map((v, i) => [v, i]).sort((a, b) => a[0] - b[0]).map((p) => p[1]);
    simplex = ord.map((i) => simplex[i]);
    fv = ord.map((i) => fv[i]);
    history.push(fv[0]);
    if (onIter) onIter({ evals, best: fv[0], x: toPhys(simplex[0]) });
    if (shouldStop && shouldStop()) break;
    if (Math.abs(fv[n] - fv[0]) < tol * (Math.abs(fv[0]) + 1e-12) && evals > 4 * n) break;
    const c = new Array(n).fill(0);
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) c[j] += simplex[i][j] / n;
    const pt = (a) => c.map((cj, j) => cj + a * (simplex[n][j] - cj));
    const xr = pt(-1).map((v) => Math.min(1, Math.max(0, v)));
    const fr = await F(xr);
    if (fr < fv[0]) {
      const xe = pt(-2).map((v) => Math.min(1, Math.max(0, v)));
      const fe = await F(xe);
      if (fe < fr) { simplex[n] = xe; fv[n] = fe; } else { simplex[n] = xr; fv[n] = fr; }
    } else if (fr < fv[n - 1]) {
      simplex[n] = xr; fv[n] = fr;
    } else {
      const xc = fr < fv[n] ? pt(-0.5) : pt(0.5);
      const fc = await F(xc);
      if (fc < Math.min(fr, fv[n])) { simplex[n] = xc; fv[n] = fc; } else {
        for (let i = 1; i <= n; i++) {
          simplex[i] = simplex[i].map((v, j) => simplex[0][j] + 0.5 * (v - simplex[0][j]));
          fv[i] = await F(simplex[i]);
        }
      }
    }
  }
  let bi = 0;
  for (let i = 1; i < fv.length; i++) if (fv[i] < fv[bi]) bi = i;
  return { x: toPhys(simplex[bi]), f: fv[bi], evals, history };
}
