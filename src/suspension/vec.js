export const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
export const len = (a) => Math.hypot(a[0], a[1], a[2]);
export const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
export const norm = (a) => { const l = len(a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };
export const mirrorY = (a) => [a[0], -a[1], a[2]];
export const deg = (r) => (r * 180) / Math.PI;
export const rad = (d) => (d * Math.PI) / 180;

/** Rotate vector v about unit axis k by angle t (Rodrigues). */
export function rotVec(v, k, t) {
  const c = Math.cos(t), s = Math.sin(t);
  const kxv = cross(k, v);
  const kdv = dot(k, v) * (1 - c);
  return [v[0] * c + kxv[0] * s + k[0] * kdv, v[1] * c + kxv[1] * s + k[1] * kdv, v[2] * c + kxv[2] * s + k[2] * kdv];
}

/** Rotate point p about the axis through a with unit direction k. */
export function rotAbout(p, a, k, t) {
  return add(a, rotVec(sub(p, a), k, t));
}

/** 3x3 rotation matrix (row-major) taking unit vector a onto unit vector b (minimal rotation). */
export function alignRot(a, b) {
  const v = cross(a, b);
  const c = dot(a, b);
  const s2 = dot(v, v);
  if (s2 < 1e-20) return c > 0 ? [1, 0, 0, 0, 1, 0, 0, 0, 1] : [-1, 0, 0, 0, -1, 0, 0, 0, 1];
  const f = (1 - c) / s2;
  const [x, y, z] = v;
  return [
    1 - f * (y * y + z * z), -z + f * x * y, y + f * x * z,
    z + f * x * y, 1 - f * (x * x + z * z), -x + f * y * z,
    -y + f * x * z, x + f * y * z, 1 - f * (x * x + y * y),
  ];
}
export const matVec = (m, v) => [
  m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
  m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
  m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
];

/** 1-D root find: secant with bisection fallback. f(x0), f(x1) need not bracket. */
export function solve1D(f, x0, x1, tol = 1e-10, maxIt = 60) {
  let f0 = f(x0), f1 = f(x1);
  for (let i = 0; i < maxIt; i++) {
    if (Math.abs(f1) < tol) return x1;
    const d = f1 - f0;
    let x2 = Math.abs(d) < 1e-300 ? x1 + 1e-6 : x1 - (f1 * (x1 - x0)) / d;
    if (!Number.isFinite(x2)) x2 = x1 + 1e-4;
    x0 = x1; f0 = f1;
    x1 = x2; f1 = f(x1);
  }
  return Math.abs(f1) < 1e-4 ? x1 : NaN;
}

/** Intersection of two 2-D lines given as point + direction; returns null if parallel. */
export function intersect2D(p1, d1, p2, d2) {
  const den = d1[0] * d2[1] - d1[1] * d2[0];
  if (Math.abs(den) < 1e-12) return null;
  const t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / den;
  return [p1[0] + t * d1[0], p1[1] + t * d1[1]];
}
