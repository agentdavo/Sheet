// Finite elements for thin-walled chassis structures.
//
//  * shellQ4  - flat 4-node shell, 6 dof/node:
//      membrane: bilinear Q4 + Wilson/Taylor incompatible modes (QM6), condensed
//      bending : Mindlin-Reissner plate with MITC4 assumed transverse shear
//                (Bathe-Dvorkin) -> no shear locking, no spurious modes
//      drilling: penalty tying the normal rotation to the in-plane rotation
//  * beam3d  - Euler-Bernoulli space frame element, 6 dof/node.
//
// Local dof order per node: u v w rx ry rz.

const G = 1 / Math.sqrt(3);
const GAUSS = [
  [-G, -G],
  [G, -G],
  [G, G],
  [-G, G],
];
const XI = [-1, 1, 1, -1];
const ETA = [-1, -1, 1, 1];
const KAPPA = 5 / 6;

function shape(xi, eta, N, Nx, Ne) {
  for (let i = 0; i < 4; i++) {
    N[i] = 0.25 * (1 + XI[i] * xi) * (1 + ETA[i] * eta);
    Nx[i] = 0.25 * XI[i] * (1 + ETA[i] * eta);
    Ne[i] = 0.25 * ETA[i] * (1 + XI[i] * xi);
  }
}

/** Local orthonormal frame of a (possibly warped) quad. Returns {R (9, rows e1,e2,e3), xl, yl, area, c} */
export function shellFrame(X) {
  const cx = (X[0] + X[3] + X[6] + X[9]) / 4;
  const cy = (X[1] + X[4] + X[7] + X[10]) / 4;
  const cz = (X[2] + X[5] + X[8] + X[11]) / 4;
  const d1x = X[6] - X[0], d1y = X[7] - X[1], d1z = X[8] - X[2];
  const d2x = X[9] - X[3], d2y = X[10] - X[4], d2z = X[11] - X[5];
  let nx = d1y * d2z - d1z * d2y;
  let ny = d1z * d2x - d1x * d2z;
  let nz = d1x * d2y - d1y * d2x;
  let nl = Math.hypot(nx, ny, nz);
  const area = 0.5 * nl;
  nx /= nl; ny /= nl; nz /= nl;
  // e1 from mid-side (0,3) to mid-side (1,2)
  let ax = (X[3] + X[6] - X[0] - X[9]) / 2;
  let ay = (X[4] + X[7] - X[1] - X[10]) / 2;
  let az = (X[5] + X[8] - X[2] - X[11]) / 2;
  const dn = ax * nx + ay * ny + az * nz;
  ax -= dn * nx; ay -= dn * ny; az -= dn * nz;
  const al = Math.hypot(ax, ay, az);
  ax /= al; ay /= al; az /= al;
  const bx = ny * az - nz * ay;
  const by = nz * ax - nx * az;
  const bz = nx * ay - ny * ax;
  const R = [ax, ay, az, bx, by, bz, nx, ny, nz];
  const xl = new Float64Array(4), yl = new Float64Array(4);
  for (let i = 0; i < 4; i++) {
    const px = X[3 * i] - cx, py = X[3 * i + 1] - cy, pz = X[3 * i + 2] - cz;
    xl[i] = px * ax + py * ay + pz * az;
    yl[i] = px * bx + py * by + pz * bz;
  }
  return { R, xl, yl, area, c: [cx, cy, cz] };
}

function jac(xl, yl, Nx, Ne) {
  let xx = 0, xy = 0, ex = 0, ey = 0;
  for (let i = 0; i < 4; i++) {
    xx += Nx[i] * xl[i]; xy += Nx[i] * yl[i];
    ex += Ne[i] * xl[i]; ey += Ne[i] * yl[i];
  }
  const det = xx * ey - xy * ex;
  // J = [[xx, xy],[ex, ey]] ; J^-1 = 1/det [[ey,-xy],[-ex,xx]]
  return { xx, xy, ex, ey, det, i00: ey / det, i01: -xy / det, i10: -ex / det, i11: xx / det };
}

function inv4(M) {
  // Gauss-Jordan on 4x4 (row-major, length 16)
  const a = Array.from(M);
  const b = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  for (let c = 0; c < 4; c++) {
    let p = c;
    for (let r = c + 1; r < 4; r++) if (Math.abs(a[r * 4 + c]) > Math.abs(a[p * 4 + c])) p = r;
    if (p !== c) for (let k = 0; k < 4; k++) {
      [a[c * 4 + k], a[p * 4 + k]] = [a[p * 4 + k], a[c * 4 + k]];
      [b[c * 4 + k], b[p * 4 + k]] = [b[p * 4 + k], b[c * 4 + k]];
    }
    const d = a[c * 4 + c];
    for (let k = 0; k < 4; k++) { a[c * 4 + k] /= d; b[c * 4 + k] /= d; }
    for (let r = 0; r < 4; r++) if (r !== c) {
      const f = a[r * 4 + c];
      if (f) for (let k = 0; k < 4; k++) { a[r * 4 + k] -= f * a[c * 4 + k]; b[r * 4 + k] -= f * b[c * 4 + k]; }
    }
  }
  return b;
}

const MEM_DOF = [0, 1, 6, 7, 12, 13, 18, 19]; // u,v of nodes 0..3 in 24-vector
const PLT_DOF = [2, 3, 4, 8, 9, 10, 14, 15, 16, 20, 21, 22]; // w,rx,ry

/**
 * Local stiffness parts of a flat shell quad.
 * Returns { Km, Kb, frame } where Km (membrane + drilling) and Kb (bending + shear)
 * are 24x24 row-major Float64Arrays in LOCAL coordinates.
 */
export function shellLocal(X, t, E, nu) {
  const fr = shellFrame(X);
  const { xl, yl } = fr;
  const Km = new Float64Array(576);
  const Kb = new Float64Array(576);
  const N = new Float64Array(4), Nx = new Float64Array(4), Ne = new Float64Array(4);
  const c = E / (1 - nu * nu);
  const Dm = [c * t, c * t * nu, 0, c * t * nu, c * t, 0, 0, 0, c * t * (1 - nu) / 2];
  const db = (E * t * t * t) / (12 * (1 - nu * nu));
  const Db = [db, db * nu, 0, db * nu, db, 0, 0, 0, db * (1 - nu) / 2];
  const ds = KAPPA * (E / (2 * (1 + nu))) * t;
  const gd = 0.01 * (E / (2 * (1 + nu))) * t; // drilling penalty

  // centre Jacobian for incompatible modes
  shape(0, 0, N, Nx, Ne);
  const J0 = jac(xl, yl, Nx, Ne);

  const Kcc = new Float64Array(64), Kci = new Float64Array(32), Kii = new Float64Array(16);
  const Bm = new Float64Array(24), Bi = new Float64Array(12), Bb = new Float64Array(36);

  // MITC4 tying-point covariant shear rows (12 plate dofs each)
  const tie = (xi, eta, dir) => {
    shape(xi, eta, N, Nx, Ne);
    const J = jac(xl, yl, Nx, Ne);
    const row = new Float64Array(12);
    const dN = dir === 0 ? Nx : Ne;
    const gx = dir === 0 ? J.xx : J.ex;
    const gy = dir === 0 ? J.xy : J.ey;
    for (let i = 0; i < 4; i++) {
      row[3 * i] = dN[i];
      row[3 * i + 1] = -N[i] * gy; // rx
      row[3 * i + 2] = N[i] * gx; // ry
    }
    return row;
  };
  const gA = tie(0, 1, 0), gC = tie(0, -1, 0), gB = tie(-1, 0, 1), gD = tie(1, 0, 1);

  for (const [xi, eta] of GAUSS) {
    shape(xi, eta, N, Nx, Ne);
    const J = jac(xl, yl, Nx, Ne);
    const w = J.det;
    // --- membrane
    for (let i = 0; i < 4; i++) {
      const dx = J.i00 * Nx[i] + J.i01 * Ne[i];
      const dy = J.i10 * Nx[i] + J.i11 * Ne[i];
      Bm[0 * 8 + 2 * i] = dx; Bm[0 * 8 + 2 * i + 1] = 0;
      Bm[1 * 8 + 2 * i] = 0; Bm[1 * 8 + 2 * i + 1] = dy;
      Bm[2 * 8 + 2 * i] = dy; Bm[2 * 8 + 2 * i + 1] = dx;
      // bending: kxx = d ry/dx ; kyy = -d rx/dy ; kxy = d ry/dy - d rx/dx
      Bb[0 * 12 + 3 * i] = 0; Bb[0 * 12 + 3 * i + 1] = 0; Bb[0 * 12 + 3 * i + 2] = dx;
      Bb[1 * 12 + 3 * i] = 0; Bb[1 * 12 + 3 * i + 1] = -dy; Bb[1 * 12 + 3 * i + 2] = 0;
      Bb[2 * 12 + 3 * i] = 0; Bb[2 * 12 + 3 * i + 1] = -dx; Bb[2 * 12 + 3 * i + 2] = dy;
    }
    // incompatible modes P1 = 1-xi^2, P2 = 1-eta^2 (Taylor correction via J0)
    const s = J0.det / J.det;
    const p1x = s * (J0.i00 * (-2 * xi)), p1y = s * (J0.i10 * (-2 * xi));
    const p2x = s * (J0.i01 * (-2 * eta)), p2y = s * (J0.i11 * (-2 * eta));
    Bi.fill(0);
    Bi[0 * 4 + 0] = p1x; Bi[1 * 4 + 1] = p1y; Bi[2 * 4 + 0] = p1y; Bi[2 * 4 + 1] = p1x;
    Bi[0 * 4 + 2] = p2x; Bi[1 * 4 + 3] = p2y; Bi[2 * 4 + 2] = p2y; Bi[2 * 4 + 3] = p2x;
    // drilling: penalise (rz - in-plane rotation), rigid-body invariant (Hughes-Brezzi type)
    const rd = new Float64Array(24);
    for (let i = 0; i < 4; i++) {
      const dx = J.i00 * Nx[i] + J.i01 * Ne[i];
      const dy = J.i10 * Nx[i] + J.i11 * Ne[i];
      rd[6 * i + 5] = N[i];
      rd[6 * i] = 0.5 * dy;
      rd[6 * i + 1] = -0.5 * dx;
    }
    for (let a = 0; a < 24; a++) if (rd[a]) for (let b = 0; b < 24; b++) if (rd[b]) Km[a * 24 + b] += gd * w * rd[a] * rd[b];
    addBtDB(Kcc, Bm, Bm, Dm, 3, 8, 8, w);
    addBtDB(Kci, Bm, Bi, Dm, 3, 8, 4, w);
    addBtDB(Kii, Bi, Bi, Dm, 3, 4, 4, w);
    // --- bending
    const Kp = new Float64Array(144);
    addBtDB(Kp, Bb, Bb, Db, 3, 12, 12, w);
    // --- MITC4 shear
    const Bs = new Float64Array(24);
    for (let k = 0; k < 12; k++) {
      const ge = 0.5 * (1 + eta) * gA[k] + 0.5 * (1 - eta) * gC[k];
      const gn = 0.5 * (1 + xi) * gD[k] + 0.5 * (1 - xi) * gB[k];
      Bs[k] = J.i00 * ge + J.i01 * gn;
      Bs[12 + k] = J.i10 * ge + J.i11 * gn;
    }
    for (let a = 0; a < 12; a++) {
      const ga = PLT_DOF[a];
      for (let b = 0; b < 12; b++) {
        const v = Kp[a * 12 + b] + ds * w * (Bs[a] * Bs[b] + Bs[12 + a] * Bs[12 + b]);
        Kb[ga * 24 + PLT_DOF[b]] += v;
      }
    }
  }
  // condense incompatible modes
  const Kiinv = inv4(Kii);
  for (let a = 0; a < 8; a++) {
    for (let b = 0; b < 8; b++) {
      let v = Kcc[a * 8 + b];
      for (let p = 0; p < 4; p++) {
        let q = 0;
        for (let r = 0; r < 4; r++) q += Kiinv[p * 4 + r] * Kci[b * 4 + r];
        v -= Kci[a * 4 + p] * q;
      }
      Km[MEM_DOF[a] * 24 + MEM_DOF[b]] += v;
    }
  }
  return { Km, Kb, frame: fr, Kiinv, Kci };
}

function addBtDB(K, Ba, Bb, D, nr, na, nb, w) {
  // K[na x nb] += Ba^T D Bb * w    (Ba nr x na, Bb nr x nb)
  const DB = new Float64Array(nr * nb);
  for (let i = 0; i < nr; i++)
    for (let j = 0; j < nb; j++) {
      let s = 0;
      for (let k = 0; k < nr; k++) s += D[i * nr + k] * Bb[k * nb + j];
      DB[i * nb + j] = s;
    }
  for (let a = 0; a < na; a++)
    for (let b = 0; b < nb; b++) {
      let s = 0;
      for (let k = 0; k < nr; k++) s += Ba[k * na + a] * DB[k * nb + b];
      K[a * nb + b] += s * w;
    }
}

/** Transform a local matrix (n x n, n = 3*blocks) to global: Kg = T^T Kl T with T = diag(R). */
export function toGlobal(Kl, n, R) {
  const Kg = new Float64Array(n * n);
  const tmp = new Float64Array(n * n);
  const nb = n / 3;
  // tmp = Kl * T   (T block diag, T_ij = R rows)
  for (let i = 0; i < n; i++)
    for (let bj = 0; bj < nb; bj++) {
      const o = 3 * bj;
      const k0 = Kl[i * n + o], k1 = Kl[i * n + o + 1], k2 = Kl[i * n + o + 2];
      for (let c = 0; c < 3; c++) tmp[i * n + o + c] = k0 * R[c] + k1 * R[3 + c] + k2 * R[6 + c];
    }
  // Kg = T^T tmp
  for (let bi = 0; bi < nb; bi++) {
    const o = 3 * bi;
    for (let j = 0; j < n; j++) {
      const t0 = tmp[o * n + j], t1 = tmp[(o + 1) * n + j], t2 = tmp[(o + 2) * n + j];
      for (let r = 0; r < 3; r++) Kg[(o + r) * n + j] = R[r] * t0 + R[3 + r] * t1 + R[6 + r] * t2;
    }
  }
  return Kg;
}

/** Rotate a global vector of 3-blocks into local: ul = T ug */
export function vecToLocal(ug, R) {
  const n = ug.length;
  const ul = new Float64Array(n);
  for (let o = 0; o < n; o += 3) {
    const a = ug[o], b = ug[o + 1], c = ug[o + 2];
    ul[o] = R[0] * a + R[1] * b + R[2] * c;
    ul[o + 1] = R[3] * a + R[4] * b + R[5] * c;
    ul[o + 2] = R[6] * a + R[7] * b + R[8] * c;
  }
  return ul;
}

export function shellGlobal(X, t, E, nu) {
  const L = shellLocal(X, t, E, nu);
  const Kl = new Float64Array(576);
  for (let i = 0; i < 576; i++) Kl[i] = L.Km[i] + L.Kb[i];
  return { K: toGlobal(Kl, 24, L.frame.R), local: L };
}

/**
 * Post-process a shell: returns membrane strains, curvatures (local), top/bottom von Mises
 * and strain-energy split.
 */
export function shellRecover(X, t, E, nu, ug) {
  const L = shellLocal(X, t, E, nu);
  const ul = vecToLocal(ug, L.frame.R);
  let Um = 0, Ub = 0;
  for (let a = 0; a < 24; a++) {
    let sm = 0, sb = 0;
    for (let b = 0; b < 24; b++) { sm += L.Km[a * 24 + b] * ul[b]; sb += L.Kb[a * 24 + b] * ul[b]; }
    Um += 0.5 * ul[a] * sm; Ub += 0.5 * ul[a] * sb;
  }
  // strains at centre
  const N = new Float64Array(4), Nx = new Float64Array(4), Ne = new Float64Array(4);
  shape(0, 0, N, Nx, Ne);
  const J = jac(L.frame.xl, L.frame.yl, Nx, Ne);
  // recover incompatible dofs: alpha = -Kii^-1 Kic u
  const um = new Float64Array(8);
  for (let a = 0; a < 8; a++) um[a] = ul[MEM_DOF[a]];
  const alpha = new Float64Array(4);
  for (let p = 0; p < 4; p++) {
    let s = 0;
    for (let r = 0; r < 4; r++) {
      let q = 0;
      for (let b = 0; b < 8; b++) q += L.Kci[b * 4 + r] * um[b];
      s += L.Kiinv[p * 4 + r] * q;
    }
    alpha[p] = -s;
  }
  let ex = 0, ey = 0, gxy = 0, kx = 0, ky = 0, kxy = 0;
  for (let i = 0; i < 4; i++) {
    const dx = J.i00 * Nx[i] + J.i01 * Ne[i];
    const dy = J.i10 * Nx[i] + J.i11 * Ne[i];
    const u = ul[6 * i], v = ul[6 * i + 1], rx = ul[6 * i + 3], ry = ul[6 * i + 4];
    ex += dx * u; ey += dy * v; gxy += dy * u + dx * v;
    kx += dx * ry; ky += -dy * rx; kxy += dy * ry - dx * rx;
  }
  // incompatible-mode derivatives vanish at the centre (dP/dxi = -2xi = 0) -> nothing to add
  void alpha;
  const c = E / (1 - nu * nu);
  const vm = (e1, e2, g) => {
    const sx = c * (e1 + nu * e2), sy = c * (e2 + nu * e1), txy = c * (1 - nu) / 2 * g;
    return Math.sqrt(sx * sx - sx * sy + sy * sy + 3 * txy * txy);
  };
  const h = t / 2;
  const top = vm(ex + h * kx, ey + h * ky, gxy + h * kxy);
  const bot = vm(ex - h * kx, ey - h * ky, gxy - h * kxy);
  const mem = vm(ex, ey, gxy);
  return { vm: Math.max(top, bot), vmMem: mem, Um, Ub, area: L.frame.area };
}

// ---------------------------------------------------------------- beams

/** Local frame of a beam: ex along axis, ez ~ perpendicular to ex in plane with `up`. */
export function beamFrame(p0, p1, up) {
  let ex = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
  const L = Math.hypot(...ex);
  ex = ex.map((v) => v / L);
  let u = up || [0, 0, 1];
  let d = ex[0] * u[0] + ex[1] * u[1] + ex[2] * u[2];
  if (Math.abs(d) > 0.99) { u = [1, 0, 0]; d = ex[0]; if (Math.abs(d) > 0.99) { u = [0, 1, 0]; d = ex[1]; } }
  let ez = [u[0] - d * ex[0], u[1] - d * ex[1], u[2] - d * ex[2]];
  const zl = Math.hypot(...ez);
  ez = ez.map((v) => v / zl);
  const ey = [ez[1] * ex[2] - ez[2] * ex[1], ez[2] * ex[0] - ez[0] * ex[2], ez[0] * ex[1] - ez[1] * ex[0]];
  return { L, R: [...ex, ...ey, ...ez] };
}

export function beamLocal(L, E, G, A, Iy, Iz, J) {
  const k = new Float64Array(144);
  const set = (i, j, v) => { k[i * 12 + j] = v; k[j * 12 + i] = v; };
  const EA = (E * A) / L, GJ = (G * J) / L;
  set(0, 0, EA); set(6, 6, EA); set(0, 6, -EA);
  set(3, 3, GJ); set(9, 9, GJ); set(3, 9, -GJ);
  const a = (12 * E * Iz) / L ** 3, b = (6 * E * Iz) / L ** 2, c = (4 * E * Iz) / L, d = (2 * E * Iz) / L;
  set(1, 1, a); set(1, 5, b); set(1, 7, -a); set(1, 11, b);
  set(5, 5, c); set(5, 7, -b); set(5, 11, d);
  set(7, 7, a); set(7, 11, -b); set(11, 11, c);
  const e = (12 * E * Iy) / L ** 3, f = (6 * E * Iy) / L ** 2, g = (4 * E * Iy) / L, h = (2 * E * Iy) / L;
  set(2, 2, e); set(2, 4, -f); set(2, 8, -e); set(2, 10, -f);
  set(4, 4, g); set(4, 8, f); set(4, 10, h);
  set(8, 8, e); set(8, 10, f); set(10, 10, g);
  return k;
}

export function beamGlobal(p0, p1, up, sec) {
  const fr = beamFrame(p0, p1, up);
  const Kl = beamLocal(fr.L, sec.E, sec.G, sec.A, sec.Iy, sec.Iz, sec.J);
  return { K: toGlobal(Kl, 12, fr.R), Kl, frame: fr };
}

/** Axial + bending stress estimate for a tube/box beam (max fibre). */
export function beamRecover(p0, p1, up, sec, ug) {
  const fr = beamFrame(p0, p1, up);
  const ul = vecToLocal(ug, fr.R);
  const Kl = beamLocal(fr.L, sec.E, sec.G, sec.A, sec.Iy, sec.Iz, sec.J);
  const f = new Float64Array(12);
  let U = 0;
  for (let a = 0; a < 12; a++) {
    let s = 0;
    for (let b = 0; b < 12; b++) s += Kl[a * 12 + b] * ul[b];
    f[a] = s; U += 0.5 * ul[a] * s;
  }
  const N = -f[0];
  const My = Math.max(Math.abs(f[4]), Math.abs(f[10]));
  const Mz = Math.max(Math.abs(f[5]), Math.abs(f[11]));
  const T = Math.abs(f[3]);
  const c = sec.c || Math.sqrt(sec.Iy / sec.A) * 1.4;
  const sig = Math.abs(N) / sec.A + (Math.hypot(My, Mz) * c) / sec.Iy;
  const tau = (T * c) / sec.J;
  return { vm: Math.sqrt(sig * sig + 3 * tau * tau), U, N };
}
