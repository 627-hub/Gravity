import { Vector3 } from 'three';
import { MU_SUN } from './units';

// Lambert's problem: given two position vectors and a time of flight, find the
// connecting two-body arc. This is the core primitive of any trajectory planner
// (transfer windows, rendezvous, flyby targeting).
//
// Solver: Izzo's 2014 reformulation (Householder iteration in the x variable),
// ported from the reference algorithm in poliastro (BSD-3-Clause). It is
// robust near the 180° singularity, handles hyperbolic arcs and arbitrary
// numbers of complete revolutions, and needs no initial guess from the caller.

export interface LambertOptions {
  /** Complete revolutions on the transfer arc: 0 (default), 1, 2, ... */
  revs?: number;
  /** Direction of motion. Prograde (default) favours the reference +z normal. */
  prograde?: boolean;
  /** Multi-rev branch: true picks the larger x (slower "left" arc), false (default) the right arc. */
  lowpath?: boolean;
  /** Householder iteration cap and absolute tolerance on x. */
  maxIter?: number;
  tol?: number;
}

export interface LambertSolution {
  /** Velocity required at r1, AU/day. */
  v1: Vector3;
  /** Velocity on arrival at r2, AU/day. */
  v2: Vector3;
}

const PI = Math.PI;

/** Hypergeometric 2F1(3, 1, 5/2, x) by its Maclaurin series (Battin). */
export function hyp2f1b(x: number): number {
  if (x >= 1) return Infinity;
  let res = 1;
  let term = 1;
  for (let ii = 0; ii < 10000; ii++) {
    term = (term * (3 + ii) * (1 + ii)) / (2.5 + ii) * (x / (ii + 1));
    const old = res;
    res += term;
    if (old === res) return res;
  }
  return res;
}

function yOf(x: number, lambda: number): number {
  return Math.sqrt(1 - lambda * lambda * (1 - x * x));
}

/** Auxiliary angle psi of Izzo's formulation (Eq. 17). */
function psiOf(x: number, y: number, lambda: number): number {
  if (x < 1) {
    return Math.acos(Math.min(1, Math.max(-1, x * y + lambda * (1 - x * x))));
  }
  if (x > 1) {
    return Math.asinh((y - x * lambda) * Math.sqrt(x * x - 1));
  }
  return 0;
}

/** Residual T(x) - T0 of the normalized time-of-flight equation. */
function tofResidual(x: number, y: number, T0: number, lambda: number, revs: number): number {
  // Near-parabolic x the closed form suffers cancellation; use the series
  // form from the paper (valid for M = 0 only).
  if (revs === 0 && x > Math.sqrt(0.6) && x < Math.sqrt(1.4)) {
    const eta = y - lambda * x;
    const s1 = (1 - lambda - x * eta) * 0.5;
    const q = (4 / 3) * hyp2f1b(s1);
    return ((eta * eta * eta * q + 4 * lambda * eta) * 0.5) - T0;
  }
  const psi = psiOf(x, y, lambda);
  return (
    (psi + revs * PI) / Math.sqrt(Math.abs(1 - x * x)) - x + lambda * y
  ) / (1 - x * x) - T0;
}

/** Normalized time of flight T(x). */
function tofValue(x: number, lambda: number, revs: number): number {
  return tofResidual(x, yOf(x, lambda), 0, lambda, revs);
}

function dTdx(x: number, y: number, T: number, lambda: number): number {
  return (3 * T * x - 2 + (2 * lambda ** 3 * x) / y) / (1 - x * x);
}

function d2Tdx2(x: number, y: number, T: number, dT: number, lambda: number): number {
  return (3 * T + 5 * x * dT + (2 * (1 - lambda * lambda) * lambda ** 3) / (y ** 3)) / (1 - x * x);
}

function d3Tdx3(x: number, y: number, dT: number, ddT: number, lambda: number): number {
  return (7 * x * ddT + 8 * dT - (6 * (1 - lambda * lambda) * lambda ** 5 * x) / (y ** 5)) / (1 - x * x);
}

/** Location and value of the minimum of T(x) on x in (-1, 1) for M >= 1 revs. */
function computeTMin(lambda: number, revs: number, maxIter: number, tol: number): { xMin: number; tMin: number } {
  if (revs === 0) return { xMin: Infinity, tMin: 0 };
  // Halley iteration on dT/dx = 0, starting from x = 0.1.
  let x = 0.1;
  const T0 = tofValue(x, lambda, revs);
  for (let iter = 0; iter < maxIter; iter++) {
    const y = yOf(x, lambda);
    const dT = dTdx(x, y, T0, lambda);
    const ddT = d2Tdx2(x, y, T0, dT, lambda);
    const dddT = d3Tdx3(x, y, dT, ddT, lambda);
    const p = x - (2 * dT * ddT) / (2 * ddT * ddT - dT * dddT);
    if (Math.abs(p - x) < tol) {
      x = p;
      break;
    }
    x = p;
  }
  return { xMin: x, tMin: tofValue(x, lambda, revs) };
}

/** Izzo's piecewise initial guess for the Householder iteration. */
function initialGuess(T: number, lambda: number, revs: number, lowpath: boolean): number {
  if (revs === 0) {
    const T0 = Math.acos(lambda) + lambda * Math.sqrt(1 - lambda * lambda);
    const T1 = (2 * (1 - lambda ** 3)) / 3;
    if (T >= T0) return (T0 / T) ** (2 / 3) - 1;
    if (T < T1) return (5 / 2) * (T1 / T) * ((T1 - T) / (1 - lambda ** 5)) + 1;
    return Math.exp((Math.log(2) * Math.log(T / T0)) / Math.log(T1 / T0)) - 1;
  }
  const a = ((revs * PI + PI) / (8 * T)) ** (2 / 3);
  const b = ((8 * T) / (revs * PI)) ** (2 / 3);
  const x0l = (a - 1) / (a + 1);
  const x0r = (b - 1) / (b + 1);
  return lowpath ? Math.max(x0l, x0r) : Math.min(x0l, x0r);
}

/** Find x such that T(x) = T, with feasibility checks against the rev budget. */
function findX(lambda: number, T: number, revs: number, lowpath: boolean, maxIter: number, tol: number): number {
  const t00 = Math.acos(lambda) + lambda * Math.sqrt(1 - lambda * lambda);
  let mMax = Math.floor(T / PI);
  if (T < t00 + mMax * PI && mMax > 0) {
    const { tMin } = computeTMin(lambda, mMax, maxIter, tol);
    if (T < tMin) mMax -= 1;
  }
  if (revs > mMax) {
    throw new Error(`Lambert: no feasible solution for revs=${revs} (max ${mMax})`);
  }

  let x = initialGuess(T, lambda, revs, lowpath);
  for (let iter = 0; iter < maxIter; iter++) {
    const y = yOf(x, lambda);
    const fval = tofResidual(x, y, T, lambda, revs);
    const tVal = fval + T;
    const dT = dTdx(x, y, tVal, lambda);
    const ddT = d2Tdx2(x, y, tVal, dT, lambda);
    const dddT = d3Tdx3(x, y, dT, ddT, lambda);
    const denominator = dT * (dT * dT - fval * ddT) + (dddT * fval * fval) / 6;
    const p = x - fval * ((dT * dT - (fval * ddT) / 2) / denominator);
    if (!Number.isFinite(p)) break;
    if (Math.abs(p - x) < tol) return p;
    x = p;
  }
  throw new Error('Lambert: Householder iteration did not converge');
}

/**
 * Solve Lambert's problem between `r1` and `r2` (AU, same frame) with time of
 * flight `tof` days and gravitational parameter `mu` (AU^3/day^2).
 */
export function lambert(
  r1: Vector3,
  r2: Vector3,
  tof: number,
  mu: number = MU_SUN,
  opts: LambertOptions = {},
): LambertSolution {
  const { revs = 0, prograde = true, lowpath = false, maxIter = 60, tol = 1e-12 } = opts;
  if (!(tof > 0)) throw new Error('Lambert: time of flight must be positive');

  const chord = new Vector3().subVectors(r2, r1);
  const cNorm = chord.length();
  const r1Norm = r1.length();
  const r2Norm = r2.length();
  const s = (r1Norm + r2Norm + cNorm) / 2;

  const ir1 = r1.clone().divideScalar(r1Norm);
  const ir2 = r2.clone().divideScalar(r2Norm);
  const ih = new Vector3().crossVectors(ir1, ir2);
  const ihNorm = ih.length();
  if (ihNorm < 1e-12) throw new Error('Lambert: r1 and r2 are collinear');
  ih.divideScalar(ihNorm);

  // Geometry: lambda encodes the transfer angle, its sign the sweep direction.
  let lambda = Math.sqrt(1 - Math.min(1, cNorm / s));
  if (lambda >= 1 - 1e-12) throw new Error('Lambert: degenerate (r1 ≈ r2)');
  let it1: Vector3;
  let it2: Vector3;
  if (ih.z < 0) {
    lambda = -lambda;
    it1 = new Vector3().crossVectors(ir1, ih);
    it2 = new Vector3().crossVectors(ir2, ih);
  } else {
    it1 = new Vector3().crossVectors(ih, ir1);
    it2 = new Vector3().crossVectors(ih, ir2);
  }
  if (!prograde) {
    lambda = -lambda;
    it1.negate();
    it2.negate();
  }

  const T = Math.sqrt((2 * mu) / (s * s * s)) * tof;
  const x = findX(lambda, T, revs, lowpath, maxIter, tol);
  const y = yOf(x, lambda);

  // Reconstruct the radial and tangential velocity components.
  const gamma = Math.sqrt((mu * s) / 2);
  const rho = (r1Norm - r2Norm) / cNorm;
  const sigma = Math.sqrt(1 - rho * rho);
  const vr1 = (gamma * ((lambda * y - x) - rho * (lambda * y + x))) / r1Norm;
  const vr2 = (-gamma * ((lambda * y - x) + rho * (lambda * y + x))) / r2Norm;
  const vt1 = (gamma * sigma * (y + lambda * x)) / r1Norm;
  const vt2 = (gamma * sigma * (y + lambda * x)) / r2Norm;

  const v1 = ir1.multiplyScalar(vr1).addScaledVector(it1, vt1);
  const v2 = ir2.multiplyScalar(vr2).addScaledVector(it2, vt2);
  return { v1, v2 };
}
