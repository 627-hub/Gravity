import { Vector3 } from 'three';
import type { StateVector } from '../physics/state';

// Two-body propagation with universal variables: one formulation covers
// elliptic, parabolic and hyperbolic motion, which is exactly what a
// navigation layer needs (transfer arcs, escape asymptotes, flybys).
// State vectors are AU / AU-per-day, mu is AU^3/day^2.

const SERIES_Z = 1e-8;

/** Stumpff C(z) = (1 - cos sqrt z)/z, continued analytically for z <= 0. */
export function stumpffC(z: number): number {
  if (z > SERIES_Z) {
    const s = Math.sqrt(z);
    return (1 - Math.cos(s)) / z;
  }
  if (z < -SERIES_Z) {
    const s = Math.sqrt(-z);
    return (Math.cosh(s) - 1) / -z;
  }
  // Maclaurin series: C(z) = 1/2 - z/24 + z^2/720 - ...
  return 1 / 2 - z / 24 + (z * z) / 720 - (z * z * z) / 40320;
}

/** Stumpff S(z) = (sqrt z - sin sqrt z)/z^(3/2), continued analytically. */
export function stumpffS(z: number): number {
  if (z > SERIES_Z) {
    const s = Math.sqrt(z);
    return (s - Math.sin(s)) / (s * s * s);
  }
  if (z < -SERIES_Z) {
    const s = Math.sqrt(-z);
    return (Math.sinh(s) - s) / (s * s * s);
  }
  // Maclaurin series: S(z) = 1/6 - z/120 + z^2/5040 - ...
  return 1 / 6 - z / 120 + (z * z) / 5040 - (z * z * z) / 362880;
}

/**
 * Solve the universal Kepler equation for the universal anomaly chi:
 *   sqrt(mu) dt = sigma0*chi^2*C(z) + (1 - r0*alpha)*chi^3*S(z) + r0*chi
 * with z = alpha*chi^2. The residual is strictly monotonic in chi, so we
 * bracket the root and then run a safeguarded Newton (falls back to bisection
 * whenever a Newton step leaves the bracket). This is robust for elliptic,
 * parabolic and hyperbolic arcs alike — no fragile case-specific starter
 * needed, and no silent NaN when a starter misbehaves.
 */
function solveChi(r0: number, sigma0: number, alpha: number, dt: number, mu: number): number {
  const sqrtMu = Math.sqrt(mu);
  const target = sqrtMu * dt;

  const residual = (c: number): number => {
    const z = alpha * c * c;
    const C = stumpffC(z);
    const S = stumpffS(z);
    return sigma0 * c * c * C + (1 - r0 * alpha) * c * c * c * S + r0 * c - target;
  };
  const derivative = (c: number): number => {
    const z = alpha * c * c;
    const C = stumpffC(z);
    const S = stumpffS(z);
    return sigma0 * c * (1 - z * S) + (1 - r0 * alpha) * c * c * C + r0;
  };

  // Bracket the root: residual(0) = -sqrt(mu)*dt, monotonic in chi.
  let lo: number;
  let hi: number;
  if (dt >= 0) {
    lo = 0;
    hi = Math.max(1e-9, target / r0);
    while (residual(hi) < 0 && hi < 1e15) hi *= 2;
  } else {
    hi = 0;
    lo = Math.min(-1e-9, target / r0);
    while (residual(lo) > 0 && lo > -1e15) lo *= 2;
  }

  // Fast starter (Curtis) for the elliptic case; otherwise start mid-bracket.
  let x = alpha > 1e-12 ? Math.min(Math.max(sqrtMu * dt * alpha, lo), hi) : 0.5 * (lo + hi);
  for (let iter = 0; iter < 200; iter++) {
    const f = residual(x);
    if (f > 0) hi = x;
    else lo = x;
    const dF = derivative(x);
    let next = x - f / dF;
    if (!(next > lo && next < hi)) next = 0.5 * (lo + hi);
    if (Math.abs(next - x) < 1e-14 * Math.max(1, Math.abs(next))) return next;
    x = next;
  }
  return x;
}

/**
 * Propagate a two-body state by `dt` days around a central body with
 * standard gravitational parameter `mu` (AU^3/day^2). Exact for pure
 * Keplerian motion; no numerical integration error accumulates.
 */
export function propagate(state: StateVector, dt: number, mu: number): StateVector {
  const r0Vec = state.pos;
  const v0Vec = state.vel;
  const r0 = r0Vec.length();
  const v0 = v0Vec.length();
  const sqrtMu = Math.sqrt(mu);
  const sigma0 = r0Vec.dot(v0Vec) / sqrtMu; // Bate-Mueller-White sigma
  const alpha = 2 / r0 - (v0 * v0) / mu; // 1/a

  const chi = solveChi(r0, sigma0, alpha, dt, mu);
  const z = alpha * chi * chi;
  const C = stumpffC(z);
  const S = stumpffS(z);

  const f = 1 - (chi * chi * C) / r0;
  const g = dt - (chi * chi * chi * S) / sqrtMu;
  const pos = new Vector3()
    .copy(r0Vec)
    .multiplyScalar(f)
    .addScaledVector(v0Vec, g);
  const r = pos.length();

  const fdot = (sqrtMu / (r0 * r)) * chi * (z * S - 1);
  const gdot = 1 - (chi * chi * C) / r;
  const vel = new Vector3()
    .copy(r0Vec)
    .multiplyScalar(fdot)
    .addScaledVector(v0Vec, gdot);

  return { pos, vel };
}

/** Specific orbital energy, AU^2/day^2 (negative = bound). */
export function specificEnergy(state: StateVector, mu: number): number {
  return state.vel.lengthSq() / 2 - mu / state.pos.length();
}

/** Specific angular momentum vector, AU^2/day. */
export function angularMomentum(state: StateVector): Vector3 {
  return new Vector3().crossVectors(state.pos, state.vel);
}
