import { Vector3 } from 'three';
import type { StateVector } from '../physics/state';
import type { AccelFn } from './perturbations';

// Adaptive Dormand–Prince 5(4) integrator for spacecraft trajectories.
// Fixed-step leapfrog (physics/nbody.ts) is fine for planets on well-behaved
// orbits, but a spacecraft crosses scales — hours near a planet, months in
// deep space — so the step size must follow the dynamics.

const A: number[][] = [
  [],
  [1 / 5],
  [3 / 40, 9 / 40],
  [44 / 45, -56 / 15, 32 / 9],
  [19372 / 6561, -25360 / 2187, 64448 / 6561, -212 / 729],
  [9017 / 3168, -355 / 33, 46732 / 5247, 49 / 176, -5103 / 18656],
  [35 / 384, 0, 500 / 1113, 125 / 192, -2187 / 6784, 11 / 84],
];
const C = [0, 1 / 5, 3 / 10, 4 / 5, 8 / 9, 1, 1];
const B5 = [35 / 384, 0, 500 / 1113, 125 / 192, -2187 / 6784, 11 / 84, 0];
const B4 = [
  5179 / 57600, 0, 7571 / 16695, 393 / 640, -92097 / 339200, 187 / 2100, 1 / 40,
];

export interface IntegrateOptions {
  /** Relative tolerance per component (default 1e-10). */
  rtol?: number;
  /** Absolute tolerance for position in AU (default 1e-12). */
  atol?: number;
  /** Step size cap in days (also controls output sampling). */
  maxStep?: number;
  /** Floor on the step size in days (default 1e-8 d ≈ 0.9 ms). */
  minStep?: number;
  /** Initial step guess in days; by default scaled from r/v. */
  initialStep?: number;
  /** Hard cap on accepted steps (default 200000, guards runaway runs). */
  maxSteps?: number;
}

export interface TrajectoryPoint {
  t: number;
  pos: Vector3;
  vel: Vector3;
}

/**
 * Integrate a two-body-plus-perturbations trajectory from `initial` at `t0`
 * over `dt` days (may be negative). Returns the state at every accepted step,
 * including t0, so the result doubles as a rendering polyline.
 */
export function integrate(
  initial: StateVector,
  t0: number,
  dt: number,
  accel: AccelFn,
  opts: IntegrateOptions = {},
): TrajectoryPoint[] {
  if (dt === 0) return [{ t: t0, pos: initial.pos.clone(), vel: initial.vel.clone() }];
  const rtol = opts.rtol ?? 1e-10;
  const atol = opts.atol ?? 1e-12;
  const minStep = opts.minStep ?? 1e-8;
  const maxSteps = opts.maxSteps ?? 200000;
  const dir = Math.sign(dt);
  const tEnd = t0 + dt;
  const maxStep = opts.maxStep ?? Math.abs(dt);

  const yPos = initial.pos.clone();
  const yVel = initial.vel.clone();
  const kPos: Vector3[] = Array.from({ length: 7 }, () => new Vector3());
  const kVel: Vector3[] = Array.from({ length: 7 }, () => new Vector3());
  const tmpPos = new Vector3();
  const tmpVel = new Vector3();
  const newPos = new Vector3();
  const newVel = new Vector3();

  let h = opts.initialStep ?? Math.min(maxStep, Math.max(minStep, 0.001 * (yPos.length() / (yVel.length() || 1e-8))));
  h = Math.min(h, maxStep) * dir;

  let t = t0;
  const points: TrajectoryPoint[] = [{ t, pos: yPos.clone(), vel: yVel.clone() }];

  for (let step = 0; step < maxSteps; step++) {
    const remaining = tEnd - t;
    if (Math.abs(h) > Math.abs(remaining)) h = remaining;
    if (Math.abs(h) < minStep) h = Math.min(minStep, Math.abs(remaining)) * dir;

    // Dormand–Prince stages.
    kPos[0].copy(yVel);
    accel(t, yPos, kVel[0]);
    for (let i = 1; i < 7; i++) {
      tmpPos.copy(yPos);
      tmpVel.copy(yVel);
      for (let j = 0; j < i; j++) {
        tmpPos.addScaledVector(kPos[j], h * A[i][j]);
        tmpVel.addScaledVector(kVel[j], h * A[i][j]);
      }
      kPos[i].copy(tmpVel);
      accel(t + C[i] * h, tmpPos, kVel[i]);
    }

    newPos.copy(yPos);
    newVel.copy(yVel);
    for (let i = 0; i < 7; i++) {
      newPos.addScaledVector(kPos[i], h * B5[i]);
      newVel.addScaledVector(kVel[i], h * B5[i]);
    }

    // Error estimate: 5th-order solution minus the embedded 4th-order one.
    let err = 0;
    for (let c = 0; c < 3; c++) {
      let dPos = 0;
      let dVel = 0;
      for (let i = 0; i < 7; i++) {
        const w = B5[i] - B4[i];
        if (w === 0) continue;
        dPos += w * kPos[i].getComponent(c);
        dVel += w * kVel[i].getComponent(c);
      }
      const posScale = atol + rtol * Math.max(Math.abs(yPos.getComponent(c)), Math.abs(newPos.getComponent(c)));
      const velScale = atol + rtol * Math.max(Math.abs(yVel.getComponent(c)), Math.abs(newVel.getComponent(c)));
      err = Math.max(err, Math.abs(h * dPos) / posScale, Math.abs(h * dVel) / velScale);
    }

    if (err <= 1) {
      t += h;
      yPos.copy(newPos);
      yVel.copy(newVel);
      points.push({ t, pos: yPos.clone(), vel: yVel.clone() });
      if (Math.abs(t - tEnd) < 1e-12 * Math.max(1, Math.abs(tEnd))) break;
    }

    const growth = err === 0 ? 5 : Math.min(5, Math.max(0.2, 0.9 * Math.pow(1 / err, 1 / 5)));
    h *= growth;
    if (Math.abs(h) > maxStep) h = maxStep * dir;
  }

  return points;
}
