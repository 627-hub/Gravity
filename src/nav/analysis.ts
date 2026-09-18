import { Vector3 } from 'three';
import type { TrajectoryPoint } from './integrate';

// Post-flight trajectory analysis: how close did we get, did we hit anything.
// Between integration samples the trajectory is reconstructed with cubic
// Hermite interpolation (we have both position and velocity at every sample),
// so events are located well below the step size.

export interface ApproachEvent {
  /** Time of the event, days since J2000. */
  t: number;
  /** Separation from the body centre at that time, AU. */
  distance: number;
}

/** Cubic Hermite interpolation of the spacecraft position on [a, b]. */
export function hermitePos(a: TrajectoryPoint, b: TrajectoryPoint, t: number): Vector3 {
  const h = b.t - a.t;
  const s = (t - a.t) / h;
  const s2 = s * s;
  const s3 = s2 * s;
  return new Vector3()
    .copy(a.pos)
    .multiplyScalar(2 * s3 - 3 * s2 + 1)
    .addScaledVector(a.vel, h * (s3 - 2 * s2 + s))
    .addScaledVector(b.pos, -2 * s3 + 3 * s2)
    .addScaledVector(b.vel, h * (s3 - s2));
}

/** Linear interpolation of a moving body between two samples. */
function bodyBetween(
  a: TrajectoryPoint,
  b: TrajectoryPoint,
  bodyA: Vector3,
  bodyB: Vector3,
  t: number,
  out: Vector3,
): Vector3 {
  return out.copy(bodyA).lerp(bodyB, (t - a.t) / (b.t - a.t));
}

/**
 * Minimum separation from a moving body over a sampled trajectory, refined by
 * a golden-section search on the Hermite-interpolated path.
 */
export function closestApproach(
  traj: TrajectoryPoint[],
  bodyPositionAt: (t: number) => Vector3,
  opts: { refine?: boolean } = {},
): ApproachEvent {
  if (traj.length === 0) throw new Error('closestApproach: empty trajectory');

  let best = 0;
  let bestD = Infinity;
  for (let i = 0; i < traj.length; i++) {
    const d = traj[i].pos.distanceTo(bodyPositionAt(traj[i].t));
    if (d < bestD) {
      bestD = d;
      best = i;
    }
  }
  if (opts.refine === false || traj.length < 2) return { t: traj[best].t, distance: bestD };

  const i0 = Math.max(0, best - 1);
  const i1 = Math.min(traj.length - 1, best + 1);
  const a = traj[i0];
  const b = traj[i1];
  const bodyA = bodyPositionAt(a.t);
  const bodyB = bodyPositionAt(b.t);
  const tmp = new Vector3();
  const f = (t: number) =>
    hermitePos(a, b, t).distanceTo(bodyBetween(a, b, bodyA, bodyB, t, tmp));

  // Golden-section search for the minimum on [a.t, b.t].
  const phi = (Math.sqrt(5) - 1) / 2;
  let lo = a.t;
  let hi = b.t;
  let c = hi - phi * (hi - lo);
  let d = lo + phi * (hi - lo);
  let fc = f(c);
  let fd = f(d);
  for (let k = 0; k < 90; k++) {
    if (fc < fd) {
      hi = d;
      d = c;
      fd = fc;
      c = hi - phi * (hi - lo);
      fc = f(c);
    } else {
      lo = c;
      c = d;
      fc = fd;
      d = lo + phi * (hi - lo);
      fd = f(d);
    }
  }
  const t = (lo + hi) / 2;
  return { t, distance: f(t) };
}

/**
 * First time the trajectory enters a body's impact sphere of radius `radius`
 * (AU), or null if it never does.
 */
export function firstImpact(
  traj: TrajectoryPoint[],
  bodyPositionAt: (t: number) => Vector3,
  radius: number,
): ApproachEvent | null {
  if (traj.length === 0) return null;
  if (traj[0].pos.distanceTo(bodyPositionAt(traj[0].t)) < radius) {
    return { t: traj[0].t, distance: traj[0].pos.distanceTo(bodyPositionAt(traj[0].t)) };
  }

  for (let i = 1; i < traj.length; i++) {
    const a = traj[i - 1];
    const b = traj[i];
    if (b.pos.distanceTo(bodyPositionAt(b.t)) >= radius) continue;

    const bodyA = bodyPositionAt(a.t);
    const bodyB = bodyPositionAt(b.t);
    const tmp = new Vector3();
    const g = (t: number) =>
      hermitePos(a, b, t).distanceTo(bodyBetween(a, b, bodyA, bodyB, t, tmp)) - radius;
    let lo = a.t;
    let hi = b.t;
    for (let k = 0; k < 80; k++) {
      const mid = 0.5 * (lo + hi);
      if (g(mid) > 0) lo = mid;
      else hi = mid;
    }
    return { t: 0.5 * (lo + hi), distance: radius };
  }
  return null;
}
