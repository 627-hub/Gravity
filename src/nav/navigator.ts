import { Vector3 } from 'three';
import type { StateVector } from '../physics/state';
import { AU_KM } from '../data/constants';
import { propagate } from './propagate';
import { MPS_TO_AUDAY, MU_SUN } from './units';

// L1 navigation: a sequential EKF that estimates the spacecraft state from
// noisy range / range-rate tracking. The spacecraft never sees truth:
// manoeuvres are computed from this estimate.
//
// Covariance model: the filter keeps a DIAGONAL covariance (per-axis
// variances) and propagates it conservatively (position gains the full
// velocity term as if the position-velocity correlation were ±1). A full
// 6x6 covariance carries the true correlations and is information-optimal,
// but in double precision it repeatedly collapses into rank deficiency, and
// null-space roundoff then amplifies measurement noise into the state. The
// conservative diagonal form is slightly pessimistic (slower convergence,
// never overconfident) and numerically bulletproof — the right trade for
// this simulator. A square-root (UD) filter is the textbook upgrade path.

export interface TrackingNoise {
  /** 1-sigma range noise, AU. */
  rangeSigma: number;
  /** 1-sigma range-rate noise, AU/day. */
  rangeRateSigma: number;
  /** 1-sigma optical (onboard camera) bearing noise, radians. */
  opticalSigma: number;
}

export interface TrackingTier {
  id: 'high' | 'medium' | 'low';
  label: string;
  /** 1-sigma range noise, km. */
  rangeKm: number;
  /** 1-sigma range-rate noise, m/s. */
  rateMps: number;
  /** 1-sigma onboard optical bearing noise, arcseconds. */
  optArcsec: number;
}

export const TRACKING_TIERS: TrackingTier[] = [
  { id: 'high', label: '高精度', rangeKm: 10, rateMps: 0.5, optArcsec: 0.5 },
  { id: 'medium', label: '中精度', rangeKm: 300, rateMps: 5, optArcsec: 5 },
  { id: 'low', label: '低精度', rangeKm: 5000, rateMps: 50, optArcsec: 20 },
];

const ARCSEC = Math.PI / 180 / 3600;

export function tierNoise(tier: TrackingTier): TrackingNoise {
  return {
    rangeSigma: tier.rangeKm / AU_KM,
    rangeRateSigma: tier.rateMps * MPS_TO_AUDAY,
    opticalSigma: tier.optArcsec * ARCSEC,
  };
}

// ---- deterministic noise -------------------------------------------------

/** Small fast PRNG so every mission replays identically. */
export function mulberry32(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** Standard normal sample (Box–Muller) from a uniform PRNG. */
export function gaussian(rng: () => number): number {
  let u = 0;
  let v = 0;
  while (u === 0) u = rng();
  while (v === 0) v = rng();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

// ---- measurement model ---------------------------------------------------

export interface RangeMeasurement {
  /** Measurement epoch, days since J2000. */
  t: number;
  /** Station-to-craft distance, AU. */
  range: number;
  /** d(range)/dt, AU/day. */
  rangeRate: number;
}

/** Noiseless range and range-rate of a craft state seen from a station state. */
export function rangeAndRangeRate(
  craft: StateVector,
  station: StateVector,
): { range: number; rangeRate: number } {
  const d = new Vector3().subVectors(craft.pos, station.pos);
  const dv = new Vector3().subVectors(craft.vel, station.vel);
  const range = d.length();
  return { range, rangeRate: d.dot(dv) / range };
}

/** A noisy tracking sample generated from the truth state. */
export function makeMeasurement(
  truth: StateVector,
  station: StateVector,
  t: number,
  noise: TrackingNoise,
  rng: () => number,
): RangeMeasurement {
  const clean = rangeAndRangeRate(truth, station);
  return {
    t,
    range: clean.range + gaussian(rng) * noise.rangeSigma,
    rangeRate: clean.rangeRate + gaussian(rng) * noise.rangeRateSigma,
  };
}

export interface OpticalMeasurement {
  /** Measurement epoch, days since J2000. */
  t: number;
  /** Measured unit direction from the craft to the beacon (target body). */
  dir: Vector3;
}

/** Orthonormal transverse basis of a unit vector. */
export function transverseBasis(n: Vector3): [Vector3, Vector3] {
  const ref = Math.abs(n.x) < 0.9 ? new Vector3(1, 0, 0) : new Vector3(0, 1, 0);
  const e1 = new Vector3().crossVectors(n, ref);
  if (e1.lengthSq() < 1e-12) e1.crossVectors(n, new Vector3(0, 0, 1));
  e1.normalize();
  const e2 = new Vector3().crossVectors(n, e1);
  return [e1, e2];
}

/** A noisy onboard bearing to the beacon, generated from the truth state. */
export function makeOpticalMeasurement(
  truth: StateVector,
  beacon: StateVector,
  t: number,
  noise: TrackingNoise,
  rng: () => number,
): OpticalMeasurement {
  const dir = new Vector3().subVectors(beacon.pos, truth.pos).normalize();
  const [e1, e2] = transverseBasis(dir);
  dir
    .addScaledVector(e1, gaussian(rng) * noise.opticalSigma)
    .addScaledVector(e2, gaussian(rng) * noise.opticalSigma)
    .normalize();
  return { t, dir };
}

// ---- the filter ----------------------------------------------------------

export interface NavigatorInit {
  noise: TrackingNoise;
  /** 1-sigma a-priori position uncertainty, AU. */
  posSigma0: number;
  /** 1-sigma a-priori velocity uncertainty, AU/day. */
  velSigma0: number;
  /** Process-noise acceleration spectral density, AU/day^2. */
  accelNoise?: number;
}

const N = 6;
/** Numerical floors: (1.5 km)^2 and (0.02 mm/s)^2, in AU units. */
const POS_VARIANCE_FLOOR = 1e-16;
const VEL_VARIANCE_FLOOR = 1e-20;

/** Sequential EKF over [x, y, z, vx, vy, vz] with a diagonal covariance. */
export class Navigator {
  measurementCount = 0;

  private x = new Float64Array(N);
  private p = new Float64Array(N); // per-axis variances
  private noise: TrackingNoise;
  private accelNoise: number;
  private t: number;

  constructor(initial: StateVector, t0: number, init: NavigatorInit) {
    this.x[0] = initial.pos.x;
    this.x[1] = initial.pos.y;
    this.x[2] = initial.pos.z;
    this.x[3] = initial.vel.x;
    this.x[4] = initial.vel.y;
    this.x[5] = initial.vel.z;
    for (let i = 0; i < 3; i++) this.p[i] = init.posSigma0 * init.posSigma0;
    for (let i = 3; i < N; i++) this.p[i] = init.velSigma0 * init.velSigma0;
    this.noise = init.noise;
    this.accelNoise = init.accelNoise ?? 3e-8;
    this.t = t0;
  }

  /** Current estimate (heliocentric ecliptic). */
  state(): StateVector {
    return {
      pos: new Vector3(this.x[0], this.x[1], this.x[2]),
      vel: new Vector3(this.x[3], this.x[4], this.x[5]),
    };
  }

  /** Current epoch of the estimate. */
  epoch(): number {
    return this.t;
  }

  /** Total position uncertainty (1-sigma, AU): sqrt of the summed variances. */
  positionSigma(): number {
    return Math.sqrt(this.p[0] + this.p[1] + this.p[2]);
  }

  velocitySigma(): number {
    return Math.sqrt(this.p[3] + this.p[4] + this.p[5]);
  }

  /** Propagate the estimate and its (conservative) covariance to `t`. */
  propagateTo(t: number): void {
    const dt = t - this.t;
    if (dt <= 0) return;
    const next = propagate(this.state(), dt, MU_SUN);
    const qPos = 0.5 * this.accelNoise * dt * dt;
    const qVel = this.accelNoise * dt;
    for (let i = 0; i < 3; i++) {
      const posVar = this.p[i];
      const velVar = this.p[i + 3];
      // Assume a +0.5 position-velocity correlation: slightly pessimistic
      // (never overconfident) yet tight enough to stay useful.
      this.p[i] =
        posVar + Math.sqrt(posVar * velVar) * dt + velVar * dt * dt + qPos * qPos;
      this.p[i + 3] = velVar + qVel * qVel;
    }
    this.x[0] = next.pos.x;
    this.x[1] = next.pos.y;
    this.x[2] = next.pos.z;
    this.x[3] = next.vel.x;
    this.x[4] = next.vel.y;
    this.x[5] = next.vel.z;
    this.t = t;
  }

  /** Fold one range/range-rate measurement into the estimate. */
  update(z: RangeMeasurement, station: StateVector): void {
    this.rangeUpdate(z.range, station);
    this.rangeRateUpdate(z.rangeRate, station);
    this.measurementCount += 1;
  }

  private rangeUpdate(zRange: number, station: StateVector): void {
    const d = new Vector3(
      this.x[0] - station.pos.x,
      this.x[1] - station.pos.y,
      this.x[2] - station.pos.z,
    );
    const range = d.length();
    if (!(range > 1e-6)) return;
    const h = [d.x / range, d.y / range, d.z / range, 0, 0, 0];
    this.scalarUpdate(h, zRange - range, this.noise.rangeSigma * this.noise.rangeSigma);
  }

  private rangeRateUpdate(zRate: number, station: StateVector): void {
    const d = new Vector3(
      this.x[0] - station.pos.x,
      this.x[1] - station.pos.y,
      this.x[2] - station.pos.z,
    );
    const dv = new Vector3(
      this.x[3] - station.vel.x,
      this.x[4] - station.vel.y,
      this.x[5] - station.vel.z,
    );
    const range = d.length();
    if (!(range > 1e-6)) return;
    const rangeRate = d.dot(dv) / range;
    const dHat = d.divideScalar(range).clone();
    const drr = new Vector3().copy(dv).addScaledVector(dHat, -rangeRate).divideScalar(range);
    const h = [drr.x, drr.y, drr.z, dHat.x, dHat.y, dHat.z];
    this.scalarUpdate(h, zRate - rangeRate, this.noise.rangeRateSigma * this.noise.rangeRateSigma);
  }

  /**
   * Fold an onboard optical bearing (craft → beacon) into the estimate: two
   * transverse angles, each with 1-sigma `opticalSigma`. Range+Doppler from a
   * ground station leaves the cross-track state poorly observable; the camera
   * fixes it (classic optical navigation).
   */
  opticalUpdate(z: OpticalMeasurement, beacon: StateVector): void {
    const d = new Vector3(
      beacon.pos.x - this.x[0],
      beacon.pos.y - this.x[1],
      beacon.pos.z - this.x[2],
    );
    const range = d.length();
    if (!(range > 1e-6)) return;
    d.divideScalar(range); // predicted direction craft -> beacon
    const [e1, e2] = transverseBasis(d);
    const residues = [z.dir.dot(e1), z.dir.dot(e2)];
    const predicted = [d.dot(e1), d.dot(e2)];
    const r = this.noise.opticalSigma * this.noise.opticalSigma;
    for (let k = 0; k < 2; k++) {
      const e = k === 0 ? e1 : e2;
      // d(direction)/d(craft position) = -(I - dd^T)/range
      const h = [-e.x / range, -e.y / range, -e.z / range, 0, 0, 0];
      this.scalarUpdate(h, residues[k] - predicted[k], r);
    }
  }

  /** One scalar update of the diagonal filter. */
  private scalarUpdate(h: number[], innovation: number, r: number): void {
    let s = r;
    for (let i = 0; i < N; i++) s += h[i] * h[i] * this.p[i];
    if (!(s > 0) || !Number.isFinite(s)) return;
    for (let i = 0; i < N; i++) {
      const k = (this.p[i] * h[i]) / s;
      this.x[i] += k * innovation;
      const pNew = this.p[i] * (1 - k * h[i]);
      this.p[i] = Math.max(pNew, i < 3 ? POS_VARIANCE_FLOOR : VEL_VARIANCE_FLOOR);
    }
  }

  /** The craft executes a commanded burn; the estimate follows the command. */
  applyBurn(dv: Vector3): void {
    this.x[3] += dv.x;
    this.x[4] += dv.y;
    this.x[5] += dv.z;
  }
}
