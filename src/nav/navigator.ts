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

/**
 * 船载推进器：把状态从 t0 推进 dt 天。默认是日心二体；任务可以传入含目标
 * 引力与太阳光压的模型（真值则是完整多体场）——即"船载模型的保真度"旋钮。
 */
export type AdvanceFn = (state: StateVector, t0: number, dt: number) => StateVector;

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
  /**
   * Process-noise acceleration spectral density, AU/day^2. Represents the
   * unmodelled part of the dynamics (third-body pulls, SRP, ...): with an
   * n-body truth and a two-body onboard model this is real model error, and
   * it is what keeps the filter honest over long coasts.
   */
  accelNoise?: number;
  /** 船载推进模型；默认日心二体。 */
  advance?: AdvanceFn;
}

const N = 6;
/** Factor floor: keeps the covariance bounded below (~1.5 km / 0.02 mm/s). */
const L_FLOOR = 1e-8;

/**
 * Sequential EKF over [x, y, z, vx, vy, vz] in SQUARE-ROOT form: the
 * covariance is carried as P = L L^T (lower-triangular Cholesky factor) and
 * never formed explicitly.
 *
 * Why: a dense covariance collapses to numerical rank deficiency under precise
 * measurements, and measurement directions aligned with the resulting null
 * space divide roundoff by roundoff (gains ~1e7) and destroy the filter. In
 * factored form the propagation is a QR re-triangularisation of [Phi L | Lq]
 * and the measurement update is a rank-1 Cholesky downdate — both are
 * positive-definite by construction, so the failure mode cannot occur.
 */
export class Navigator {
  measurementCount = 0;

  private x = new Float64Array(N);
  private L = new Float64Array(N * N);
  private noise: TrackingNoise;
  private accelNoise: number;
  private t: number;
  private advance: AdvanceFn;

  constructor(initial: StateVector, t0: number, init: NavigatorInit) {
    this.x[0] = initial.pos.x;
    this.x[1] = initial.pos.y;
    this.x[2] = initial.pos.z;
    this.x[3] = initial.vel.x;
    this.x[4] = initial.vel.y;
    this.x[5] = initial.vel.z;
    for (let i = 0; i < N; i++) {
      this.L[i * N + i] = i < 3 ? init.posSigma0 : init.velSigma0;
    }
    this.noise = init.noise;
    this.accelNoise = init.accelNoise ?? 1e-8;
    this.advance = init.advance ?? ((st, _t0, dt) => propagate(st, dt, MU_SUN));
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

  private positionVariance(): number {
    let v = 0;
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j <= i; j++) v += this.L[i * N + j] ** 2;
    }
    return v;
  }

  private velocityVariance(): number {
    let v = 0;
    for (let i = 3; i < N; i++) {
      for (let j = 0; j <= i; j++) v += this.L[i * N + j] ** 2;
    }
    return v;
  }

  /** Total position uncertainty (1-sigma, AU). */
  positionSigma(): number {
    return Math.sqrt(this.positionVariance());
  }

  velocitySigma(): number {
    return Math.sqrt(this.velocityVariance());
  }

  /** Propagate the estimate and its covariance factor to `t`. */
  propagateTo(t: number): void {
    const dt = t - this.t;
    if (dt <= 0) return;
    const adv = this.advance;
    const t0 = this.t;
    const next = adv(this.state(), t0, dt);

    // State transition matrix via central differences (smooth flow over days).
    const Phi = new Float64Array(N * N);
    const epsPos = 1e-8; // AU
    const epsVel = 1e-10; // AU/day
    for (let i = 0; i < N; i++) {
      const eps = i < 3 ? epsPos : epsVel;
      const xp = this.x.slice();
      const xm = this.x.slice();
      xp[i] += eps;
      xm[i] -= eps;
      const fp = adv(arrayState(xp), t0, dt);
      const fm = adv(arrayState(xm), t0, dt);
      const fpArr = [fp.pos.x, fp.pos.y, fp.pos.z, fp.vel.x, fp.vel.y, fp.vel.z];
      const fmArr = [fm.pos.x, fm.pos.y, fm.pos.z, fm.vel.x, fm.vel.y, fm.vel.z];
      for (let r = 0; r < N; r++) Phi[r * N + i] = (fpArr[r] - fmArr[r]) / (2 * eps);
    }

    // Square-root propagation: P+ = Phi P Phi^T + Q. Stack B = [Phi L ; Lq]
    // (2N x N) and re-triangularise: B^T B = R^T R, so L+ = R^T.
    const half = 0.5 * this.accelNoise * dt * dt;
    const qPos = Math.sqrt(half * half);
    const qVel = Math.sqrt((this.accelNoise * dt) ** 2);
    const B = new Float64Array(2 * N * N);
    for (let i = 0; i < N; i++) {
      for (let j = 0; j < N; j++) {
        // B row i = column i of (Phi L), so that B^T B = (Phi L)(Phi L)^T.
        let acc = 0;
        for (let k = i; k < N; k++) acc += Phi[j * N + k] * this.L[k * N + i];
        B[i * N + j] = acc;
      }
    }
    for (let i = 0; i < N; i++) B[(N + i) * N + i] = i < 3 ? qPos : qVel;
    const R = householderR(B, 2 * N, N);
    // Householder QR leaves arbitrary row signs; normalise them so L = R^T is
    // a proper Cholesky factor (P = R^T R is invariant under row sign flips).
    for (let i = 0; i < N; i++) {
      if (R[i * N + i] < 0) {
        for (let j = i; j < N; j++) R[i * N + j] = -R[i * N + j];
      }
    }
    for (let i = 0; i < N; i++) {
      for (let j = 0; j < N; j++) {
        this.L[i * N + j] = i >= j ? R[j * N + i] : 0; // L = R^T (lower)
      }
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
    this.scalarUpdate(h, zRange - range, this.noise.rangeSigma ** 2);
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
    this.scalarUpdate(h, zRate - rangeRate, this.noise.rangeRateSigma ** 2);
  }

  /**
   * Onboard optical bearing (craft -> beacon): two transverse angles, each
   * with 1-sigma `opticalSigma`. Range+Doppler from a ground station leaves
   * the cross-track state poorly observable; the camera fixes it.
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
    const r = this.noise.opticalSigma ** 2;
    for (let k = 0; k < 2; k++) {
      const e = k === 0 ? e1 : e2;
      const h = [-e.x / range, -e.y / range, -e.z / range, 0, 0, 0];
      this.scalarUpdate(h, residues[k] - predicted[k], r);
    }
  }

  /**
   * One scalar update in factored form. The gain comes from Ph = P h computed
   * through the factor; the covariance becomes the Joseph form
   *   P+ = (I - K h^T) P (I - K h^T)^T + K r K^T,
   * which is stacked as B = [M L | sqrt(r) K] and re-triangularised by QR.
   * Positive definite by construction — no explicit downdate.
   */
  private scalarUpdate(h: number[], innovation: number, r: number): void {
    const tv = new Float64Array(N);
    for (let i = N - 1; i >= 0; i--) {
      let acc = 0;
      for (let j = i; j < N; j++) acc += this.L[j * N + i] * h[j];
      tv[i] = acc;
    }
    const ph = new Float64Array(N);
    for (let i = 0; i < N; i++) {
      let acc = 0;
      for (let j = 0; j <= i; j++) acc += this.L[i * N + j] * tv[j];
      ph[i] = acc;
    }
    let s = r;
    for (let i = 0; i < N; i++) s += h[i] * ph[i];
    if (!(s > 0) || !Number.isFinite(s)) return;

    const K = new Float64Array(N);
    for (let i = 0; i < N; i++) K[i] = ph[i] / s;
    for (let i = 0; i < N; i++) this.x[i] += K[i] * innovation;

    // Stack B = [ (I - K h^T) L ; sqrt(r) K ]^T as a (N+1) x N matrix so the
    // QR returns an N x N R; P+ = B^T B = R^T R, hence L+ = R^T.
    const W = N + 1;
    const B = new Float64Array(W * N);
    for (let i = 0; i < N; i++) {
      for (let j = 0; j < N; j++) {
        // B row i = column i of (M L), so that B^T B = (M L)(M L)^T.
        let acc = 0;
        for (let k = i; k < N; k++) {
          const jk = j === k ? 1 : 0;
          acc += (jk - K[j] * h[k]) * this.L[k * N + i];
        }
        B[i * N + j] = acc;
      }
    }
    for (let i = 0; i < N; i++) B[N * N + i] = Math.sqrt(r) * K[i];
    const R = householderR(B, W, N);
    for (let i = 0; i < N; i++) {
      if (R[i * N + i] < 0) {
        for (let j = i; j < N; j++) R[i * N + j] = -R[i * N + j];
      }
    }
    for (let i = 0; i < N; i++) {
      for (let j = 0; j < N; j++) {
        this.L[i * N + j] = i >= j ? R[j * N + i] : 0;
      }
    }
  }

  /** The craft executes a commanded burn; the estimate follows the command. */
  applyBurn(dv: Vector3): void {
    this.x[3] += dv.x;
    this.x[4] += dv.y;
    this.x[5] += dv.z;
  }
}

/** Householder QR of an m x n (m >= n) row-major matrix; returns R (n x n upper). */
export function householderR(A: Float64Array, m: number, n: number): Float64Array {
  const a = A.slice();
  const v = new Float64Array(m);
  for (let k = 0; k < n; k++) {
    let norm = 0;
    for (let i = k; i < m; i++) norm += a[i * n + k] ** 2;
    norm = Math.sqrt(norm);
    if (norm < 1e-300) continue;
    const alpha = a[k * n + k] >= 0 ? -norm : norm;
    // Capture the Householder vector BEFORE applying the reflection: applying
    // it in place would zero the column that the trailing columns still need.
    v[0] = a[k * n + k] - alpha;
    for (let i = k + 1; i < m; i++) v[i - k] = a[i * n + k];
    let vnorm2 = 0;
    for (let i = 0; i < m - k; i++) vnorm2 += v[i] ** 2;
    if (vnorm2 < 1e-300) continue;
    for (let j = k; j < n; j++) {
      let dot = 0;
      for (let i = k; i < m; i++) dot += v[i - k] * a[i * n + j];
      const f = (2 * dot) / vnorm2;
      for (let i = k; i < m; i++) a[i * n + j] -= f * v[i - k];
    }
  }
  const R = new Float64Array(n * n);
  for (let i = 0; i < n; i++) {
    for (let j = i; j < n; j++) R[i * n + j] = a[i * n + j];
  }
  return R;
}

function arrayState(a: Float64Array): StateVector {
  return {
    pos: new Vector3(a[0], a[1], a[2]),
    vel: new Vector3(a[3], a[4], a[5]),
  };
}
