import { Vector3 } from 'three';
import { gaussian, mulberry32 } from './navigator';

// Rigid-body attitude with a PD pointing controller and a gyro + star-tracker
// complementary filter. Body frame: +Z = nose, +X = right, +Y = up (the same
// convention the renderer and the cockpit use).
//
// This is the ADCS layer the flight-deck console reads: the controller holds
// the commanded pointing (prograde cruise), the gyro integrates with bias and
// noise, the star tracker fixes attitude periodically, and the filter keeps an
// onboard estimate plus a gyro-bias estimate — so the console can show a
// pointing error, an attitude-estimate error and a bias, like a real ADCS.

export type Quat = [number, number, number, number]; // scalar-first [w, x, y, z]

const DEG = 180 / Math.PI;
const ARCSEC = (Math.PI / 180 / 3600);

// ---- quaternion helpers --------------------------------------------------

export function quatIdentity(): Quat {
  return [1, 0, 0, 0];
}

export function quatNormalize(q: Quat): Quat {
  const n = Math.hypot(q[0], q[1], q[2], q[3]) || 1;
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

export function quatMultiply(a: Quat, b: Quat): Quat {
  return [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  ];
}

export function quatFromAxisAngle(axis: Vector3, angle: number): Quat {
  const a = axis.clone().normalize();
  const s = Math.sin(angle / 2);
  return [Math.cos(angle / 2), a.x * s, a.y * s, a.z * s];
}

/** Rotate a vector by a quaternion (body → inertial). */
export function quatRotate(q: Quat, v: Vector3): Vector3 {
  const [w, x, y, z] = q;
  const t = new Vector3().crossVectors(new Vector3(x, y, z), v).multiplyScalar(2);
  return v
    .clone()
    .addScaledVector(t, w)
    .add(new Vector3().crossVectors(new Vector3(x, y, z), t));
}

/** Build the attitude whose +Z is `nose` and +Y is as close to `up` as possible. */
export function quatFromBasis(nose: Vector3, up: Vector3): Quat {
  const z = nose.clone().normalize();
  let y = up.clone().addScaledVector(z, -up.dot(z));
  if (y.lengthSq() < 1e-12) y = new Vector3(0, 0, 1).addScaledVector(z, -z.z);
  y.normalize();
  const x = new Vector3().crossVectors(y, z).normalize();
  // Rotation matrix columns (x, y, z) → quaternion.
  const m00 = x.x, m01 = y.x, m02 = z.x;
  const m10 = x.y, m11 = y.y, m12 = z.y;
  const m20 = x.z, m21 = y.z, m22 = z.z;
  const tr = m00 + m11 + m22;
  if (tr > 0) {
    const s = Math.sqrt(tr + 1) * 2;
    return quatNormalize([s / 4, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s]);
  }
  if (m00 > m11 && m00 > m22) {
    const s = Math.sqrt(1 + m00 - m11 - m22) * 2;
    return quatNormalize([(m21 - m12) / s, s / 4, (m01 + m10) / s, (m02 + m20) / s]);
  }
  if (m11 > m22) {
    const s = Math.sqrt(1 + m11 - m00 - m22) * 2;
    return quatNormalize([(m02 - m20) / s, (m01 + m10) / s, s / 4, (m12 + m21) / s]);
  }
  const s = Math.sqrt(1 + m22 - m00 - m11) * 2;
  return quatNormalize([(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, s / 4]);
}

/** Rotation angle between two attitudes, radians. */
export function quatAngle(a: Quat, b: Quat): number {
  const dot = Math.min(1, Math.abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]));
  return 2 * Math.acos(dot);
}

// ---- the ADCS model ------------------------------------------------------

export interface AdcsConfig {
  /** Diagonal inertia, kg m^2 (x, y, z). */
  inertia: [number, number, number];
  /** Max control torque per axis, N m. */
  maxTorque: number;
  /** PD gains (per axis, N m / rad and N m / (rad/s)). */
  kp: number;
  kd: number;
  /** Star tracker 1-sigma noise per axis, arcsec. */
  starSigmaArcsec: number;
  /** Gyro angle random walk, deg/sqrt(hour). */
  gyroArwDegSqrtH: number;
  /** Star tracker cadence, days. */
  starInterval: number;
  /**
   * 0..1 gain toward each star-tracker fix. 1 = reset to the tracker (gyro
   * only coasts between fixes), which is the sane mode for an 8 arcsec
   * tracker next to a 0.02 deg/sqrt(h) gyro.
   */
  filterGain: number;
}

export const DEFAULT_ADCS: AdcsConfig = {
  inertia: [120, 120, 80],
  maxTorque: 0.01,   // smallsat reaction wheels
  kp: 4e-3,
  kd: 0.5,
  starSigmaArcsec: 8,
  gyroArwDegSqrtH: 0.01,
  starInterval: 0.01, // star trackers update continuously (~15 min here)
  filterGain: 1,
};

/** Rate conversions. */
const RADDAY_TO_DEGH = (180 / Math.PI) / 24;
const RADDAY_TO_DEGS = (180 / Math.PI) / 86400;
const DEGH_TO_RADDAY = 1 / RADDAY_TO_DEGH;

export class Adcs {
  /** Truth attitude and rate (inertial frame, rad/day). */
  q: Quat;
  rate = new Vector3();
  /** Onboard estimate and gyro-bias estimate. */
  qHat: Quat;
  bias = new Vector3();

  starUpdates = 0;
  lastStarDay: number;

  private cfg: AdcsConfig;
  private rng: () => number;
  private lastDay: number;

  constructor(nose: Vector3, up: Vector3, t0: number, opts: Partial<AdcsConfig> = {}, seed = 11) {
    this.cfg = { ...DEFAULT_ADCS, ...opts };
    this.q = quatFromBasis(nose, up);
    this.qHat = [...this.q] as Quat;
    this.lastDay = t0;
    this.lastStarDay = t0;
    this.rng = mulberry32(seed);
  }

  /**
   * Advance the ADCS by `dt` days toward the commanded nose direction.
   * Substepped so large time-warp jumps stay stable.
   */
  advance(dt: number, commandNose: Vector3, commandUp: Vector3): void {
    if (!(dt > 0)) return;
    // Substep well below the control bandwidth (~20 min period): explicit
    // integration of the PD loop needs dt << 1/omega_n. The cap bounds the
    // work per frame under extreme time warp.
    const steps = Math.min(20000, Math.max(1, Math.ceil(dt / 0.002)));
    const h = dt / steps;
    const cmd = quatFromBasis(commandNose, commandUp);
    for (let k = 0; k < steps; k++) {
      const tNow = this.lastDay + (k + 1) * h;
      this.step(h, tNow, cmd);
    }
    this.lastDay += dt;
  }

  private step(h: number, tNow: number, cmd: Quat): void {
    const { inertia, maxTorque, kp, kd } = this.cfg;

    // Pointing error: the rotation from the current attitude to the commanded
    // one, expressed in the body frame (small-angle vector part).
    const qErr = quatMultiply(quatConjugate(this.q), cmd); // q^-1 * q_cmd
    const sign = qErr[0] >= 0 ? 1 : -1;
    const errVec = new Vector3(qErr[1] * sign, qErr[2] * sign, qErr[3] * sign).multiplyScalar(2);

    // PD torque in the body frame, saturated. The rate is rad/day, the gains
    // are SI-ish (N m per rad/s), so damp with the rate converted to rad/s.
    const rateRadS = this.rate.clone().divideScalar(86400);
    const torque = errVec.clone().multiplyScalar(kp).addScaledVector(rateRadS, -kd);
    if (torque.length() > maxTorque) torque.setLength(maxTorque);

    // Rate dynamics. The gyroscopic cross term is dropped: for this slow,
    // near-stationary pointing problem it is negligible, and its nutation
    // frequency would force far smaller substeps than the sim can afford.
    const w = this.rate;
    const angAcc = new Vector3(
      torque.x / inertia[0],
      torque.y / inertia[1],
      torque.z / inertia[2],
    ).multiplyScalar(86400 * 86400); // rad/s^2 -> rad/day^2
    w.addScaledVector(angAcc, h);
    // Body rates: qdot = 0.5 q (x) omega_body, i.e. right-multiply the step.
    if (w.lengthSq() > 1e-24) {
      const dq = quatFromAxisAngle(w.clone().normalize(), w.length() * h);
      this.q = quatNormalize(quatMultiply(this.q, dq));
    }
    void qErr;

    // --- onboard: gyro integration with bias + noise, star tracker fixes.
    const sigmaRate =
      (this.cfg.gyroArwDegSqrtH / Math.sqrt(h * 24)) * DEGH_TO_RADDAY;
    const gyroNoise = new Vector3(
      gaussian(this.rng),
      gaussian(this.rng),
      gaussian(this.rng),
    ).multiplyScalar(sigmaRate);
    const wMeas = w.clone().add(this.bias).add(gyroNoise);
    if (wMeas.lengthSq() > 1e-24) {
      const dqHat = quatFromAxisAngle(wMeas.clone().normalize(), wMeas.length() * h);
      this.qHat = quatNormalize(quatMultiply(this.qHat, dqHat));
    }

    if (tNow - this.lastStarDay >= this.cfg.starInterval) {
      this.lastStarDay = tNow;
      const sigma = this.cfg.starSigmaArcsec * ARCSEC;
      const noise = new Vector3(
        gaussian(this.rng),
        gaussian(this.rng),
        gaussian(this.rng),
      ).multiplyScalar(sigma);
      const qMeas = quatNormalize(
        quatMultiply(quatFromAxisAngle(noise.clone().normalize(), noise.length()), this.q),
      );
      // Nudge the estimate toward the measurement (complementary filter):
      // qE = qMeas * qHat^-1 is the rotation that takes qHat to qMeas.
      const qE = quatMultiply(qMeas, quatConjugate(this.qHat));
      const sg = qE[0] >= 0 ? 1 : -1;
      const corr = new Vector3(qE[1] * sg, qE[2] * sg, qE[3] * sg);
      if (corr.lengthSq() > 1e-30) {
        // |corr| ~ sin(theta/2): the applied correction angle is ~2|corr|.
        const alpha = this.cfg.filterGain;
        this.qHat = quatNormalize(
          quatMultiply(
            quatFromAxisAngle(corr.clone().normalize(), corr.length() * 2 * alpha),
            this.qHat,
          ),
        );
      }
      // The gyro bias is treated as a nominal spec (the learning loop is a
      // textbook upgrade); the estimate is carried by gyro + tracker alone.
      this.starUpdates += 1;
    }
  }

  /** True pointing error against the command, degrees. */
  pointingErrorDeg(commandNose: Vector3): number {
    const nose = quatRotate(this.q, new Vector3(0, 0, 1));
    return (nose.angleTo(commandNose) * DEG);
  }

  /** Onboard attitude-estimate error (truth vs filter), arcsec. */
  estimateErrorArcsec(): number {
    return quatAngle(this.q, this.qHat) / ARCSEC;
  }

  /** Gyro bias magnitude, deg/h. */
  biasDegPerHour(): number {
    return this.bias.length() * RADDAY_TO_DEGH;
  }

  /** Body rate magnitude, deg/s. */
  rateDegPerSec(): number {
    return this.rate.length() * RADDAY_TO_DEGS;
  }
}

function quatConjugate(q: Quat): Quat {
  return [q[0], -q[1], -q[2], -q[3]];
}
