import { Vector3 } from 'three';
import { AU, DAY } from '../data/constants';
import type { StateVector } from '../physics/state';
import type { AccelFn } from './perturbations';

/**
 * 连续推力（含质量流）。推力恒定、推进剂按 ṁ = F/v_e 消耗，所以飞船质量随
 * 时间线性下降、加速度随之上升：a(t) = F / (m0 − ṁ·t)。这正是火箭方程
 * 在连续工作下的样子，也是"电推螺旋"必须按连续推力积分而不是脉冲近似的原因。
 */
export interface ThrustModel {
  /** 推力方向（单位矢量，日心黄道系），由当前状态决定（如顺行 = v̂）。 */
  direction: (pos: Vector3, vel: Vector3, out: Vector3) => Vector3;
  /**
   * 无工质推进（光帆/束能）：直接给出该位置的加速度（m/s²），此时不用
   * F/m、也不消耗质量——动量来自外部光子，不来自携带的工质。
   * 太阳帆 a ∝ 1/r²；束能帆（激光阵）在射程内近似常数。
   */
  accelAt?: (pos: Vector3) => number;
  /**
   * 更一般的无工质力：直接给出该状态下的加速度矢量（m/s²，日心黄道系）。
   * 用于依赖速度/磁场的力（电动力缆绳 F = I·L×(v_rel×B)）。
   */
  accelVec?: (pos: Vector3, vel: Vector3, out: Vector3) => Vector3;
  /** 推力，N（有工质推进用）。 */
  thrustN?: number;
  /** 排气速度 v_e = Isp·g0，km/s。 */
  exhaustKms?: number;
  /** 点火开始时的质量，kg。 */
  mass0Kg?: number;
  /** 干重，kg（推进剂耗尽后不再减重）。 */
  dryMassKg?: number;
}

/** 恒定推力下的质量流，kg/天。 */
export function massFlowKgPerDay(thrustN: number, exhaustKms: number): number {
  return (thrustN / (exhaustKms * 1000)) * DAY;
}

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
  /** 连续推力（含质量流）；缺省为无推力自由飞行。 */
  thrust?: ThrustModel;
}

export interface TrajectoryPoint {
  t: number;
  pos: Vector3;
  vel: Vector3;
  /** 该采样点的飞船质量，kg（启用推力时才有意义）。 */
  massKg?: number;
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

  const thrust = opts.thrust;
  const rocket = thrust && !thrust.accelAt && !thrust.accelVec ? thrust : null; // 有工质（火箭）模式
  const mdot = rocket ? massFlowKgPerDay(rocket.thrustN!, rocket.exhaustKms!) : 0;
  const massAt = (tt: number): number => {
    if (!rocket) return 0;
    return Math.max(rocket.dryMassKg!, rocket.mass0Kg! - mdot * (tt - t0));
  };
  const thrustAccelAUday = rocket ? (rocket.thrustN! * DAY * DAY) / AU : 0;
  const MPS2_TO_AUDAY2 = (DAY * DAY) / AU;
  const tDir = new Vector3();

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
  const points: TrajectoryPoint[] = [
    { t, pos: yPos.clone(), vel: yVel.clone(), massKg: rocket ? massAt(t) : undefined },
  ];

  for (let step = 0; step < maxSteps; step++) {
    const remaining = tEnd - t;
    if (Math.abs(h) > Math.abs(remaining)) h = remaining;
    if (Math.abs(h) < minStep) h = Math.min(minStep, Math.abs(remaining)) * dir;

    // 推力项：与引力叠加。
    //  · 火箭模式：质量随点火线性下降，同一推力下加速度越来越大；
    //  · 光帆/束能：加速度来自外部光子（a ∝ 1/r² 或常数），不消耗质量。
    const addThrust = (tt: number, pos: Vector3, vel: Vector3, out: Vector3): void => {
      if (!thrust) return;
      if (thrust.accelVec) {
        thrust.accelVec(pos, vel, tDir);
        out.addScaledVector(tDir, MPS2_TO_AUDAY2);
        return;
      }
      if (thrust.accelAt) {
        const a = thrust.accelAt(pos); // m/s²
        if (a <= 0) return;
        thrust.direction(pos, vel, tDir);
        out.addScaledVector(tDir, a * MPS2_TO_AUDAY2);
        return;
      }
      if (!rocket || mdot <= 0) return;
      const m = massAt(tt);
      if (m <= rocket.dryMassKg! + 1e-9) return; // 推进剂耗尽
      thrust.direction(pos, vel, tDir);
      out.addScaledVector(tDir, thrustAccelAUday / m);
    };

    // Dormand–Prince stages.
    kPos[0].copy(yVel);
    accel(t, yPos, kVel[0]);
    addThrust(t, yPos, yVel, kVel[0]);
    for (let i = 1; i < 7; i++) {
      tmpPos.copy(yPos);
      tmpVel.copy(yVel);
      for (let j = 0; j < i; j++) {
        tmpPos.addScaledVector(kPos[j], h * A[i][j]);
        tmpVel.addScaledVector(kVel[j], h * A[i][j]);
      }
      kPos[i].copy(tmpVel);
      accel(t + C[i] * h, tmpPos, kVel[i]);
      addThrust(t + C[i] * h, tmpPos, tmpVel, kVel[i]);
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
      points.push({ t, pos: yPos.clone(), vel: yVel.clone(), massKg: rocket ? massAt(t) : undefined });
      if (Math.abs(t - tEnd) < 1e-12 * Math.max(1, Math.abs(tEnd))) break;
    }

    const growth = err === 0 ? 5 : Math.min(5, Math.max(0.2, 0.9 * Math.pow(1 / err, 1 / 5)));
    h *= growth;
    if (Math.abs(h) > maxStep) h = maxStep * dir;
  }

  return points;
}
