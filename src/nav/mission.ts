import { Vector3 } from 'three';
import type { StateVector } from '../physics/state';
import { AU, AU_KM, DAY } from '../data/constants';
import { PLANETS } from '../data/bodies';
import { bodyEphemeris, type Ephemeris } from './ephemeris';
import { integrate } from './integrate';
import { lambert } from './lambert';
import {
  makeMeasurement, makeOpticalMeasurement, mulberry32, Navigator, tierNoise,
  type OpticalMeasurement, type RangeMeasurement, type TrackingNoise, type TrackingTier,
} from './navigator';
import type { TransferPlan } from './plan';
import { makeForceModel, type AccelFn, type GravitySource } from './perturbations';
import { DRIVES, massRatioVe, thrustDirection, type Drive, type ThrustDir } from './propulsion';
import { DIPOLE_MOMENT, tetherForceN, windSailThrustN } from './tether';
import { propagate } from './propagate';
import { solarSystemSources } from './sources';
import { DEFAULT_SRP, TruthTrajectory, truthSoftening } from './truth';
import { fromKms, MPS_TO_AUDAY, MU_SUN, toKms } from './units';

// A flown mission: the planned Lambert arc plus execution errors, noisy
// tracking and TCM (trajectory correction manoeuvre) history.
//
// TRUTH physics default to N-BODY: the trajectory is integrated in the full
// force field (Sun + planets + moons + SRP), while the onboard computer keeps
// its two-body model. The difference is genuine model error — the L2 problem
// the filter and the TCMs must live with. 'two-body' recovers the classic
// exact arcs.
//
// The mission now starts and ends at a **synchronous-orbit spaceport**, not at
// a body centre: the initial state is the post-escape-burn state in the port
// orbit, so the departure/arrival bodies must stay in the force model (their
// gravity bends the escape hyperbola and the capture). Surface ↔ port traffic
// is a different vehicle and is not modelled here.
//
// Two state tracks live here:
//   - the TRUTH state (what actually happens), propagated between burns;
//   - the onboard ESTIMATE (an EKF fed with noisy range/range-rate), which is
//     all the spacecraft is allowed to see when planning a correction.
// Without tracking the estimate equals the truth (the L0 model); with tracking
// corrections are imperfect and leave a residual miss — the L1 story.

/** 解 3×3 线性方程组 J·x = b（克拉默法则）；奇异返回 null。 */
function solve3x3(cols: Vector3[], b: Vector3): Vector3 | null {
  const m = (c: Vector3[], k: number, r: number): number => (c[k].getComponent(r));
  const det =
    m(cols, 0, 0) * (m(cols, 1, 1) * m(cols, 2, 2) - m(cols, 1, 2) * m(cols, 2, 1)) -
    m(cols, 1, 0) * (m(cols, 0, 1) * m(cols, 2, 2) - m(cols, 0, 2) * m(cols, 2, 1)) +
    m(cols, 2, 0) * (m(cols, 0, 1) * m(cols, 1, 2) - m(cols, 0, 2) * m(cols, 1, 1));
  if (Math.abs(det) < 1e-30) return null;
  const detX =
    b.x * (m(cols, 1, 1) * m(cols, 2, 2) - m(cols, 1, 2) * m(cols, 2, 1)) -
    m(cols, 1, 0) * (b.y * m(cols, 2, 2) - b.z * m(cols, 2, 1)) +
    m(cols, 2, 0) * (b.y * m(cols, 1, 2) - b.z * m(cols, 1, 1));
  const detY =
    m(cols, 0, 0) * (b.y * m(cols, 2, 2) - b.z * m(cols, 2, 1)) -
    b.x * (m(cols, 0, 1) * m(cols, 2, 2) - m(cols, 0, 2) * m(cols, 2, 1)) +
    m(cols, 2, 0) * (m(cols, 0, 1) * b.z - m(cols, 0, 2) * b.y);
  const detZ =
    m(cols, 0, 0) * (m(cols, 1, 1) * b.z - m(cols, 1, 2) * b.y) -
    m(cols, 1, 0) * (m(cols, 0, 1) * b.z - m(cols, 0, 2) * b.y) +
    b.x * (m(cols, 0, 1) * m(cols, 1, 2) - m(cols, 0, 2) * m(cols, 1, 1));
  return new Vector3(detX / det, detY / det, detZ / det);
}

export interface FlightSegment {
  /** Truth state immediately after the segment start (launch or TCM). */
  state: StateVector;
  startDay: number;
  /** Rendezvous epoch. */
  endDay: number;
}

export interface TcmSolution {
  /** Required velocity change, computed from the onboard estimate, AU/day. */
  dv: Vector3;
  dvKms: number;
  /** Estimated state after executing the burn. */
  state: StateVector;
}

/** 离港点火执行误差（演示用）：逃逸点火量的 0.25%——误差源是发动机，不是日心速度。 */
export const LAUNCH_ERROR_FRACTION = 0.0025;

/** Tracking cadence, days between ground-station passes. */
export const TRACK_INTERVAL_DAYS = 2;

/** A-priori state uncertainty before any tracking (loose launch knowledge). */
const P0_POS_KM = 1000;
const P0_VEL_AUDAY = 0.05;

/**
 * 船载力模型的太阳光压系数偏差。船载模型 = 太阳 + 出发/目标天体 + 太阳光压，
 * 但 Cr 只知其先验值到 ~10%（真实值 1.3，船载用 1.43）——这 10% 就是滤波器
 * 必须靠跟踪吸收的模型误差。没有它，船载模型就等于真值，导航问题会假掉。
 */
const ONBOARD_SRP_CR_FACTOR = 1.1;

/** 飞船质量模型：干重 + 初始推进剂。 */
export const DRY_MASS_KG = 900;
export const PROPELLANT_KG0 = 600;

/** 终端瞄准的迭代容差（1e-8 AU ≈ 1.5 km，受积分器精度限制）。 */
const AIM_TOL_AU = 1e-8;
/** 单步最大修正量（AU/day，≈1.7 km/s）；超过说明在发散。 */
const AIM_MAX_STEP = 1e-3;

export interface MissionOptions {
  injectError?: boolean;
  /** Ground tracking quality; null/undefined = no tracking (truth navigation). */
  tracking?: TrackingTier | null;
  /** Tracking station ephemeris; defaults to Earth. */
  station?: Ephemeris;
  /**
   * Truth physics. 'n-body' (default) integrates the real trajectory in the
   * full force field (Sun + planets + moons + SRP) while the onboard computer
   * keeps believing its two-body model — the model error the filter and the
   * TCMs must live with. 'two-body' keeps the classic exact-arc behaviour.
   */
  physics?: 'two-body' | 'n-body';
  /** Solar radiation pressure in the truth model (n-body only; default on). */
  srp?: boolean;
}

export class Mission {
  readonly plan: TransferPlan;
  readonly segments: FlightSegment[] = [];
  /** Actual flown path samples (heliocentric AU), grown as time advances. */
  readonly flown: Vector3[] = [];
  /** The original planned arc (no dispersion), for reference rendering. */
  readonly planArcPath: Vector3[];
  tcmCount = 0;
  tcmUsedKms = 0;
  /** 手动点火次数与累计 Δv（km/s）。 */
  manualCount = 0;
  manualUsedKms = 0;
  /** 连续推力点火次数。 */
  throttleBurns = 0;

  private injectError: boolean;
  private truthPhysics: 'two-body' | 'n-body';
  private srp: boolean;
  private trajectories: TruthTrajectory[] = [];
  private station: Ephemeris;
  private beacon: Ephemeris | null;
  private navigator: Navigator | null = null;
  private trackNoise: TrackingNoise | null = null;
  private rng: () => number;
  private nextTrackDay = 0;
  private lastDay = -Infinity;
  private sampleStep: number;
  private nextSampleDay = 0;
  private predicted: Vector3[] = [];
  private lastPredictDay = -Infinity;
  private lastRangeMeas: RangeMeasurement | null = null;
  private lastOpticalMeas: OpticalMeasurement | null = null;
  /** 船载加速度模型（太阳 + 出发/目标天体 + SRP）与其推进器。 */
  private onboardAccel: AccelFn;
  /** 推进系统与油门：当前点火指令（null = 关机）。 */
  private drive: Drive;
  private throttle: { dir: ThrustDir; level: number; startDay: number; mass0Kg: number } | null = null;
  /** 累计消耗的推进剂（kg）——关机后也要记住。 */
  private burnedKg = 0;

  constructor(plan: TransferPlan, opts: MissionOptions = {}) {
    this.plan = plan;
    this.injectError = opts.injectError ?? false;
    this.truthPhysics = opts.physics ?? 'n-body';
    this.srp = opts.srp ?? true;
    const earth = PLANETS.find((b) => b.id === 'earth')!;
    this.station = opts.station ?? bodyEphemeris(earth);
    const target = PLANETS.find((b) => b.id === plan.targetId);
    this.beacon = target ? bodyEphemeris(target) : null;
    if (opts.tracking) this.trackNoise = tierNoise(opts.tracking);
    this.rng = mulberry32(Math.floor(plan.departureDay * 7919 + plan.targetId.length * 104729));
    this.drive = DRIVES.find((d) => d.id === 'chemical')!;
    // 船载力模型。n-body 模式：太阳 + 出发/目标天体 + 太阳光压（Cr 有 10% 先验
    // 偏差）——目标引力进模型后，终端瞄准才不再像纯二体那样差出 1e5 km；出发
    // 天体也必须进来，因为飞船此刻就在它的引力井里点火逃逸。
    // 二体模式：太阳 only（经典 patched-conic 理想化）。
    const nbody = this.truthPhysics === 'n-body';
    const wanted = nbody
      ? new Set(['sun', plan.departureId, plan.targetId])
      : new Set(['sun']);
    const onboardSources: GravitySource[] = solarSystemSources().filter((src) => wanted.has(src.id));
    this.onboardAccel = makeForceModel(
      onboardSources,
      nbody ? { ...DEFAULT_SRP, cr: DEFAULT_SRP.cr * ONBOARD_SRP_CR_FACTOR } : undefined,
      nbody ? truthSoftening : 0,
    );
    this.sampleStep = Math.max(0.5, plan.tof / 512);
    this.planArcPath = this.sampleArc(
      { pos: plan.r1.clone(), vel: plan.v1.clone() },
      plan.departureDay,
      plan.arrivalDay,
    );
    this.init();
  }

  /** (Re)initialise the flown mission from the plan — used on time resets. */
  private init(): void {
    this.segments.length = 0;
    this.flown.length = 0;
    this.tcmCount = 0;
    this.tcmUsedKms = 0;
    this.manualCount = 0;
    this.manualUsedKms = 0;
    this.burnedKg = 0;
    this.throttle = null;
    this.throttleBurns = 0;
    this.lastDay = -Infinity;
    this.nextTrackDay = this.plan.departureDay;
    this.nextSampleDay = this.plan.departureDay;
    this.lastRangeMeas = null;
    this.lastOpticalMeas = null;

    // n-body 真值从太空港点火后的逃逸状态出发（行星引力参与，逃逸双曲线是真的）。
    // 二体模式是经典的 patched-conic 理想化：日心弧从港口位置以转移速度出发，
    // 逃逸/捕获被当作瞬时——所以它必须走 Lambert 端点，否则会带着"未爬出行星
    // 引力井"的 2.3 km/s 在日心系里飞错轨道。两条口径都从太空港端点起步。
    const start =
      this.truthPhysics === 'n-body' && this.plan.departureState
        ? this.plan.departureState
        : { pos: this.plan.r1.clone(), vel: this.plan.v1.clone() };

    // 船载先验状态 = 同一个起点（飞船当然知道自己点火后的状态），与船载力模型
    // 自洽：n-body 模式下它就处在出发天体的引力井里，模型和状态必须匹配。
    this.navigator = this.trackNoise
      ? new Navigator(
          { pos: start.pos.clone(), vel: start.vel.clone() },
          this.plan.departureDay,
          {
            noise: this.trackNoise,
            posSigma0: P0_POS_KM / AU_KM,
            velSigma0: P0_VEL_AUDAY,
            advance: (st, t0, dt) => this.advanceOnboard(st, t0, dt),
          },
        )
      : null;

    const v = start.vel.clone();
    if (this.injectError) {
      const burnAuday = this.plan.burnDepart
        ? fromKms(this.plan.burnDepart.dvKms)
        : v.length();
      v.addScaledVector(this.launchErrorDir(), burnAuday * LAUNCH_ERROR_FRACTION);
    }
    const first: FlightSegment = {
      state: { pos: start.pos.clone(), vel: v },
      startDay: this.plan.departureDay,
      endDay: this.plan.arrivalDay,
    };
    this.segments.push(first);
    this.trajectories = [];
    if (this.truthPhysics === 'n-body') {
      this.trajectories.push(
        new TruthTrajectory(first.state, first.startDay, first.endDay, {
          srp: this.srp,
          mass0Kg: DRY_MASS_KG + PROPELLANT_KG0,
        }),
      );
    }
    this.predicted = this.sampleArc(first.state, first.startDay, first.endDay);
    this.lastPredictDay = -Infinity;
  }

  /** 当前质量（kg）：干重 + 剩余推进剂（含本次点火已烧掉的量）。 */
  massKg(t: number): number {
    const burning = this.throttleRateKgPerDay();
    const dt = this.throttle ? Math.max(0, t - this.throttle.startDay) : 0;
    const soFar = Math.min(PROPELLANT_KG0 - this.burnedKg, burning * dt);
    return DRY_MASS_KG + PROPELLANT_KG0 - this.burnedKg - soFar;
  }

  /** 把当前点火烧掉的推进剂结转到累计账上（改指令/关机时调用）。 */
  private commitBurn(upToDay: number): void {
    if (!this.throttle) return;
    const rate = this.throttleRateKgPerDay();
    const dt = Math.max(0, upToDay - this.throttle.startDay);
    this.burnedKg = Math.min(PROPELLANT_KG0, this.burnedKg + rate * dt);
  }

  /** 当前油门指令（UI 读数用）。 */
  throttleState(): { dir: ThrustDir; level: number; active: boolean; drive: Drive } | null {
    if (!this.throttle) return null;
    return {
      dir: this.throttle.dir,
      level: this.throttle.level,
      active: this.throttle.level > 0 && this.massKg(this.lastDay) > DRY_MASS_KG + 1e-9,
      drive: this.drive,
    };
  }

  private throttleRateKgPerDay(): number {
    const f = this.drive.exhaustKms;
    if (this.drive.kind !== 'rocket') return 0; // 光帆不耗工质
    if (!Number.isFinite(f) || !this.throttle || this.throttle.level <= 0) return 0;
    const thrustN = this.throttle.level * this.maxThrustN();
    return (thrustN / (f * 1000)) * 86400;
  }

  /** 满油门推力（N）：按给定加速度量级与初始质量定义。 */
  private maxThrustN(): number {
    return this.drive.accelMps2 * (DRY_MASS_KG + PROPELLANT_KG0);
  }

  /** 是否是无工质推进——质量不随点火变化。 */
  get isPropellantless(): boolean {
    return this.drive.kind !== 'rocket';
  }

  /** 缆绳所依附的天体（取出发点；无磁场天体则退化为无推力）。 */
  private tetherBody(): { id: string; momentAm2: number; omegaRadPerDay: number } | null {
    const id = this.plan.departureId;
    const body = PLANETS.find((b) => b.id === id);
    if (!body) return null;
    const moment = DIPOLE_MOMENT[id] ?? 0;
    const omega = body.rotationPeriod !== 0 ? (2 * Math.PI) / body.rotationPeriod : 0;
    return { id, momentAm2: moment, omegaRadPerDay: omega };
  }

  /** 当前推力加速度（m/s²）与 T/W 比值。 */
  thrustAccelMps2(t: number): number {
    if (!this.throttle || this.throttle.level <= 0) return 0;
    const m = this.massKg(t);
    if (m <= DRY_MASS_KG + 1e-9) return 0;
    return (this.throttle.level * this.maxThrustN()) / m;
  }

  /**
   * 设定油门：把连续推力写进真值轨迹（新开一段带 ThrustModel 的积分）。
   * level = 0 关机；dir 为推力方向（顺行/逆行/径向/法向）。
   */
  setThrottle(t: number, driveId: string, level: number, dir: ThrustDir): void {
    const drive = DRIVES.find((d) => d.id === driveId) ?? this.drive;
    this.commitBurn(t);
    this.drive = drive;
    const mass0Kg = this.massKg(t);
    const wasBurning = this.throttle !== null && this.throttle.level > 0;
    this.throttle = level > 0 ? { dir, level, startDay: t, mass0Kg } : null;
    if (!this.truthPhysics || !this.plan) return;
    // 新开一段真值轨迹（含推力 + 质量流）
    this.updateFlown(t);
    const truth = this.stateAt(t);
    this.segments.push({ state: { pos: truth.pos.clone(), vel: truth.vel.clone() }, startDay: t, endDay: this.plan.arrivalDay });
    if (this.truthPhysics === 'n-body') {
      this.trajectories.push(
        new TruthTrajectory({ pos: truth.pos.clone(), vel: truth.vel.clone() }, t, this.plan.arrivalDay, {
          srp: this.srp,
          mass0Kg: mass0Kg,
          thrust: level > 0
            ? drive.kind === 'rocket'
              ? {
                  direction: (pos, vel, out) => thrustDirection(dir, pos, vel, out),
                  thrustN: level * this.maxThrustN(),
                  exhaustKms: drive.exhaustKms,
                  mass0Kg,
                  dryMassKg: DRY_MASS_KG,
                }
              : drive.kind === 'tether'
              ? {
                  // 电动力缆绳：力依赖相对行星的 r 与 v（含自转牵连）
                  direction: (_pos, _vel, out) => out.set(0, 0, 1),
                  accelVec: (pos, vel, out) => {
                    const tb = this.tetherBody();
                    const body = tb ? PLANETS.find((b) => b.id === tb.id) : undefined;
                    if (!tb || !body || tb.momentAm2 === 0) return out.set(0, 0, 0);
                    const st = bodyEphemeris(body).stateAt(t);
                    // 导航层是 AU / AU·day⁻¹；缆绳模块要 SI —— 这里换算
                    const relPosM = new Vector3().subVectors(pos, st.pos).multiplyScalar(AU);
                    const omegaPerSec = new Vector3(0, 0, tb.omegaRadPerDay / DAY);
                    const vRelMps = new Vector3().subVectors(vel, st.vel)
                      .multiplyScalar(AU / DAY)
                      .sub(new Vector3().crossVectors(omegaPerSec, relPosM));
                    const fN = tetherForceN({ lengthKm: 10, currentA: level * 20 }, tb.momentAm2, relPosM, vRelMps);
                    return out.copy(fN).multiplyScalar(1 / (DRY_MASS_KG + PROPELLANT_KG0)); // m/s²
                  },
                }
              : {
                  // 无工质：加速度由外部光子给，质量不变
                  direction: (pos, vel, out) => thrustDirection(dir, pos, vel, out),
                  accelAt: drive.kind === 'sail-solar'
                    ? (pos) => level * drive.accelMps2 / Math.max(pos.lengthSq(), 1e-6)
                    : drive.kind === 'wind-sail'
                      ? (pos) => (() => {
                          const p = drive.id === 'magsail'
                            ? { kind: 'magsail' as const, size: 100 }
                            : { kind: 'esail' as const, size: 20, wires: 100 };
                          return (level * windSailThrustN(p, pos.length())) / (DRY_MASS_KG + PROPELLANT_KG0);
                        })()
                      : () => level * drive.accelMps2,
                }
            : undefined,
        }),
      );
    }
    if (wasBurning !== (level > 0)) this.throttleBurns += level > 0 ? 1 : 0;
    this.refreshPrediction(t);
  }

  /** 船载模型推进（n-body 模式含出发/目标引力与 SRP；二体模式即太阳二体）。 */
  private advanceOnboard(state: StateVector, t0: number, dt: number): StateVector {
    // 推力是**指令**，船载系统当然知道：把它一并放进滤波器模型，
    // 否则连续点火期间估计会被当成模型误差而跑飞。
    const burning = this.throttle !== null && this.throttle.level > 0
      && this.massKg(t0) > DRY_MASS_KG + 1e-9;
    const thrust = burning
      ? {
          direction: (pos: Vector3, vel: Vector3, out: Vector3) =>
            thrustDirection(this.throttle!.dir, pos, vel, out),
          thrustN: this.throttle!.level * this.maxThrustN(),
          exhaustKms: this.drive.exhaustKms,
          mass0Kg: this.massKg(t0),
          dryMassKg: DRY_MASS_KG,
        }
      : undefined;
    const pts = integrate(state, t0, dt, this.onboardAccel, {
      maxStep: this.truthPhysics === 'n-body' ? 1.5 : 5,
      rtol: 1e-9,
      atol: 1e-11,
      thrust,
    });
    const last = pts[pts.length - 1];
    return { pos: last.pos.clone(), vel: last.vel.clone() };
  }

  /** 从 `pos`/`vel` 用船载模型飞到抵达时刻，返回相对目标港的偏差。 */
  private aimMiss(pos: Vector3, vel: Vector3, t: number): Vector3 {
    const tof = this.plan.arrivalDay - t;
    const end = this.advanceOnboard({ pos, vel }, t, tof);
    return this.plan.r2.clone().sub(end.pos);
  }

  /**
   * 终端瞄准：用**含目标引力的船载模型**做微分修正，解出命中港口的出发速度。
   * 以二体 Lambert 解为初值，数值雅可比 3×3，最多 8 次迭代。
   * 返回 null 表示不收敛（调用方回退到二体 Lambert 解）。
   */
  private aimAtPort(pos: Vector3, t: number): Vector3 | null {
    const tof = this.plan.arrivalDay - t;
    let v: Vector3;
    try {
      v = lambert(pos, this.plan.r2, tof, MU_SUN).v1;
    } catch {
      return null;
    }
    const eps = 1e-9; // AU/day
    const col = [new Vector3(), new Vector3(), new Vector3()];
    for (let iter = 0; iter < 8; iter++) {
      const miss = this.aimMiss(pos, v, t);
      if (miss.length() < AIM_TOL_AU) return v;
      // 数值雅可比 ∂r_f/∂v（列 = 对每个速度分量扰动后的落点变化）
      for (let c = 0; c < 3; c++) {
        const vp = v.clone();
        vp.setComponent(c, vp.getComponent(c) + eps);
        const mp = this.aimMiss(pos, vp, t);
        col[c].copy(mp).sub(miss).divideScalar(eps);
      }
      // 对偏差函数求雅可比，所以 Newton 步解的是 J·δv = −miss。
      const dv = solve3x3(col, miss.clone().multiplyScalar(-1));
      if (!dv || dv.length() > AIM_MAX_STEP) return null; // 发散：交给回退解
      v.add(dv);
    }
    // 迭代被 8 次截断时也要验收：落点必须真的到港（15 km 以内）。
    return this.aimMiss(pos, v, t).length() < 1e-7 ? v : null;
  }

  /** Deterministic pseudo-random dispersion direction (stable per mission). */
  private launchErrorDir(): Vector3 {
    const seed = this.plan.departureDay * 0.37 + this.plan.targetId.length * 1.93;
    return new Vector3(
      Math.sin(seed),
      Math.cos(seed * 1.7),
      Math.sin(seed * 2.3) * 0.4,
    ).normalize();
  }

  private segmentFor(t: number): FlightSegment {
    for (let i = this.segments.length - 1; i >= 0; i--) {
      if (t >= this.segments[i].startDay) return this.segments[i];
    }
    return this.segments[0];
  }

  private sampleArc(state: StateVector, startDay: number, endDay: number): Vector3[] {
    const n = 512;
    const out: Vector3[] = [];
    const span = endDay - startDay;
    for (let k = 0; k <= n; k++) {
      out.push(propagate(state, (span * k) / n, MU_SUN).pos);
    }
    return out;
  }

  /**
   * Advance the onboard navigation solution to `t`: propagate the filter and
   * fold in every scheduled tracking pass. Call once per frame. A backwards
   * time jump (user reset) replays the mission from scratch.
   */
  advance(t: number): void {
    if (t < this.lastDay - 1e-9) this.init();
    this.lastDay = Math.max(this.lastDay, t);
    if (!this.navigator || !this.trackNoise) return;
    const end = Math.min(t, this.plan.arrivalDay);
    while (this.nextTrackDay <= end) {
      const tMeas = this.nextTrackDay;
      this.navigator.propagateTo(tMeas);
      const truth = this.stateAt(tMeas);
      const station = this.station.stateAt(tMeas);
      const z = makeMeasurement(truth, station, tMeas, this.trackNoise, this.rng);
      this.lastRangeMeas = z;
      this.navigator.update(z, station);
      if (this.beacon && this.trackNoise.opticalSigma > 0) {
        const bx = this.beacon.stateAt(tMeas);
        const opt = makeOpticalMeasurement(truth, bx, tMeas, this.trackNoise, this.rng);
        this.lastOpticalMeas = opt;
        this.navigator.opticalUpdate(opt, bx);
      }
      this.nextTrackDay += TRACK_INTERVAL_DAYS;
    }
    this.navigator.propagateTo(end);
    if (this.navigator && end - this.lastPredictDay > 5) this.refreshPrediction(end);
  }

  /** Truth state at `t` (valid for t >= departureDay). */
  stateAt(t: number): StateVector {
    if (this.truthPhysics === 'n-body') {
      // Pick the trajectory covering t (segments are ordered by start day).
      for (let i = this.trajectories.length - 1; i >= 0; i--) {
        const tr = this.trajectories[i];
        if (t >= tr.startDay) return tr.stateAt(t);
      }
      if (this.trajectories.length) return this.trajectories[0].stateAt(t);
    }
    const seg = this.segmentFor(t);
    return propagate(seg.state, t - seg.startDay, MU_SUN);
  }

  /** What the spacecraft believes its state is at `t` (advances the filter). */
  estimateState(t: number): StateVector {
    this.advance(t);
    return this.navigator ? this.navigator.state() : this.stateAt(t);
  }

  /**
   * Predicted arc the ONBOARD navigation believes in (from the current
   * estimate to the rendezvous point), refreshed as tracking updates arrive.
   * Without tracking the estimate equals the truth (the L0 model).
   */
  predictedPath(): Vector3[] {
    return this.predicted;
  }

  /** Rebuild the onboard predicted arc from the current estimate (船载模型). */
  private refreshPrediction(t: number): void {
    const est = this.navigator ? this.navigator.state() : this.stateAt(t);
    const span = this.plan.arrivalDay - t;
    const pts = integrate(est, t, span, this.onboardAccel, {
      maxStep: 2,
      rtol: 1e-9,
      atol: 1e-11,
    });
    const step = Math.max(1, Math.floor(pts.length / 200));
    const out: Vector3[] = [];
    for (let k = 0; k < pts.length; k += step) out.push(pts[k].pos.clone());
    out.push(pts[pts.length - 1].pos.clone());
    this.predicted = out;
    this.lastPredictDay = t;
  }

  /** True separation from the rendezvous point at arrival, AU. */
  truthMissDistance(): number {
    const arrive = this.stateAt(this.plan.arrivalDay);
    return arrive.pos.distanceTo(this.plan.r2);
  }

  /** Separation the spacecraft expects at arrival from its own estimate, AU. */
  estimatedMissDistance(t: number): number {
    const est = this.estimateState(t);
    const arrive = propagate(est, this.plan.arrivalDay - t, MU_SUN);
    return arrive.pos.distanceTo(this.plan.r2);
  }

  /** Distance between the onboard estimate and the truth, AU. */
  estimateError(t: number): number {
    const est = this.estimateState(t);
    return est.pos.distanceTo(this.stateAt(t).pos);
  }

  /** Onboard position uncertainty (1-sigma), AU. */
  positionSigma(): number {
    return this.navigator ? this.navigator.positionSigma() : 0;
  }

  /** Onboard velocity uncertainty (1-sigma), AU/day. */
  velocitySigma(): number {
    return this.navigator ? this.navigator.velocitySigma() : 0;
  }

  get trackingCount(): number {
    return this.navigator ? this.navigator.measurementCount : 0;
  }

  /** Raw instrument readings from the most recent tracking pass. */
  lastMeasurement(): { range: RangeMeasurement | null; optical: OpticalMeasurement | null } {
    return { range: this.lastRangeMeas, optical: this.lastOpticalMeas };
  }

  /** Noise spec of the active tracking tier (null without tracking). */
  trackingNoise(): TrackingNoise | null {
    return this.trackNoise;
  }

  get hasTracking(): boolean {
    return this.navigator !== null;
  }

  /**
   * TCM solution computed from the ONBOARD ESTIMATE (not truth) that would
   * rendezvous at the fixed arrival epoch — this is what the spacecraft can
   * actually know. Returns null when unavailable.
   */
  solveTcm(t: number): TcmSolution | null {
    if (t < this.plan.departureDay || t >= this.plan.arrivalDay) return null;
    const est = this.estimateState(t);
    // 优先用含目标引力的微分修正；不收敛时回退到二体 Lambert（保守但可用）。
    let v = this.aimAtPort(est.pos, t);
    if (!v) {
      try {
        v = lambert(est.pos, this.plan.r2, this.plan.arrivalDay - t, MU_SUN).v1;
      } catch {
        return null;
      }
    }
    const dv = v.clone().sub(est.vel);
    if (toKms(dv.length()) > 20) return null; // 离谱的解不要（不是修正，是重做任务）
    return { dv, dvKms: toKms(dv.length()), state: { pos: est.pos.clone(), vel: v.clone() } };
  }

  /**
   * Execute a TCM at `t`: the burn is computed from the estimate but applied
   * to the truth trajectory, so estimation error becomes residual miss.
   * Returns the commanded delta-v in km/s, or null.
   */
  applyTcm(t: number): number | null {
    const sol = this.solveTcm(t);
    if (!sol) return null;
    this.updateFlown(t); // capture the actual path up to the burn
    const truth = this.stateAt(t);
    const truthAfter: StateVector = {
      pos: truth.pos.clone(),
      vel: truth.vel.clone().add(sol.dv),
    };
    this.segments.push({ state: truthAfter, startDay: t, endDay: this.plan.arrivalDay });
    if (this.truthPhysics === 'n-body') {
      this.trajectories.push(
        new TruthTrajectory(truthAfter, t, this.plan.arrivalDay, {
          srp: this.srp,
        }),
      );
    }
    this.navigator?.applyBurn(sol.dv);
    this.refreshPrediction(t);
    this.tcmCount += 1;
    this.tcmUsedKms += sol.dvKms;
    return sol.dvKms;
  }

  /**
   * 手动点火：把用户指定的 Δv（AU/day，日心系）施加到真值轨迹上，并让滤波器
   * 知道这次点火（它由加速度计/指令记录可知）。返回施加的 Δv（km/s）。
   */
  applyManualBurn(t: number, dv: Vector3): number | null {
    if (t < this.plan.departureDay || t >= this.plan.arrivalDay) return null;
    const dvKms = toKms(dv.length());
    if (dvKms <= 0) return null;
    this.updateFlown(t);
    const truth = this.stateAt(t);
    const truthAfter: StateVector = {
      pos: truth.pos.clone(),
      vel: truth.vel.clone().add(dv),
    };
    this.segments.push({ state: truthAfter, startDay: t, endDay: this.plan.arrivalDay });
    if (this.truthPhysics === 'n-body') {
      this.trajectories.push(
        new TruthTrajectory(truthAfter, t, this.plan.arrivalDay, { srp: this.srp }),
      );
    }
    this.navigator?.applyBurn(dv);
    this.refreshPrediction(t);
    this.manualCount += 1;
    this.manualUsedKms += dvKms;
    return dvKms;
  }

  /** Append actual-path samples up to `t`; safe to call every frame. */
  updateFlown(t: number): void {
    const end = Math.min(t, this.plan.arrivalDay);
    if (end < this.plan.departureDay) return;
    while (this.nextSampleDay <= end) {
      // 采样真值（Hermite 插值）而不是二体推算：离港/入泊段是真值引力主导的
      // 双曲线，二体外推在那里完全不对，画出来的航迹会与飞船实际位置分家。
      this.flown.push(this.stateAt(this.nextSampleDay).pos.clone());
      this.nextSampleDay += this.sampleStep;
    }
  }

  /** 1-sigma velocity uncertainty in m/s (convenience for the UI). */
  velocitySigmaMps(): number {
    return this.velocitySigma() / MPS_TO_AUDAY;
  }
}
