import { Vector3 } from 'three';
import type { StateVector } from '../physics/state';
import { AU_KM } from '../data/constants';
import { PLANETS } from '../data/bodies';
import { bodyEphemeris, type Ephemeris } from './ephemeris';
import { lambert } from './lambert';
import {
  makeMeasurement, makeOpticalMeasurement, mulberry32, Navigator, tierNoise,
  type OpticalMeasurement, type RangeMeasurement, type TrackingNoise, type TrackingTier,
} from './navigator';
import type { TransferPlan } from './plan';
import { propagate } from './propagate';
import { TruthTrajectory } from './truth';
import { MPS_TO_AUDAY, MU_SUN, toKms } from './units';

// A flown mission: the planned Lambert arc plus execution errors, noisy
// tracking and TCM (trajectory correction manoeuvre) history.
//
// TRUTH physics default to N-BODY: the trajectory is integrated in the full
// force field (Sun + planets + moons + SRP, patched-conic exclusions for the
// departure/arrival body), while the onboard computer keeps its two-body
// model. The difference is genuine model error — the L2 problem the filter
// and the TCMs must live with. 'two-body' recovers the classic exact arcs.
//
// Two state tracks live here:
//   - the TRUTH state (what actually happens), propagated between burns;
//   - the onboard ESTIMATE (an EKF fed with noisy range/range-rate), which is
//     all the spacecraft is allowed to see when planning a correction.
// Without tracking the estimate equals the truth (the L0 model); with tracking
// corrections are imperfect and leave a residual miss — the L1 story.

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

/** Launch dispersion injected for demonstration: 0.25% of the velocity. */
export const LAUNCH_ERROR_FRACTION = 0.0025;

/** Tracking cadence, days between ground-station passes. */
export const TRACK_INTERVAL_DAYS = 2;

/** A-priori state uncertainty before any tracking (loose launch knowledge). */
const P0_POS_KM = 1000;
const P0_VEL_AUDAY = 0.05;

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
    this.lastDay = -Infinity;
    this.nextTrackDay = this.plan.departureDay;
    this.nextSampleDay = this.plan.departureDay;
    this.lastRangeMeas = null;
    this.lastOpticalMeas = null;
    this.navigator = this.trackNoise
      ? new Navigator(
          { pos: this.plan.r1.clone(), vel: this.plan.v1.clone() },
          this.plan.departureDay,
          {
            noise: this.trackNoise,
            posSigma0: P0_POS_KM / AU_KM,
            velSigma0: P0_VEL_AUDAY,
          },
        )
      : null;

    const v = this.plan.v1.clone();
    if (this.injectError) {
      v.addScaledVector(this.launchErrorDir(), v.length() * LAUNCH_ERROR_FRACTION);
    }
    const first: FlightSegment = {
      state: { pos: this.plan.r1.clone(), vel: v },
      startDay: this.plan.departureDay,
      endDay: this.plan.arrivalDay,
    };
    this.segments.push(first);
    this.trajectories = [];
    if (this.truthPhysics === 'n-body') {
      this.trajectories.push(
        new TruthTrajectory(first.state, first.startDay, first.endDay, {
          srp: this.srp,
          exclude: [this.plan.departureId, this.plan.targetId],
        }),
      );
    }
    this.predicted = this.sampleArc(first.state, first.startDay, first.endDay);
    this.lastPredictDay = -Infinity;
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

  /** Rebuild the onboard predicted arc from the current estimate. */
  private refreshPrediction(t: number): void {
    const est = this.navigator ? this.navigator.state() : this.stateAt(t);
    const n = 160;
    const out: Vector3[] = [];
    const span = this.plan.arrivalDay - t;
    for (let k = 0; k <= n; k++) {
      out.push(propagate(est, (span * k) / n, MU_SUN).pos);
    }
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
    try {
      const { v1 } = lambert(est.pos, this.plan.r2, this.plan.arrivalDay - t, MU_SUN);
      const dv = v1.clone().sub(est.vel);
      return { dv, dvKms: toKms(dv.length()), state: { pos: est.pos.clone(), vel: v1 } };
    } catch {
      return null;
    }
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
          exclude: [this.plan.departureId, this.plan.targetId],
        }),
      );
    }
    this.navigator?.applyBurn(sol.dv);
    this.refreshPrediction(t);
    this.tcmCount += 1;
    this.tcmUsedKms += sol.dvKms;
    return sol.dvKms;
  }

  /** Append actual-path samples up to `t`; safe to call every frame. */
  updateFlown(t: number): void {
    const end = Math.min(t, this.plan.arrivalDay);
    if (end < this.plan.departureDay) return;
    while (this.nextSampleDay <= end) {
      const seg = this.segmentFor(this.nextSampleDay);
      this.flown.push(propagate(seg.state, this.nextSampleDay - seg.startDay, MU_SUN).pos);
      this.nextSampleDay += this.sampleStep;
    }
  }

  /** 1-sigma velocity uncertainty in m/s (convenience for the UI). */
  velocitySigmaMps(): number {
    return this.velocitySigma() / MPS_TO_AUDAY;
  }
}
