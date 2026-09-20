import type { StateVector } from '../physics/state';
import { hermiteState } from './analysis';
import { integrate, type TrajectoryPoint } from './integrate';
import { makeForceModel, type SrpParams } from './perturbations';
import { bodyRadiusKm, solarSystemSources } from './sources';
import { GravitySource } from './perturbations';

// Truth trajectory for the flown mission: the spacecraft is integrated in the
// FULL force field (Sun + planets + moons + optional solar radiation
// pressure) instead of the two-body arc the onboard computer believes in.
// The difference between the two is real model error, which is what makes the
// navigation problem (and course corrections) interesting.

/** Default SRP model for a ~20 m^2 / 500 kg probe. */
export const DEFAULT_SRP: SrpParams = { cr: 1.3, areaM2: 20, massKg: 500 };

import { AU_KM } from '../data/constants';

/**
 * Per-source gravity softening = the body's own radius. The spacecraft now
 * starts in a *bound orbit* around the departure body (its synchronous-orbit
 * spaceport), so that body's point mass must stay in the force model — with a
 * softening scaled to the body, otherwise the port orbit would be destroyed by
 * a fixed large softening (or, without any, the launch phase would be singular).
 * Inside the body's radius the point mass is capped; the cruise is unaffected
 * (at 1 AU the term is < 1e-6 of solar gravity).
 */
export function truthSoftening(src: GravitySource): number {
  return bodyRadiusKm(src.id) / AU_KM;
}

export interface TruthOptions {
  /** Include solar radiation pressure (on by default). */
  srp?: boolean;
  /**
   * Body ids whose point mass is NOT applied during the cruise. The patched-
   * conic story: the departure/arrival body's gravity belongs to the launch /
   * arrival phase inside its sphere of influence, not to the heliocentric
   * arc — applying it here would produce a spurious pull towards a body the
   * arc starts (or ends) exactly on top of.
   */
  exclude?: string[];
}

export class TruthTrajectory {
  readonly startDay: number;
  readonly endDay: number;
  private samples: TrajectoryPoint[];

  constructor(state: StateVector, startDay: number, endDay: number, opts: TruthOptions = {}) {
    this.startDay = startDay;
    this.endDay = endDay;
    const skip = new Set(opts.exclude ?? []);
    const sources = solarSystemSources().filter((src) => !skip.has(src.id));
    const accel = makeForceModel(
      sources,
      (opts.srp ?? true) ? DEFAULT_SRP : undefined,
      truthSoftening,
    );
    this.samples = integrate(state, startDay, endDay - startDay, accel, {
      maxStep: 2,
      rtol: 1e-10,
      atol: 1e-12,
    });
  }

  /** Integrator samples (t, pos, vel) — also usable as a display path. */
  get path(): TrajectoryPoint[] {
    return this.samples;
  }

  /** Truth state at `t` within [startDay, endDay] (Hermite interpolation). */
  stateAt(t: number): StateVector {
    const s = this.samples;
    if (t <= s[0].t) return { pos: s[0].pos.clone(), vel: s[0].vel.clone() };
    if (t >= s[s.length - 1].t) {
      const last = s[s.length - 1];
      return { pos: last.pos.clone(), vel: last.vel.clone() };
    }
    let lo = 0;
    let hi = s.length - 1;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (s[mid].t <= t) lo = mid;
      else hi = mid;
    }
    return hermiteState(s[lo], s[hi], t);
  }
}
