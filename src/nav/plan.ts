import type { Vector3 } from 'three';
import type { Ephemeris } from './ephemeris';
import { lambert } from './lambert';
import { MU_SUN, toKms } from './units';

// High-level mission planning: pick the cheapest direct transfer between two
// bodies over a rolling departure horizon. This is what the navigation UI
// calls when the user asks "when should I leave, and how much delta-v?".

export interface TransferPlan {
  departureId: string;
  targetId: string;
  /** Departure epoch, days since J2000. */
  departureDay: number;
  /** Time of flight, days. */
  tof: number;
  arrivalDay: number;
  /** Hyperbolic excess speeds at both ends, km/s. */
  dvDepart: number;
  dvArrive: number;
  dvTotal: number;
  /** Heliocentric ecliptic states: position/velocity at departure and arrival. */
  r1: Vector3;
  v1: Vector3;
  r2: Vector3;
  v2: Vector3;
}

export interface BestTransferOptions {
  departure: Ephemeris;
  target: Ephemeris;
  departureId?: string;
  targetId?: string;
  /** Current epoch, days since J2000 (the horizon starts here). */
  tNow: number;
  horizonDays?: number;
  departStep?: number;
  tofMin?: number;
  tofMax?: number;
  tofStep?: number;
  mu?: number;
}

/** Cheapest direct transfer found on the (departure x tof) grid, or null. */
export function bestTransfer(opts: BestTransferOptions): TransferPlan | null {
  const mu = opts.mu ?? MU_SUN;
  const horizon = opts.horizonDays ?? 1200;
  const dStep = opts.departStep ?? 5;
  const tofMin = opts.tofMin ?? 100;
  const tofMax = opts.tofMax ?? 420;
  const tofStep = opts.tofStep ?? 5;

  let best: TransferPlan | null = null;
  for (let dt = 0; dt <= horizon; dt += dStep) {
    const t = opts.tNow + dt;
    const s1 = opts.departure.stateAt(t);
    for (let tof = tofMin; tof <= tofMax; tof += tofStep) {
      const s2 = opts.target.stateAt(t + tof);
      try {
        const { v1, v2 } = lambert(s1.pos, s2.pos, tof, mu);
        const dvDepart = toKms(v1.clone().sub(s1.vel).length());
        const dvArrive = toKms(v2.clone().sub(s2.vel).length());
        const dvTotal = dvDepart + dvArrive;
        if (!best || dvTotal < best.dvTotal) {
          best = {
            departureId: opts.departureId ?? '',
            targetId: opts.targetId ?? '',
            departureDay: t,
            tof,
            arrivalDay: t + tof,
            dvDepart,
            dvArrive,
            dvTotal,
            r1: s1.pos.clone(),
            v1,
            r2: s2.pos.clone(),
            v2,
          };
        }
      } catch {
        // Infeasible geometry for this cell (degenerate/collinear positions).
      }
    }
  }
  return best;
}
