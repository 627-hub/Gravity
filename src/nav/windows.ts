import { lambert } from './lambert';
import { MU_SUN, toKms } from './units';
import type { Ephemeris } from './ephemeris';

// Launch-window scanning: cross every departure date with every time of flight,
// solve Lambert for each pair and record the hyperbolic excess speeds. The
// output is a plain grid (Float64Array) so a UI can map it straight onto a
// porkchop plot. Values are km/s; cells with no feasible solution are NaN.

export interface PorkchopOptions {
  departure: Ephemeris;
  arrival: Ephemeris;
  /** Departure epoch range, days since J2000. */
  departFrom: number;
  departTo: number;
  departStep: number;
  /** Time-of-flight range, days. */
  tofMin: number;
  tofMax: number;
  tofStep: number;
  mu?: number;
  prograde?: boolean;
  revs?: number;
}

export interface PorkchopResult {
  /** Grid axes: departure epochs and times of flight, days. */
  departDays: Float64Array;
  tofDays: Float64Array;
  /** v_inf at departure / arrival and their sum, km/s; NaN = infeasible. */
  dvDepart: Float64Array;
  dvArrive: Float64Array;
  dvTotal: Float64Array;
  /** Index into dvTotal of the minimum-dV cell (row-major: i * tofCount + j). */
  bestIndex: number;
}

/** Evenly spaced grid values from `from` to `to` inclusive. */
export function daysGrid(from: number, to: number, step: number): Float64Array {
  if (!(step > 0)) throw new Error('daysGrid: step must be positive');
  const n = Math.floor((to - from) / step + 1e-9) + 1;
  const out = new Float64Array(Math.max(1, n));
  for (let i = 0; i < out.length; i++) out[i] = from + i * step;
  return out;
}

export function porkchop(opts: PorkchopOptions): PorkchopResult {
  const { departure, arrival, departFrom, departTo, departStep, tofMin, tofMax, tofStep } = opts;
  const mu = opts.mu ?? MU_SUN;
  const prograde = opts.prograde ?? true;
  const revs = opts.revs ?? 0;

  const departDays = daysGrid(departFrom, departTo, departStep);
  const tofDays = daysGrid(tofMin, tofMax, tofStep);
  const size = departDays.length * tofDays.length;
  const dvDepart = new Float64Array(size);
  const dvArrive = new Float64Array(size);
  const dvTotal = new Float64Array(size);

  let bestIndex = -1;
  let bestDv = Infinity;

  for (let i = 0; i < departDays.length; i++) {
    const t0 = departDays[i];
    const s1 = departure.stateAt(t0);
    for (let j = 0; j < tofDays.length; j++) {
      const tof = tofDays[j];
      const k = i * tofDays.length + j;
      const s2 = arrival.stateAt(t0 + tof);
      let total = NaN;
      try {
        const { v1, v2 } = lambert(s1.pos, s2.pos, tof, mu, { prograde, revs });
        const dvD = toKms(v1.clone().sub(s1.vel).length());
        const dvA = toKms(v2.clone().sub(s2.vel).length());
        dvDepart[k] = dvD;
        dvArrive[k] = dvA;
        total = dvD + dvA;
        dvTotal[k] = total;
      } catch {
        // Infeasible geometry for this rev budget (or degenerate positions).
        dvDepart[k] = NaN;
        dvArrive[k] = NaN;
        dvTotal[k] = NaN;
      }
      if (total < bestDv) {
        bestDv = total;
        bestIndex = k;
      }
    }
  }

  return { departDays, tofDays, dvDepart, dvArrive, dvTotal, bestIndex };
}

/** Decode a grid index into departure/tof indices. */
export function decodeIndex(result: PorkchopResult, index: number): { i: number; j: number } {
  return { i: Math.floor(index / result.tofDays.length), j: index % result.tofDays.length };
}
