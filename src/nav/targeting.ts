import { Vector3 } from 'three';
import type { StateVector } from '../physics/state';
import type { Ephemeris } from './ephemeris';
import { flybyOutbound } from './flyby';
import type { TrajectoryPoint } from './integrate';
import { propagate } from './propagate';
import { MU_SUN } from './units';

// Gravity-assist targeting: choose the flyby (periapsis radius + B-plane angle)
// so that, after the encounter, the spacecraft reaches a target body at a
// given epoch. The search is patched-conic: the flyby is the instantaneous
// v_inf rotation from flyby.ts, and the cruise arc is a heliocentric two-body
// propagation — the standard preliminary-design model, and fast enough for a
// two-dimensional scan followed by local refinement.

export interface ShootFlybyOptions {
  /** Flyby body ephemeris (provides position and velocity). */
  planet: Ephemeris;
  /** Flyby body GM, AU^3/day^2. */
  muPlanet: number;
  /** Epoch of closest approach, days since J2000. */
  tFlyby: number;
  /** Incoming v_inf relative to the flyby body, AU/day. */
  vInfIn: Vector3;
  /** Target body ephemeris (or a fixed point in space). */
  target: Ephemeris;
  /** Epoch at which the target must be reached. */
  tTarget: number;
  /** Periapsis search range, AU. */
  rpMin: number;
  rpMax: number;
  /** Initial grid resolution (defaults: 24 x 48). */
  rpSteps?: number;
  bAngleSteps?: number;
  /** Refinement effort: Nelder-Mead iterations per seed (default 200). */
  refineIterations?: number;
  /** Number of well-separated grid seeds to refine (default 4). */
  seeds?: number;
  /** Heliocentric mu, default Sun. */
  mu?: number;
  /** Trajectory samples for display (default 200). */
  samples?: number;
}

export interface FlybyAim {
  rp: number;
  bAngle: number;
  /** Separation from the target at tTarget, AU. */
  missDistance: number;
  vInfOut: Vector3;
  stateAfterFlyby: StateVector;
  /** Heliocentric cruise arc samples for display. */
  trajectory: TrajectoryPoint[];
}

interface Candidate {
  miss: number;
  vInfOut: Vector3;
  state: StateVector;
}

function evaluate(opts: ShootFlybyOptions, rp: number, bAngle: number, mu: number): Candidate {
  const planet = opts.planet.stateAt(opts.tFlyby);
  const vInfOut = flybyOutbound(opts.vInfIn, opts.muPlanet, rp, bAngle).vInfOut;
  const state: StateVector = {
    pos: planet.pos.clone(),
    vel: planet.vel.clone().add(vInfOut),
  };
  const arrive = propagate(state, opts.tTarget - opts.tFlyby, mu);
  const targetPos = opts.target.stateAt(opts.tTarget).pos;
  return { miss: arrive.pos.distanceTo(targetPos), vInfOut, state };
}

/** Nelder-Mead simplex minimisation of a 2D function. */
function nelderMead(
  f: (rp: number, bAngle: number) => number,
  start: { rp: number; bAngle: number },
  stepRp: number,
  stepB: number,
  maxIter: number,
): { rp: number; bAngle: number; f: number } {
  const alpha = 1;
  const gamma = 2;
  const rho = 0.5;
  const sigma = 0.5;
  const pts = [
    { rp: start.rp, bAngle: start.bAngle },
    { rp: start.rp + stepRp, bAngle: start.bAngle },
    { rp: start.rp, bAngle: start.bAngle + stepB },
  ].map((p) => ({ ...p, f: f(p.rp, p.bAngle) }));

  for (let iter = 0; iter < maxIter; iter++) {
    pts.sort((a, b) => a.f - b.f);
    const best = pts[0];
    const good = pts[1];
    const worst = pts[2];
    const crp = 0.5 * (best.rp + good.rp);
    const cb = 0.5 * (best.bAngle + good.bAngle);
    const xr = { rp: crp + alpha * (crp - worst.rp), bAngle: cb + alpha * (cb - worst.bAngle) };
    const fr = f(xr.rp, xr.bAngle);
    if (fr < best.f) {
      const xe = { rp: crp + gamma * (crp - worst.rp), bAngle: cb + gamma * (cb - worst.bAngle) };
      const fe = f(xe.rp, xe.bAngle);
      pts[2] = fe < fr ? { ...xe, f: fe } : { ...xr, f: fr };
    } else if (fr < good.f) {
      pts[2] = { ...xr, f: fr };
    } else {
      const xc = { rp: crp + rho * (worst.rp - crp), bAngle: cb + rho * (worst.bAngle - cb) };
      const fc = f(xc.rp, xc.bAngle);
      if (fc < worst.f) {
        pts[2] = { ...xc, f: fc };
      } else {
        for (let i = 1; i < 3; i++) {
          const shrunk = {
            rp: best.rp + sigma * (pts[i].rp - best.rp),
            bAngle: best.bAngle + sigma * (pts[i].bAngle - best.bAngle),
          };
          pts[i] = { ...shrunk, f: f(shrunk.rp, shrunk.bAngle) };
        }
      }
    }
  }
  pts.sort((a, b) => a.f - b.f);
  return pts[0];
}

const clamp = (x: number, lo: number, hi: number) => Math.min(hi, Math.max(lo, x));

/**
 * Find the flyby geometry that best reaches the target. Scans the (rp, bAngle)
 * plane on a grid, then refines the most promising, well-separated cells with
 * coordinate descent (golden-section sweeps) — a single-seed zoom can stall in
 * a wrong local minimum.
 */
export function shootFlyby(opts: ShootFlybyOptions): FlybyAim {
  const mu = opts.mu ?? MU_SUN;
  const rpSteps = opts.rpSteps ?? 24;
  const bAngleSteps = opts.bAngleSteps ?? 48;
  const refineIterations = opts.refineIterations ?? 200;
  const seeds = opts.seeds ?? 4;
  const samples = opts.samples ?? 200;

  // Coarse grid.
  const grid: ({ rp: number; bAngle: number } & Candidate)[] = [];
  for (let i = 0; i < rpSteps; i++) {
    const rp = rpSteps === 1 ? opts.rpMin : opts.rpMin + ((opts.rpMax - opts.rpMin) * i) / (rpSteps - 1);
    for (let j = 0; j < bAngleSteps; j++) {
      const bAngle = (2 * Math.PI * j) / bAngleSteps;
      grid.push({ rp, bAngle, ...evaluate(opts, rp, bAngle, mu) });
    }
  }
  grid.sort((a, b) => a.miss - b.miss);

  // Keep well-separated seeds so refinement explores different basins.
  const rpSpacing = (opts.rpMax - opts.rpMin) / Math.max(1, rpSteps - 1);
  const bSpacing = (2 * Math.PI) / bAngleSteps;
  const chosen: typeof grid = [];
  for (const cell of grid) {
    if (chosen.length >= seeds) break;
    const far = chosen.every(
      (c) =>
        Math.abs(c.rp - cell.rp) > 2 * rpSpacing || Math.abs(c.bAngle - cell.bAngle) > 2 * bSpacing,
    );
    if (far) chosen.push(cell);
  }

  let best = chosen[0];
  for (const seed of chosen) {
    const res = nelderMead(
      (rp, bAngle) => evaluate(opts, clamp(rp, opts.rpMin, opts.rpMax), bAngle, mu).miss,
      { rp: seed.rp, bAngle: seed.bAngle },
      rpSpacing * 0.5,
      bSpacing * 0.5,
      refineIterations,
    );
    if (res.f < best.miss) {
      const rp = clamp(res.rp, opts.rpMin, opts.rpMax);
      best = { rp, bAngle: res.bAngle, ...evaluate(opts, rp, res.bAngle, mu) };
    }
  }

  // Final cruise arc for display.
  const span = opts.tTarget - opts.tFlyby;
  const trajectory: TrajectoryPoint[] = [];
  for (let k = 0; k <= samples; k++) {
    const dt = (span * k) / samples;
    const s = propagate(best.state, dt, mu);
    trajectory.push({ t: opts.tFlyby + dt, pos: s.pos, vel: s.vel });
  }

  return {
    rp: best.rp,
    bAngle: best.bAngle,
    missDistance: best.miss,
    vInfOut: best.vInfOut,
    stateAfterFlyby: best.state,
    trajectory,
  };
}
