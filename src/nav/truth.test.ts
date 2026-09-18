import { describe, expect, it } from 'vitest';
import { PLANETS } from '../data/bodies';
import { AU } from '../data/constants';
import { bodyEphemeris } from './ephemeris';
import { bestTransfer } from './plan';
import { propagate } from './propagate';
import { TruthTrajectory } from './truth';
import { MU_SUN } from './units';

function j2000Days(y: number, m: number, d: number): number {
  return (Date.UTC(y, m - 1, d) - Date.UTC(2000, 0, 1, 12)) / 86400000;
}

const earth = PLANETS.find((b) => b.id === 'earth')!;
const mars = PLANETS.find((b) => b.id === 'mars')!;

const plan = bestTransfer({
  departure: bodyEphemeris(earth), target: bodyEphemeris(mars),
  departureId: 'earth', targetId: 'mars',
  tNow: j2000Days(2026, 1, 1),
  horizonDays: 900, departStep: 5, tofMin: 150, tofMax: 350, tofStep: 5,
})!;

describe('n-body truth trajectory', () => {
  it('covers the whole flight and integrates fast', () => {
    const t0 = performance.now();
    const tr = new TruthTrajectory(
      { pos: plan.r1.clone(), vel: plan.v1.clone() },
      plan.departureDay, plan.arrivalDay,
      { exclude: [plan.departureId, plan.targetId] },
    );
    const ms = performance.now() - t0;
    expect(tr.path.length).toBeGreaterThan(50);
    expect(tr.path[tr.path.length - 1].t).toBeCloseTo(plan.arrivalDay, 6);
    expect(ms).toBeLessThan(1500); // was 6 s before the softening fix
  });

  it('accumulates a realistic model error against the two-body plan', () => {
    const tr = new TruthTrajectory(
      { pos: plan.r1.clone(), vel: plan.v1.clone() },
      plan.departureDay, plan.arrivalDay,
      { exclude: [plan.departureId, plan.targetId] },
    );
    const driftKm = (t: number): number => {
      const twoBody = propagate({ pos: plan.r1, vel: plan.v1 }, t - plan.departureDay, MU_SUN);
      return (tr.stateAt(t).pos.distanceTo(twoBody.pos) * AU) / 1000;
    };
    // Early on the trajectories agree to ~100 km; by arrival the unmodelled
    // SRP + third bodies drift the truth ~10^5 km away from the plan.
    expect(driftKm(plan.departureDay + 16)).toBeLessThan(2000);
    const end = driftKm(plan.arrivalDay);
    expect(end).toBeGreaterThan(2e4);
    expect(end).toBeLessThan(1e6);
  });
});
