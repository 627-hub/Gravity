import { Vector3 } from 'three';
import { describe, expect, it } from 'vitest';
import { PLANETS } from '../data/bodies';
import { bodyEphemeris } from './ephemeris';
import { bestTransfer } from './plan';
import { propagate } from './propagate';
import { MU_SUN } from './units';

const earth = PLANETS.find((b) => b.id === 'earth')!;
const mars = PLANETS.find((b) => b.id === 'mars')!;

function j2000Days(y: number, m: number, d: number): number {
  return (Date.UTC(y, m - 1, d) - Date.UTC(2000, 0, 1, 12)) / 86400000;
}

describe('bestTransfer', () => {
  it('finds the late-2026 Earth→Mars window from a rolling horizon', () => {
    const plan = bestTransfer({
      departure: bodyEphemeris(earth),
      target: bodyEphemeris(mars),
      departureId: 'earth',
      targetId: 'mars',
      tNow: j2000Days(2026, 1, 1),
      horizonDays: 900,
      departStep: 5,
      tofMin: 150,
      tofMax: 350,
      tofStep: 5,
    });
    expect(plan).not.toBeNull();
    if (!plan) return;
    expect(plan.dvTotal).toBeGreaterThan(4);
    expect(plan.dvTotal).toBeLessThan(9);
    expect(plan.departureDay).toBeGreaterThan(j2000Days(2026, 7, 1));
    expect(plan.departureDay).toBeLessThan(j2000Days(2027, 4, 1));
    expect(plan.tof).toBeGreaterThanOrEqual(150);
    expect(plan.tof).toBeLessThanOrEqual(350);
    expect(plan.arrivalDay).toBeCloseTo(plan.departureDay + plan.tof, 9);

    // The planned states must actually connect: propagate v1 for tof → r2.
    const arrived = propagate({ pos: plan.r1, vel: plan.v1 }, plan.tof, MU_SUN);
    expect(arrived.pos.distanceTo(plan.r2)).toBeLessThan(1e-7);
  });

  it('returns null when every cell is degenerate', () => {
    const fixed = {
      stateAt: () => ({ pos: new Vector3(1, 0, 0), vel: new Vector3(0, 0.017, 0) }),
    };
    const plan = bestTransfer({
      departure: fixed,
      target: fixed,
      tNow: 0,
      horizonDays: 10,
      departStep: 5,
      tofMin: 100,
      tofMax: 110,
      tofStep: 5,
    });
    expect(plan).toBeNull();
  });
});
