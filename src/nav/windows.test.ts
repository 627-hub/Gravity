import { describe, expect, it } from 'vitest';
import { PLANETS } from '../data/bodies';
import { bodyEphemeris } from './ephemeris';
import { daysGrid, decodeIndex, porkchop } from './windows';

/** Days since J2000 (2000-01-01 12:00 UTC) for a calendar date. */
function j2000Days(y: number, m: number, d: number): number {
  return (Date.UTC(y, m - 1, d) - Date.UTC(2000, 0, 1, 12)) / 86400000;
}

describe('launch-window scanning', () => {
  it('builds inclusive day grids', () => {
    expect(Array.from(daysGrid(0, 10, 5))).toEqual([0, 5, 10]);
    expect(Array.from(daysGrid(0, 9, 5))).toEqual([0, 5]);
    expect(() => daysGrid(0, 10, 0)).toThrow();
  });

  it('finds the late-2026 Earth→Mars launch window', () => {
    const earth = PLANETS.find((b) => b.id === 'earth')!;
    const mars = PLANETS.find((b) => b.id === 'mars')!;
    const result = porkchop({
      departure: bodyEphemeris(earth),
      arrival: bodyEphemeris(mars),
      departFrom: j2000Days(2026, 1, 1),
      departTo: j2000Days(2028, 6, 1),
      departStep: 5,
      tofMin: 100,
      tofMax: 400,
      tofStep: 5,
    });

    expect(result.bestIndex).toBeGreaterThanOrEqual(0);
    const total = result.dvTotal[result.bestIndex];
    expect(total).toBeGreaterThan(4); // real eccentric-orbit transfers need ~5-7 km/s
    expect(total).toBeLessThan(9);

    // Mars windows in this synodic cycle depart Oct–Dec 2026 and arrive mid-2027.
    const { i, j } = decodeIndex(result, result.bestIndex);
    const departDay = result.departDays[i];
    const tof = result.tofDays[j];
    expect(departDay).toBeGreaterThan(j2000Days(2026, 7, 1));
    expect(departDay).toBeLessThan(j2000Days(2027, 4, 1));
    expect(tof).toBeGreaterThan(150);
    expect(tof).toBeLessThan(350);

    // With a 0-rev solver every non-degenerate cell should be feasible.
    let nan = 0;
    for (let k = 0; k < result.dvTotal.length; k++) {
      if (Number.isNaN(result.dvTotal[k])) nan++;
    }
    expect(nan).toBeLessThan(result.dvTotal.length * 0.1);
  });
});
