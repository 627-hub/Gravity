import { Vector3 } from 'three';
import { describe, expect, it } from 'vitest';
import { PLANETS } from '../data/bodies';
import { keplerState } from '../physics/state';
import { lambert } from './lambert';
import { propagate, specificEnergy } from './propagate';
import { MU_SUN, toKms } from './units';

describe('Lambert solver (Izzo)', () => {
  it('round-trips random elliptic transfers', () => {
    // Deterministic LCG so a failure is reproducible.
    let seed = 42;
    const rand = () => {
      seed = (seed * 1664525 + 1013904223) >>> 0;
      return seed / 2 ** 32;
    };
    const randPos = (rMin: number, rMax: number) => {
      const r = rMin + rand() * (rMax - rMin);
      const theta = rand() * 2 * Math.PI;
      const z = (rand() - 0.5) * 0.3;
      return new Vector3(r * Math.cos(theta), r * Math.sin(theta), z);
    };

    for (let n = 0; n < 40; n++) {
      const r1 = randPos(0.7, 2.0);
      const r2 = randPos(0.7, 3.5);
      const tof = 60 + rand() * 500;
      const { v1, v2 } = lambert(r1, r2, tof, MU_SUN);
      const arrived = propagate({ pos: r1, vel: v1 }, tof, MU_SUN);
      expect(arrived.pos.distanceTo(r2)).toBeLessThan(1e-7);
      expect(arrived.vel.distanceTo(v2)).toBeLessThan(1e-8);
    }
  });

  it('solves hyperbolic arcs', () => {
    const r1 = new Vector3(1, 0, 0);
    const r2 = new Vector3(3.2, 1.5, 0);
    const tof = 90;
    const { v1, v2 } = lambert(r1, r2, tof, MU_SUN);
    const arrived = propagate({ pos: r1, vel: v1 }, tof, MU_SUN);
    expect(arrived.pos.distanceTo(r2)).toBeLessThan(1e-7);
    expect(arrived.vel.distanceTo(v2)).toBeLessThan(1e-8);
    expect(specificEnergy({ pos: r1, vel: v1 }, MU_SUN)).toBeGreaterThan(0);
  });

  it('handles retrograde arcs', () => {
    const r1 = new Vector3(1, 0, 0);
    const r2 = new Vector3(0, -1.4, 0);
    const tof = 250;
    const { v1, v2 } = lambert(r1, r2, tof, MU_SUN, { prograde: false });
    const arrived = propagate({ pos: r1, vel: v1 }, tof, MU_SUN);
    expect(arrived.pos.distanceTo(r2)).toBeLessThan(1e-7);
    expect(arrived.vel.distanceTo(v2)).toBeLessThan(1e-8);
    expect(new Vector3().crossVectors(r1, v1).z).toBeLessThan(0);
  });

  it('recovers an analytic multi-revolution ellipse (M = 1)', () => {
    // Closed-form ellipse: a = 1.5 AU, e = 0.2, in the ecliptic plane.
    const a = 1.5;
    const e = 0.2;
    const p = a * (1 - e * e);
    const n = Math.sqrt(MU_SUN / (a * a * a));
    const stateAtNu = (nu: number) => {
      const r = p / (1 + e * Math.cos(nu));
      const pos = new Vector3(r * Math.cos(nu), r * Math.sin(nu), 0);
      const vr = Math.sqrt(MU_SUN / p) * e * Math.sin(nu);
      const vt = Math.sqrt(MU_SUN / p) * (1 + e * Math.cos(nu));
      const rHat = pos.clone().divideScalar(r);
      const tHat = new Vector3(-rHat.y, rHat.x, 0);
      const vel = rHat.clone().multiplyScalar(vr).addScaledVector(tHat, vt);
      const E = 2 * Math.atan2(Math.sqrt(1 - e) * Math.sin(nu / 2), Math.sqrt(1 + e) * Math.cos(nu / 2));
      const M = E - e * Math.sin(E);
      return { pos, vel, M };
    };

    const s1 = stateAtNu((30 * Math.PI) / 180);
    const s2 = stateAtNu((250 * Math.PI) / 180);
    const period = (2 * Math.PI) / n;
    const single = ((s2.M - s1.M + 2 * Math.PI) % (2 * Math.PI)) / n;
    const tof = single + period; // one extra full revolution

    // Two M = 1 solutions exist; the analytic ellipse is one of them.
    const matches = [true, false]
      .map((lowpath) => {
        try {
          const { v1, v2 } = lambert(s1.pos, s2.pos, tof, MU_SUN, { revs: 1, lowpath });
          return v1.distanceTo(s1.vel) + v2.distanceTo(s2.vel);
        } catch {
          return Infinity;
        }
      })
      .filter((err) => err < 1e-8);
    expect(matches.length).toBeGreaterThan(0);
  });

  it('produces an Earth→Mars transfer in the right ballpark', () => {
    const earth = PLANETS.find((b) => b.id === 'earth')!;
    const mars = PLANETS.find((b) => b.id === 'mars')!;
    // Best cell of the late-2026 Mars window (found by windows.test.ts).
    const t0 = (Date.UTC(2026, 9, 28) - Date.UTC(2000, 0, 1, 12)) / 86400000;
    const tof = 310;
    const s1 = keplerState(earth.orbit!, t0);
    const s2 = keplerState(mars.orbit!, t0 + tof);
    const { v1, v2 } = lambert(s1.pos, s2.pos, tof, MU_SUN);
    const dvDepart = toKms(v1.clone().sub(s1.vel).length());
    const dvArrive = toKms(v2.clone().sub(s2.vel).length());
    expect(dvDepart).toBeCloseTo(3.0, 0); // textbook Earth→Mars v_inf ~ 3 km/s
    expect(dvArrive).toBeCloseTo(2.6, 0);
    const arrived = propagate({ pos: s1.pos, vel: v1 }, tof, MU_SUN);
    expect(arrived.pos.distanceTo(s2.pos)).toBeLessThan(1e-7);
  });

  it('stays finite through the near-180° singularity', () => {
    // Mid-October 2026 with a 240-day flight is within ~2° of a collinear
    // transfer: the plane is then set by tiny out-of-plane components, so the
    // required dV legitimately spikes. The solver must still return an exact
    // arc (no NaN) rather than picking a wrong branch.
    const earth = PLANETS.find((b) => b.id === 'earth')!;
    const mars = PLANETS.find((b) => b.id === 'mars')!;
    const t0 = (Date.UTC(2026, 9, 15) - Date.UTC(2000, 0, 1, 12)) / 86400000;
    const tof = 240;
    const s1 = keplerState(earth.orbit!, t0);
    const s2 = keplerState(mars.orbit!, t0 + tof);
    const { v1, v2 } = lambert(s1.pos, s2.pos, tof, MU_SUN);
    expect(Number.isFinite(v1.x + v1.y + v1.z + v2.x + v2.y + v2.z)).toBe(true);
    const arrived = propagate({ pos: s1.pos, vel: v1 }, tof, MU_SUN);
    expect(arrived.pos.distanceTo(s2.pos)).toBeLessThan(1e-7);
    expect(arrived.vel.distanceTo(v2)).toBeLessThan(1e-6);
  });

  it('rejects degenerate inputs', () => {
    expect(() => lambert(new Vector3(1, 0, 0), new Vector3(-1, 0, 0), 200, MU_SUN)).toThrow();
    expect(() => lambert(new Vector3(1, 0, 0), new Vector3(0, 1, 0), 0, MU_SUN)).toThrow();
  });
});
