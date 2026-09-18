import { Vector3 } from 'three';
import { describe, expect, it } from 'vitest';
import { PLANETS } from '../data/bodies';
import { keplerState } from '../physics/state';
import { elementsFromState } from './elements';
import { orbitalPeriodDays } from '../physics/kepler';

describe('elementsFromState', () => {
  it('recovers the orbital elements the state was built from', () => {
    for (const id of ['earth', 'mars', 'mercury']) {
      const body = PLANETS.find((b) => b.id === id)!;
      const el = body.orbit!;
      const state = keplerState(el, 1234);
      const out = elementsFromState(state);
      expect(out.a).toBeCloseTo(el.a, 6);
      expect(out.e).toBeCloseTo(el.e, 6);
      expect(out.iDeg).toBeCloseTo(Math.abs(el.i), 6); // computed i is non-negative
      expect(out.energy).toBeLessThan(0);
      expect(out.periodDays).not.toBeNull();
      if (out.periodDays !== null) {
        expect(out.periodDays).toBeCloseTo(orbitalPeriodDays(el.a), 3);
      }
    }
  });

  it('handles hyperbolic arcs (negative a, no period)', () => {
    const state = {
      pos: new Vector3(1, 0, 0),
      vel: new Vector3(0, 0.03, 0),
    };
    const out = elementsFromState(state);
    expect(out.energy).toBeGreaterThan(0);
    expect(out.a).toBeLessThan(0);
    expect(out.e).toBeGreaterThan(1);
    expect(out.periodDays).toBeNull();
  });
});
