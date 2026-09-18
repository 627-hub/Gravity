import { Vector3 } from 'three';
import { describe, expect, it } from 'vitest';
import { PLANETS } from '../data/bodies';
import { keplerState } from '../physics/state';
import { angularMomentum, propagate, specificEnergy } from './propagate';
import { escapeSpeed } from './maneuvers';
import { MU_SUN } from './units';

function planet(id: string) {
  const p = PLANETS.find((b) => b.id === id);
  if (!p?.orbit) throw new Error(`missing planet ${id}`);
  return p;
}

describe('universal-variable propagation', () => {
  it.each(['mercury', 'earth', 'mars', 'jupiter'])(
    'matches keplerState for %s over 500 days',
    (id) => {
      const el = planet(id).orbit!;
      const s0 = keplerState(el, 0);
      const ref = keplerState(el, 500);
      const s1 = propagate(s0, 500, MU_SUN);
      expect(s1.pos.distanceTo(ref.pos)).toBeLessThan(1e-9);
      expect(s1.vel.distanceTo(ref.vel)).toBeLessThan(1e-11);
    },
  );

  it('propagates backwards exactly', () => {
    const el = planet('earth').orbit!;
    const s0 = keplerState(el, 1000);
    const ref = keplerState(el, 400);
    const s1 = propagate(s0, -600, MU_SUN);
    expect(s1.pos.distanceTo(ref.pos)).toBeLessThan(1e-9);
    expect(s1.vel.distanceTo(ref.vel)).toBeLessThan(1e-11);
  });

  it('conserves specific energy and angular momentum over 10 years', () => {
    const el = planet('mars').orbit!;
    let s = keplerState(el, 0);
    const e0 = specificEnergy(s, MU_SUN);
    const h0 = angularMomentum(s);
    for (let k = 0; k < 100; k++) s = propagate(s, 36.525, MU_SUN);
    expect(Math.abs((specificEnergy(s, MU_SUN) - e0) / e0)).toBeLessThan(1e-12);
    expect(angularMomentum(s).distanceTo(h0)).toBeLessThan(1e-13);
  });

  it('tracks a hyperbolic arc against an RK4 reference', () => {
    const state = {
      pos: new Vector3(1, 0, 0),
      vel: new Vector3(0, 1.2 * escapeSpeed(1, MU_SUN), 0),
    };
    const dt = 200;
    const s1 = propagate(state, dt, MU_SUN);
    expect(specificEnergy(state, MU_SUN)).toBeGreaterThan(0);

    // Independent RK4 integration of dr/dt = v, dv/dt = -mu r/|r|^3.
    let pos = state.pos.clone();
    let vel = state.vel.clone();
    const steps = 40000;
    const h = dt / steps;
    const acc = (p: Vector3) => p.clone().multiplyScalar(-MU_SUN / (p.lengthSq() * p.length()));
    for (let k = 0; k < steps; k++) {
      const k1v = acc(pos);
      const k1p = vel.clone();
      const k2v = acc(pos.clone().addScaledVector(k1p, h / 2));
      const k2p = vel.clone().addScaledVector(k1v, h / 2);
      const k3v = acc(pos.clone().addScaledVector(k2p, h / 2));
      const k3p = vel.clone().addScaledVector(k2v, h / 2);
      const k4v = acc(pos.clone().addScaledVector(k3p, h));
      const k4p = vel.clone().addScaledVector(k3v, h);
      pos.addScaledVector(k1p, h / 6).addScaledVector(k2p, h / 3).addScaledVector(k3p, h / 3).addScaledVector(k4p, h / 6);
      vel.addScaledVector(k1v, h / 6).addScaledVector(k2v, h / 3).addScaledVector(k3v, h / 3).addScaledVector(k4v, h / 6);
    }
    expect(s1.pos.distanceTo(pos)).toBeLessThan(1e-6);
    expect(s1.vel.distanceTo(vel)).toBeLessThan(1e-8);
  });
});
