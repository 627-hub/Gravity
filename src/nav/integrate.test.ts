import { Vector3 } from 'three';
import { describe, expect, it } from 'vitest';
import { PLANETS } from '../data/bodies';
import { AU } from '../data/constants';
import type { OrbitalElements } from '../data/bodies';
import { keplerState } from '../physics/state';
import { closestApproach, firstImpact } from './analysis';
import { bodyEphemeris } from './ephemeris';
import { integrate } from './integrate';
import { makeForceModel } from './perturbations';
import { propagate, specificEnergy } from './propagate';
import { solarSystemSources } from './sources';
import { MU_SUN, muAuOfMass } from './units';

const earth = PLANETS.find((b) => b.id === 'earth')!;
const sunOnly = () => makeForceModel([solarSystemSources()[0]]);

describe('adaptive trajectory integration', () => {
  it('reproduces analytic two-body propagation', () => {
    const s0 = keplerState(earth.orbit!, 0);
    const traj = integrate(s0, 0, 400, sunOnly(), { rtol: 1e-10, atol: 1e-12 });
    const end = traj[traj.length - 1];
    const ref = propagate(s0, 400, MU_SUN);
    expect(end.pos.distanceTo(ref.pos)).toBeLessThan(1e-8);
    expect(end.vel.distanceTo(ref.vel)).toBeLessThan(1e-9);
    expect(Math.abs(specificEnergy(end, MU_SUN) / specificEnergy(s0, MU_SUN) - 1)).toBeLessThan(1e-9);
  });

  it('integrates backwards to where it started', () => {
    const s0 = keplerState(earth.orbit!, 500);
    const forward = integrate(s0, 500, 300, sunOnly());
    const end = forward[forward.length - 1];
    const back = integrate({ pos: end.pos, vel: end.vel }, 800, -300, sunOnly());
    const home = back[back.length - 1];
    expect(home.pos.distanceTo(s0.pos)).toBeLessThan(1e-7);
    expect(home.vel.distanceTo(s0.vel)).toBeLessThan(1e-8);
  });

  it('keeps a 7000 km Earth orbit bound in the full solar system model', () => {
    const accel = makeForceModel(solarSystemSources());
    const r = 7000 / (AU / 1000); // km -> AU
    const e0 = bodyEphemeris(earth).stateAt(0);
    const vCirc = Math.sqrt(muAuOfMass(earth.mass) / r);
    const initial = {
      pos: e0.pos.clone().add(new Vector3(r, 0, 0)),
      vel: e0.vel.clone().add(new Vector3(0, vCirc, 0)),
    };
    const traj = integrate(initial, 0, 2, accel, { rtol: 1e-9, atol: 1e-14 });

    let min = Infinity;
    let max = 0;
    for (const p of traj) {
      const d = p.pos.distanceTo(bodyEphemeris(earth).stateAt(p.t).pos);
      if (d < min) min = d;
      if (d > max) max = d;
    }
    expect(min).toBeGreaterThan(r * 0.95);
    expect(max).toBeLessThan(r * 1.05);
  });

  it('adds a measurable third-body deflection over two years', () => {
    const el: OrbitalElements = { a: 1.5, e: 0.2, i: 3, node: 10, peri: 20, meanLongitude: 90 };
    const s0 = keplerState(el, 0);
    const twoBody = integrate(s0, 0, 800, sunOnly());
    const full = integrate(s0, 0, 800, makeForceModel(solarSystemSources()));
    const diff = twoBody[twoBody.length - 1].pos.distanceTo(full[full.length - 1].pos);
    expect(diff).toBeGreaterThan(1e-5);
    expect(diff).toBeLessThan(0.5);
  });
});

describe('closest approach and impact analysis', () => {
  const ellipse: OrbitalElements = { a: 1.2, e: 0.3, i: 0, node: 0, peri: 0, meanLongitude: 180 };
  const period = 2 * Math.PI * Math.sqrt(1.2 ** 3 / MU_SUN);

  it('locates perihelion on an eccentric heliocentric orbit', () => {
    const traj = integrate(keplerState(ellipse, 0), 0, period, sunOnly(), { rtol: 1e-11, atol: 1e-13 });
    const ca = closestApproach(traj, () => new Vector3());
    expect(ca.distance).toBeCloseTo(1.2 * (1 - 0.3), 4); // perihelion 0.84 AU
  });

  it('detects and locates an impact crossing', () => {
    const traj = integrate(keplerState(ellipse, 0), 0, period, sunOnly(), { rtol: 1e-11, atol: 1e-13 });
    const hit = firstImpact(traj, () => new Vector3(), 0.9);
    expect(hit).not.toBeNull();
    if (hit) {
      expect(hit.t).toBeGreaterThan(0.3 * period);
      expect(hit.t).toBeLessThan(0.6 * period);
    }
    expect(firstImpact(traj, () => new Vector3(), 0.5)).toBeNull();
  });
});
