import { Vector3 } from 'three';
import { describe, expect, it } from 'vitest';
import { PLANETS, SUN } from '../data/bodies';
import { AU } from '../data/constants';
import { bodyEphemeris } from './ephemeris';
import { makeForceModel } from './perturbations';
import { soiRadius, solarSystemSources } from './sources';
import { MU_SUN, muAuOfMass } from './units';

const earth = PLANETS.find((b) => b.id === 'earth')!;

describe('gravity sources and force model', () => {
  it('computes the Earth sphere of influence', () => {
    const r = soiRadius(1, SUN.mass, earth.mass); // mass ratio, length in AU
    const km = (r * AU) / 1000;
    expect(km).toBeGreaterThan(850000);
    expect(km).toBeLessThan(1000000); // textbook value ≈ 924,000 km
  });

  it('lists the Sun and every planet as sources', () => {
    const sources = solarSystemSources();
    expect(sources[0].id).toBe('sun');
    expect(sources.length).toBe(1 + PLANETS.length);
    const withMoons = solarSystemSources({ includeMoons: true });
    expect(withMoons.length).toBeGreaterThan(sources.length);
  });

  it('reproduces Sun-only gravity exactly', () => {
    const [sun] = solarSystemSources();
    const accel = makeForceModel([sun]);
    const pos = new Vector3(0.7, -0.4, 0.05);
    const a = accel(0, pos, new Vector3());
    expect(a.length()).toBeCloseTo(MU_SUN / pos.lengthSq(), 12);
    expect(a.clone().normalize().distanceTo(pos.clone().normalize().negate())).toBeLessThan(1e-12);
  });

  it('is dominated by Earth gravity 7000 km from Earth', () => {
    const accel = makeForceModel(solarSystemSources());
    const e0 = bodyEphemeris(earth).stateAt(0);
    const r = 7000 / ((AU / 1000) * 1); // 7000 km in AU
    const pos = e0.pos.clone().add(new Vector3(r, 0, 0));
    const a = accel(0, pos, new Vector3());
    const expected = muAuOfMass(earth.mass) / (r * r);
    expect(a.length()).toBeGreaterThan(expected * 0.9);
    expect(a.length()).toBeLessThan(expected * 1.1); // ~0.406 AU/day^2
  });

  it('models solar radiation pressure pushing away from the Sun', () => {
    const [sun] = solarSystemSources();
    const accel = makeForceModel([sun], { cr: 1, areaM2: 100, massKg: 10000 });
    const pos = new Vector3(1, 0, 0);
    const a = accel(0, pos, new Vector3());
    // Gravity minus a small outward push: net points inward but is weakened.
    const gravity = MU_SUN / 1;
    const srp = gravity - a.length();
    // (4.56e-6 N/m^2 * 0.01 m^2/kg) at 1 AU ≈ 2.28e-9 AU/day^2
    expect(srp).toBeGreaterThan(2.0e-9);
    expect(srp).toBeLessThan(2.6e-9);
  });
});
