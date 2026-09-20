import { PLANETS, SUN } from '../data/bodies';
import { buildSimBodies, descriptorState } from '../data/system';
import type { GravitySource } from './perturbations';
import { muAuOfMass } from './units';

// Gravity sources built from the simulator's own body data, so the navigation
// layer and the renderer agree on where everything is at any epoch.

/**
 * Laplace sphere-of-influence radius: the practical handoff distance between
 * patched-conic segments, r_SOI = a (m / M)^(2/5). Same length unit as `a`.
 */
export function soiRadius(a: number, primaryMu: number, secondaryMu: number): number {
  return a * Math.pow(secondaryMu / primaryMu, 2 / 5);
}

/** 天体半径（km，含月球）——真值积分按天体自身尺度做软化。 */
export function bodyRadiusKm(id: string): number {
  const planet = PLANETS.find((p) => p.id === id);
  if (planet) return planet.radius;
  for (const p of PLANETS) {
    const moon = p.moons?.find((m) => m.id === id);
    if (moon) return moon.radius;
  }
  return SUN.radius;
}

/**
 * Sun + planets (optionally dynamically significant moons as well) as moving
 * point-mass sources in heliocentric coordinates.
 */
export function solarSystemSources(opts: { includeMoons?: boolean } = {}): GravitySource[] {
  return buildSimBodies(opts.includeMoons ?? false).map((d) => ({
    id: d.id,
    mu: muAuOfMass(d.mass),
    positionAt: (tDays) => descriptorState(d, tDays).pos,
  }));
}
