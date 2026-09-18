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
