import type { Body, OrbitalElements } from '../data/bodies';
import { keplerState, StateVector } from '../physics/state';

// Adapter between the simulator's body data and the navigation layer. Keeping
// this interface small lets the planner work against any ephemeris source
// later (e.g. JPL Horizons vectors, or a star catalogue for interstellar
// routes) without touching the solver code.

export interface Ephemeris {
  /** Heliocentric state (AU, AU/day, ecliptic J2000) at `tDays` since J2000. */
  stateAt(tDays: number): StateVector;
}

/** Analytic Keplerian ephemeris for one body. */
export function keplerEphemeris(elements: OrbitalElements): Ephemeris {
  return { stateAt: (tDays) => keplerState(elements, tDays) };
}

/** Keplerian ephemeris built from the simulator's planet/moon data. */
export function bodyEphemeris(body: Body): Ephemeris {
  if (!body.orbit) throw new Error(`Ephemeris: body "${body.id}" has no orbital elements`);
  return keplerEphemeris(body.orbit);
}
