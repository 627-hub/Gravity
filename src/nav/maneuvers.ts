import { MU_SUN } from './units';

// Closed-form impulse maneuvers for preliminary mission design. All lengths in
// AU, times in days, mu in AU^3/day^2. Delta-v values are signed along the
// velocity direction (negative = retrograde burn); dvTotal sums magnitudes.

export interface ImpulsePlan {
  /** Signed burn magnitudes in AU/day (convert with toKms for km/s). */
  burns: number[];
  /** Sum of |burn|, AU/day. */
  dvTotal: number;
  /** Time of flight, days. */
  tof: number;
}

export interface HohmannPlan extends ImpulsePlan {
  dv1: number;
  dv2: number;
}

/** Circular orbit speed (AU/day). */
export function circularSpeed(r: number, mu: number = MU_SUN): number {
  return Math.sqrt(mu / r);
}

/** Escape speed from radius r (AU/day). */
export function escapeSpeed(r: number, mu: number = MU_SUN): number {
  return Math.sqrt((2 * mu) / r);
}

/**
 * Hohmann transfer between two circular coplanar orbits: two tangent burns on
 * a half-ellipse. Optimal in dV for radius ratios below ~11.94.
 */
export function hohmannTransfer(r1: number, r2: number, mu: number = MU_SUN): HohmannPlan {
  const aT = (r1 + r2) / 2;
  const v1 = circularSpeed(r1, mu);
  const v2 = circularSpeed(r2, mu);
  const vp = Math.sqrt(mu * (2 / r1 - 1 / aT));
  const va = Math.sqrt(mu * (2 / r2 - 1 / aT));
  const dv1 = vp - v1;
  const dv2 = v2 - va;
  const tof = Math.PI * Math.sqrt((aT * aT * aT) / mu);
  return { dv1, dv2, burns: [dv1, dv2], dvTotal: Math.abs(dv1) + Math.abs(dv2), tof };
}

/**
 * Bi-elliptic transfer via an intermediate apoapsis rApo: three burns.
 * Beats Hohmann above a radius ratio of ~15.58 (for a sufficiently distant
 * intermediate apoapsis).
 */
export function biEllipticTransfer(
  r1: number,
  r2: number,
  rApo: number,
  mu: number = MU_SUN,
): ImpulsePlan {
  if (rApo < Math.max(r1, r2)) {
    throw new Error('bi-elliptic: intermediate apoapsis must exceed both radii');
  }
  const a1 = (r1 + rApo) / 2;
  const a2 = (r2 + rApo) / 2;
  const v1 = circularSpeed(r1, mu);
  const v2 = circularSpeed(r2, mu);
  const dv1 = Math.sqrt(mu * (2 / r1 - 1 / a1)) - v1;
  const dv2 =
    Math.sign(r2 - r1) *
    Math.abs(Math.sqrt(mu * (2 / rApo - 1 / a2)) - Math.sqrt(mu * (2 / rApo - 1 / a1)));
  const dv3 = v2 - Math.sqrt(mu * (2 / r2 - 1 / a2));
  const tof = Math.PI * (Math.sqrt((a1 * a1 * a1) / mu) + Math.sqrt((a2 * a2 * a2) / mu));
  return {
    burns: [dv1, dv2, dv3],
    dvTotal: Math.abs(dv1) + Math.abs(dv2) + Math.abs(dv3),
    tof,
  };
}

/** Speed change for a pure plane change at speed v by angle `angleDeg` (deg). */
export function planeChangeDeltaV(v: number, angleDeg: number): number {
  return 2 * v * Math.sin((angleDeg * Math.PI) / 180 / 2);
}

/**
 * Burn needed to leave a circular parking orbit of radius rPark onto a
 * hyperbola with the given hyperbolic excess speed vInf (both km/s, muPlanet
 * in km^3/s^2). Same formula in reverse gives the capture burn.
 */
export function parkingOrbitDeltaV(rPark: number, vInfKms: number, muPlanetKms: number): number {
  const vPeri = Math.sqrt(vInfKms * vInfKms + (2 * muPlanetKms) / rPark);
  const vCirc = Math.sqrt(muPlanetKms / rPark);
  return vPeri - vCirc;
}
