import { Vector3 } from 'three';
import { AU, DAY } from '../data/constants';

// Force model for trajectory propagation: point-mass gravity from any number
// of moving sources (Sun, planets, moons) plus optional solar radiation
// pressure. This is the "other bodies matter" layer — patched conics ignores
// third bodies entirely, direct N-body integration does not.

export interface GravitySource {
  id: string;
  /** Standard gravitational parameter, AU^3/day^2. */
  mu: number;
  /** Heliocentric position at `tDays` since J2000, AU. */
  positionAt(tDays: number): Vector3;
}

export interface SrpParams {
  /** Radiation pressure coefficient: 1 = perfect absorber, 2 = perfect mirror. */
  cr: number;
  areaM2: number;
  massKg: number;
}

/** Acceleration callback: writes d2r/dt2 (AU/day^2) for a state at time t. */
export type AccelFn = (tDays: number, pos: Vector3, out: Vector3) => Vector3;

/** Solar radiation pressure at 1 AU, N/m^2. */
export const P_SRP_1AU = 4.56e-6;

/**
 * Build the acceleration function for a spacecraft in heliocentric ecliptic
 * coordinates. Every source pulls with mu/r^2 towards its instantaneous
 * position; SRP pushes radially away from the Sun (only if a Sun source is
 * present and `srp` is given).
 */
export function makeForceModel(sources: GravitySource[], srp?: SrpParams): AccelFn {
  const sun = sources.find((s) => s.id === 'sun');
  // (m/s^2) -> (AU/day^2) for a unit 1/r^2 factor.
  const srpFactor = srp
    ? ((P_SRP_1AU * srp.cr * srp.areaM2) / srp.massKg) * ((DAY * DAY) / AU)
    : 0;

  const d = new Vector3();
  return (t: number, pos: Vector3, out: Vector3): Vector3 => {
    out.set(0, 0, 0);
    for (const s of sources) {
      d.copy(s.positionAt(t)).sub(pos);
      const r2 = d.lengthSq();
      const invR3 = 1 / (r2 * Math.sqrt(r2));
      out.addScaledVector(d, s.mu * invR3);
    }
    if (srpFactor > 0 && sun) {
      d.copy(pos).sub(sun.positionAt(t));
      const r = d.length();
      out.addScaledVector(d.divideScalar(r), srpFactor / (r * r));
    }
    return out;
  };
}
