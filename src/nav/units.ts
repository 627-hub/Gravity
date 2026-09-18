import { Vector3 } from 'three';
import { AU, DAY, G, GM_SUN } from '../data/constants';

// Unit conventions for the navigation layer: AU for distances, days for time.
// This matches the simulator's master clock (days since J2000) and the state
// vectors produced by the physics layer, so no conversion sits in the hot path.
// km/s is kept for human-facing maneuver numbers only.

/** Sun's standard gravitational parameter in AU^3/day^2 (≈ 2.959122082855911e-4). */
export const MU_SUN = (GM_SUN * DAY * DAY) / (AU * AU * AU);

/** 1 AU/day expressed in km/s (≈ 1731.4568). */
export const AUDAY_TO_KMS = AU / DAY / 1000;
export const KMS_TO_AUDAY = 1 / AUDAY_TO_KMS;

/** Convert a speed in AU/day to km/s. */
export function toKms(v: number): number {
  return v * AUDAY_TO_KMS;
}

/** 1 m/s expressed in AU/day (≈ 5.775e-7). */
export const MPS_TO_AUDAY = 1 / (AUDAY_TO_KMS * 1000);

/** Convert a speed in km/s to AU/day. */
export function fromKms(v: number): number {
  return v * KMS_TO_AUDAY;
}

/** Magnitude of an AU/day velocity vector in km/s. */
export function speedKms(v: Vector3): number {
  return toKms(v.length());
}

/** Standard gravitational parameter GM (m^3/s^2) for a mass in kg. */
export function muOfMass(massKg: number): number {
  return G * massKg;
}

/** Standard gravitational parameter in AU^3/day^2 for a mass in kg. */
export function muAuOfMass(massKg: number): number {
  return (G * massKg * DAY * DAY) / (AU * AU * AU);
}
