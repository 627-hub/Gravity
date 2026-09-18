import { Vector3 } from 'three';
import type { StateVector } from '../physics/state';
import { MU_SUN } from './units';

// Classical orbital elements from a state vector — the onboard ephemeris
// readout a flight computer shows next to raw position/velocity. Works for
// elliptic and hyperbolic arcs (a < 0, periodDays null).

export interface ClassicalElements {
  /** Semi-major axis, AU (negative for hyperbolic). */
  a: number;
  e: number;
  /** Inclination to the ecliptic, degrees. */
  iDeg: number;
  /** Longitude of the ascending node, degrees. */
  nodeDeg: number;
  /** Argument of periapsis, degrees. */
  periDeg: number;
  /** True anomaly, degrees. */
  trueAnomalyDeg: number;
  /** Sidereal period, days (null when a <= 0). */
  periodDays: number | null;
  /** Specific orbital energy, AU^2/day^2. */
  energy: number;
  /** Flight path angle (velocity above the local horizon), degrees. */
  flightPathAngleDeg: number;
}

const DEG = 180 / Math.PI;
const clampUnit = (x: number): number => Math.min(1, Math.max(-1, x));

export function elementsFromState(state: StateVector, mu: number = MU_SUN): ClassicalElements {
  const { pos: r, vel: v } = state;
  const rMag = r.length();
  const vMag = v.length();
  const h = new Vector3().crossVectors(r, v);
  const hMag = h.length();
  const rv = r.dot(v);

  const energy = (vMag * vMag) / 2 - mu / rMag;
  const a = -mu / (2 * energy);

  const eVec = new Vector3()
    .copy(r)
    .multiplyScalar(vMag * vMag - mu / rMag)
    .addScaledVector(v, -rv)
    .divideScalar(mu);
  const e = eVec.length();

  const iDeg = Math.acos(clampUnit(h.z / hMag)) * DEG;

  const node = new Vector3(-h.y, h.x, 0); // ẑ × h
  const nMag = node.length();
  const nodeDeg = nMag > 1e-12 ? Math.atan2(node.y, node.x) * DEG : 0;

  let periDeg = 0;
  if (e > 1e-12) {
    if (nMag > 1e-12) {
      const w = Math.acos(clampUnit(node.dot(eVec) / (nMag * e)));
      periDeg = (eVec.z < 0 ? 2 * Math.PI - w : w) * DEG;
    } else {
      periDeg = Math.atan2(eVec.y, eVec.x) * DEG;
    }
  }

  let trueAnomalyDeg = 0;
  if (e > 1e-12) {
    const nu = Math.acos(clampUnit(eVec.dot(r) / (e * rMag)));
    trueAnomalyDeg = (rv < 0 ? 2 * Math.PI - nu : nu) * DEG;
  }

  const flightPathAngleDeg = Math.atan2(rv, hMag) * DEG;
  const periodDays = a > 0 ? 2 * Math.PI * Math.sqrt((a * a * a) / mu) : null;

  return {
    a,
    e,
    iDeg,
    nodeDeg,
    periDeg,
    trueAnomalyDeg,
    periodDays,
    energy,
    flightPathAngleDeg,
  };
}
