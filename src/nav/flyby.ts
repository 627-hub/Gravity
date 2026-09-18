import { Vector3 } from 'three';
import type { StateVector } from '../physics/state';

// Gravity-assist mechanics in the patched-conic picture. During a flyby the
// hyperbolic excess velocity v_inf keeps its magnitude in the body's frame —
// that is the free "delta-v" of a gravity assist — while its direction rotates
// by an angle set by the periapsis radius. Orientation is parameterised with
// the classic B-plane: the plane through the body perpendicular to v_inf,
// with the impact vector B (T/R components) selecting which side the
// spacecraft passes. Patching back to the heliocentric frame adds the
// planet's velocity and produces the actual velocity change.

export interface BPlaneFrame {
  /** Along the incoming v_inf. */
  s: Vector3;
  /** First B-plane axis (ecliptic-projected). */
  t: Vector3;
  /** Second B-plane axis; (s, t, r) is right-handed. */
  r: Vector3;
}

/** Right-handed B-plane triad for an incoming v_inf direction. */
export function bPlaneFrame(vInfIn: Vector3, pole: Vector3 = new Vector3(0, 0, 1)): BPlaneFrame {
  const s = vInfIn.clone().normalize();
  const t = new Vector3().crossVectors(pole, s);
  if (t.lengthSq() < 1e-12) {
    const alt = Math.abs(s.x) < 0.9 ? new Vector3(1, 0, 0) : new Vector3(0, 1, 0);
    t.crossVectors(alt, s);
  }
  t.normalize();
  const r = new Vector3().crossVectors(s, t);
  return { s, t, r };
}

/** Deflection angle δ for a flyby at periapsis radius rp: sin(δ/2) = 1/(1 + rp v²/μ). */
export function turnAngle(vInf: number, rp: number, mu: number): number {
  return 2 * Math.asin(1 / (1 + (rp * vInf * vInf) / mu));
}

/** B-plane impact parameter b: miss distance of the incoming asymptote. */
export function impactParameter(vInf: number, rp: number, mu: number): number {
  return rp * Math.sqrt(1 + (2 * mu) / (rp * vInf * vInf));
}

/** Periapsis radius that produces a given deflection angle. */
export function periapsisForTurn(vInf: number, turn: number, mu: number): number {
  const s = Math.sin(turn / 2);
  if (s <= 0) return Infinity;
  return (mu / (vInf * vInf)) * (1 / s - 1);
}

export interface FlybyOutbound {
  /** Outgoing v_inf (same magnitude as incoming). */
  vInfOut: Vector3;
  /** Direction from the body centre to periapsis. */
  periapsisDir: Vector3;
  /** Deflection angle, radians. */
  turn: number;
  /** Impact parameter b, AU. */
  b: number;
}

/**
 * Outgoing v_inf for a flyby with periapsis radius `rp` and B-plane angle
 * `bAngle` (0 = +T axis, π/2 = +R axis). The velocity bends towards -B.
 */
export function flybyOutbound(
  vInfIn: Vector3,
  mu: number,
  rp: number,
  bAngle: number,
  pole?: Vector3,
): FlybyOutbound {
  const frame = bPlaneFrame(vInfIn, pole);
  const vInf = vInfIn.length();
  const turn = turnAngle(vInf, rp, mu);
  const bHat = new Vector3()
    .copy(frame.t)
    .multiplyScalar(Math.cos(bAngle))
    .addScaledVector(frame.r, Math.sin(bAngle));
  const axis = new Vector3().crossVectors(bHat, frame.s).normalize();
  const vInfOut = frame.s.clone().applyAxisAngle(axis, turn).multiplyScalar(vInf);
  const periapsisDir = bHat.clone().applyAxisAngle(axis, turn / 2);
  return { vInfOut, periapsisDir, turn, b: impactParameter(vInf, rp, mu) };
}

export interface FlybySolution {
  rp: number;
  bAngle: number;
  turn: number;
}

/**
 * Inverse problem: find the flyby (rp, B-plane angle) that turns `vInfIn`
 * into `vInfOut`. Returns null when the vectors differ in magnitude (the
 * assist cannot change |v_inf| in the body frame) or are collinear.
 */
export function solveFlyby(
  vInfIn: Vector3,
  vInfOut: Vector3,
  mu: number,
  pole?: Vector3,
): FlybySolution | null {
  const vIn = vInfIn.length();
  const vOut = vInfOut.length();
  if (Math.abs(vIn - vOut) > 1e-9 * vIn) return null;
  const sIn = vInfIn.clone().normalize();
  const sOut = vInfOut.clone().normalize();
  const turn = Math.acos(Math.min(1, Math.max(-1, sIn.dot(sOut))));
  if (turn < 1e-12) return null;
  const axis = new Vector3().crossVectors(sIn, sOut);
  if (axis.lengthSq() < 1e-24) return null;
  axis.normalize();
  // The solver's rotation axis is B × S, so B = S × axis (both unit, ⊥).
  const bHat = new Vector3().crossVectors(sIn, axis).normalize();
  const frame = bPlaneFrame(vInfIn, pole);
  const bAngle = Math.atan2(bHat.dot(frame.r), bHat.dot(frame.t));
  return { rp: periapsisForTurn(vIn, turn, mu), bAngle, turn };
}

export interface GravityAssist {
  /** Incoming/outgoing v_inf in the body frame. */
  vInfIn: Vector3;
  vInfOut: Vector3;
  /** Heliocentric velocity change: vInfOut - vInfIn (free delta-v). */
  deltaV: Vector3;
  /**
   * Change of heliocentric specific energy: v_planet · Δv_inf. Positive means
   * the assist raised the orbit.
   */
  energyChange: number;
  /** Deflection angle, radians. */
  turn: number;
}

/** Heliocentric view of a flyby: what the assist does to the spacecraft's orbit. */
export function gravityAssist(
  planetState: StateVector,
  vInfIn: Vector3,
  mu: number,
  rp: number,
  bAngle: number,
  pole?: Vector3,
): GravityAssist {
  const out = flybyOutbound(vInfIn, mu, rp, bAngle, pole);
  const deltaV = new Vector3().subVectors(out.vInfOut, vInfIn);
  return {
    vInfIn: vInfIn.clone(),
    vInfOut: out.vInfOut,
    deltaV,
    energyChange: planetState.vel.dot(deltaV),
    turn: out.turn,
  };
}

/** v_inf of a spacecraft state relative to a body state, AU/day. */
export function vInfinity(spacecraft: StateVector, body: StateVector): Vector3 {
  return new Vector3().subVectors(spacecraft.vel, body.vel);
}
