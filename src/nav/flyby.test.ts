import { Vector3 } from 'three';
import { describe, expect, it } from 'vitest';
import { PLANETS } from '../data/bodies';
import { AU } from '../data/constants';
import {
  bPlaneFrame,
  flybyOutbound,
  gravityAssist,
  impactParameter,
  periapsisForTurn,
  solveFlyby,
  turnAngle,
} from './flyby';
import { integrate } from './integrate';
import { makeForceModel } from './perturbations';
import { fromKms, muAuOfMass, toKms } from './units';

const earth = PLANETS.find((b) => b.id === 'earth')!;
const jupiter = PLANETS.find((b) => b.id === 'jupiter')!;
const muEarth = muAuOfMass(earth.mass);
const muJupiter = muAuOfMass(jupiter.mass);

describe('flyby mechanics (B-plane model)', () => {
  it('validates the modelled deflection against direct integration', () => {
    const vInf = fromKms(6);
    const rp = 12000 / (AU / 1000);
    const bAngle = 0.7;
    const vInfIn = new Vector3(0, 1, 0).multiplyScalar(vInf);
    const out = flybyOutbound(vInfIn, muEarth, rp, bAngle);

    // Build the periapsis state: velocity there bisects v_inf in/out.
    const vp = Math.sqrt(vInf * vInf + (2 * muEarth) / rp);
    const bisector = vInfIn
      .clone()
      .normalize()
      .add(out.vInfOut.clone().normalize())
      .normalize();
    const initial = {
      pos: out.periapsisDir.clone().multiplyScalar(rp),
      vel: bisector.clone().multiplyScalar(vp),
    };
    // The state is a true periapsis: radius perpendicular to velocity.
    expect(Math.abs(initial.pos.dot(initial.vel)) / (rp * vp)).toBeLessThan(1e-9);

    // Integrate the planet-centred hyperbola out towards the asymptote.
    const planetAtOrigin = { id: 'planet', mu: muEarth, positionAt: () => new Vector3() };
    const accel = makeForceModel([planetAtOrigin]);
    const traj = integrate(initial, 0, 30, accel, { rtol: 1e-12, atol: 1e-14 });
    const end = traj[traj.length - 1];

    // Speed has converged to v_inf and the direction matches the model.
    expect(Math.abs(end.vel.length() / vInf - 1)).toBeLessThan(3e-3);
    const dirError = end.vel.clone().normalize().angleTo(out.vInfOut.clone().normalize());
    expect(dirError).toBeLessThan(3e-3);
    const measuredTurn = end.vel.clone().normalize().angleTo(vInfIn.clone().normalize());
    expect(measuredTurn).toBeCloseTo(out.turn, 2);
  });

  it('conserves |v_inf| and yields the classic assist delta-v', () => {
    const vInfIn = new Vector3(fromKms(7), fromKms(2), fromKms(1));
    const rp = 15000 / (AU / 1000);
    const out = flybyOutbound(vInfIn, muEarth, rp, 2.1);
    expect(out.vInfOut.length()).toBeCloseTo(vInfIn.length(), 12);

    // |Δv_inf| = 2 v_inf sin(δ/2): the textbook gravity-assist relation.
    const dv = out.vInfOut.clone().sub(vInfIn).length();
    expect(dv).toBeCloseTo(2 * vInfIn.length() * Math.sin(out.turn / 2), 12);

    // Heliocentric energy gain equals v_planet · Δv_inf.
    const planetVel = new Vector3(0, fromKms(29.8), 0);
    const assist = gravityAssist({ pos: new Vector3(1, 0, 0), vel: planetVel }, vInfIn, muEarth, rp, 2.1);
    expect(assist.energyChange).toBeCloseTo(
      planetVel.dot(out.vInfOut.clone().sub(vInfIn)),
      14,
    );
  });

  it('locates the B-plane frame consistently', () => {
    const frame = bPlaneFrame(new Vector3(1, 0, 0));
    expect(frame.s.distanceTo(new Vector3(1, 0, 0))).toBeLessThan(1e-12);
    expect(frame.t.distanceTo(new Vector3(0, 1, 0))).toBeLessThan(1e-12);
    expect(frame.r.distanceTo(new Vector3(0, 0, 1))).toBeLessThan(1e-12);
    // Degenerate: v_inf along the pole must still produce a valid triad.
    const deg = bPlaneFrame(new Vector3(0, 0, 1));
    expect(Math.abs(deg.t.length() - 1)).toBeLessThan(1e-12);
    expect(Math.abs(deg.s.dot(deg.t))).toBeLessThan(1e-12);
  });

  it('inverts flybyOutbound with solveFlyby', () => {
    const vInfIn = new Vector3(fromKms(4), fromKms(3.5), fromKms(-2));
    for (const [rpKm, bAngle] of [
      [12000, 0.3],
      [25000, 4.0],
    ]) {
      const rp = rpKm / (AU / 1000);
      const out = flybyOutbound(vInfIn, muEarth, rp, bAngle);
      const sol = solveFlyby(vInfIn, out.vInfOut, muEarth);
      expect(sol).not.toBeNull();
      if (!sol) continue;
      expect(sol.rp).toBeCloseTo(rp, 9);
      const wrap = Math.atan2(Math.sin(sol.bAngle - bAngle), Math.cos(sol.bAngle - bAngle));
      expect(Math.abs(wrap)).toBeLessThan(1e-6);
    }
    // Magnitude mismatch and no-deflection cases are rejected.
    expect(solveFlyby(vInfIn, vInfIn.clone().multiplyScalar(1.1), muEarth)).toBeNull();
    expect(solveFlyby(vInfIn, vInfIn.clone(), muEarth)).toBeNull();
  });

  it('produces Jupiter-class assist magnitudes', () => {
    const vInf = fromKms(10);
    const rp = 100000 / (AU / 1000);
    const turn = turnAngle(vInf, rp, muJupiter);
    const dvKmS = toKms(2 * vInf * Math.sin(turn / 2));
    expect(dvKmS).toBeGreaterThan(10);
    expect(dvKmS).toBeLessThan(20); // ~18.5 km/s at 100,000 km, 10 km/s v_inf
    expect(impactParameter(vInf, rp, muJupiter)).toBeGreaterThan(rp);
    expect(periapsisForTurn(vInf, turn, muJupiter)).toBeCloseTo(rp, 9);
  });
});
