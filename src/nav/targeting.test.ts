import { Vector3 } from 'three';
import { describe, expect, it } from 'vitest';
import { PLANETS } from '../data/bodies';
import { AU } from '../data/constants';
import { bodyEphemeris } from './ephemeris';
import { flybyOutbound } from './flyby';
import { propagate } from './propagate';
import { shootFlyby } from './targeting';
import { MU_SUN, fromKms, muAuOfMass } from './units';

const earth = PLANETS.find((b) => b.id === 'earth')!;
const muEarth = muAuOfMass(earth.mass);

describe('gravity-assist targeting', () => {
  it.each([
    [15000, 1.0],
    [25000, 4.0],
  ])('recovers a known flyby solution (rp=%i km, bAngle=%s)', (rpKm, bAngleTrue) => {
    const tFlyby = 0;
    const tTarget = 400;
    const vInfIn = new Vector3(fromKms(-3), fromKms(4), fromKms(1));
    const rpTrue = rpKm / (AU / 1000);

    // Generate the "truth": fly by Earth, then cruise for 400 days.
    const truth = flybyOutbound(vInfIn, muEarth, rpTrue, bAngleTrue);
    const planet = bodyEphemeris(earth).stateAt(tFlyby);
    const after = { pos: planet.pos.clone(), vel: planet.vel.clone().add(truth.vInfOut) };
    const endpoint = propagate(after, tTarget - tFlyby, MU_SUN).pos;
    const fixedTarget = {
      stateAt: () => ({ pos: endpoint.clone(), vel: new Vector3() }),
    };

    const aim = shootFlyby({
      planet: bodyEphemeris(earth),
      muPlanet: muEarth,
      tFlyby,
      vInfIn,
      target: fixedTarget,
      tTarget,
      rpMin: 8000 / (AU / 1000),
      rpMax: 40000 / (AU / 1000),
      rpSteps: 32,
      bAngleSteps: 64,
    });

    expect(aim.missDistance).toBeLessThan(1e-5); // ~1500 km at Mars-orbit distances
    expect(aim.trajectory.length).toBeGreaterThan(100);
    // The found solution must reproduce the flyby velocity from its own rp/bAngle.
    const repro = flybyOutbound(vInfIn, muEarth, aim.rp, aim.bAngle).vInfOut;
    expect(repro.distanceTo(aim.vInfOut)).toBeLessThan(1e-12);
  });
});
