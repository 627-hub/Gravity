import { Vector3 } from 'three';
import { describe, expect, it } from 'vitest';
import { PLANETS } from '../data/bodies';
import { AU_KM } from '../data/constants';
import { bodyEphemeris } from './ephemeris';
import {
  gaussian, makeMeasurement, makeOpticalMeasurement, mulberry32, Navigator,
  rangeAndRangeRate, tierNoise, TRACKING_TIERS,
} from './navigator';
import { bestTransfer } from './plan';
import { propagate } from './propagate';
import { MU_SUN } from './units';

const earth = PLANETS.find((b) => b.id === 'earth')!;
const mars = PLANETS.find((b) => b.id === 'mars')!;

function j2000Days(y: number, m: number, d: number): number {
  return (Date.UTC(y, m - 1, d) - Date.UTC(2000, 0, 1, 12)) / 86400000;
}

const plan = bestTransfer({
  departure: bodyEphemeris(earth),
  target: bodyEphemeris(mars),
  departureId: 'earth',
  targetId: 'mars',
  tNow: j2000Days(2026, 1, 1),
  horizonDays: 900,
  departStep: 5,
  tofMin: 150,
  tofMax: 350,
  tofStep: 5,
})!;

/** A realistic truth: the planned arc plus a small launch dispersion. */
function dispersedTruth(): { pos: Vector3; vel: Vector3 } {
  const vel = plan.v1.clone();
  vel.addScaledVector(new Vector3(0.3, 0.9, 0.2).normalize(), vel.length() * 0.0025);
  return { pos: plan.r1.clone(), vel };
}

describe('range/range-rate measurements', () => {
  it('computes geometry directly', () => {
    const station = { pos: new Vector3(), vel: new Vector3() };
    const radial = { pos: new Vector3(1, 0, 0), vel: new Vector3(0.01, 0, 0) };
    expect(rangeAndRangeRate(radial, station).range).toBeCloseTo(1, 12);
    expect(rangeAndRangeRate(radial, station).rangeRate).toBeCloseTo(0.01, 12);
    const tangential = { pos: new Vector3(1, 0, 0), vel: new Vector3(0, 0.01, 0) };
    expect(rangeAndRangeRate(tangential, station).rangeRate).toBeCloseTo(0, 12);
  });

  it('produces zero-mean noise with the requested spread', () => {
    const rng = mulberry32(7);
    const sigma = 0.5;
    let sum = 0;
    let sumSq = 0;
    const n = 4000;
    for (let i = 0; i < n; i++) {
      const g = gaussian(rng) * sigma;
      sum += g;
      sumSq += g * g;
    }
    const mean = sum / n;
    const std = Math.sqrt(sumSq / n - mean * mean);
    expect(Math.abs(mean)).toBeLessThan(0.05);
    expect(std).toBeGreaterThan(sigma * 0.9);
    expect(std).toBeLessThan(sigma * 1.1);
  });

  it('replays identically for the same seed', () => {
    const station = bodyEphemeris(earth).stateAt(0);
    const truth = { pos: new Vector3(1.2, 0.3, 0.1), vel: new Vector3(0, 0.015, 0) };
    const noise = tierNoise(TRACKING_TIERS[1]);
    const a = makeMeasurement(truth, station, 0, noise, mulberry32(99));
    const b = makeMeasurement(truth, station, 0, noise, mulberry32(99));
    expect(a.range).toBe(b.range);
    expect(a.rangeRate).toBe(b.rangeRate);
    const oa = makeOpticalMeasurement(truth, { pos: new Vector3(1.5, 0.2, 0), vel: new Vector3() }, 0, noise, mulberry32(5));
    const ob = makeOpticalMeasurement(truth, { pos: new Vector3(1.5, 0.2, 0), vel: new Vector3() }, 0, noise, mulberry32(5));
    expect(oa.dir.distanceTo(ob.dir)).toBe(0);
  });
});

describe('navigator EKF', () => {
  it('converges with range/range-rate plus optical tracking', () => {
    const truth = dispersedTruth();
    const noise = tierNoise(TRACKING_TIERS[1]); // 300 km / 5 m/s / 5 arcsec
    const nav = new Navigator({ pos: plan.r1.clone(), vel: plan.v1.clone() }, plan.departureDay, {
      posSigma0: 1000 / AU_KM, velSigma0: 0.05, noise,
    });
    const station = bodyEphemeris(earth);
    const beacon = bodyEphemeris(mars);
    const rng = mulberry32(4242);

    let lastT = plan.departureDay;
    for (let t = plan.departureDay + 5; t <= plan.departureDay + 60; t += 2) {
      nav.propagateTo(t);
      const truthState = propagate(truth, t - plan.departureDay, MU_SUN);
      const st = station.stateAt(t);
      nav.update(makeMeasurement(truthState, st, t, noise, rng), st);
      const bx = beacon.stateAt(t);
      nav.opticalUpdate(makeOpticalMeasurement(truthState, bx, t, noise, rng), bx);
      lastT = t;
    }
    // Compare at the filter's own epoch (a one-day offset is millions of km).
    const late = nav.state().pos.distanceTo(propagate(truth, lastT - plan.departureDay, MU_SUN).pos);
    expect(nav.measurementCount).toBeGreaterThan(25);
    // Ground ranging + onboard optical navigation hold the estimate far below
    // the ~470,000 km the raw dispersion would accumulate over 60 days.
    expect(late * AU_KM).toBeLessThan(1e5);
    expect(nav.positionSigma()).toBeGreaterThan(0); // and reports an uncertainty
  });

  it('replays identically for the same seed', () => {
    const truth = dispersedTruth();
    const noise = tierNoise(TRACKING_TIERS[2]);
    const station = bodyEphemeris(earth);
    const run = () => {
      const nav = new Navigator({ pos: plan.r1.clone(), vel: plan.v1.clone() }, plan.departureDay, {
        posSigma0: 1000 / AU_KM, velSigma0: 0.05, noise,
      });
      const rng = mulberry32(7);
      for (let t = plan.departureDay + 5; t <= plan.departureDay + 60; t += 2) {
        nav.propagateTo(t);
        const ts = propagate(truth, t - plan.departureDay, MU_SUN);
        const st = station.stateAt(t);
        nav.update(makeMeasurement(ts, st, t, noise, rng), st);
      }
      return nav.state();
    };
    const a = run();
    const b = run();
    expect(a.pos.distanceTo(b.pos)).toBe(0);
    expect(a.vel.distanceTo(b.vel)).toBe(0);
  });
});
