import { Vector3 } from 'three';
import { describe, expect, it } from 'vitest';
import { PLANETS } from '../data/bodies';
import { AU } from '../data/constants';
import { bodyEphemeris } from './ephemeris';
import { Mission } from './mission';
import { TRACKING_TIERS } from './navigator';
import { bestTransfer } from './plan';

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

const km = (au: number): number => (au * AU) / 1000;

describe('mission with TCM corrections', () => {
  it('a perfect launch arrives on target', () => {
    const m = new Mission(plan, { injectError: false });
    expect(m.truthMissDistance()).toBeLessThan(1e-7);
    const arrive = m.stateAt(plan.arrivalDay);
    expect(arrive.pos.distanceTo(plan.r2)).toBeLessThan(1e-7);
    expect(m.predictedPath().length).toBe(513);
  });

  it('a dispersive launch misses by a visible margin', () => {
    const m = new Mission(plan, { injectError: true });
    const missKm = (m.truthMissDistance() * AU) / 1000;
    expect(missKm).toBeGreaterThan(50000);      // clearly off target
    expect(m.truthMissDistance()).toBeLessThan(0.5); // but not absurdly so
  });

  it('re-solves Lambert mid-flight and arrives exactly after the TCM', () => {
    const m = new Mission(plan, { injectError: true });
    const tMid = plan.departureDay + plan.tof * 0.5;
    const sol = m.solveTcm(tMid);
    expect(sol).not.toBeNull();
    if (!sol) return;
    expect(sol.dvKms).toBeGreaterThan(0);
    expect(sol.dvKms).toBeLessThan(2); // a correction, not a new mission

    const applied = m.applyTcm(tMid);
    expect(applied).toBeCloseTo(sol.dvKms, 12);
    expect(m.tcmCount).toBe(1);
    expect(m.tcmUsedKms).toBeCloseTo(sol.dvKms, 12);
    expect(m.truthMissDistance()).toBeLessThan(1e-7);
    expect(m.stateAt(plan.arrivalDay).pos.distanceTo(plan.r2)).toBeLessThan(1e-7);

    // The old (uncorrected) segment was shorter: the flown path still spans
    // the whole flight and ends at the rendezvous point.
    m.updateFlown(plan.arrivalDay);
    expect(m.flown.length).toBeGreaterThan(400);
    const last = m.flown[m.flown.length - 1];
    expect(last.distanceTo(plan.r2)).toBeLessThan(0.05);
  });

  it('L1: noisy tracking holds the estimate near the truth', () => {
    const m = new Mission(plan, { injectError: true, tracking: TRACKING_TIERS[1] });
    const t = plan.departureDay + plan.tof * 0.75;
    m.advance(t);
    expect(m.hasTracking).toBe(true);
    expect(m.trackingCount).toBeGreaterThan(100);
    expect(m.positionSigma()).toBeGreaterThan(0);
    // Ground ranging + onboard optical navigation keep the estimate within
    // ~10^4 km, where the raw dispersion would be ~10^6 km off by now.
    expect(km(m.estimateError(t))).toBeLessThan(2e5);
  });

  it('L1: corrections computed from the estimate are imperfect but converge', () => {
    const m = new Mission(plan, { injectError: true, tracking: TRACKING_TIERS[2] });
    const t1 = plan.departureDay + plan.tof * 0.75;
    m.advance(t1);
    const uncorrected = km(m.truthMissDistance());
    expect(uncorrected).toBeGreaterThan(3e5); // the flight really would miss
    const dv1 = m.applyTcm(t1);
    expect(dv1).not.toBeNull();
    const after1 = km(m.truthMissDistance());
    expect(after1).toBeLessThan(uncorrected); // the correction helps
    expect(after1).toBeGreaterThan(0);        // but estimation error leaves a residual

    const t2 = plan.departureDay + plan.tof * 0.92;
    m.advance(t2);
    m.applyTcm(t2);
    expect(km(m.truthMissDistance())).toBeLessThan(after1); // and improves again
    expect(m.tcmCount).toBe(2);
  });

  it('supports multiple corrections and refuses past arrival', () => {
    const m = new Mission(plan, { injectError: true });
    const t1 = plan.departureDay + plan.tof * 0.3;
    const t2 = plan.departureDay + plan.tof * 0.7;
    // Inject another dispersion by nudging the velocity after the first burn.
    expect(m.applyTcm(t1)).not.toBeNull();
    m.segments[m.segments.length - 1].state.vel.addScaledVector(
      new Vector3(0, 1, 0),
      m.segments[m.segments.length - 1].state.vel.length() * 0.001,
    );
    expect(m.truthMissDistance()).toBeGreaterThan(1e-6);
    expect(m.applyTcm(t2)).not.toBeNull();
    expect(m.tcmCount).toBe(2);
    expect(m.truthMissDistance()).toBeLessThan(1e-7);
    expect(m.solveTcm(plan.arrivalDay)).toBeNull();
  });
});
