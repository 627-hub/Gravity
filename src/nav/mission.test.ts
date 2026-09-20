import { Vector3 } from 'three';
import { describe, expect, it } from 'vitest';
import { PLANETS } from '../data/bodies';
import { AU } from '../data/constants';
import { bodyEphemeris } from './ephemeris';
import { Mission } from './mission';
import { fromKms, toKms } from './units';
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
  it('two-body truth arrives exactly on target', () => {
    const m = new Mission(plan, { injectError: false, physics: 'two-body' });
    expect(m.truthMissDistance()).toBeLessThan(1e-7);
    const arrive = m.stateAt(plan.arrivalDay);
    expect(arrive.pos.distanceTo(plan.r2)).toBeLessThan(1e-7);
    expect(m.predictedPath().length).toBe(513);
  });

  it('n-body truth drifts off the two-body plan (real model error)', () => {
    const m = new Mission(plan, { injectError: false }); // n-body by default
    const missKm = km(m.truthMissDistance());
    expect(missKm).toBeGreaterThan(1e4);  // SRP + third bodies push it off
    expect(missKm).toBeLessThan(1e7);
  });

  it('a dispersive launch misses by a visible margin', () => {
    const m = new Mission(plan, { injectError: true });
    const missKm = (m.truthMissDistance() * AU) / 1000;
    expect(missKm).toBeGreaterThan(50000);      // clearly off target
    expect(m.truthMissDistance()).toBeLessThan(0.5); // but not absurdly so
  });

  it('re-solves Lambert mid-flight and arrives exactly after the TCM', () => {
    const m = new Mission(plan, { injectError: true, physics: 'two-body' });
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
    const t = plan.departureDay + plan.tof * 0.9;
    m.advance(t);
    expect(m.hasTracking).toBe(true);
    expect(m.trackingCount).toBeGreaterThan(100);
    expect(m.positionSigma()).toBeGreaterThan(0);
    // Ground ranging + onboard optical navigation pull the estimate down to
    // ~10^4 km. Note the truth is the full n-body field (including SRP, which
    // the onboard two-body model does not know about): the residual is the
    // unmodelled SRP, ~10^7 km of drift over a 305-day cruise — real missions
    // carry the SRP model in their filter for exactly this reason.
    expect(km(m.estimateError(t))).toBeLessThan(2e4);
  });

  it('L1: 修正逐级收敛到港口精度（终端瞄准含目标引力）', () => {
    const m = new Mission(plan, { injectError: true, tracking: TRACKING_TIERS[2] });

    const t1 = plan.departureDay + plan.tof * 0.5;
    m.advance(t1);
    const uncorrected = km(m.truthMissDistance());
    expect(uncorrected).toBeGreaterThan(1e6); // 不修正的话确实会飞掉
    expect(m.applyTcm(t1)).not.toBeNull();
    const after1 = km(m.truthMissDistance());
    expect(after1).toBeLessThan(uncorrected / 10);

    const t2 = plan.departureDay + plan.tof * 0.75;
    m.advance(t2);
    m.applyTcm(t2);
    const after2 = km(m.truthMissDistance());
    expect(after2).toBeLessThan(after1 / 3);

    // 终端瞄准用**含目标引力**的船载模型：不再撞纯二体那 ~6e4 km 的模型下限，
    // 而是收敛到与跟踪质量相称的入泊精度（这里是最低精度档）。
    const t3 = plan.departureDay + plan.tof * 0.95;
    m.advance(t3);
    m.applyTcm(t3);
    expect(km(m.truthMissDistance())).toBeLessThan(2e4);
    expect(m.tcmCount).toBe(3);
    expect(m.tcmUsedKms).toBeLessThan(1);
  });

  it('高精度跟踪 + 终端修正 -> 亚千公里入泊（港口精度）', () => {
    const m = new Mission(plan, { injectError: true, tracking: TRACKING_TIERS[0] });
    const t = plan.departureDay + plan.tof * 0.95;
    m.advance(t);
    expect(m.applyTcm(t)).not.toBeNull();
    expect(km(m.truthMissDistance())).toBeLessThan(1000);
  });

  it('船载模型含太阳光压后，巡航段估计误差降到 10^4 km 以内', () => {
    const m = new Mission(plan, { injectError: true, tracking: TRACKING_TIERS[1] });
    const t = plan.departureDay + plan.tof * 0.5;
    m.advance(t);
    // 不建模 SRP 时这里的模型误差是 ~1e7 km（305 天），估计根本跟不上。
    expect(km(m.estimateError(t))).toBeLessThan(1e4);
  });

  it('手动点火：真实改变弹道并计入台账（可手动“开”飞船）', () => {
    const m = new Mission(plan, { injectError: false });
    const t = plan.departureDay + plan.tof * 0.5;
    const v0 = m.stateAt(t).vel.length();

    // 顺行 100 m/s
    const fwd = m.stateAt(t).vel.clone().normalize();
    const applied = m.applyManualBurn(t, fwd.multiplyScalar(fromKms(0.1)));
    expect(applied).toBeCloseTo(0.1, 9);
    expect(m.manualCount).toBe(1);
    expect(m.manualUsedKms).toBeCloseTo(0.1, 9);
    // 点火后瞬时速率正好加 0.1 km/s（同向）
    expect(toKms(m.stateAt(t).vel.length() - v0)).toBeCloseTo(0.1, 6);
    // 弹道被改变：原本按计划抵达，现在会错过（100 m/s 在 150 天后是百万公里量级）
    expect(km(m.truthMissDistance())).toBeGreaterThan(1e5);

    // 逆行点火让速率下降，台账累加
    const t2 = plan.departureDay + plan.tof * 0.6;
    const v1 = m.stateAt(t2).vel.length();
    m.applyManualBurn(t2, m.stateAt(t2).vel.clone().normalize().multiplyScalar(fromKms(-0.05)));
    expect(toKms(m.stateAt(t2).vel.length() - v1)).toBeCloseTo(-0.05, 6);
    expect(m.manualCount).toBe(2);
    expect(m.manualUsedKms).toBeCloseTo(0.15, 9);

    // 抵达之后不再接受点火
    expect(m.applyManualBurn(plan.arrivalDay, fwd)).toBeNull();
  });

  it('supports multiple corrections and refuses past arrival', () => {
    const m = new Mission(plan, { injectError: true, physics: 'two-body' });
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
