import { Vector3 } from 'three';
import { describe, expect, it } from 'vitest';
import { AU, AU_KM, DAY } from '../data/constants';
import { integrate, massFlowKgPerDay } from './integrate';
import { makeForceModel } from './perturbations';
import { DRIVES, massRatioVe, thrustDirection } from './propulsion';
import { AUDAY_TO_KMS, MU_SUN } from './units';

const nep = DRIVES.find((d) => d.id === 'nep')!; // 核电：v_e 29.4 km/s

/** 位力：圆轨道速度（AU/day）。 */
const vCirc = (rAu: number): number => Math.sqrt(MU_SUN / rAu);

describe('连续推力 + 质量流', () => {
  const m0 = 1500;
  const dry = 1000;
  const F = 0.5; // N
  const days = 200;
  const r0 = 1; // AU
  const mdot = massFlowKgPerDay(F, nep.exhaustKms);

  const run = () => {
    const accel = makeForceModel([{ id: 'sun', mu: MU_SUN, positionAt: () => new Vector3() }]);
    return integrate(
      { pos: new Vector3(r0, 0, 0), vel: new Vector3(0, vCirc(r0), 0) },
      0,
      days,
      accel,
      {
        rtol: 1e-10,
        atol: 1e-13,
        maxStep: 0.5,
        thrust: {
          direction: (_p, v, out) => out.copy(v).normalize(),
          thrustN: F,
          exhaustKms: nep.exhaustKms,
          mass0Kg: m0,
          dryMassKg: dry,
        },
      },
    );
  };

  it('质量按 ṁ = F/v_e 线性下降', () => {
    const pts = run();
    const last = pts[pts.length - 1];
    expect(last.massKg!).toBeCloseTo(m0 - mdot * days, 6);
    expect(mdot).toBeCloseTo((F / (nep.exhaustKms * 1000)) * DAY, 8);
    // 消耗 294 kg（< 500 kg 可用推进剂，没烧干）
    expect(m0 - last.massKg!).toBeGreaterThan(250);
    expect(m0 - last.massKg!).toBeLessThan(350);
  });

  it('累积 Δv 与火箭方程一致：Δv = v_e·ln(m0/mf)', () => {
    const pts = run();
    // 对采样点数值积分 ∫ F/m dt（换算 km/s），应与火箭方程吻合
    let dv = 0;
    for (let i = 1; i < pts.length; i++) {
      const dtt = pts[i].t - pts[i - 1].t;
      const m = Math.max(dry, m0 - mdot * pts[i - 1].t);
      dv += ((F / m) * dtt * DAY) / 1000; // m/s -> km/s
    }
    const mf = m0 - mdot * days;
    const rocket = nep.exhaustKms * Math.log(m0 / mf);
    expect(dv).toBeCloseTo(rocket, 2);
    expect(rocket).toBeGreaterThan(6);
  });

  it('顺行推力把轨道抬高（螺旋外扩），不是脉冲式的霍曼', () => {
    const pts = run();
    const last = pts[pts.length - 1];
    // 圆轨道半径变大：从 1 AU 外扩
    expect(last.pos.length()).toBeGreaterThan(1.2);
    // 轨道速度反而下降（v = √(μ/r)），这正是"螺旋"的指纹
    expect(last.vel.length()).toBeLessThan(vCirc(r0));
    // 比能量上升
    const energy = (p: Vector3, v: Vector3) => (v.lengthSq() / 2) - MU_SUN / p.length();
    expect(energy(last.pos, last.vel)).toBeGreaterThan(energy(new Vector3(r0, 0, 0), new Vector3(0, vCirc(r0), 0)));
  });

  it('推进剂耗尽后自动停止减重与加速', () => {
    const pts = run();
    const mf = pts[pts.length - 1].massKg!;
    // 干重地板：无论烧多久都不会低于 dry
    const long = integrate(
      { pos: new Vector3(r0, 0, 0), vel: new Vector3(0, vCirc(r0), 0) },
      0,
      5000,
      makeForceModel([{ id: 'sun', mu: MU_SUN, positionAt: () => new Vector3() }]),
      {
        maxStep: 1,
        thrust: {
          direction: (_p, v, out) => out.copy(v).normalize(),
          thrustN: F, exhaustKms: nep.exhaustKms, mass0Kg: m0, dryMassKg: dry,
        },
      },
    );
    expect(long[long.length - 1].massKg!).toBeCloseTo(dry, 6);
    expect(mf).toBeGreaterThan(dry);
  });

  it('光帆：不耗工质，锥角切向分量把轨道螺旋推出去', () => {
    const a0 = 1e-3; // 1 AU 处加速度，m/s²（放大 10 倍便于测试）
    const accel = makeForceModel([{ id: 'sun', mu: MU_SUN, positionAt: () => new Vector3() }]);
    const r0 = 1;
    const state = { pos: new Vector3(r0, 0, 0), vel: new Vector3(0, vCirc(r0), 0) };
    const energy = (p: Vector3, v: Vector3) => v.lengthSq() / 2 - MU_SUN / p.length();
    const e0 = energy(state.pos, state.vel);

    // 功的即时判据（不用积分，最干净）：力与速度的夹角决定做不做功。
    const sailDir = thrustDirection('sailOut', state.pos, state.vel);
    const radialDir = thrustDirection('radialOut', state.pos, state.vel);
    const vHat = state.vel.clone().normalize();
    expect(sailDir.dot(vHat)).toBeGreaterThan(0.5);      // 锥角帆有明显切向分量
    expect(Math.abs(radialDir.dot(vHat))).toBeLessThan(1e-12); // 纯径向：F·v = 0

    // 真正飞 400 天：光帆外扩（真实注入能量）
    const pts = integrate(state, 0, 400, accel, {
      maxStep: 1,
      thrust: {
        direction: (pos, vel, out) => thrustDirection('sailOut', pos, vel, out),
        accelAt: (pos) => a0 / Math.max(pos.lengthSq(), 1e-9), // a ∝ 1/r²
      },
    });
    const last = pts[pts.length - 1];
    expect(last.pos.length()).toBeGreaterThan(1.05);            // 半径外扩
    expect(energy(last.pos, last.vel)).toBeGreaterThan(e0);     // 比能量上升（做了功）
    expect(last.massKg).toBeUndefined();                        // 不携带工质、不耗质量
  });

  it('同样推力下高比冲的 Δv 更大（但加速度更小）', () => {
    const ratioNep = massRatioVe(6, nep.exhaustKms);
    const chem = DRIVES.find((d) => d.id === 'chemical')!;
    expect(ratioNep).toBeLessThan(massRatioVe(6, chem.exhaustKms));
    // 核电 v_e 大 6.7 倍 -> 同样质量比下 Δv 大 6.7 倍
    expect(nep.exhaustKms / chem.exhaustKms).toBeGreaterThan(6);
    // 单位换算自检
    expect(AUDAY_TO_KMS).toBeCloseTo(AU / DAY / 1000, 6);
    expect(AU_KM).toBeGreaterThan(1.49e8);
  });
});
