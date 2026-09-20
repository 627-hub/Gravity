import { Vector3 } from 'three';
import { PLANETS } from '../data/bodies';
import { integrate } from './integrate';
import { muAuOfMass } from './units';
import { describe, expect, it } from 'vitest';
import {
  DIPOLE_MOMENT, dipoleField, tetherForceN, tetherPowerW, windSailThrustN,
} from './tether';

describe('电动力缆绳：与行星磁场做动量交换', () => {
  const earth = DIPOLE_MOMENT.earth;
  const r7000 = new Vector3(0, 0, 7e6); // 赤道面上（θ=90°）

  it('偶极场：赤道面场强 ~3e-5 T，且赤道上是径向的', () => {
    const b = dipoleField(earth, r7000, new Vector3());
    expect(b.length()).toBeGreaterThan(1e-5);
    expect(b.length()).toBeLessThan(5e-5);
    // 赤道：B 基本沿 −Z（径向）
    expect(Math.abs(b.clone().normalize().z)).toBeGreaterThan(0.99);
    // 1/r³ 衰减
    const far = dipoleField(earth, new Vector3(0, 0, 1.4e7), new Vector3());
    expect(far.length() / b.length()).toBeCloseTo(0.125, 2);
  });

  it('径向缆绳的力恒为东西向，且赤道面上为零（所以真实任务走倾斜轨道）', () => {
    const vRel = new Vector3(7.5e3, 0, 0); // 东向运动（v−Ω×r）
    const fEq = tetherForceN({ lengthKm: 10, currentA: 5 }, earth, r7000, vRel);
    expect(fEq.length()).toBeLessThan(1e-9); // 赤道：sinθ=0

    // 中纬度（θ=45°）：出现东西向的力
    const r45 = new Vector3(0, 7e6 * Math.SQRT1_2, 7e6 * Math.SQRT1_2);
    const f45 = tetherForceN({ lengthKm: 10, currentA: 5 }, earth, r45, vRel);
    expect(f45.length()).toBeGreaterThan(0.1);  // 几牛的量级
    expect(f45.length()).toBeLessThan(5);
    // 力 ⟂ 缆绳（径向）⟂ 磁场
    const rHat = r45.clone().normalize();
    expect(Math.abs(f45.clone().normalize().dot(rHat))).toBeLessThan(1e-9);
  });

  it('量级合理：10 km / 5 A 在 7000 km 轨道上是 N 级推力，功率 kW 级', () => {
    const r45 = new Vector3(0, 7e6 * Math.SQRT1_2, 7e6 * Math.SQRT1_2);
    const vRel = new Vector3(7.5e3, 0, 0);
    const f = tetherForceN({ lengthKm: 10, currentA: 5 }, earth, r45, vRel);
    // 500 kg 飞船：a = F/m ~ 1e-3 m/s²（比 1 AU 的太阳帆强 3 个量级）
    expect(f.length() / 500).toBeGreaterThan(1e-3);
    const p = tetherPowerW({ lengthKm: 10, currentA: 5 }, earth, r45, vRel);
    expect(p / 1000).toBeGreaterThan(1);   // >1 kW
    expect(p / 1000).toBeLessThan(100);    // <100 kW
  });

  it('磁场随行星自转 -> 相对速度里含 Ω×r；电流反向则推力反向（电动/发电）', () => {
    const r45 = new Vector3(0, 7e6 * Math.SQRT1_2, 7e6 * Math.SQRT1_2);
    const vRel = new Vector3(7.5e3, 0, 0);
    const up = tetherForceN({ lengthKm: 10, currentA: 5 }, earth, r45, vRel);
    const down = tetherForceN({ lengthKm: 10, currentA: -5 }, earth, r45, vRel);
    expect(up.dot(down)).toBeLessThan(0); // 严格反向
  });

  it('火星/金星没有可用内禀磁场 —— 缆绳在那里没用', () => {
    expect(DIPOLE_MOMENT.mars).toBe(0);
    expect(DIPOLE_MOMENT.venus).toBe(0);
    const f = tetherForceN(
      { lengthKm: 20, currentA: 10 },
      DIPOLE_MOMENT.mars,
      new Vector3(0, 4e6 * Math.SQRT1_2, 4e6 * Math.SQRT1_2),
      new Vector3(3e3, 0, 0),
    );
    expect(f.length()).toBe(0);
  });
});

describe('太阳风帆（磁帆 / 电帆）', () => {
  it('磁帆 F ∝ 1/r²：100 km 磁层顶在 1 AU 是 N 级', () => {
    const f1 = windSailThrustN({ kind: 'magsail', size: 100 }, 1);
    expect(f1).toBeGreaterThan(1);
    expect(f1).toBeLessThan(100);
    const f2 = windSailThrustN({ kind: 'magsail', size: 100 }, 2);
    expect(f1 / f2).toBeCloseTo(4, 6); // 1/r²
  });

  it('电帆 F ∝ 1/r：20 km × 100 根导线在 1 AU ~1 N', () => {
    const f1 = windSailThrustN({ kind: 'esail', size: 20, wires: 100 }, 1);
    expect(f1).toBeGreaterThan(0.5);
    expect(f1).toBeLessThan(2);
    const f2 = windSailThrustN({ kind: 'esail', size: 20, wires: 100 }, 2);
    expect(f1 / f2).toBeCloseTo(2, 6); // 1/r
  });

  it('太阳风动压比光压弱 ~300 倍（但等效面积大得多）', () => {
    expect(1.34e-9 / 4.56e-6).toBeGreaterThan(0.0002);
    expect(1.34e-9 / 4.56e-6).toBeLessThan(0.001);
  });
});

describe('缆绳接进积分器：真的能改轨道', () => {
  it('accelVec 钩子生效；电动模式抬升轨道、发电模式下降（同一条缆绳反向电流）', () => {
    const earthDipole = DIPOLE_MOMENT.earth;
    const earthBody = PLANETS.find((b) => b.id === 'earth')!;
    const muAu = muAuOfMass(earthBody.mass); // AU³/day²
    const rAu = 7300 / 149597870.7;          // 7300 km 圆轨道
    const vAu = Math.sqrt(muAu / rAu);

    const r0 = new Vector3(0, rAu * Math.SQRT1_2, rAu * Math.SQRT1_2); // 45° 倾斜
    const vHat = new Vector3(1, 0, 0);                                  // 东向
    const v0 = vHat.clone().multiplyScalar(vAu);

    const gravity = (_t: number, pos: Vector3, out: Vector3) =>
      out.copy(pos).multiplyScalar(-muAu / Math.pow(pos.length(), 3));

    const thrust = (currentA: number) => ({
      direction: (_p: Vector3, _v: Vector3, out: Vector3) => out.set(0, 0, 1),
      // 导航层单位 AU / AU·day⁻¹ -> SI，再调缆绳模块
      accelVec: (pos: Vector3, vel: Vector3, out: Vector3) => {
        const AU = 1.495978707e11;
        const DAY = 86400;
        const f = tetherForceN(
          { lengthKm: 10, currentA },
          earthDipole,
          pos.clone().multiplyScalar(AU),
          vel.clone().multiplyScalar(AU / DAY),
        );
        return out.copy(f).multiplyScalar(1 / 1000); // 1000 kg -> m/s²
      },
    });

    const energy = (p: Vector3, v: Vector3) => v.lengthSq() / 2 - muAu / p.length();
    const run = (currentA: number) => {
      const pts = integrate({ pos: r0.clone(), vel: v0.clone() }, 0, 0.4, gravity, {
        maxStep: 0.005,
        thrust: thrust(currentA) as never,
      });
      return pts[pts.length - 1];
    };

    const e0 = energy(r0, v0);
    const up = run(-20);
    const down = run(20);
    const dUp = energy(up.pos, up.vel) - e0;
    const dDown = energy(down.pos, down.vel) - e0;
    // 电流符号决定推力的东西向，从而决定抬升/下降（本几何下 +I 为西向、减速）
    expect(dUp).toBeGreaterThan(0);                 // 电动模式：抬轨道
    expect(dDown).toBeLessThan(0);                  // 发电模式：降轨道（动能转电）
    expect(Math.abs(dUp)).toBeGreaterThan(1e-7);
    expect(Math.abs(dUp + dDown)).toBeLessThan(Math.abs(dUp)); // 近似对称
  });
});
