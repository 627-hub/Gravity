import { describe, expect, it } from 'vitest';
import {
  DRIVES, burnDays, interstellarYears, jetPowerW, massRatioVe,
  propellantFractionVe, sailAccel, thrustRegime,
} from './propulsion';

const drive = (id: string) => DRIVES.find((d) => d.id === id)!;

describe('推进系统：Δv -> 可行性', () => {
  it('化学 vs 核热 vs 核电：同一 Δv，质量比按 v_e 指数拉开', () => {
    const dv = 4.23; // 地→火最佳窗口
    const chem = massRatioVe(dv, drive('chemical').exhaustKms);
    const ntr = massRatioVe(dv, drive('nuclear').exhaustKms);
    const nep = massRatioVe(dv, drive('nep').exhaustKms);
    expect(chem).toBeGreaterThan(2.5);   // 61% 推进剂
    expect(ntr).toBeLessThan(chem);      // 核热好一截
    expect(nep).toBeLessThan(1.2);       // 电推几乎不花推进剂
    expect(propellantFractionVe(dv, drive('nep'))).toBeLessThan(0.2);
    expect(propellantFractionVe(dv, drive('solar-sail'))).toBe(0); // 光帆不带推进剂
  });

  it('点火时长：化学是"点数分钟"，电推是"数月"连续推', () => {
    // 4.23 km/s：化学 ~3.5 分钟；电推 ~163 天
    expect(burnDays(4.23, drive('chemical').accelMps2)).toBeLessThan(0.01);
    expect(burnDays(4.23, drive('nep').accelMps2)).toBeGreaterThan(100);
    expect(thrustRegime(burnDays(4.23, drive('chemical').accelMps2), 250)).toBe('impulsive');
    expect(thrustRegime(burnDays(4.23, drive('nep').accelMps2), 250)).toBe('low-thrust');
  });

  it('星际门槛：v_e 必须与目标速度同量级，化学完全无解', () => {
    const dvInterstellar = 30000; // 0.1c
    // 化学：质量比 10^2955，任何语言都写不出来
    expect(massRatioVe(dvInterstellar, drive('chemical').exhaustKms)).toBeGreaterThan(1e300);
    expect(massRatioVe(dvInterstellar, drive('nep').exhaustKms)).toBeGreaterThan(1e100);
    // 聚变 v_e=1000 km/s：m0/mf = e^30 ≈ 1e13，仍然不可能
    expect(massRatioVe(dvInterstellar, drive('fusion').exhaustKms)).toBeGreaterThan(1e12);
    // v_e = 0.3c（反物质）：m0/mf ≈ 1.35，宽裕
    expect(massRatioVe(dvInterstellar, drive('antimatter').exhaustKms)).toBeLessThan(2);
  });

  it('最近的恒星：以 Voyager 的速度要 7 万年，0.1c 要 42 年', () => {
    expect(interstellarYears(4.24, 17)).toBeGreaterThan(70000);
    const y = interstellarYears(4.24, 30000);
    expect(y).toBeGreaterThan(40);
    expect(y).toBeLessThan(45);
  });

  it('电推的墙是电站质量：P ∝ F·v_e，比功率 α 让电站比载荷还重', () => {
    const p = jetPowerW(1000, drive('nep').exhaustKms); // 1000 N @ 29.4 km/s
    expect(p).toBeGreaterThan(1e7);                     // ~15 MW 喷流功率
    const plantKg = (p / 1000) * 5;                     // α = 5 kg/kW（含散热器更糟）
    expect(plantKg).toBeGreaterThan(5e4);               // >50 t 电站
    // 同样推力、v_e 提高 34 倍（聚变）：功率同倍增长
    expect(jetPowerW(1000, drive('fusion').exhaustKms) / p).toBeGreaterThan(30);
    // 光帆把能量留在家里：1 g 载荷配 100 GW 激光 -> 上万 g 的加速度（Starshot 量级）
    expect(sailAccel(1e11, 1e-3)).toBeGreaterThan(1e4);
  });
});
