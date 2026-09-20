import { describe, expect, it } from 'vitest';
import {
  brakingDistanceLy, beamSailAccelMps2, ismForceN, ramPressurePa, ramjetPowerFluxWm2,
} from './interstellar';

describe('星际介质里的磁帆：是刹车，不是引擎', () => {
  it('0.1c 下动压已远超太阳风（速度平方的威力）', () => {
    const p = ramPressurePa(0.1 * 299792.458); // 0.1c
    expect(p).toBeGreaterThan(5e-8);            // ~0.075 µPa
    expect(p / 1.34e-9).toBeGreaterThan(50);    // 比 1AU 太阳风还强
  });

  it('1000 km 等效面积在 0.1c 下是 MN 级阻力 —— 根本不是"微弱推进"', () => {
    const f = ismForceN(1000 * 1000, 0.1 * 299792.458); // 面积 km² -> 注意是半径 1000 km 的帆
    expect(f).toBeGreaterThan(1e5);   // >0.1 MN
    // 100 t 飞船 -> 加速度 m/s² 量级：飞不到巡航，会当场刹住
    expect(f / 1e5).toBeGreaterThan(1);
  });

  it('从 0.2c 刹到 0.05c 的刹车距离是"几光年"量级 —— 可作到站方案', () => {
    const d = brakingDistanceLy(1e5, 1000 * 1000, 0.2 * 299792.458, 0.05 * 299792.458);
    expect(d).toBeGreaterThan(0.1);   // >0.1 ly
    expect(d).toBeLessThan(100);      // 但远小于 4.24 ly
  });

  it('太阳风里能推、星际介质里只能拖：介质必须相对你有流动', () => {
    // 同样面积，太阳风(1AU, 400km/s, 5/cm³) vs 星际(0.1c, 0.1/cm³)
    const solarWind = 1.34e-9 * 1e12;   // 面积 1e12 m²
    const ism = ismForceN(1000 * 1000, 0.1 * 299792.458);
    // 太阳风给的是"推力"（向外流），ISM 给的是"阻力"（迎面而来）——方向相反
    expect(solarWind).toBeGreaterThan(100);
    expect(ism).toBeGreaterThan(solarWind); // 但星际的那份是拖你的
  });

  it('冲压发动机：单位进气口面积的可用功率很低（阻力/加热是致命伤）', () => {
    const flux = ramjetPowerFluxWm2(0.1 * 299792.458);
    expect(flux).toBeLessThan(1e-3); // W/m² 量级
  });

  it('束能帆：束斑扩散让加速度随距离急降（所以激光阵要够大）', () => {
    const near = beamSailAccelMps2(1e11, 1, 0.01);
    const far = beamSailAccelMps2(1e11, 1, 10);
    expect(near).toBeGreaterThan(far);
    expect(far / near).toBeLessThan(1e-3); // 至少三个量级的衰减
  });
});
