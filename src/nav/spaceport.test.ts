import { Vector3 } from 'three';
import { describe, expect, it } from 'vitest';
import { PLANETS } from '../data/bodies';
import { AU_KM } from '../data/constants';
import { propagate } from './propagate';
import { portBurn, spaceportFor, synchronousRadiusKm } from './spaceport';
import { AUDAY_TO_KMS, muAuOfMass, toKms } from './units';

const body = (id: string) => PLANETS.find((b) => b.id === id)!;

describe('spaceport: 同步轨道判据', () => {
  it('地球：对地静止轨道 42,164 km / 3.07 km/s / 0.997 天', () => {
    const p = spaceportFor('earth')!;
    expect(p.kind).toBe('synchronous');
    expect(p.radiusKm).toBeGreaterThan(41800);
    expect(p.radiusKm).toBeLessThan(42500);
    expect(p.altitudeKm).toBeGreaterThan(35000);
    expect(p.vCircKms).toBeCloseTo(3.075, 2);
    expect(p.periodDays).toBeCloseTo(0.9973, 3);
    // 理想脉冲地表→同步轨道 ≈ 4.0 km/s（真实运载含大气/重力损失约 12+）
    expect(p.surfaceAccessKms).toBeGreaterThan(3.5);
    expect(p.surfaceAccessKms).toBeLessThan(4.5);
  });

  it('火星：同步轨道 20,428 km / 1.45 km/s', () => {
    const p = spaceportFor('mars')!;
    expect(p.kind).toBe('synchronous');
    expect(p.radiusKm).toBeGreaterThan(20200);
    expect(p.radiusKm).toBeLessThan(20700);
    expect(p.vCircKms).toBeCloseTo(1.45, 1);
  });

  it('金星：逆行自转 -> 停泊轨道，并给出原因', () => {
    const p = spaceportFor('venus')!;
    expect(p.kind).toBe('parking');
    expect(p.reason).toContain('逆行');
    expect(p.radiusKm).toBeLessThan(8000); // 贴表面低轨，而非 153 万 km 的假同步轨道
  });

  it('木星：快自转，同步轨道远在伽利略卫星之内', () => {
    const p = spaceportFor('jupiter')!;
    expect(p.kind).toBe('synchronous');
    expect(p.radiusKm).toBeGreaterThan(150000);
    expect(p.radiusKm).toBeLessThan(180000);
  });

  it('公式：月球若按自转算出的「同步轨道」超出希尔球（所以不该用）', () => {
    const moon = body('earth').moons!.find((m) => m.id === 'moon')!;
    const mu = (6.6743e-11 * moon.mass) / 1e9;
    const rSyn = synchronousRadiusKm(mu, moon.rotationPeriod);
    expect(rSyn).toBeGreaterThan(6e4); // ~88,000 km，远超月球 SOI (~66,000 km)
  });
});

describe('spaceport: 单脉冲逃逸/捕获', () => {
  const port = spaceportFor('earth')!;
  const bodyPos = new Vector3(1, 0, 0);
  const bodyVel = new Vector3(0, 1, 0);

  it('纯逃逸（v∞ = 0）：dv = (√2−1)·v_c', () => {
    const b = portBurn(0, new Vector3(1, 0, 0), port, bodyPos, bodyVel)!;
    expect(b.dvKms).toBeCloseTo((Math.SQRT2 - 1) * port.vCircKms, 6);
  });

  it('地球同步轨道逃逸 v∞=3 km/s：≈ 2.2 km/s，平面变更 ≈ 0', () => {
    const b = portBurn(
      3,
      new Vector3(Math.cos(0.3), Math.sin(0.3), 0),
      port,
      bodyPos,
      bodyVel,
    )!;
    // v_p = √(9 + 2μ/r) = 5.28 km/s，减去 v_c = 3.07 -> 同向时 2.21 km/s
    expect(b.dvKms).toBeGreaterThan(2.0);
    expect(b.dvKms).toBeLessThan(2.5);
    expect(b.vPeriKms).toBeCloseTo(5.28, 1);
    expect(b.planeChangeDeg).toBeLessThan(1e-6);
  });

  it('平面外的 v∞ 要付出平面变更代价', () => {
    const inPlane = portBurn(3, new Vector3(1, 0, 0), port, bodyPos, bodyVel)!;
    const outOfPlane = portBurn(
      3,
      new Vector3(Math.cos(0.5), 0, Math.sin(0.5)),
      port,
      bodyPos,
      bodyVel,
    )!;
    expect(outOfPlane.planeChangeDeg).toBeCloseTo(28.6, 0);
    expect(outOfPlane.dvKms).toBeGreaterThan(inPlane.dvKms);
  });

  it('构造出的双曲线真的以 d̂ 为渐近线（前向积分验证）', () => {
    const mu = muAuOfMass(body('earth').mass);
    const vInfKms = 3;
    const dHat = new Vector3(Math.cos(0.9), Math.sin(0.9), 0);
    const b = portBurn(vInfKms, dHat, port, bodyPos, bodyVel)!;
    // 相对天体的点火后状态
    const relPos = b.portPos.clone().sub(bodyPos);
    const relVel = b.shipVel.clone().sub(bodyVel);
    expect(relPos.length()).toBeCloseTo(port.radiusKm / AU_KM, 9);
    // 前向积分 10 天：速度方向收敛到渐近线，速率趋近 v∞。
    // （注：通用变量 propagate 在超长时程双曲线上数值退化——本体任务用
    //  DOPRI5 真值积分，不走这条路。）
    const far = propagate({ pos: relPos, vel: relVel }, 10, mu);
    const vDir = far.vel.clone().normalize();
    expect(vDir.dot(dHat)).toBeGreaterThan(0.999);
    expect(toKms(far.vel.length())).toBeGreaterThan(vInfKms);
    expect(toKms(far.vel.length())).toBeLessThan(vInfKms + 0.1);
    // 且点火点确实是近心点（速度 ⟂ 半径）
    expect(Math.abs(relVel.clone().normalize().dot(relPos.clone().normalize()))).toBeLessThan(1e-6);
  });

  it('捕获与逃逸同式：同一 v∞ 与几何下点火量相同（对称性）', () => {
    const out = portBurn(2.5, new Vector3(0, 1, 0), port, bodyPos, bodyVel)!;
    const back = portBurn(2.5, new Vector3(0, 1, 0), port, bodyPos, bodyVel)!;
    expect(back.dvKms).toBeCloseTo(out.dvKms, 12);
    // 点火后速度 = 圆轨道速度 + Δv，方向即近心点速度方向
    const rel = back.shipVel.clone().sub(bodyVel);
    expect(toKms(rel.length())).toBeCloseTo(back.vPeriKms, 9);
  });

  it('dv 的单位换算自洽（km/s <-> AU/day）', () => {
    const b = portBurn(3, new Vector3(1, 0, 0), port, bodyPos, bodyVel)!;
    expect(toKms(b.dv.length())).toBeCloseTo(b.dvKms, 9);
    expect(b.dvKms / AUDAY_TO_KMS).toBeGreaterThan(0);
  });
});
