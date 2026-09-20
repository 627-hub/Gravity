import { Vector3 } from 'three';
import { PLANETS, SUN } from '../data/bodies';
import { AU_KM, DAY, G } from '../data/constants';
import { soiRadius } from './sources';
import { fromKms, toKms } from './units';

// 太空港 (spaceport)：行星际飞船的出发/抵达端点。
//
// 飞船不再从行星中心出发，而是停靠在**同步轨道**上的太空港（对地静止式
// 轨道，周期 = 天体自转周期）。地表 ↔ 太空港由另一套运输系统承担（大气层内
// 飞行、气动、热防护、大推力——设计约束与行星际飞船完全不同），本模块不建模
// 它，只给出理想脉冲量级作为「另案参考」。
//
// 两个物理判据决定了港口类型：
//   1. 自转必须顺行且足够快，同步半径 = (μT²/4π²)^(1/3) 才存在且在表面之上；
//   2. 同步半径必须落在希尔球稳定范围内（取 0.4 × SOI）。潮汐锁定天体
//      （月球：T = 27.3 天，同步半径 ~88,000 km，远超其希尔球）与慢自转/
//      逆行天体（金星：243 天逆行）都不满足 —— 这些天体降级为**停泊轨道**
//      太空港（贴表面的低圆轨道），并在界面标注原因。
//
// 港口轨道面取天体轨道面（行星即黄道面）。真实的同步港口在赤道面上，倾角带来
// 的平面变更是发射场纬度/相位吸收的二级效应，这里统一按黄道面口径计算。

/** 天体 μ，km³/s²。 */
export function muKmsOfMass(massKg: number): number {
  return (G * massKg) / 1e9;
}

/** 同步轨道半径：r = (μT²/4π²)^(1/3)，km。 */
export function synchronousRadiusKm(muKms: number, periodDays: number): number {
  const T = Math.abs(periodDays) * DAY;
  return Math.cbrt((muKms * T * T) / (4 * Math.PI * Math.PI));
}

export interface Spaceport {
  bodyId: string;
  bodyName: string;
  /** 'synchronous' = 真同步轨道；'parking' = 停泊轨道（该天体无可用同步轨道）。 */
  kind: 'synchronous' | 'parking';
  /** 轨道半径（距天体中心）与高度，km。 */
  radiusKm: number;
  altitudeKm: number;
  /** 轨道周期，天；圆轨道速度，km/s。 */
  periodDays: number;
  vCircKms: number;
  /** 天体 μ，km³/s²；天体半径，km。 */
  muKms: number;
  bodyRadiusKm: number;
  /** 地表↔港口的理想脉冲（Hohmann，不含大气/重力损失）——另案系统的量级参考。 */
  surfaceAccessKms: number;
  /** 降级原因（仅 parking）。 */
  reason?: string;
}

/** 港口必须明显高于表面。 */
const MIN_CLEARANCE_KM = 200;
/** 同步半径超过 SOI 的这个比例即认为不稳定（第三体摄动/无法长期保持）。 */
const STABILITY_FRACTION = 0.4;

/** 停泊轨道高度：贴表面的低轨，按天体尺度缩放。 */
function parkingAltitudeKm(bodyRadiusKm: number): number {
  return Math.min(400, Math.max(100, 0.06 * bodyRadiusKm));
}

/** 从表面到港口圆轨道的理想双脉冲（Hohmann）速度增量，km/s。 */
export function surfaceAccessKms(muKms: number, r1: number, r2: number): number {
  const a = (r1 + r2) / 2;
  const vCirc1 = Math.sqrt(muKms / r1);
  const vPeri1 = Math.sqrt(muKms * (2 / r1 - 1 / a));
  const vApo2 = Math.sqrt(muKms * (2 / r2 - 1 / a));
  const vCirc2 = Math.sqrt(muKms / r2);
  return vPeri1 - vCirc1 + (vCirc2 - vApo2);
}

/** 某天体的太空港；无轨道数据（如太阳）返回 null。 */
export function spaceportFor(bodyId: string): Spaceport | null {
  const body = PLANETS.find((b) => b.id === bodyId);
  if (!body || !body.orbit) return null;

  const muKms = muKmsOfMass(body.mass);
  const rot = body.rotationPeriod;
  const rSynKm = rot > 0 ? synchronousRadiusKm(muKms, rot) : NaN;
  const soiKm = soiRadius(body.orbit.a, muKmsOfMass(SUN.mass), muKms) * AU_KM;

  let kind: 'synchronous' | 'parking' = 'synchronous';
  let reason: string | undefined;
  if (!(rot > 0)) {
    kind = 'parking';
    reason = '自转逆行，同步轨道不存在';
  } else if (rSynKm - body.radius < MIN_CLEARANCE_KM) {
    kind = 'parking';
    reason = `同步轨道高度仅 ${Math.round(rSynKm - body.radius)} km，低于净空要求`;
  } else if (rSynKm > STABILITY_FRACTION * soiKm) {
    kind = 'parking';
    reason = `同步半径 ${Math.round(rSynKm / 1000)} 千km 超出希尔球稳定范围（SOI 的 40% = ${Math.round((STABILITY_FRACTION * soiKm) / 1000)} 千km）`;
  }

  const radiusKm =
    kind === 'synchronous' ? rSynKm : body.radius + parkingAltitudeKm(body.radius);
  const periodDays =
    kind === 'synchronous'
      ? Math.abs(rot)
      : (2 * Math.PI * Math.sqrt((radiusKm * radiusKm * radiusKm) / muKms)) / DAY;

  return {
    bodyId: body.id,
    bodyName: body.name,
    kind,
    radiusKm,
    altitudeKm: radiusKm - body.radius,
    periodDays,
    vCircKms: Math.sqrt(muKms / radiusKm),
    muKms,
    bodyRadiusKm: body.radius,
    surfaceAccessKms: surfaceAccessKms(muKms, body.radius, radiusKm),
    reason,
  };
}

export interface PortBurn {
  /** 点火量，km/s。 */
  dvKms: number;
  /** Δv 矢量（天体相对，黄道坐标，AU/day）。 */
  dv: Vector3;
  /** 点火时刻港口相对天体的位置方向（单位矢量，黄道坐标）。 */
  portDir: Vector3;
  /** 港口相位（弧度，从 +X 起算）。 */
  phaseRad: number;
  /** 点火后飞船的日心位置/速度（AU, AU/day）。 */
  portPos: Vector3;
  shipVel: Vector3;
  /** 港口圆轨道速度；双曲线近心点速度，km/s。 */
  vCircKms: number;
  vPeriKms: number;
  /** 渐近线方向相对港口轨道面的倾角（度）——需要吸收的平面变更。 */
  planeChangeDeg: number;
  /** 天体半径（km），便于调用方做软化/显示。 */
  bodyRadiusKm: number;
}

/**
 * 从港口圆轨道到给定双曲线（离开：逃逸；抵达：捕获制动）的**单脉冲**解。
 *
 * 一阶 patched conic（忽略大气/重力损失）。几何是硬的：
 *  1. 偏心率 e = 1 + r v∞²/μ -> 渐近线真近点角 ν∞ = acos(-1/e)（> 90°）；
 *  2. 单脉冲能量给出 |v_p| = √(v∞² + 2μ/r)，且**点火点必须是近心点**（速度 ⟂
 *     半径）。因此渐近线与点火点方向的夹角被锁死为 ν∞，不能任意取；
 *  3. 于是点火点方向 r̂_p 是「港口轨道面内的单位圆」与「与 d̂ 夹角 ν∞ 的圆锥」
 *     的交点 —— 两个候选，取代价小的那个；
 *  4. 近心点速度方向 = 把 r̂_p 绕平面法线 (r̂_p × d̂) 转 +90°；Δv =
 *     |v_p v̂_p − v_c v̂_c|（单脉冲余弦定理，平面变更自动计入）。
 *
 * 港口相位即发射窗口内等一个自转周期以内的自由设计量，这里取最省燃料的相位。
 * 返回 null 表示几何退化（渐近线几乎垂直于港口轨道面且无解）。
 */
export function portBurn(
  vInfKms: number,
  vInfDir: Vector3,
  port: Spaceport,
  bodyPos: Vector3,
  bodyVel: Vector3,
): PortBurn | null {
  const r = port.radiusKm;
  const mu = port.muKms;
  const vc = port.vCircKms;
  const bodyRadiusKm = port.bodyRadiusKm;
  const zAxis = new Vector3(0, 0, 1);

  const build = (
    rHat: Vector3,
    vHatP: Vector3,
  ): PortBurn => {
    const dv = vHatP.clone().multiplyScalar(fromKms(vp)).sub(vHatC.clone().multiplyScalar(fromKms(vc)));
    return {
      dvKms: toKms(dv.length()),
      dv,
      portDir: rHat.clone(),
      phaseRad: Math.atan2(rHat.y, rHat.x),
      portPos: bodyPos.clone().addScaledVector(rHat, r / AU_KM),
      // 点火后：飞船相对天体的速度 = 港口圆轨道速度 + Δv = v_p·v̂_p
      shipVel: bodyVel.clone().addScaledVector(vHatP, fromKms(vp)),
      vCircKms: vc,
      vPeriKms: vp,
      planeChangeDeg: (Math.asin(Math.min(1, Math.abs(d.z))) * 180) / Math.PI,
      bodyRadiusKm,
    };
  };

  const d = vInfDir.clone().normalize();
  if (vInfKms < 1e-9) {
    // 纯逃逸（C3 = 0）：抛物线，dv = (√2 − 1) v_c，沿港口顺行方向切向点火。
    const rHat = new Vector3(1, 0, 0);
    const vHat = new Vector3(0, 1, 0);
    const dvKms = (Math.SQRT2 - 1) * vc;
    return {
      dvKms,
      dv: vHat.clone().multiplyScalar(fromKms(dvKms)),
      portDir: rHat.clone(),
      phaseRad: 0,
      portPos: bodyPos.clone().addScaledVector(rHat, r / AU_KM),
      shipVel: bodyVel.clone().addScaledVector(vHat, fromKms(Math.SQRT2 * vc)),
      vCircKms: vc,
      vPeriKms: Math.SQRT2 * vc,
      planeChangeDeg: 0,
      bodyRadiusKm,
    };
  }

  const vp = Math.sqrt(vInfKms * vInfKms + (2 * mu) / r);
  const e = 1 + (r * vInfKms * vInfKms) / mu;
  const cosNu = Math.cos(Math.acos(-1 / e)); // < 0

  // 港口轨道面 = 黄道面：面内基底 û（沿 d̂ 的面内投影）与 ŵ。
  const dPlane = new Vector3(d.x, d.y, 0);
  const dPlaneLen = dPlane.length();
  const uHat = dPlaneLen > 1e-9 ? dPlane.clone().normalize() : new Vector3(1, 0, 0);
  const wHat = new Vector3().crossVectors(zAxis, uHat);

  // r̂_p = cosδ·û + sinδ·ŵ，要求 r̂_p·d̂ = cosδ·|d_Π| = cos ν∞。
  const candidates: number[] = [];
  if (dPlaneLen > 1e-9) {
    const c = cosNu / dPlaneLen;
    if (Math.abs(c) <= 1) {
      const base = Math.acos(c);
      candidates.push(base, -base);
    }
  }
  if (!candidates.length) return null; // 渐近线几乎 ⟂ 港口轨道面：单脉冲近心点解不存在

  let best: PortBurn | null = null;
  const rHat = new Vector3();
  const vHatP = new Vector3();
  const vHatC = new Vector3();
  const n = new Vector3();
  for (const delta of candidates) {
    rHat.copy(uHat).multiplyScalar(Math.cos(delta)).addScaledVector(wHat, Math.sin(delta));
    n.crossVectors(rHat, d);
    if (n.lengthSq() < 1e-12) continue;
    n.normalize();
    // 近心点速度方向：r̂_p 绕双曲线平面法线转 +90°，此时轨道法线 = n̂。
    vHatP.copy(rHat).applyAxisAngle(n, Math.PI / 2).normalize();
    // 港口顺行圆轨道速度方向（+Z 自转）。
    vHatC.crossVectors(zAxis, rHat).normalize();
    const b = build(rHat, vHatP);
    if (!best || b.dvKms < best.dvKms) best = b;
  }
  return best;
}
