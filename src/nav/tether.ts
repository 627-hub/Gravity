import { Vector3 } from 'three';

// 无工质推进的第二类：靠**外部磁场/等离子体**拿动量。
//
// 1) 电动力缆绳 (electrodynamic tether)：导电缆绳在行星磁场里运动产生
//    EMF，通电后受洛伦兹力 F = I·L×(v_rel×B)。关键在 **v_rel = v − Ω×r**：
//    磁场随行星自转，所以缆绳是在和行星自转做动量/能量交换 —— 这才是完整的
//    "外部动量"。两种工作模式：
//      · 电动模式（耗电）：注入电流 → 力抬升轨道；
//      · 发电模式（取电）：让感应电流流出 → 力使轨道下降，同时把动能转成电。
//    偶极场下，径向缆绳的力恒为**东西向**（∝ sinθ 磁纬），赤道面上为零——
//    这是真实缆绳任务都在倾斜轨道上的原因。
//
// 2) 磁帆 / 电帆：太阳风动压 ρv² 的动量（不是光压）。磁帆用超导线圈撑出
//    磁层顶等效面积；电帆用带电导线偏转质子。两者推力都沿太阳风（径向外），
//    可小幅偏摆产生切向分量。磁帆 F ∝ 1/r²，电帆 F ∝ 1/r。
//
// 太阳风动压 (~1.3 nPa @1AU) 比光压 (~4.6 µPa) 弱约 300 倍，但等效面积可以
// 大到万倍（磁层顶 ~百 km），所以量级可比。

/** 天体磁偶极矩 M（A·m²，取赤道表面场反推）；0 = 无可用的内禀磁场。 */
export const DIPOLE_MOMENT: Record<string, number> = {
  earth: 7.94e22,
  jupiter: 1.55e27,
  saturn: 4.6e25,
  uranus: 3.8e24,
  neptune: 1.4e24,
  ganymede: 1.3e20,
  mars: 0, // 只有地壳剩磁，行星际导航用不上
  venus: 0,
  mercury: 0, // 弱场（~300 nT），工程上不予考虑
};

const MU0_OVER_4PI = 1e-7;

/**
 * 偶极磁场在 r 处的方向与大小（T）。偶极轴取黄道法线（+Z）——我们并没有
 * 建模各行星自转轴的空间取向，这是明确的简化。
 * B = (µ0 M / 4π r³) · (2 cosθ r̂ + sinθ θ̂)
 */
export function dipoleField(momentAm2: number, rVec: Vector3, out: Vector3): Vector3 {
  const r = rVec.length();
  if (momentAm2 === 0 || r < 1) return out.set(0, 0, 0);
  const rHat = out.copy(rVec).divideScalar(r);
  const cosT = rHat.z;
  const sinT = Math.sqrt(Math.max(0, 1 - cosT * cosT));
  // θ̂：沿 θ 增大的方向（从 +Z 转向赤道面）
  const thetaHat = new Vector3(rHat.x * cosT, rHat.y * cosT, -sinT);
  if (thetaHat.lengthSq() > 1e-12) thetaHat.normalize();
  const k = (MU0_OVER_4PI * momentAm2) / (r * r * r);
  return out.copy(rHat).multiplyScalar(2 * cosT * k).addScaledVector(thetaHat, sinT * k);
}

export interface TetherParams {
  /** 缆绳长度，km（沿局地垂线/径向展开）。 */
  lengthKm: number;
  /** 电流，A（正 = 电动模式抬升轨道，负 = 发电模式下降轨道）。 */
  currentA: number;
}

/**
 * 径向缆绳的洛伦兹力，N（矢量，日心黄道系方向）。
 * rVec/vRel 需为**相对行星**的矢量（vRel 已扣掉行星自转的牵连速度）。
 */
export function tetherForceN(
  tether: TetherParams,
  momentAm2: number,
  rVec: Vector3,
  vRel: Vector3,
  out: Vector3 = new Vector3(),
): Vector3 {
  const b = dipoleField(momentAm2, rVec, new Vector3());
  // F = I·L·(L̂ × B)，L̂ = r̂
  const rHat = new Vector3().copy(rVec).normalize();
  const lCrossB = new Vector3().crossVectors(rHat, b);
  return out.copy(lCrossB).multiplyScalar(tether.currentA * tether.lengthKm * 1000);
}

/** 电动模式所需电功率，W（EMF = (v_rel × B)·L，P = I·EMF）。 */
export function tetherPowerW(
  tether: TetherParams,
  momentAm2: number,
  rVec: Vector3,
  vRel: Vector3,
): number {
  const b = dipoleField(momentAm2, rVec, new Vector3());
  const emf = new Vector3().crossVectors(vRel, b).length() * tether.lengthKm * 1000;
  return Math.abs(tether.currentA) * emf;
}

// ---- 太阳风帆（磁帆 / 电帆） ----------------------------------------------

/** 1 AU 处太阳风动压 ρv²，Pa（n ≈ 5 cm^-3，v ≈ 400 km/s）。 */
export const SOLAR_WIND_PRESSURE_1AU = 1.34e-9;

export interface WindSailParams {
  /** 'magsail' = 超导磁层顶；'esail' = 带电导线阵。 */
  kind: 'magsail' | 'esail';
  /** 磁帆：磁层顶等效半径 km；电帆：导线总长 km。 */
  size: number;
  /** 电帆导线数（磁帆忽略）。 */
  wires?: number;
}

/**
 * 太阳风帆推力，N（1 AU 处），方向沿太阳风（径向外）。
 * 磁帆：F = p·πR_mp²，F ∝ 1/r²；
 * 电帆：F = 5e-7 N/m · 导线总长，F ∝ 1/r（德拜鞘尺度修正）。
 *   量级依据：文献给出的电帆性能 ≈ 0.5 N / 1000 km 导线 @1 AU。
 *   从太阳风动压直接推导会差一个德拜鞘半径的因子（量级量），故采用文献值。
 */
export function windSailThrustN(params: WindSailParams, rAu: number): number {
  const r = Math.max(rAu, 1e-3);
  if (params.kind === 'magsail') {
    const rmp = params.size * 1000; // km -> m
    return SOLAR_WIND_PRESSURE_1AU * Math.PI * rmp * rmp / (r * r);
  }
  const wires = params.wires ?? 1;
  const lengthM = params.size * 1000 * wires;
  const perMetre = 5e-7; // N/m @1AU（≈0.5 N / 1000 km 导线）
  return (perMetre * lengthM) / r;
}
