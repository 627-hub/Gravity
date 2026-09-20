import { Vector3 } from 'three';

// 星际介质 (ISM) 里的"磁帆/超导线圈"到底有没有用。
//
// 关键不是磁场强不强，而是**磁场有没有连着动量源**：
//   · 行星磁层：场随行星自转（Ω×r）→ 可以做动量交换 → 已实现（电动力缆绳）。
//   · 星际介质：场被冻结在等离子体里，而等离子体相对恒星几乎静止 →
//     你相对它运动时它只会**拖你**。阻力 F = ½Cρv²A 与速度平方成正比，
//     在 0.1c 下即使 0.1 cm⁻³ 的密度也是 MN 级 —— 于是磁帆在星际尺度上
//     是**刹车**，不是引擎。这正是"到站减速"方案的物理基础，
//     也正是 Bussard 冲压发动机的致死伤：进气口阻力超过推力。
//
// 静态磁场里的"梯度/感应"推进同样不行：梯度力是保守力（只能把势能换动能，
// 绕一圈净功为零），感应电流按楞次定律只会反对相对运动（只能减速/发电）。
// 想要净冲量，必须有个**带动量的第三方**跟你交换。

/** 星际介质质子密度，cm^-3（本地泡约 0.1；稠密云可到 10²）。 */
export const ISM_DENSITY_CM3 = 0.1;

/** 质子质量，kg。 */
const M_P = 1.67262192e-27;

/** 太阳风动压 @1AU，Pa（对照用）。 */
export const SOLAR_WIND_DYNAMIC_PRESSURE_1AU = 1.34e-9;

/** 相对介质运动的动压，Pa。 */
export function ramPressurePa(vRelKms: number, densityCm3: number = ISM_DENSITY_CM3): number {
  const rho = densityCm3 * 1e6 * M_P;      // cm^-3 -> kg/m^3
  const v = vRelKms * 1000;                 // km/s -> m/s
  return 0.5 * rho * v * v;
}

/**
 * 磁帆/线圈相对介质运动的力，N。drag > 0 表示**被拖住**（减速）——
 * 星际介质里这是唯一的结果；只有介质相对你朝有利方向流动（恒星风、
 * 磁化流）时才可能变成推力，方向由流速定。
 */
export function ismForceN(
  areaKm2: number,
  vRelKms: number,
  densityCm3: number = ISM_DENSITY_CM3,
  dragCoeff = 1.5,
): number {
  const area = areaKm2 * 1e6; // km² -> m²
  return dragCoeff * ramPressurePa(vRelKms, densityCm3) * area;
}

/**
 * 以给定等效面积从 v0 减速到 vf 需要的距离，ly。
 * （能量法：½ m v² 的变化 = ∫F ds，F ∝ v²，故 d = m/(CρA) · ln(v0/vf)）
 */
export function brakingDistanceLy(
  massKg: number,
  areaKm2: number,
  v0Kms: number,
  vfKms: number,
  densityCm3: number = ISM_DENSITY_CM3,
  dragCoeff = 1.5,
): number {
  const rho = densityCm3 * 1e6 * M_P;
  const area = areaKm2 * 1e6;
  // m dv/dt = -½CρAv²  =>  ds = -(2m/(CρA)) dv/v  =>  s = (2m/(CρA))·ln(v0/vf)
  const k = massKg / (0.5 * dragCoeff * rho * area);  // 阻尼长度，m
  const dM = k * Math.log(v0Kms / Math.max(vfKms, 1e-9));
  return dM / 9.4607e15;                               // m -> ly
}

/**
 * Bussard 冲压发动机的进气口阻力与推力的量级比。
 * 磁进气口半径 R 的阻力 ∝ ρv²·πR²，可用的聚变功率 ∝ ρv·πR²·(每个质子能量)。
 * 结果：低速时阻力压过推力；经典分析给出的门槛在百分之几 c 量级。
 * 这里只返回"单位进气口面积的可用功率密度"，W/m²（用于量级判断）。
 */
export function ramjetPowerFluxWm2(vKms: number, densityCm3: number = ISM_DENSITY_CM3): number {
  const rho = densityCm3 * 1e6 * M_P;
  return rho * (vKms * 1000) * 6.5e6 * 1.6e-19; // p-p 聚变 ~6.5 MeV/质子 * 效率
}

/** 光帆在激光束下的加速度：a = 2P/(c·m)，束斑扩散后功率密度按 1/L² 衰减。 */
export function beamSailAccelMps2(
  laserW: number,
  sailMassKg: number,
  rangeAu: number,
  diffractionLimitM = 1e4, // D·λ 的等效发射口径，决定远方束斑
): number {
  const rangeM = rangeAu * 1.495978707e11;
  // 束斑半径 ~ λL/D；用等效口径近似：半径 = 衍射极限 × 距离(km)/1e6
  const spotM = Math.max(1, (diffractionLimitM * rangeM) / 1e6);
  const density = laserW / (Math.PI * spotM * spotM);
  const area = Math.max(1, sailMassKg / 1e-3); // 1 g/m² 级薄膜
  const force = (2 * density * area) / 299792458;
  return force / Math.max(sailMassKg, 1e-9);
}

/** 由状态给一个方向（与 propulsion.thrustDirection 的极简版）：径向外。 */
export function radialOut(pos: Vector3, out: Vector3 = new Vector3()): Vector3 {
  return out.copy(pos).normalize();
}
