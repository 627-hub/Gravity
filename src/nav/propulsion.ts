import { Vector3 } from 'three';

// 推进系统：把 Δv 换算成"能不能做到"。
//
// 火箭方程的暴政：为了推动推进剂本身，需要的推进剂按 **指数** 增长，
//   m0/mf = exp(Δv / v_e)
// 所以推进系统的关键不是推力大小，而是**排气速度 v_e = Isp·g0**：
//   - 化学  v_e ≈ 4.4 km/s   → 行星际够用，星际绝无可能（v_e 比目标速度小 4 个量级）；
//   - 核热  v_e ≈ 8.8 km/s   → 同 Δv 下质量比从 e^x 变成 e^(x/2)；
//   - 核电  v_e ≈ 30 km/s    → 质量比可观，但推力极小（mN~N）→ 走螺旋不清爽；
//   - 聚变  v_e ~ 10^3 km/s  → 星际的门槛（v_e 需与目标速度同量级）；
//   - 反物质 v_e ~ 10^5 km/s → 宽裕；
//   - 光帆/束能：**不携带推进剂**，能量留在家里（外部提供）→ 完全绕开火箭方程。
//
// 另一个硬约束是功率：P_jet = F·v_e/2。高 v_e 意味着同样推力要天文数字的功率，
// 而电推的电站+散热器质量往往比推进剂还重。这是"核能/电推"路线真正的墙。

/** 推力/点火方向（相对飞船当前状态）。 */
export type ThrustDir =
  | 'prograde' | 'retrograde' | 'radialOut' | 'radialIn' | 'normal' | 'antiNormal'
  /** 光帆外扩：帆面法线偏离日心方向一个锥角并朝顺行侧——纯径向光压不做功
   *  （F·v = 0），必须靠这个切向分量才能把轨道螺旋推出去。 */
  | 'sailOut';

/** 光帆锥角（度）：真实帆的典型工作点。 */
export const SAIL_CONE_DEG = 35;

/** 由状态解出方向单位矢量（日心黄道系）。 */
export function thrustDirection(
  dir: ThrustDir,
  pos: Vector3,
  vel: Vector3,
  out: Vector3 = new Vector3(),
): Vector3 {
  if (dir === 'sailOut') {
    const rHat = out.copy(pos).normalize();
    const vHat = new Vector3().copy(vel).normalize();
    const a = (SAIL_CONE_DEG * Math.PI) / 180;
    return rHat.multiplyScalar(Math.cos(a)).addScaledVector(vHat, Math.sin(a)).normalize();
  }
  if (dir === 'prograde') return out.copy(vel).normalize();
  if (dir === 'retrograde') return out.copy(vel).normalize().negate();
  if (dir === 'radialOut') return out.copy(pos).normalize();
  if (dir === 'radialIn') return out.copy(pos).normalize().negate();
  const h = new Vector3().crossVectors(pos, vel).normalize();
  return dir === 'normal' ? out.copy(h) : out.copy(h).negate();
}

export interface Drive {
  id: string;
  label: string;
  /** 推进方式：自带工质 / 太阳帆（a∝1/r²） / 束能帆（射程内近似常数）。 */
  kind: 'rocket' | 'sail-solar' | 'sail-beamed';
  /** 有效排气速度 v_e = Isp·g0，km/s（光帆=Infinity，不携带推进剂）。 */
  exhaustKms: number;
  /** 工作加速度量级，m/s²（决定一次点火要烧多久）。 */
  accelMps2: number;
  /** 是否自带推进剂。 */
  propellant: boolean;
  /** Isp（s）；光帆无。 */
  ispS: number | null;
  note: string;
}

export const DRIVES: Drive[] = [
  { kind: 'rocket', id: 'chemical', label: '化学', exhaustKms: 4.413, accelMps2: 20, propellant: true, ispS: 450,
    note: 'Isp 450 s：推质比高、比冲低。行星际的主力，星际没戏。' },
  { kind: 'rocket', id: 'nuclear', label: '核热', exhaustKms: 8.83, accelMps2: 5, propellant: true, ispS: 900,
    note: 'Isp 900 s：同 Δv 的质量比开方，地面样机验证过。' },
  { kind: 'rocket', id: 'nep', label: '核电推进', exhaustKms: 29.4, accelMps2: 3e-4, propellant: true, ispS: 3000,
    note: 'Isp 3000 s：质量比可观，但推力 mN~N 级，只能走螺旋，数月连续点火。' },
  { kind: 'rocket', id: 'fusion', label: '聚变', exhaustKms: 1000, accelMps2: 1e-3, propellant: true, ispS: 102000,
    note: 'v_e ~10³ km/s：星际的门槛（v_e 与目标速度同量级才行），尚未实现。' },
  { kind: 'rocket', id: 'antimatter', label: '反物质', exhaustKms: 1e5, accelMps2: 1e-2, propellant: true, ispS: 1.02e7,
    note: 'v_e ~0.3c：质量比宽裕，但反物质产量以 ng 计、储存极难。' },
  { kind: 'sail-solar', id: 'solar-sail', label: '太阳帆', exhaustKms: Infinity, accelMps2: 1e-4, propellant: false, ispS: null,
    note: '不携带推进剂：动量来自太阳光子（a ∝ 1/r²，1 AU 处 ~0.1 mm/s²）。必须偏锥角才有切向推力。' },
  { kind: 'sail-beamed', id: 'beam-sail', label: '束能帆', exhaustKms: Infinity, accelMps2: 1e-3, propellant: false, ispS: null,
    note: '不携带推进剂：激光阵从母星持续供能供动量，射程内加速度近似常数、方向可指。' },
];

/** 用排气速度表达的质量比 m0/mf。 */
export function massRatioVe(dvKms: number, exhaustKms: number): number {
  return Math.exp(dvKms / exhaustKms);
}

/** 推进剂质量分数（单级，不含结构质量）。光帆为 0（不携带推进剂）。 */
export function propellantFractionVe(dvKms: number, drive: Drive): number {
  if (!drive.propellant) return 0;
  return 1 - 1 / massRatioVe(dvKms, drive.exhaustKms);
}

/** 一次点火时长（天）：Δv / 加速度（假设质量近似不变，仅作量级判断）。 */
export function burnDays(dvKms: number, accelMps2: number): number {
  return (dvKms * 1000) / accelMps2 / 86400;
}

/**
 * 推力模式：一次点火时长相对转移时间是"脉冲"还是"螺旋"。
 * 脉冲式（化学/核热）= 现有 Lambert/TCM 模型成立；螺旋式（电推）= 需要
 * 连续推力积分，且窗口约束基本消失。
 */
export function thrustRegime(days: number, tofDays: number): 'impulsive' | 'low-thrust' {
  return days < 0.02 * tofDays ? 'impulsive' : 'low-thrust';
}

/** 喷流功率 P = F·v_e/2，W（电推真正的墙：电站与散热器质量）。 */
export function jetPowerW(thrustN: number, exhaustKms: number): number {
  if (!Number.isFinite(exhaustKms)) return thrustN * 3e8 / 2; // 光帆：F = P/c
  return (thrustN * exhaustKms * 1000) / 2;
}

/** 光帆/束能：给定激光功率与帆+载荷质量，得到的加速度（m/s²）。 */
export function sailAccel(laserW: number, massKg: number): number {
  return laserW / 3e8 / massKg; // 完全反射：F = 2P/c，这里按 1 个光子动量计保守值
}

/** 星际航时（年）：距离（ly）与巡航速度（km/s），忽略加速段。 */
export function interstellarYears(ly: number, vKms: number): number {
  const kmPerLy = 9.4607e12;
  return (ly * kmPerLy) / vKms / 3.15576e7;
}
