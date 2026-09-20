import type { Ephemeris } from './ephemeris';
import { lambert } from './lambert';
import { Vector3 } from 'three';
import { portBurn, spaceportFor, type PortBurn, type Spaceport } from './spaceport';
import { fromKms, MU_SUN, toKms } from './units';

/** 港口顺行圆轨道速度方向（黄道面法线 × 港口方向）。 */
function portTangent(b: PortBurn): Vector3 {
  return new Vector3(0, 0, 1).cross(b.portDir).normalize();
}

// High-level mission planning: pick the cheapest direct transfer between two
// bodies over a rolling departure horizon. This is what the navigation UI
// calls when the user asks "when should I leave, and how much delta-v?".
//
// 端点不是行星中心，而是**同步轨道太空港**（spaceport.ts）。深空段仍然按
// 天体中心做 Lambert（SOI 半径 ~1e-4 AU，对日心弧的影响可忽略），但真正记账
// 的是两项点火：
//   - 离港：从港口圆轨道点火逃逸，Δv = f(v∞, r_port, μ)——含平面变更；
//   - 入泊：双曲线接近后制动进入目标港口圆轨道。
// 这两项才是飞船要带的推进剂（v∞ 只是双曲线渐近速度，不是点火量）。
// 地表 ↔ 太空港由另一套运输系统承担，不计入飞船预算（见 Spaceport.surfaceAccessKms）。

export interface TransferPlan {
  departureId: string;
  targetId: string;
  /** Departure epoch, days since J2000. */
  departureDay: number;
  /** Time of flight, days. */
  tof: number;
  arrivalDay: number;
  /** 双曲剩余速度 v∞（km/s）：离港/到港的双曲线渐近速度，不是点火量。 */
  vinfDepartKms: number;
  vinfArriveKms: number;
  /** 真实点火量（km/s）：离港逃逸 + 入泊捕获（含平面变更）。 */
  dvDepart: number;
  dvArrive: number;
  dvTotal: number;
  /** 出发/抵达太空港（无轨道数据时为 null，退回中心到中心口径）。 */
  departPort: Spaceport | null;
  arrivePort: Spaceport | null;
  /** 点火方案（null = 无港口或几何退化）。 */
  burnDepart: PortBurn | null;
  burnArrive: PortBurn | null;
  /** 飞船在出发港点火后的日心状态（AU, AU/day）——n-body 真值任务从这里开始。 */
  departureState: { pos: Vector3; vel: Vector3 } | null;
  /** 抵达时停靠在目标港的状态（日心 AO, AU/day）——显示/入泊用。 */
  arrivalState: { pos: Vector3; vel: Vector3 } | null;
  /** Heliocentric ecliptic states: 港口位置（有港口时）/ 天体中心。 */
  r1: Vector3;
  v1: Vector3;
  r2: Vector3;
  v2: Vector3;
}

export interface BestTransferOptions {
  departure: Ephemeris;
  target: Ephemeris;
  departureId?: string;
  targetId?: string;
  /** Current epoch, days since J2000 (the horizon starts here). */
  tNow: number;
  horizonDays?: number;
  departStep?: number;
  tofMin?: number;
  tofMax?: number;
  tofStep?: number;
  mu?: number;
}

/** Cheapest direct transfer found on the (departure x tof) grid, or null. */
export function bestTransfer(opts: BestTransferOptions): TransferPlan | null {
  const mu = opts.mu ?? MU_SUN;
  const horizon = opts.horizonDays ?? 1200;
  const dStep = opts.departStep ?? 5;
  const tofMin = opts.tofMin ?? 100;
  const tofMax = opts.tofMax ?? 420;
  const tofStep = opts.tofStep ?? 5;

  const departPort = opts.departureId ? spaceportFor(opts.departureId) : null;
  const arrivePort = opts.targetId ? spaceportFor(opts.targetId) : null;

  let best: TransferPlan | null = null;
  for (let dt = 0; dt <= horizon; dt += dStep) {
    const t = opts.tNow + dt;
    const s1 = opts.departure.stateAt(t);
    for (let tof = tofMin; tof <= tofMax; tof += tofStep) {
      const s2 = opts.target.stateAt(t + tof);
      try {
        // 一轮细化：先用中心到中心的 Lambert 定 v∞ 方向与港口相位，
        // 再用**港口位置**做最终 Lambert —— 端点不再是行星中心。
        // 港口离中心 ~1e-4 AU，一轮足够收敛。
        const { v1: v1c, v2: v2c } = lambert(s1.pos, s2.pos, tof, mu);
        const vi0 = v1c.clone().sub(s1.vel);
        const va0 = v2c.clone().sub(s2.vel);
        const b0Dep =
          departPort && toKms(vi0.length()) > 1e-9
            ? portBurn(toKms(vi0.length()), vi0, departPort, s1.pos, s1.vel)
            : null;
        const b0Arr =
          arrivePort && toKms(va0.length()) > 1e-9
            ? portBurn(toKms(va0.length()), va0, arrivePort, s2.pos, s2.vel)
            : null;
        const p1 = b0Dep ? b0Dep.portPos : s1.pos;
        const p2 = b0Arr ? b0Arr.portPos : s2.pos;

        const { v1, v2 } = lambert(p1, p2, tof, mu);
        const vInfDep = v1.clone().sub(s1.vel);
        const vInfArr = v2.clone().sub(s2.vel);
        const vinfDepartKms = toKms(vInfDep.length());
        const vinfArriveKms = toKms(vInfArr.length());
        const burnDepart =
          departPort && vinfDepartKms > 1e-9
            ? portBurn(vinfDepartKms, vInfDep, departPort, s1.pos, s1.vel)
            : null;
        const burnArrive =
          arrivePort && vinfArriveKms > 1e-9
            ? portBurn(vinfArriveKms, vInfArr, arrivePort, s2.pos, s2.vel)
            : null;
        const dvDepart = burnDepart ? burnDepart.dvKms : vinfDepartKms;
        const dvArrive = burnArrive ? burnArrive.dvKms : vinfArriveKms;
        const dvTotal = dvDepart + dvArrive;
        if (!best || dvTotal < best.dvTotal) {
          best = {
            departureId: opts.departureId ?? '',
            targetId: opts.targetId ?? '',
            departureDay: t,
            tof,
            arrivalDay: t + tof,
            vinfDepartKms,
            vinfArriveKms,
            dvDepart,
            dvArrive,
            dvTotal,
            departPort,
            arrivePort,
            burnDepart,
            burnArrive,
            departureState: burnDepart
              ? { pos: p1.clone(), vel: burnDepart.shipVel.clone() }
              : null,
            arrivalState: burnArrive
              ? { pos: p2.clone(), vel: s2.vel.clone().addScaledVector(portTangent(burnArrive), fromKms(burnArrive.vCircKms)) }
              : null,
            r1: p1.clone(),
            v1,
            r2: p2.clone(),
            v2,
          };
        }
      } catch {
        // Infeasible geometry for this cell (degenerate/collinear positions).
      }
    }
  }
  return best;
}
