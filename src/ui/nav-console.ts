import { PLANETS } from '../data/bodies';
import { ISP_PRESETS, propellantFraction } from '../nav/maneuvers';
import type { Spaceport } from '../nav/spaceport';
import type { ManualBurnDir, NavSnapshot, PointingReading, World } from '../scene/world';

// Spacecraft flight-deck console: an attitude/pointing indicator (where the
// Sun, Earth, the target and the gravity vector sit relative to the nose),
// instrument readouts (accelerometer, ground ranging, onboard optical
// navigation), the osculating orbit and the delta-v ledger. The mini-map at
// the bottom keeps the heliocentric picture.

const ADI_W = 264;
const ADI_H = 158;
const MAP_W = 240;
const MAP_H = 136;

const phaseLabel = (phase: NavSnapshot['phase']): string =>
  phase === 'docked' ? '港内待发' : phase === 'cruise' ? '巡航中' : '已入泊';

const fmtDate = (days: number): string =>
  new Date(Date.UTC(2000, 0, 1, 12) + days * 86400000).toISOString().slice(0, 10);

const fmtKm = (km: number): string => {
  const v = Math.abs(km);
  if (v >= 1e8) return `${(km / 1e8).toFixed(2)} 亿 km`;
  if (v >= 1e4) return `${(km / 1e4).toFixed(1)} 万 km`;
  return `${Math.round(km)} km`;
};

const bodyName = (id: string): string => PLANETS.find((p) => p.id === id)?.name ?? id;
const signed = (x: number, digits = 1): string => `${x >= 0 ? '+' : ''}${x.toFixed(digits)}`;

export function buildNavConsole(world: World): () => void {
  const app = document.getElementById('app') ?? document.body;
  const root = document.createElement('section');
  root.id = 'navConsole';
  root.className = 'panel nav-console';
  root.style.display = 'none';
  root.innerHTML = `
    <div class="nc-head">飞船驾驶导航台<span class="nc-phase" id="ncPhase"></span></div>
    <canvas id="ncAdi" width="${ADI_W}" height="${ADI_H}"></canvas>
    <div class="nc-legend">
      <span><i class="vel"></i>速度轴</span>
      <span><i class="sun"></i>太阳</span>
      <span><i class="earth"></i>地球</span>
      <span><i class="tgt"></i>目标</span>
      <span><i class="grav"></i>引力</span>
    </div>
    <div class="nc-sec">姿态 · 指向</div>
    <div class="nc-row"><span>航向 / 俯仰（黄道系）</span><b id="ncYaw"></b></div>
    <div class="nc-row"><span>转向率（需保持）</span><b id="ncGyro"></b></div>
    <div class="nc-row"><span>指向误差（PD）</span><b id="ncPoint"></b></div>
    <div class="nc-row"><span>定姿误差（星敏）</span><b id="ncAttEst"></b></div>
    <div class="nc-row"><span>陀螺 / 星敏</span><b id="ncSensors"></b></div>
    <div class="nc-row"><span>太阳偏角 / 入射面板</span><b id="ncSun"></b></div>
    <div class="nc-row"><span>目标偏角 / 地球偏角</span><b id="ncBear"></b></div>
    <div class="nc-sec">传感器 · 测量</div>
    <div class="nc-row"><span>加速计（非引力）</span><b id="ncAccN"></b></div>
    <div class="nc-row"><span>引力加速度</span><b id="ncAccG"></b></div>
    <div class="nc-row"><span>地面测距</span><b id="ncRange"></b></div>
    <div class="nc-row"><span>测距率（多普勒）</span><b id="ncRate"></b></div>
    <div class="nc-row"><span>光学导航（目标）</span><b id="ncOpt"></b></div>
    <div class="nc-sec">位置 · 速度 · 轨道</div>
    <div class="nc-row"><span>日心距离</span><b id="ncR"></b></div>
    <div class="nc-row"><span>日心速度</span><b id="ncV"></b></div>
    <div class="nc-row"><span>太阳逃逸速度比</span><b id="ncVesc"></b></div>
    <div class="nc-row"><span>轨道 a / e / i</span><b id="ncEl"></b></div>
    <div class="nc-row"><span>周期 / 飞行路径角</span><b id="ncPer"></b></div>
    <div class="nc-row"><span>目标距离 / 接近速度</span><b id="ncTgt"></b></div>
    <div class="nc-sec">导航（L1 定轨）</div>
    <div class="nc-row"><span>估计误差 / σ</span><b id="ncErr"></b></div>
    <div class="nc-row"><span>跟踪</span><b id="ncTrk"></b></div>
    <div class="nc-row"><span>真实漏距 / 船载预期</span><b id="ncMiss"></b></div>
    <div class="nc-sec">太空港 · Δv 预算</div>
    <div class="nc-row"><span>出发港</span><b id="ncPortA"></b></div>
    <div class="nc-row"><span>目标港</span><b id="ncPortB"></b></div>
    <div class="nc-row"><span>离港逃逸 + 入泊捕获</span><b id="ncDvPlan"></b></div>
    <div class="nc-row"><span>修正已用</span><b id="ncDvUsed"></b></div>
    <div class="nc-row"><span>地表↔港（另案系统）</span><b id="ncSurf"></b></div>
    <div class="nc-row"><span>燃料（单级）</span><b id="ncFuel"></b></div>
    <div class="nc-row"><span>手动点火</span><b id="ncManual"></b></div>
    <div class="row nc-burn">
      <select id="ncBurnDir">
        <option value="prograde">顺行 +v</option>
        <option value="retrograde">逆行 −v</option>
        <option value="radialOut">径向外</option>
        <option value="radialIn">径向内</option>
        <option value="normal">轨道法向</option>
        <option value="antiNormal">反法向</option>
      </select>
      <input type="number" id="ncBurnDv" value="50" min="0.1" max="5000" step="10" title="Δv (m/s)" />
      <button id="ncBurnGo">点火</button>
    </div>
    <div class="nc-sec">日心航线</div>
    <canvas id="ncMap" width="${MAP_W}" height="${MAP_H}"></canvas>
    <div class="nc-legend">
      <span><i class="plan"></i>计划</span>
      <span><i class="pred"></i>船载预测</span>
      <span><i class="flown"></i>实飞</span>
    </div>
    <div class="nc-route" id="ncRoute"></div>
    <div class="nc-bar"><span id="ncBar"></span></div>
    <div class="nc-meta" id="ncMeta"></div>
    <button id="ncFollow" class="nc-follow">🎯 跟随飞船</button>
    <div class="nc-note">传感为模拟读数：地面站测距/测距率、船载相机测目标方位；姿态由 ADCS 物理模型给出（PD 指向 + 陀螺/星敏）。</div>
  `;
  app.appendChild(root);

  const setupCanvas = (id: string, w: number, h: number): CanvasRenderingContext2D | null => {
    const c = root.querySelector(`#${id}`) as HTMLCanvasElement;
    const ctx = c.getContext('2d');
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    c.width = w * dpr;
    c.height = h * dpr;
    c.style.width = `${w}px`;
    c.style.height = `${h}px`;
    if (ctx) ctx.scale(dpr, dpr);
    return ctx;
  };
  const adi = setupCanvas('ncAdi', ADI_W, ADI_H);
  const map = setupCanvas('ncMap', MAP_W, MAP_H);
  const bar = root.querySelector('#ncBar') as HTMLElement;

  (root.querySelector('#ncFollow') as HTMLButtonElement).addEventListener('click', () => {
    world.followCraft();
  });

  // Manual burn: fly the ship by hand (Δv along a chosen body-frame direction).
  const burnDir = root.querySelector('#ncBurnDir') as HTMLSelectElement;
  const burnDv = root.querySelector('#ncBurnDv') as HTMLInputElement;
  const burnGo = root.querySelector('#ncBurnGo') as HTMLButtonElement;
  burnGo.addEventListener('click', () => {
    const mps = Number(burnDv.value);
    if (!Number.isFinite(mps) || mps <= 0) return;
    const applied = world.manualBurn(mps / 1000, burnDir.value as ManualBurnDir);
    burnGo.textContent = applied === null ? '无效' : `已点火 ${mps.toFixed(0)} m/s`;
    setTimeout(() => { burnGo.textContent = '点火'; }, 1200);
  });

  const set = (id: string, text: string): void => {
    const el = root.querySelector(`#${id}`) as HTMLElement | null;
    if (el && el.textContent !== text) el.textContent = text;
  };

  // ---- attitude / pointing indicator -------------------------------------
  const drawAdi = (snap: NavSnapshot): void => {
    if (!adi) return;
    adi.clearRect(0, 0, ADI_W, ADI_H);
    const cx = ADI_W / 2;
    const cy = ADI_H / 2 + 4;
    const R = 62;

    // Off-nose rings (30/60/90 degrees) and the nose crosshair.
    adi.strokeStyle = 'rgba(255,255,255,0.12)';
    adi.lineWidth = 1;
    for (const deg of [30, 60, 90]) {
      adi.beginPath();
      adi.arc(cx, cy, (R * deg) / 90, 0, Math.PI * 2);
      adi.stroke();
    }
    adi.strokeStyle = 'rgba(255,255,255,0.18)';
    adi.beginPath();
    adi.moveTo(cx - R, cy);
    adi.lineTo(cx + R, cy);
    adi.moveTo(cx, cy - R);
    adi.lineTo(cx, cy + R);
    adi.stroke();

    const polar = (p: PointingReading): [number, number] => {
      const r = (R * Math.min(p.offDeg, 90)) / 90;
      const psi = (p.posDeg * Math.PI) / 180;
      return [cx + r * Math.cos(psi), cy - r * Math.sin(psi)];
    };
    const marker = (
      p: PointingReading,
      color: string,
      label: string,
    ): void => {
      const [x, y] = polar(p);
      const behind = p.offDeg > 90;
      adi.strokeStyle = color;
      adi.fillStyle = color;
      adi.lineWidth = 1.2;
      adi.beginPath();
      adi.arc(x, y, behind ? 3.6 : 3, 0, Math.PI * 2);
      if (behind) adi.stroke();
      else adi.fill();
      adi.globalAlpha = 0.85;
      adi.font = '9px -apple-system, "PingFang SC", sans-serif';
      adi.fillText(label, x + 6, y + 3);
      adi.globalAlpha = 1;
    };

    marker(snap.attitude.sun, '#ffd75e', '日');
    marker(snap.attitude.earth, '#6aa9ff', '地');
    marker(snap.attitude.target, '#ff9a6b', '标');
    if (snap.accel.gravity) marker(snap.accel.gravity, '#ff6b6b', '引');

    // Velocity axis sits at the centre (nose is held prograde).
    adi.fillStyle = '#7af0a6';
    adi.beginPath();
    adi.arc(cx, cy, 3, 0, Math.PI * 2);
    adi.fill();
    adi.font = '9px -apple-system, "PingFang SC", sans-serif';
    adi.fillText('速', cx + 6, cy + 3);

    adi.fillStyle = 'rgba(255,255,255,0.35)';
    adi.font = '8.5px -apple-system, "PingFang SC", sans-serif';
    adi.fillText('船头 +', cx - 16, cy - R - 4);
  };

  // ---- heliocentric mini-map ---------------------------------------------
  const drawMap = (snap: NavSnapshot): void => {
    if (!map) return;
    map.clearRect(0, 0, MAP_W, MAP_H);
    const cx = MAP_W / 2;
    const cy = MAP_H / 2;
    let extent = Math.max(snap.shipAU.length(), snap.targetAU.length(), 0.3);
    const scan = (pts: { x: number; y: number }[]) => {
      for (let i = 0; i < pts.length; i += 4) {
        const r = Math.hypot(pts[i].x, pts[i].y);
        if (r > extent) extent = r;
      }
    };
    scan(snap.planArc);
    scan(snap.flown);
    scan(snap.predicted);
    extent *= 1.18;
    const s = (Math.min(MAP_W, MAP_H) / 2 - 5) / extent;
    const px = (p: { x: number; y: number }) => [cx + p.x * s, cy - p.y * s] as const;

    map.strokeStyle = 'rgba(255,255,255,0.07)';
    map.lineWidth = 1;
    map.beginPath();
    map.arc(cx, cy, s, 0, Math.PI * 2);
    map.stroke();

    const path = (pts: { x: number; y: number }[], color: string, width: number, alpha: number) => {
      if (pts.length < 2) return;
      map.globalAlpha = alpha;
      map.strokeStyle = color;
      map.lineWidth = width;
      map.beginPath();
      const [x0, y0] = px(pts[0]);
      map.moveTo(x0, y0);
      for (let i = 4; i < pts.length; i += 4) {
        const [x, y] = px(pts[i]);
        map.lineTo(x, y);
      }
      const [xl, yl] = px(pts[pts.length - 1]);
      map.lineTo(xl, yl);
      map.stroke();
      map.globalAlpha = 1;
    };
    path(snap.planArc, '#6fe3ff', 1, 0.4);
    path(snap.predicted, '#ffb454', 1.1, 0.95);
    path(snap.flown, '#ffffff', 1.1, 0.9);

    const dot = (p: { x: number; y: number }, r: number, color: string) => {
      const [x, y] = px(p);
      map.fillStyle = color;
      map.beginPath();
      map.arc(x, y, r, 0, Math.PI * 2);
      map.fill();
    };
    if (snap.planArc.length) {
      dot(snap.planArc[0], 2, '#9ad9ff');
      dot(snap.planArc[snap.planArc.length - 1], 2, '#ffd27a');
    }
    dot(snap.targetAU, 2.6, '#ff9a6b');
    dot({ x: 0, y: 0 }, 2.4, '#ffcf66');
    const [sx, sy] = px(snap.shipAU);
    map.strokeStyle = '#ffffff';
    map.lineWidth = 1;
    map.beginPath();
    map.arc(sx, sy, 4, 0, Math.PI * 2);
    map.stroke();
    dot(snap.shipAU, 2, '#ffffff');
  };

  let lastDrawDay = NaN;
  let visible = false;

  return () => {
    const snap = world.navSnapshot();
    if (!snap) {
      if (visible) {
        root.style.display = 'none';
        visible = false;
        lastDrawDay = NaN;
      }
      return;
    }
    if (!visible) {
      root.style.display = 'block';
      visible = true;
    }
    const st = snap.status;
    const att = snap.attitude;
    const m = snap.measurement;
    const hasTrack = !!st.trackingLabel;

    set('ncPhase', phaseLabel(snap.phase));
    set('ncRoute', `${bodyName(snap.departureId)} → ${bodyName(snap.targetId)}`);

    set('ncYaw', `${att.yawDeg.toFixed(1)}° / ${signed(att.pitchDeg)}°`);
    set(
      'ncGyro',
      snap.accel.gravity ? `${att.gyroDegPerDay.toFixed(2)} °/天` : '—（停靠中）',
    );
    const adcs = snap.adcs;
    set('ncPoint', adcs ? `${adcs.pointingArcsec.toFixed(1)}″ · 角速率 ${adcs.rateDegS.toExponential(1)} °/s` : '—（巡航后激活）');
    set('ncAttEst', adcs ? `${adcs.estimateArcsec.toFixed(1)}″（星敏 ${adcs.starSigmaArcsec}″）` : '—');
    set(
      'ncSensors',
      adcs
        ? `ARW ${adcs.gyroArwDegSqrtH} °/√h · 星敏更新 ${adcs.starUpdates} 次`
        : '—',
    );
    // Panel normal is the craft's up axis: incidence = 90° − sun elevation.
    set('ncSun', `${att.sun.offDeg.toFixed(1)}° / ${(90 - att.sun.elDeg).toFixed(1)}°`);
    set('ncBear', `${att.target.offDeg.toFixed(1)}° / ${att.earth.offDeg.toFixed(1)}°`);

    set('ncAccN', `${snap.accel.nonGravUg.toFixed(1)} μg`);
    set(
      'ncAccG',
      snap.accel.gravityMms2 !== null
        ? `${snap.accel.gravityMms2.toFixed(2)} mm/s²`
        : '—（停靠中）',
    );
    set(
      'ncRange',
      m && m.rangeKm !== null
        ? `${fmtKm(m.rangeKm)}${m.rangeSigmaKm ? ` ± ${fmtKm(m.rangeSigmaKm)}` : ''}`
        : '—',
    );
    set(
      'ncRate',
      m && m.rangeRateKms !== null
        ? `${m.rangeRateKms.toFixed(3)} km/s${m.rateSigmaKms ? ` ± ${(m.rateSigmaKms * 1000).toFixed(1)} m/s` : ''}`
        : '—',
    );
    set(
      'ncOpt',
      m && m.opticalOffDeg !== null
        ? `${m.opticalOffDeg.toFixed(1)}° 偏角${m.opticalSigmaArcsec ? ` ± ${m.opticalSigmaArcsec.toFixed(1)}″` : ''}`
        : '—',
    );

    set('ncR', `${snap.shipAU.length().toFixed(3)} AU`);
    set('ncV', `${snap.shipSpeedKms.toFixed(2)} km/s`);
    set(
      'ncVesc',
      `v/v_esc ${snap.escapeFraction.toFixed(2)}（逃逸 ${snap.escapeSpeedKms.toFixed(1)} km/s）`
        + (snap.escapeFraction >= 1 ? ' · 已超太阳逃逸' : ''),
    );
    set('ncEl', `${snap.orbit.a.toFixed(3)} / ${snap.orbit.e.toFixed(3)} / ${snap.orbit.iDeg.toFixed(2)}°`);
    set(
      'ncPer',
      `${snap.orbit.periodDays ? `${snap.orbit.periodDays.toFixed(1)} 天` : '双曲'} / ${signed(snap.orbit.flightPathAngleDeg)}°`,
    );
    set(
      'ncTgt',
      `${fmtKm(snap.targetDistKm)} / ${snap.closingKms >= 0 ? '接近 ' : '远离 '}${Math.abs(snap.closingKms).toFixed(2)} km/s`,
    );

    set('ncErr', hasTrack ? `${fmtKm(st.estErrorKm)} / ±${fmtKm(st.posSigmaKm)}` : '—（真值导航）');
    set('ncTrk', hasTrack ? `${st.trackCount} 次 · ${st.trackingLabel}` : '无');
    set('ncMiss', `${fmtKm(st.missKm)} / ${fmtKm(st.estMissKm)}`);

    const portInfo = (p: Spaceport | null, fallback: string): string =>
      p
        ? `${p.bodyName} · ${p.kind === 'synchronous' ? '同步轨道' : '停泊轨道'} ${Math.round(p.altitudeKm).toLocaleString()} km 高 · T=${p.periodDays.toFixed(2)} 天`
        : fallback;
    set('ncPortA', portInfo(st.departPort, '—（中心口径）'));
    set('ncPortB', portInfo(st.arrivePort, '—（中心口径）'));
    set(
      'ncDvPlan',
      `${st.dvDepart.toFixed(2)} + ${st.dvArrive.toFixed(2)} = ${st.dvTotal.toFixed(2)} km/s · v∞ ${st.vinfDepartKms.toFixed(2)}/${st.vinfArriveKms.toFixed(2)}`,
    );
    set(
      'ncSurf',
      st.departPort && st.arrivePort
        ? `${st.departPort.surfaceAccessKms.toFixed(1)} / ${st.arrivePort.surfaceAccessKms.toFixed(1)} km/s（理想脉冲，不含大气损失；由穿梭机/电梯承担）`
        : '—',
    );
    set('ncDvUsed', st.tcmCount ? `${st.tcmUsedKms.toFixed(3)} km/s · ${st.tcmCount} 次` : '未修正');
    set(
      'ncFuel',
      ISP_PRESETS.map(
        (p) => `${p.label} ${(propellantFraction(st.dvTotal, p.ispS) * 100).toFixed(0)}%`,
      ).join(' · ') + `（Δv ${st.dvTotal.toFixed(2)} km/s，不含结构质量）`,
    );
    set(
      'ncManual',
      st.manualCount
        ? `${st.manualUsedKms.toFixed(3)} km/s · ${st.manualCount} 次`
        : '—（驾驶台右侧可手动点火）',
    );
    burnGo.disabled = snap.phase === 'arrived';

    bar.style.width = `${(snap.progress * 100).toFixed(1)}%`;
    const tRel = Math.round(snap.simDays - snap.departureDay);
    set(
      'ncMeta',
      `${fmtDate(snap.simDays)} · T${tRel >= 0 ? '+' : ''}${tRel} 天 · ${
        snap.phase === 'arrived'
          ? '已抵达'
          : `剩 ${Math.max(0, Math.round(snap.arrivalDay - snap.simDays))} 天`
      }`,
    );

    if (!(Math.abs(snap.simDays - lastDrawDay) < 0.05)) {
      drawAdi(snap);
      drawMap(snap);
      lastDrawDay = snap.simDays;
    }
  };
}
