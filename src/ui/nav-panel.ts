import { PLANETS } from '../data/bodies';
import { bodyEphemeris } from '../nav/ephemeris';
import { TRACKING_TIERS, type TrackingTier } from '../nav/navigator';
import { bestTransfer, type TransferPlan } from '../nav/plan';
import type { World } from '../scene/world';

// Navigation console: pick a departure and a target planet, scan the cheapest
// Lambert transfer over a rolling horizon, then fly it. The scan is
// synchronous but heavy-ish (thousands of solver calls), so it runs after a
// paint tick via setTimeout.

const dateFmt = (days: number): string =>
  new Date(Date.UTC(2000, 0, 1, 12) + days * 86400000).toISOString().slice(0, 10);

export function buildNavPanel(world: World): () => void {
  const controls = document.getElementById('controls');
  const tourBtn = controls?.querySelector('#tourBtn');
  if (!controls || !tourBtn) return () => {};

  tourBtn.insertAdjacentHTML('beforebegin', `
    <div class="group" id="navGroup">
      <div class="glabel">星际航行导航 <span class="nav-build">r4</span></div>
      <div class="row">
        <select id="navFrom"></select>
        <select id="navTo"></select>
      </div>
      <div class="row">
        <button id="navScan">扫描转移窗口</button>
        <button id="navNow">立即出发</button>
      </div>
      <label class="chk" id="navErrRow"><input type="checkbox" id="navErr" checked> 模拟发射误差</label>
      <div class="row"><select id="navTrack">
        <option value="none">导航：真值（无测量噪声）</option>
        <option value="high">导航：高精度跟踪</option>
        <option value="medium" selected>导航：中精度跟踪</option>
        <option value="low">导航：低精度跟踪</option>
      </select></div>
      <div class="row">
        <button id="navLaunch" disabled>🚀 发射飞船</button>
        <button id="navClear" disabled>清除</button>
      </div>
      <div class="row"><button id="navTcm" disabled>🛠 修正航向（TCM）</button></div>
      <div class="hint" id="navHint">选择出发与目标行星，扫描最优转移窗口。</div>
      <div class="hint" id="navTcmHint"></div>
    </div>
  `);

  const fromSel = controls.querySelector('#navFrom') as HTMLSelectElement;
  const toSel = controls.querySelector('#navTo') as HTMLSelectElement;
  const scanBtn = controls.querySelector('#navScan') as HTMLButtonElement;
  const nowBtn = controls.querySelector('#navNow') as HTMLButtonElement;
  const launchBtn = controls.querySelector('#navLaunch') as HTMLButtonElement;
  const clearBtn = controls.querySelector('#navClear') as HTMLButtonElement;
  const tcmBtn = controls.querySelector('#navTcm') as HTMLButtonElement;
  const errCheck = controls.querySelector('#navErr') as HTMLInputElement;
  const trackSel = controls.querySelector('#navTrack') as HTMLSelectElement;
  const hint = controls.querySelector('#navHint') as HTMLElement;
  const tcmHint = controls.querySelector('#navTcmHint') as HTMLElement;

  for (const planet of PLANETS) {
    for (const sel of [fromSel, toSel]) {
      const opt = document.createElement('option');
      opt.value = planet.id;
      opt.textContent = planet.name;
      sel.appendChild(opt);
    }
  }
  fromSel.value = 'earth';
  toSel.value = 'mars';

  const setHint = (text: string): void => { hint.textContent = text; };
  const flashHint = (): void => {
    hint.classList.add('nav-flash');
    window.setTimeout(() => hint.classList.remove('nav-flash'), 1600);
  };

  let plan: TransferPlan | null = null;
  let planText = '';
  let lastPanel = '';

  const runScan = (immediate: boolean): void => {
    const from = PLANETS.find((p) => p.id === fromSel.value);
    const to = PLANETS.find((p) => p.id === toSel.value);
    if (!from || !to) return;
    if (from.id === to.id) {
      setHint('出发与目标不能相同。');
      return;
    }
    scanBtn.disabled = true;
    nowBtn.disabled = true;
    const idle = scanBtn.textContent ?? '扫描转移窗口';
    scanBtn.textContent = immediate ? '计算中…' : '扫描中…';
    setHint(immediate ? '计算立即出发的转移…' : '扫描转移窗口…');
    flashHint();
    window.setTimeout(() => {
      let t: TransferPlan | null = null;
      try {
        t = bestTransfer({
          departure: bodyEphemeris(from),
          target: bodyEphemeris(to),
          departureId: from.id,
          targetId: to.id,
          tNow: world.simDays,
          horizonDays: immediate ? 0 : 1400,
          departStep: 5,
          tofMin: 100,
          tofMax: 450,
          tofStep: 5,
        });
      } catch (err) {
        console.error('[nav] scan failed', err);
        setHint(`扫描失败：${err instanceof Error ? err.message : String(err)}`);
        flashHint();
        return;
      } finally {
        scanBtn.disabled = false;
        nowBtn.disabled = false;
        scanBtn.textContent = idle;
      }
      if (!t) {
        plan = null;
        planText = '';
        launchBtn.disabled = true;
        setHint('未找到可行转移 —— 换个目标或稍后再试。');
        return;
      }
      try {
        plan = t;
          planText = `${from.name} → ${to.name} · ${dateFmt(t.departureDay)} 出发 · 航程 ${Math.round(t.tof)} 天`
          + ` · ${dateFmt(t.arrivalDay)} 抵达 · Δv ${t.dvDepart.toFixed(2)} + ${t.dvArrive.toFixed(2)}`
          + ` = ${t.dvTotal.toFixed(2)} km/s`;
        launchBtn.disabled = false;
        const wait = t.departureDay - world.simDays;
        if (immediate) {
          // Fly right away: no waiting phase, the cockpit is live immediately.
          world.launchMission(t, { injectError: errCheck.checked, tracking: trackTier() });
          world.followCraft();
          launchBtn.disabled = true;
          setHint(planText);
          flashHint();
          return;
        }
        const replacing = world.missionStatus() !== null ? '（发射将替换当前任务）' : '';
        setHint(planText + (wait > 1 ? `（${Math.round(wait)} 天后出发，可调快时间）` : '') + replacing);
        flashHint();
      } catch (err) {
        console.error('[nav] plan failed', err);
        setHint(`规划显示失败：${err instanceof Error ? err.message : String(err)}`);
      }
    }, 30);
  };

  scanBtn.addEventListener('click', () => runScan(false));
  nowBtn.addEventListener('click', () => runScan(true));

  const trackTier = (): TrackingTier | null =>
    TRACKING_TIERS.find((t) => t.id === trackSel.value) ?? null;

  launchBtn.addEventListener('click', () => {
    if (!plan) return;
    world.launchMission(plan, { injectError: errCheck.checked, tracking: trackTier() });
    world.followCraft();
    launchBtn.disabled = true;
  });

  tcmBtn.addEventListener('click', () => {
    const dv = world.applyTcm();
    if (dv !== null) lastPanel = ''; // force a status refresh this frame
  });

  clearBtn.addEventListener('click', () => {
    world.clearMission();
    clearBtn.disabled = true;
    tcmBtn.disabled = true;
    tcmHint.textContent = '';
    lastPanel = '';
    if (plan) launchBtn.disabled = false;
    setHint(plan ? planText : '选择出发与目标行星，扫描最优转移窗口。');
  });

  // Per-frame status refresh (cheap: only writes when the text changes).
  return () => {
    const st = world.missionStatus();
    if (st) {
      const name = PLANETS.find((p) => p.id === st.targetId)?.name ?? st.targetId;
      const phase = st.phase === 'docked'
        ? `发射前待机 · ${Math.max(0, st.daysToDeparture).toFixed(0)} 天后出发`
        : st.phase === 'cruise'
          ? `巡航中 · ${Math.max(0, st.daysToArrival).toFixed(0)} 天后抵达`
          : '已抵达目标';
      const text = `任务 ${name}：${phase} · 全程 ${Math.round(st.tof)} 天 · Δv ${st.dvTotal.toFixed(2)} km/s`;

      const navLine = st.trackingLabel
        ? `导航（${st.trackingLabel}）：估计误差 ${fmtKm(st.estErrorKm)} · σ ±${fmtKm(st.posSigmaKm)} · 跟踪 ${st.trackCount} 次`
        : '真值导航（无测量噪声）';
      let tcmText = '';
      if (st.phase === 'cruise' && st.tcmDvKms !== null) {
        tcmText = `抵达偏差：真实 ${fmtKm(st.missKm)} / 船载估计 ${fmtKm(st.estMissKm)} · 修正需 Δv ${fmtDv(st.tcmDvKms)}`;
        tcmBtn.disabled = st.tcmDvKms < 0.0005; // below ~0.5 m/s: negligible
      } else if (st.phase === 'arrived') {
        tcmText = `抵达结算：修正 ${st.tcmCount} 次，累计 ${fmtDv(st.tcmUsedKms)} · 真实漏距 ${fmtKm(st.missKm)}`;
        tcmBtn.disabled = true;
      } else {
        tcmText = `预计抵达偏差 ${fmtKm(st.missKm)}（巡航后可按需修正）`;
        tcmBtn.disabled = true;
      }
      const panel = `${escapeHtml(text)}<br>${escapeHtml(tcmText)}<br>${escapeHtml(navLine)}`;
      if (panel !== lastPanel) {
        lastPanel = panel;
        tcmHint.innerHTML = panel;
      }

      clearBtn.disabled = false;
      launchBtn.disabled = !plan; // a scanned plan can always (re)launch
    } else {
      if (lastPanel !== '') {
        lastPanel = '';
        tcmHint.textContent = '';
      }
      tcmBtn.disabled = true;
    }
  };
}

function escapeHtml(text: string): string {
  return text.replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c] as string));
}

function fmtKm(km: number): string {
  if (km >= 1e4) return `${(km / 1e4).toFixed(1)} 万 km`;
  return `${Math.round(km)} km`;
}

function fmtDv(kms: number): string {
  if (kms < 0.01) return `${(kms * 1000).toFixed(1)} m/s`;
  return `${kms.toFixed(3)} km/s`;
}
