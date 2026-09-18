// @vitest-environment happy-dom
import { describe, expect, it, vi } from 'vitest';
import { buildNavPanel } from '../ui/nav-panel';
import { buildUI } from '../ui/panel';
import type { World } from '../scene/world';

// DOM-level test for the navigation panel wiring: build the real control panel
// into a fake document, then drive it with a stubbed World and check that the
// scan/launch flow actually responds to clicks.

function stubWorld(): World {
  const state = {
    scaleMode: 'visual',
    physics: 'kepler',
    twoD: 0,
    showOrbits: true,
    showProjection: false,
    showLabels: true,
    showMoons: false,
    paused: false,
    daysPerSecond: 1,
  };
  const stub = {
    state,
    simDays: 0,
    missionStatus: () => null,
    applyTcm: () => null,
    launchMission: vi.fn(),
    clearMission: vi.fn(),
    followCraft: vi.fn(),
    setScaleMode: vi.fn(),
    setPhysics: vi.fn(),
    setTwoD: vi.fn(),
    setShowMoons: vi.fn(),
    focusOn: vi.fn(),
    stopFollow: vi.fn(),
  };
  return stub as unknown as World;
}

function buildDom(): void {
  document.body.innerHTML = `
    <canvas id="scene"></canvas>
    <div id="app"></div>
    <div id="preloader"></div>
  `;
}

describe('nav panel wiring (DOM)', () => {
  it('scan updates the hint and enables launch; launch calls the world', async () => {
    buildDom();
    const world = stubWorld();
    buildUI(world, () => {});
    buildNavPanel(world);

    const scan = document.querySelector('#navScan') as HTMLButtonElement;
    const launch = document.querySelector('#navLaunch') as HTMLButtonElement;
    const hint = document.querySelector('#navHint') as HTMLElement;
    expect(scan).toBeTruthy();
    expect(launch).toBeTruthy();
    expect(launch.disabled).toBe(true);
    const before = hint.textContent;

    scan.click();
    expect(scan.disabled).toBe(true); // feedback while scanning
    expect(scan.textContent).toBe('扫描中…'); // synchronous click echo
    expect(hint.textContent).toBe('扫描转移窗口…');
    // The scan runs in a setTimeout; wait for it to finish.
    await vi.waitFor(() => expect(scan.disabled).toBe(false), { timeout: 5000 });

    expect(hint.textContent).not.toBe(before);
    expect(hint.textContent).toContain('地球');
    expect(hint.textContent).toContain('火星');
    expect(launch.disabled).toBe(false);
    expect(scan.textContent).toBe('扫描转移窗口'); // restored after the scan

    launch.click();
    expect(world.launchMission).toHaveBeenCalledTimes(1);
    expect(world.followCraft).toHaveBeenCalledTimes(1);
  });

  it('with an active mission, a new scan can still (re)launch', async () => {
    buildDom();
    const world = stubWorld();
    const status = {
      departureId: 'earth', targetId: 'mars', phase: 'cruise' as const,
      departureDay: 0, arrivalDay: 300, tof: 300,
      dvDepart: 3, dvArrive: 2.5, dvTotal: 5.5,
      daysToDeparture: 0, daysToArrival: 120,
      missKm: 1e5, estMissKm: 1.2e5, tcmDvKms: 0.05, tcmCount: 0, tcmUsedKms: 0,
      trackingLabel: null, trackCount: 0, estErrorKm: 0, posSigmaKm: 0,
    };
    (world as unknown as { missionStatus: () => unknown }).missionStatus = () => status;
    buildUI(world, () => {});
    const sync = buildNavPanel(world);

    sync(); // mission active: launch must not be force-disabled beyond "no plan"
    const scan = document.querySelector('#navScan') as HTMLButtonElement;
    const launch = document.querySelector('#navLaunch') as HTMLButtonElement;
    scan.click();
    await vi.waitFor(() => expect(scan.disabled).toBe(false), { timeout: 5000 });
    expect(launch.disabled).toBe(false);
    launch.click();
    expect(world.launchMission).toHaveBeenCalledTimes(1);
  });

  it('immediate launch skips the wait and flies at once', async () => {
    buildDom();
    const world = stubWorld();
    buildUI(world, () => {});
    buildNavPanel(world);

    const now = document.querySelector('#navNow') as HTMLButtonElement;
    now.click();
    await vi.waitFor(() => expect(world.launchMission).toHaveBeenCalled(), { timeout: 5000 });
    const plan = (world.launchMission as unknown as { mock: { calls: unknown[][] } }).mock.calls[0][0] as { departureDay: number };
    expect(plan.departureDay).toBe(0); // departs now
  });
});
