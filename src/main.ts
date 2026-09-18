import { World } from './scene/world';
import { buildUI } from './ui/panel';
import { buildNavPanel } from './ui/nav-panel';
import { buildNavConsole } from './ui/nav-console';
import { Tour } from './ui/tour';
import { initMusic } from './ui/music';

const canvas = document.getElementById('scene') as HTMLCanvasElement;
const world = new World(canvas);

// Build the panel first (it sets #app innerHTML); then the navigation console
// and the Tour overlay append into that DOM. The tour mutates world state
// directly, so `sync` refreshes the panel controls to match when it exits.
let tour: Tour;
const sync = buildUI(world, () => tour.restart());
const syncNav = buildNavPanel(world);
const syncConsole = buildNavConsole(world);
tour = new Tour(world, () => sync());

// Begin in the guided walkthrough (honoring any #step deep link in the URL).
tour.start();

// Once the scene has rendered its first frame, drop the preloader and — only
// then, when nothing else is competing for the network — start loading the
// background-music track.
let booted = false;
function boot(): void {
  if (booted) return;
  booted = true;
  const pre = document.getElementById('preloader');
  if (pre) {
    pre.classList.add('hidden');
    setTimeout(() => pre.remove(), 600);
  }
  const idle: (cb: () => void) => void =
    (window as Window & { requestIdleCallback?: (cb: () => void) => void }).requestIdleCallback ??
    ((cb) => window.setTimeout(cb, 300));
  idle(() => initMusic());
}

let last = performance.now();
function loop(now: number): void {
  const dt = Math.min((now - last) / 1000, 0.1); // clamp big gaps (tab switch)
  last = now;
  try {
    world.update(dt);
    syncNav();
    syncConsole();
  } catch (err) {
    // Never let one bad frame kill the whole app (rAF would stop for good).
    console.error('[loop]', err);
  }
  if (!booted) boot();
  requestAnimationFrame(loop);
}
requestAnimationFrame(loop);
