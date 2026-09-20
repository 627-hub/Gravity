# CLAUDE.md

Guidance for AI assistants (and humans) working in this repo.

## What this is

**Gravity · Solar System Simulator** — an interactive, physically grounded model
of the solar system (TypeScript + Three.js + Vite), used as a **flight model for
interplanetary navigation**: free flight over the live system, plus mission
planning (transfer windows, spaceport departure/arrival), onboard navigation
(tracking + square-root EKF), course corrections and a flight-deck console.
Everything is driven by real astronomical data; only the *scale* is faked (and
that's a user toggle).

**There is no guided tour and no teaching demos.** The app boots straight into
free flight over the live system. `world.ts` keeps only the renderer/engine
core (bodies, moons, orbits, labels, scale, physics, camera, time) plus the
navigation mission visuals. The old walkthrough machinery — `DemoMode`, the
particle/vector/Probe/Spacetime/Precession/Voyager slide code, the parallax
starfield and the teaching-vector overlays — has been **deleted**.

## Commands

```bash
npm install
npm run dev      # vite dev server (http://localhost:5173)
npm test         # vitest unit tests (src/nav navigation algorithms)
npm run build    # tsc --noEmit-style type-check + vite production build → dist/
npm run preview  # serve the production build locally
```

Always run `npm run build` (or `npx tsc --noEmit && npx vite build`) before
considering a change done — the build does a full type-check.

Deployment: pushing to `main` triggers `.github/workflows/deploy.yml`, which
builds and publishes `dist/` to GitHub Pages. The Vite `base` is `/Gravity/`
for production builds (see `vite.config.ts`); keep that in sync with the repo
name if it ever changes.

## Architecture

```
src/
  data/        physical constants, real planet/moon data (radii, masses, J2000 elements)
  physics/     kepler.ts (analytic two-body), state.ts (state vectors), nbody.ts (leapfrog)
  nav/         navigation core (AU / AU-per-day, matching physics/):
               units.ts (km/s conversions), propagate.ts (universal-variable two-body),
               lambert.ts (Izzo solver), maneuvers.ts (Hohmann, bi-elliptic, dV budgets),
               ephemeris.ts (body-state adapters), windows.ts (porkchop scans),
               perturbations.ts (multi-body force model + SRP), sources.ts (solar-system
               gravity sources, SOI radii), integrate.ts (adaptive DOPRI5),
               analysis.ts (closest approach, impact events),
               flyby.ts (B-plane gravity assists), targeting.ts (shoot a flyby to hit a target),
               plan.ts (bestTransfer: cheapest window; endpoints are spaceports),
               propulsion.ts (drive table: exhaust velocity -> mass ratio, burn
               duration, impulsive vs low-thrust regime, jet power, sail/interstellar),
               spaceport.ts (synchronous-orbit spaceports: r = (mu T^2/4pi^2)^(1/3),
               parking-orbit fallback for slow/retrograde/locked bodies, and the
               single-impulse escape/capture burn model with plane change),
               mission.ts (flown mission: dispersion, TCM re-solves, flown path),
               navigator.ts (L1 tracking: square-root EKF on range/range-rate + optical),
               truth.ts (n-body truth trajectory: full force field + SRP, Hermite sampling),
               attitude.ts (ADCS: PD pointing, gyro ARW + star tracker, quaternion math),
               elements.ts (state → classical orbital elements)
  scene/
    scale.ts     real vs visual scale models
    textures.ts  procedural canvas surface textures (offline; Earth uses a real image)
    world.ts     the Three.js engine — scene, bodies, moons, orbits, camera,
                 time/scale/physics, and the navigation-mission visuals
    nav-viz.ts   spacecraft model + transfer arc rendering (NavViz, CRAFT_ID)
  ui/
    panel.ts     main control panel (scale, physics, focus, time, toggles)
    nav-panel.ts mission controls (pick from/to, scan, launch, tracking tier, TCM);
    nav-console.ts flight-deck console: attitude/pointing indicator (ADI), instrument
                 readouts (accelerometer, ranging, optical nav), orbit + mini-map
  main.ts        wiring + animation loop
public/          static assets served at root (e.g. earth_daymap.jpg)
```

### `world.ts` layout

`world.ts` is the central file: `buildBodies`/`buildMoons` create the render
objects (mesh, orbit line, label) and `update()` advances the clock, positions
everything, eases the camera, and drives the mission visuals via
`updateMission()`. Camera moves go through an eased `flyTo`; `followBody` keeps
a moving subject framed while preserving the user's own orbit/zoom pose.

## Conventions

- Match the surrounding code's style: terse, purposeful comments that explain
  *why*, real units in the physics layer, scene units in the renderer.
- Coordinate frame: physics is heliocentric ecliptic AU; `eclToScene(v)` maps
  ecliptic → render space as `(v.x, v.z, -v.y)`.
- Keep things working offline — procedural textures, bundled assets.
- Verify visual changes in the browser when possible; the slides are tuned by
  eye.

## Credits

Earth texture: Solar System Scope (CC BY 4.0). Built by
[qunabu](https://github.com/qunabu/Gravity).
