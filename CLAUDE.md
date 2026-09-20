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

**There is no guided tour.** The app boots straight into free flight. The
teaching demos that the tour used to drive (`inertia`, `accretion`, `helix`,
`orbit-intro`, `rocket`, `soi`, `flyby`, `spacetime`, `precession` — the
`DemoMode` branches in `world.ts`) are still in the code but have **no UI entry
point** any more; they are candidates for deletion.

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
    world.ts     the Three.js engine — scene, bodies, orbits, vectors, and every demo
    nav-viz.ts   spacecraft model + transfer arc rendering (NavViz, CRAFT_ID)
  ui/
    panel.ts     main control panel (scale, physics, focus, time, toggles)
    nav-panel.ts mission controls (pick from/to, scan, launch, tracking tier, TCM);
    nav-console.ts flight-deck console: attitude/pointing indicator (ADI), instrument
                 readouts (accelerometer, ranging, optical nav), orbit + mini-map
  main.ts        wiring + animation loop
public/          static assets served at root (e.g. earth_daymap.jpg)
```

### How demos work (`src/scene/world.ts`)

`world.ts` is the large, central file. A `DemoMode` string selects special
behavior in the per-frame `update()` loop. Each mode has a `startX()` method
(sets camera + state) and a branch in the update loop / `updateAstro()`. Only
`normal` is reachable from the UI now — the teaching modes (`inertia`,
`accretion`, `helix`, `orbit-intro`, `rocket`, `soi`, `flyby`, `spacetime`,
`precession`) are legacy and un-driven. Camera moves via an eased `flyTo`.

### Mission endpooints: spaceports, not body centres

Missions launch from and arrive at a **spaceport in synchronous orbit** (for
bodies where one exists — fast prograde rotators: Earth 42,164 km, Mars 20,428 km,
Jupiter ~160,000 km) and fall back to a low **parking orbit** otherwise (Venus
spins retrograde in 243 days; tidally-locked moons have their synchronous radius
outside the Hill sphere). The Δv ledger is therefore: escape burn (port orbit →
hyperbolic departure) + TCMs + capture burn (hyperbolic arrival → port orbit).
Surface-to-port traffic is a **different vehicle** (aerodynamics, thermal, high
thrust) and is deliberately not modelled — `Spaceport.surfaceAccessKms` gives the
ideal-impulse reference only.

The n-body truth integrates *with* the departure/arrival bodies in the force
model (per-source softening = the body's radius): the escape and capture
hyperbolas are real orbits about them.

The onboard force model is **Sun + departure/target bodies + SRP** (the SRP
coefficient carries a deliberate ~10% a priori error, so a real residual
remains). It drives the estimator's propagation, the predicted arc, and the
terminal targeting — a differential corrector that aims at the port with the
target's gravity in the loop (`Mission.aimAtPort`, numerically Jacobian'd,
Newton with a step cap and a two-body Lambert fallback). Navigation quality is
therefore set by the *tracking tier*, not by a model floor: with high-tier
tracking a terminal TCM lands within ~300 km of the port, ~10^4 km on the
lowest tier.

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
