# Gravity · Solar System Simulator

An interactive, physically grounded model of the solar system that demonstrates
how gravity shapes orbits — built with **TypeScript + Three.js + Vite**.

It is a **flight model for interplanetary navigation**: free flight over the
live solar system, plus mission planning (transfer windows, synchronous-orbit
spaceports), onboard navigation (tracking, square-root EKF, course
corrections) and a flight-deck console. Everything is driven by real
astronomical data; the only thing the renderer ever fakes is the *scale* (and
that is a toggle you control).

## Visuals

- **SpaceX-style UI** — pure black, hairline borders with HUD corner ticks,
  thin wide-tracked uppercase labels, white-fill active toggles, and monospace
  telemetry numerals (Inter + Roboto Mono).
- **Procedural textures** — every Sun/planet/moon surface is generated on a
  canvas from 3D value-noise (`src/scene/textures.ts`): solar granulation,
  banded gas giants with a Great Red Spot, a cloudy blue Earth, cratered rocky
  moons, and a radial Saturn ring with the Cassini gap. No image files, so it
  works fully offline.
- **Smooth transitions** — camera moves ease to their target (cancelled the
  instant you grab the controls) and bodies/orbits fade in and out rather than
  snapping.

## Run it

```bash
npm install
npm run dev      # opens http://localhost:5173
npm run build    # type-check + production bundle into dist/
```

## 星际航行 (interplanetary navigation)

Pick a departure and a target body and the planner scans a rolling window for
the cheapest transfer. Missions leave from — and arrive at — a **spaceport in
synchronous orbit** around the body (Earth's port sits at 42,164 km, one day
period), not from the planet's centre; the Δv ledger is the escape burn plus
course corrections plus the capture burn. Surface-to-port traffic is a separate
vehicle and is deliberately out of scope.

While the craft flies, the flight-deck console shows the onboard state: the
attitude/pointing indicator, the instrument readouts (accelerometer, ground
ranging, onboard optical navigation), the osculating orbit and a heliocentric
mini-map. The onboard computer carries a square-root EKF over range/range-rate
tracking; **trajectory correction manoeuvres are computed from that estimate,
not from truth**, so their quality follows the tracking tier. The truth
trajectory is integrated in the full force field (Sun + planets + moons + solar
radiation pressure).

## What's real

- **Sizes** — every body uses its real mean radius (Sun 696 340 km → Pluto
  1 188 km) and mass.
- **Orbits** — real J2000.0 heliocentric Keplerian elements (semi-major axis,
  eccentricity, inclination, node, perihelion, mean longitude) from the
  JPL/IAU approximate-element tables. Kepler's equation is solved per frame.
- **Dates** — the clock is real: T=0 is the J2000 epoch (2000-01-01 12:00).

## The two hard problems, and how they're handled

**Scale.** At true scale the Sun is 0.00465 AU across while Neptune orbits at
30 AU — you cannot show real sizes *and* real distances and see anything. So
there are two interchangeable scale models behind one interface
(`src/scene/scale.ts`):

- **True scale** — sizes and distances share one linear factor. Accurate, but
  planets become the specks they really are; zoom in to find them.
- **Visual scale** — a monotonic radial remap pulls the outer planets inward and
  a logarithmic size map keeps both the Sun and tiny Mercury visible at once.
  Physics still runs in true AU; only rendering is remapped.

**Physics.** Two toggleable models, seeded from identical real initial
conditions:

- **Keplerian** (`src/physics/kepler.ts`) — analytic two-body positions from the
  orbital elements. Exact and perfectly stable.
- **N-body** (`src/physics/nbody.ts`) — direct all-pairs Newtonian gravity
  integrated with a symplectic kick–drift–kick **leapfrog** scheme. This is
  gravity *simulated* rather than prescribed. The info panel shows live energy
  drift (typically ~10⁻⁶ %), which is how you know the integrator is honest.

## Moons

Every planet carries its major moon(s) with real radii, masses, and orbital
parameters. They always render on accurate Keplerian paths around their planet
(visually exaggerated in Visual mode so they're not sub-pixel). The **Moons**
toggle controls them — it's flagged "heavier" because turning it on in N-body
mode also feeds the dynamically significant moons into the integrator, which
needs a finer timestep (the short-period moons drive it).

Only moons massive enough to actually perturb their planet are simulated
gravitationally (the Moon ≈ 1.2 % of Earth; **Charon ≈ 12 % of Pluto**, a true
binary; the Galileans; Titan; Triton). Negligible moons like Phobos (~10⁻⁸ of
Mars) are render-only — including them would burn compute for no visible effect.

## Project layout

```
src/
  data/
    constants.ts   physical constants (G, AU, masses, …)
    bodies.ts      real planet + moon data (radii, masses, J2000 elements)
    system.ts      builds the N-body body set; moon-relative Kepler helpers
  physics/
    kepler.ts      Kepler's-equation solver + analytic positions
    state.ts       position+velocity state vectors from elements
    nbody.ts       leapfrog N-body integrator (SI internally)
  scene/
    scale.ts       real vs visual scale models
    textures.ts    procedural canvas surface textures (offline, no images)
    world.ts       Three.js scene, bodies, orbits, vectors, accretion, 2D↔3D
  ui/
    panel.ts       control + info panels
    tour.ts        the guided walkthrough (steps + deep links)
  main.ts          wiring + animation loop
```

## Caveats

- Moon J2000 node/argument/phase are approximate (real values are messy and
  precess); semi-major axis, eccentricity, inclination, and period are real.
- Visual scale is a non-linear radial remap, so in that mode orbit ellipses are
  *near*-ellipses by construction. Switch to True scale for exact geometry.
- At very high time-multipliers in N-body the substep count is capped, so the
  fastest moons lose accuracy gracefully rather than freezing the tab.
