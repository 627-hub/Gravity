import { Vector3 } from 'three';
import { describe, expect, it } from 'vitest';
import { householderR, Navigator } from './navigator';

// Controlled checks of the square-root measurement update against closed-form
// Kalman results — a diagnosis tool for the filter internals.

const station = { pos: new Vector3(0, 0, 0), vel: new Vector3(0, 0, 0) };

describe('householderR QR', () => {
  it('preserves the Gram matrix: R^T R == B^T B', () => {
    let seed = 7;
    const rand = () => {
      seed = (seed * 1664525 + 1013904223) >>> 0;
      return seed / 2 ** 32 - 0.5;
    };
    const m = 7;
    const n = 6;
    const B = new Float64Array(m * n);
    for (let i = 0; i < m * n; i++) B[i] = rand() * (i % 3 === 0 ? 100 : 1); // scale mix
    const R = householderR(B, m, n);
    for (let i = 0; i < n; i++) {
      for (let j = 0; j < n; j++) {
        let btb = 0;
        let rtr = 0;
        for (let k = 0; k < m; k++) btb += B[k * n + i] * B[k * n + j];
        for (let k = 0; k < n; k++) rtr += R[k * n + i] * R[k * n + j];
        expect(Math.abs(btb - rtr)).toBeLessThan(1e-9 * Math.max(1, Math.abs(btb)));
      }
    }
  });
});

describe('square-root update, closed form', () => {
  it('range-only update: one axis collapses, others untouched', () => {
    const sp = 1e-3; // AU
    const sv = 1e-5; // AU/day
    const sr = 1e-6; // AU
    const nav = new Navigator(
      { pos: new Vector3(1, 0, 0), vel: new Vector3(0, 1e-2, 0) },
      0,
      { noise: { rangeSigma: sr, rangeRateSigma: 1e-9, opticalSigma: 1e-9 }, posSigma0: sp, velSigma0: sv },
    );
    const delta = 5e-5; // craft is 5e-5 AU farther out than believed
    // Consistent measurement: radially at rest (v is tangential), range off by delta.
    nav.update({ t: 0, range: 1 + delta, rangeRate: 0 }, station);

    // x must absorb almost the whole innovation (measurement is far more
    // precise than the prior); y/z must stay put.
    expect(nav.state().pos.x - 1).toBeCloseTo(delta, 8);
    expect(Math.abs(nav.state().pos.y)).toBeLessThan(1e-12);
    expect(Math.abs(nav.state().pos.z)).toBeLessThan(1e-12);

    // The x (line-of-sight) axis collapses to ~sr; y/z keep the prior. The
    // range-rate update also trims a little off y (its Jacobian mixes the y
    // position with the x velocity), so assert structural bounds.
    const sigma = nav.positionSigma();
    expect(sigma).toBeGreaterThan(sp * 0.95);
    expect(sigma).toBeLessThan(sp * Math.SQRT2 * 1.001);

    // vx collapses (a stationary craft implies vx = 0); y/z velocity keep prior.
    expect(Math.abs(nav.state().vel.x)).toBeLessThan(1e-9);
    // vx keeps half its variance (K_vx = 0.5 from the geometry), vy/vz keep
    // theirs: sigma = sqrt(0.5 + 1 + 1) sv = 1.581 sv.
    expect(nav.velocitySigma()).toBeCloseTo(Math.sqrt(2.5) * sv, 9);
  });

  it('covariance stays finite and positive under repeated precise updates', () => {
    const nav = new Navigator(
      { pos: new Vector3(1, 0, 0), vel: new Vector3(0, 1e-2, 0) },
      0,
      { noise: { rangeSigma: 1e-9, rangeRateSigma: 1e-12, opticalSigma: 1e-9 }, posSigma0: 1e-3, velSigma0: 1e-5 },
    );
    for (let k = 0; k < 200; k++) {
      nav.update({ t: k, range: 1 + 1e-9 * Math.sin(k), rangeRate: 1e-2 }, station);
    }
    expect(Number.isFinite(nav.positionSigma())).toBe(true);
    expect(nav.positionSigma()).toBeGreaterThan(0);
    expect(Number.isFinite(nav.velocitySigma())).toBe(true);
  });
});
