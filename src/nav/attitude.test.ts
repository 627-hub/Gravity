import { Vector3 } from 'three';
import { describe, expect, it } from 'vitest';
import { Adcs, quatAngle, quatFromBasis, quatRotate } from './attitude';

describe('quaternion helpers', () => {
  it('rotates axes consistently with the basis convention', () => {
    const q = quatFromBasis(new Vector3(0, 0, 1), new Vector3(0, 1, 0));
    expect(quatRotate(q, new Vector3(0, 0, 1)).distanceTo(new Vector3(0, 0, 1))).toBeLessThan(1e-12);
    expect(quatRotate(q, new Vector3(0, 1, 0)).distanceTo(new Vector3(0, 1, 0))).toBeLessThan(1e-12);
    const tilted = quatFromBasis(new Vector3(1, 0, 0), new Vector3(0, 0, 1));
    expect(quatRotate(tilted, new Vector3(0, 0, 1)).distanceTo(new Vector3(1, 0, 0))).toBeLessThan(1e-12);
    expect(quatAngle(tilted, q)).toBeGreaterThan(1); // 90 degrees apart
  });
});

describe('ADCS', () => {
  const up = new Vector3(0, 0, 1);
  const prograde = new Vector3(0, 1, 0);

  it('slews onto the command and damps the rate', () => {
    // Start 30 degrees off, tumbling at 0.05 deg/s (a gentle detumble).
    const off = new Vector3(Math.sin(0.5), Math.cos(0.5), 0);
    const adcs = new Adcs(off, up, 0, {}, 3);
    adcs.rate.set(0, 0, 0.05 / (180 / Math.PI) * 86400); // rad/day
    adcs.advance(2, prograde, up);
    expect(adcs.pointingErrorDeg(prograde)).toBeLessThan(0.05);
    expect(adcs.rateDegPerSec()).toBeLessThan(0.005);
  });

  it('holds pointing with a small residual under control torque limits', () => {
    const adcs = new Adcs(prograde, up, 0, {}, 4);
    for (let k = 0; k < 20; k++) adcs.advance(0.25, prograde, up);
    expect(adcs.pointingErrorDeg(prograde)).toBeLessThan(0.05);
  });

  it('keeps the onboard attitude estimate and bias bounded', () => {
    const adcs = new Adcs(prograde, up, 0, {}, 5);
    const target = new Vector3(0.2, 0.95, 0.05).normalize();
    for (let k = 0; k < 40; k++) adcs.advance(0.5, target, up);
    // 8 arcsec star tracker: the filter estimate should stay at that order
    // (well under a few hundred arcsec) and the gyro bias should be tracked.
    expect(adcs.estimateErrorArcsec()).toBeLessThan(120);
    expect(adcs.biasDegPerHour()).toBeLessThan(0.5);
    expect(adcs.starUpdates).toBeGreaterThan(30);
    // The pointing itself is unaffected by sensor noise (control uses truth).
    expect(adcs.pointingErrorDeg(target)).toBeLessThan(0.1);
  });

  it('replays identically for the same seed', () => {
    const run = () => {
      const a = new Adcs(prograde, up, 0, {}, 7);
      for (let k = 0; k < 10; k++) a.advance(0.5, prograde, up);
      return a.estimateErrorArcsec();
    };
    expect(run()).toBe(run());
  });
});
