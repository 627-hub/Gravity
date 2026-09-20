import { describe, expect, it } from 'vitest';
import { massRatio, propellantFraction } from './maneuvers';
import { DRIVES } from './propulsion';
import {
  biEllipticTransfer,
  circularSpeed,
  escapeSpeed,
  hohmannTransfer,
  parkingOrbitDeltaV,
  planeChangeDeltaV,
} from './maneuvers';
import { MU_SUN, toKms } from './units';

describe('impulsive maneuvers', () => {
  it('reproduces the textbook Earth→Mars Hohmann transfer', () => {
    const h = hohmannTransfer(1, 1.5237, MU_SUN);
    expect(toKms(h.dv1)).toBeCloseTo(2.94, 1); // ~2.94 km/s departure
    expect(toKms(h.dv2)).toBeCloseTo(2.65, 1); // ~2.65 km/s arrival
    expect(toKms(h.dvTotal)).toBeCloseTo(5.59, 0);
    expect(h.tof).toBeGreaterThan(255);
    expect(h.tof).toBeLessThan(262); // ~259 days
  });

  it('bi-elliptic beats Hohmann only for large radius ratios', () => {
    const bigH = hohmannTransfer(1, 20, MU_SUN);
    const bigB = biEllipticTransfer(1, 20, 100, MU_SUN);
    expect(bigB.dvTotal).toBeLessThan(bigH.dvTotal);

    const smallH = hohmannTransfer(1, 1.5, MU_SUN);
    const smallB = biEllipticTransfer(1, 1.5, 3, MU_SUN);
    expect(smallB.dvTotal).toBeGreaterThan(smallH.dvTotal);
  });

  it('keeps escape and circular speeds consistent', () => {
    expect(escapeSpeed(1, MU_SUN) / circularSpeed(1, MU_SUN)).toBeCloseTo(Math.SQRT2, 12);
    expect(toKms(circularSpeed(1, MU_SUN))).toBeCloseTo(29.78, 0); // Earth's ~29.8 km/s
  });

  it('computes a trans-lunar-injection-like burn', () => {
    // 200 km LEO: r = 6778 km, mu=398600.4 km^3/s^2, v_inf = 3 km/s.
    const dv = parkingOrbitDeltaV(6778, 3, 398600.4);
    expect(dv).toBeCloseTo(3.58, 1);
  });

  it('computes plane-change burns', () => {
    expect(planeChangeDeltaV(10, 60)).toBeCloseTo(10, 12);
    expect(planeChangeDeltaV(10, 0)).toBeCloseTo(0, 12);
  });
});

describe('rocket equation (Δv -> propellant)', () => {
  it('质量比按 e^(Δv/v_e)，不是线性', () => {
    // Isp 450 s -> v_e = 4.413 km/s
    expect(massRatio(4.413, 450)).toBeCloseTo(Math.E, 3);
    expect(massRatio(0, 450)).toBeCloseTo(1, 12);
    // 地→火窗口 4.23 vs 窗口外 23.54：Δv ×5.6 -> 质量比 ×~90
    const windowMr = massRatio(4.23, 450);
    const offMr = massRatio(23.54, 450);
    expect(windowMr).toBeGreaterThan(2.5);
    expect(windowMr).toBeLessThan(2.7);
    expect(offMr / windowMr).toBeGreaterThan(50);
    // 高比冲（电推）把同样的 Δv 变得可行
    expect(propellantFraction(23.54, DRIVES[2].ispS!)).toBeLessThan(0.6);
    expect(propellantFraction(23.54, DRIVES[0].ispS!)).toBeGreaterThan(0.99);
  });
});
