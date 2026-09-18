import { describe, expect, it } from 'vitest';
import { AU, DAY, GM_SUN } from '../data/constants';
import { AUDAY_TO_KMS, MU_SUN, fromKms, toKms } from './units';

describe('nav units', () => {
  it('derives the solar mu in AU^3/day^2 consistently', () => {
    expect(MU_SUN).toBeCloseTo((GM_SUN * DAY * DAY) / (AU * AU * AU), 18);
    expect(AUDAY_TO_KMS).toBeCloseTo(1731.46, 1);
  });

  it('round-trips km/s conversions', () => {
    expect(toKms(fromKms(12.3))).toBeCloseTo(12.3, 12);
    expect(toKms(1)).toBeCloseTo(1731.4568, 3);
  });
});
