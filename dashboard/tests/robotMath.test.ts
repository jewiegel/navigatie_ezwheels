import { describe, expect, it } from 'vitest';
import { covarianceEllipse, poseFromMsg } from '../src/features/robot/robotMath';
import type { PoseWithCovarianceStamped } from '../src/types';

const msg = (x: number, y: number, yaw: number, cov: number[] = new Array(36).fill(0)): PoseWithCovarianceStamped => ({
  header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'map' },
  pose: {
    pose: {
      position: { x, y, z: 0 },
      orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) },
    },
    covariance: cov,
  },
});

const cov6 = (xx: number, xy: number, yy: number) => {
  const c = new Array(36).fill(0);
  c[0] = xx; c[1] = xy; c[6] = xy; c[7] = yy;
  return c;
};

describe('poseFromMsg', () => {
  it('extracts x, y, yaw', () => {
    const p = poseFromMsg(msg(1.5, -2, 1.2))!;
    expect(p.x).toBe(1.5);
    expect(p.y).toBe(-2);
    expect(p.yaw).toBeCloseTo(1.2, 9);
  });
  it('returns null for null / malformed / non-finite', () => {
    expect(poseFromMsg(null)).toBeNull();
    expect(poseFromMsg({} as PoseWithCovarianceStamped)).toBeNull();
    expect(poseFromMsg(msg(NaN, 0, 0))).toBeNull();
  });
});

describe('covarianceEllipse', () => {
  it('axis-aligned: sqrt of variances, angle 0', () => {
    const e = covarianceEllipse(cov6(4, 0, 1))!;
    expect(e.major).toBeCloseTo(2);
    expect(e.minor).toBeCloseTo(1);
    expect(e.angle).toBeCloseTo(0);
  });
  it('larger y variance: major along y (angle ±π/2)', () => {
    const e = covarianceEllipse(cov6(1, 0, 9))!;
    expect(e.major).toBeCloseTo(3);
    expect(e.minor).toBeCloseTo(1);
    expect(Math.abs(e.angle)).toBeCloseTo(Math.PI / 2);
  });
  it('correlated: major axis along the eigenvector', () => {
    // eigenvalues 3 and 1, major eigenvector (1,1)/√2
    const e = covarianceEllipse(cov6(2, 1, 2))!;
    expect(e.major).toBeCloseTo(Math.sqrt(3));
    expect(e.minor).toBeCloseTo(1);
    expect(e.angle).toBeCloseTo(Math.PI / 4);
  });
  it('isotropic and zero covariance do not produce NaN', () => {
    for (const c of [cov6(1, 0, 1), cov6(0, 0, 0)]) {
      const e = covarianceEllipse(c)!;
      expect(Number.isFinite(e.major) && Number.isFinite(e.minor) && Number.isFinite(e.angle)).toBe(true);
      expect(e.major).toBeCloseTo(e.minor);
    }
  });
  it('null for missing/short/NaN covariance', () => {
    expect(covarianceEllipse(undefined)).toBeNull();
    expect(covarianceEllipse([1, 2, 3])).toBeNull();
    expect(covarianceEllipse(cov6(NaN, 0, 1))).toBeNull();
  });
});
