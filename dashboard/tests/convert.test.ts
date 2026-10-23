import { afterEach, describe, expect, it, vi } from 'vitest';
import { quaternionToYaw, rosTimeNow, yawToQuaternion } from '../src/ros/convert';

describe('yaw <-> quaternion', () => {
  it('round-trips yaw in (-π, π]', () => {
    for (let i = -99; i <= 100; i++) {
      const yaw = (i / 100) * Math.PI;
      const q = yawToQuaternion(yaw);
      expect(Math.hypot(q.x, q.y, q.z, q.w)).toBeCloseTo(1, 12);
      expect(q.x).toBe(0);
      expect(q.y).toBe(0);
      const back = quaternionToYaw(q);
      expect(back).toBeGreaterThan(-Math.PI - 1e-12);
      expect(back).toBeLessThanOrEqual(Math.PI + 1e-12);
      // ±π are the same heading
      expect(Math.atan2(Math.sin(back - yaw), Math.cos(back - yaw))).toBeCloseTo(0, 9);
    }
  });

  it('known values', () => {
    expect(yawToQuaternion(0)).toMatchObject({ z: 0, w: 1 });
    const q = yawToQuaternion(Math.PI / 2);
    expect(q.z).toBeCloseTo(Math.SQRT1_2);
    expect(q.w).toBeCloseTo(Math.SQRT1_2);
    expect(quaternionToYaw({ x: 0, y: 0, z: 1, w: 0 })).toBeCloseTo(Math.PI);
  });

  it('quaternionToYaw treats q and -q as the same rotation', () => {
    const q = yawToQuaternion(1.1);
    expect(quaternionToYaw({ x: 0, y: 0, z: -q.z, w: -q.w })).toBeCloseTo(1.1);
  });
});

describe('rosTimeNow', () => {
  afterEach(() => vi.useRealTimers());
  it('splits Date.now() into sec + nanosec', () => {
    vi.useFakeTimers();
    vi.setSystemTime(1_700_000_123_456);
    const t = rosTimeNow();
    expect(t.sec).toBe(1_700_000_123);
    expect(t.nanosec).toBe(456_000_000);
    expect(Number.isInteger(t.sec) && Number.isInteger(t.nanosec)).toBe(true);
  });
});
