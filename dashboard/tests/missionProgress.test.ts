import { describe, expect, it } from 'vitest';
import { formatDuration, missionProgress, remainingDistance } from '../src/features/routes/missionProgress';

describe('missionProgress', () => {
  it('counts passed waypoints from the remaining list (current target included)', () => {
    expect(missionProgress('DRIVING', 5, 5)).toEqual({ passed: 0, total: 5, fraction: 0, current: 1 });
    expect(missionProgress('DRIVING', 5, 2)).toEqual({ passed: 3, total: 5, fraction: 0.6, current: 4 });
  });
  it('arrived means complete, whatever remaining says', () => {
    expect(missionProgress('ARRIVED', 4, 1)).toMatchObject({ passed: 4, fraction: 1, current: null });
  });
  it('unknown total or remaining gives no fraction', () => {
    expect(missionProgress('DRIVING', null, 3).fraction).toBeNull();
    expect(missionProgress('DRIVING', 3, null).fraction).toBeNull();
    expect(missionProgress('ARRIVED', null, null).fraction).toBeNull();
  });
  it('clamps a remaining list longer than the route (stale message from a previous route)', () => {
    expect(missionProgress('DRIVING', 3, 7)).toMatchObject({ passed: 0, fraction: 0 });
    expect(missionProgress('DRIVING', 3, 0)).toMatchObject({ passed: 3, fraction: 1, current: null });
  });
  it('no current target when not driving', () => {
    expect(missionProgress('IDLE', 3, 2).current).toBeNull();
    expect(missionProgress('BLOCKED', 3, 2).current).toBe(2);
  });
});

describe('remainingDistance', () => {
  it('sums robot→next and the segments after it', () => {
    expect(remainingDistance({ x: 0, y: 0 }, [{ x: 3, y: 4 }, { x: 3, y: 10 }])).toEqual({ toNext: 5, total: 11 });
  });
  it('is unknown without a robot pose or without remaining points', () => {
    expect(remainingDistance(null, [{ x: 1, y: 1 }])).toEqual({ toNext: null, total: null });
    expect(remainingDistance({ x: 0, y: 0 }, [])).toEqual({ toNext: null, total: null });
  });
});

describe('formatDuration', () => {
  it('formats seconds, minutes and hours', () => {
    expect(formatDuration(0)).toBe('0 s');
    expect(formatDuration(59.6)).toBe('1 min 00 s');
    expect(formatDuration(125)).toBe('2 min 05 s');
    expect(formatDuration(3780)).toBe('1 u 03 min');
    expect(formatDuration(null)).toBe('–');
    expect(formatDuration(NaN)).toBe('–');
    expect(formatDuration(-1)).toBe('–');
  });
});
