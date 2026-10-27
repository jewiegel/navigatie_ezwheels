import { describe, expect, it } from 'vitest';
import {
  addWaypoint,
  computeYaws,
  hitTest,
  moveWaypoint,
  navStatusFromFeedback,
  reorderWaypoints,
  removeWaypoint,
  toNavigateThroughPosesGoal,
  type NavigateThroughPosesFeedback,
} from '../src/features/route/routeLogic';
import type { Point, Waypoint } from '../src/types';

const build = (pts: [number, number][]) =>
  pts.reduce<Waypoint[]>((l, [x, y], i) => addWaypoint(l, { x, y }, `w${i}`), []);
const ids = (l: Waypoint[]) => l.map((w) => w.id);
const yawOfQ = (q: { z: number; w: number; x: number; y: number }) =>
  Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
const angDiff = (a: number, b: number) => Math.atan2(Math.sin(a - b), Math.cos(a - b));

describe('add/move/remove/reorder', () => {
  it('addWaypoint appends in order with given ids and unique default ids', () => {
    const l = build([[0, 0], [1, 0], [1, 1]]);
    expect(ids(l)).toEqual(['w0', 'w1', 'w2']);
    expect(l.map((w) => [w.x, w.y])).toEqual([[0, 0], [1, 0], [1, 1]]);
    const a = addWaypoint([], { x: 0, y: 0 });
    const b = addWaypoint(a, { x: 1, y: 0 });
    expect(new Set(ids(b)).size).toBe(2);
  });

  it('never mutates the input list or its items', () => {
    const l = build([[0, 0], [1, 0], [1, 1]]);
    const snap = JSON.stringify(l);
    const frozen = Object.freeze(l.map((w) => Object.freeze({ ...w }))) as unknown as Waypoint[];
    addWaypoint(frozen, { x: 5, y: 5 });
    moveWaypoint(frozen, 'w1', { x: 9, y: 9 });
    removeWaypoint(frozen, 'w0');
    reorderWaypoints(frozen, 0, 2);
    computeYaws(frozen);
    addWaypoint(l, { x: 5, y: 5 });
    moveWaypoint(l, 'w1', { x: 9, y: 9 });
    expect(JSON.stringify(l)).toBe(snap);
  });

  it('moveWaypoint changes only the target and recomputes yaws', () => {
    const l = build([[0, 0], [1, 0], [2, 0]]);
    const m = moveWaypoint(l, 'w1', { x: 1, y: 1 });
    expect(ids(m)).toEqual(ids(l));
    expect(m[1]).toMatchObject({ x: 1, y: 1 });
    expect(m[0].yaw).toBeCloseTo(Math.PI / 4);
    expect(moveWaypoint(l, 'nope', { x: 3, y: 3 }).map((w) => [w.x, w.y])).toEqual(l.map((w) => [w.x, w.y]));
  });

  it('removeWaypoint keeps the order of the rest', () => {
    const l = build([[0, 0], [1, 0], [2, 0], [3, 0]]);
    expect(ids(removeWaypoint(l, 'w1'))).toEqual(['w0', 'w2', 'w3']);
    expect(ids(removeWaypoint(l, 'missing'))).toEqual(ids(l));
  });

  it('reorderWaypoints moves from -> to and ignores out-of-range', () => {
    const l = build([[0, 0], [1, 0], [2, 0], [3, 0]]);
    expect(ids(reorderWaypoints(l, 0, 2))).toEqual(['w1', 'w2', 'w0', 'w3']);
    expect(ids(reorderWaypoints(l, 3, 0))).toEqual(['w3', 'w0', 'w1', 'w2']);
    expect(ids(reorderWaypoints(l, 1, 1))).toEqual(ids(l));
    expect(ids(reorderWaypoints(l, -1, 2))).toEqual(ids(l));
    expect(ids(reorderWaypoints(l, 0, 4))).toEqual(ids(l));
  });
});

describe('computeYaws', () => {
  it('each yaw points to the next waypoint; last takes the previous segment heading', () => {
    const pts: Point[] = [{ x: 0, y: 0 }, { x: 1, y: 0 }, { x: 1, y: 2 }, { x: -1, y: 2 }];
    const l = computeYaws(pts.map((p, i) => ({ ...p, id: `${i}`, yaw: 123 })));
    for (let k = 0; k < l.length - 1; k++) {
      expect(l[k].yaw).toBeCloseTo(Math.atan2(l[k + 1].y - l[k].y, l[k + 1].x - l[k].x));
    }
    expect(Math.abs(l[3].yaw)).toBeCloseTo(Math.PI); // segment 2->3 heads -x
  });

  it('single waypoint keeps its yaw, empty list ok', () => {
    expect(computeYaws([{ id: 'a', x: 1, y: 1, yaw: 0.7 }])[0].yaw).toBe(0.7);
    expect(computeYaws([])).toEqual([]);
  });

  it('zero-length segment inherits a real heading instead of a silent 0', () => {
    // duplicate point in the middle, all headings are +y (π/2)
    const l = build([[0, 0], [0, 1], [0, 1], [0, 2]]);
    for (const w of l) expect(w.yaw).toBeCloseTo(Math.PI / 2);
    // duplicate at the start: first point uses the first valid heading
    const s = build([[3, 3], [3, 3], [2, 3]]);
    for (const w of s) expect(Math.abs(w.yaw)).toBeCloseTo(Math.PI);
    // duplicate at the end
    const e = build([[0, 0], [0, -1], [0, -1]]);
    for (const w of e) expect(w.yaw).toBeCloseTo(-Math.PI / 2);
  });
});

describe('hitTest', () => {
  const api = { worldToScreen: (p: Point) => ({ sx: p.x * 10, sy: -p.y * 10 }) };
  const l = build([[0, 0], [1, 0], [1.2, 0]]);
  it('returns the closest waypoint within the radius', () => {
    expect(hitTest(l, { sx: 1, sy: 1 }, api, 5)).toBe('w0');
    expect(hitTest(l, { sx: 11.5, sy: 0 }, api, 5)).toBe('w2');
    expect(hitTest(l, { sx: 10.5, sy: 0 }, api, 5)).toBe('w1');
  });
  it('null when nothing within the radius', () => {
    expect(hitTest(l, { sx: 50, sy: 50 }, api, 5)).toBeNull();
    expect(hitTest([], { sx: 0, sy: 0 }, api, 5)).toBeNull();
  });
});

describe('toNavigateThroughPosesGoal', () => {
  it("frame 'map', poses in order, quaternion matches yaw", () => {
    const l = build([[0, 0], [1, 0], [1, 1], [0, 1]]);
    const g = toNavigateThroughPosesGoal(l);
    expect(g.poses).toHaveLength(4);
    g.poses.forEach((p, i) => {
      expect(p.header.frame_id).toBe('map');
      expect(p.pose.position).toEqual({ x: l[i].x, y: l[i].y, z: 0 });
      const q = p.pose.orientation;
      expect(Math.hypot(q.x, q.y, q.z, q.w)).toBeCloseTo(1);
      expect(q.x).toBeCloseTo(0);
      expect(q.y).toBeCloseTo(0);
      expect(angDiff(yawOfQ(q), l[i].yaw)).toBeCloseTo(0);
    });
    expect(g.behavior_tree).toBe('');
  });
  it('passes through custom frame and stamp', () => {
    const g = toNavigateThroughPosesGoal(build([[0, 0]]), 'odom', { sec: 5, nanosec: 7 });
    expect(g.poses[0].header).toEqual({ frame_id: 'odom', stamp: { sec: 5, nanosec: 7 } });
  });
});

describe('navStatusFromFeedback', () => {
  const fb: NavigateThroughPosesFeedback = {
    current_pose: { header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'map' }, pose: { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } },
    navigation_time: { sec: 3, nanosec: 0 },
    estimated_time_remaining: { sec: 12, nanosec: 500_000_000 },
    number_of_recoveries: 2,
    distance_remaining: 4.5,
    number_of_poses_remaining: 3,
  };
  it('maps feedback fields', () => {
    expect(navStatusFromFeedback('active', 5, fb)).toMatchObject({
      goal: 'active', totalWaypoints: 5, remainingWaypoints: 3, distanceRemaining: 4.5, etaSeconds: 12.5, recoveries: 2,
    });
  });
  it('nulls without feedback, error message overrides', () => {
    const s = navStatusFromFeedback('idle', 0, null);
    expect(s).toMatchObject({ remainingWaypoints: null, distanceRemaining: null, etaSeconds: null, recoveries: null });
    expect(navStatusFromFeedback('error', 2, null, 'boom').message).toBe('boom');
    expect(navStatusFromFeedback('aborted', 2, null).message).toBeTruthy();
  });
});

describe('loadWaypoints', () => {
  it('keeps given x/y/yaw exactly (no recompute) and assigns unique ids', async () => {
    const { loadWaypoints } = await import('../src/features/route/routeLogic');
    const pts = [{ x: 0, y: 0, yaw: 1.5 }, { x: 1, y: 0, yaw: -3 }, { x: 1, y: 0, yaw: 0.25 }];
    const l = loadWaypoints(pts);
    expect(l.map(({ x, y, yaw }) => ({ x, y, yaw }))).toEqual(pts);
    expect(new Set(ids(l)).size).toBe(3);
    expect(new Set([...ids(l), ...ids(loadWaypoints(pts))]).size).toBe(6);
    expect(loadWaypoints([])).toEqual([]);
  });
});
