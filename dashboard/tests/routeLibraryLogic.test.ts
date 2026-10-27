import { describe, expect, it } from 'vitest';
import {
  fromWaypointMsgs,
  hasUnsavedChanges,
  serviceErrorText,
  sortRouteNames,
  toWaypointMsgs,
  validateRouteName,
} from '../src/features/routes/routeLibraryLogic';
import type { Pose2D } from '../src/types';

const P = (x: number, y: number, yaw = 0): Pose2D => ({ x, y, yaw });

describe('validateRouteName', () => {
  it.each(['a', 'route_1', 'Hal-A', 'ABC_def-123', '-', '_'])('accepts %j', (n) => {
    expect(validateRouteName(n)).toBeNull();
  });

  it.each(['', ' ', '   ', '\t\n'])('rejects empty/whitespace %j', (n) => {
    expect(validateRouteName(n)).toMatch(/naam/i);
  });

  it.each(['my route', 'a/b', '../etc', 'a\\b', 'route.yaml', 'café', 'ruta_ñ', '日本', 'emoji🙂', 'a:b', 'a*'])(
    'rejects %j',
    (n) => {
      expect(validateRouteName(n)).not.toBeNull();
    },
  );

  it('ignores surrounding whitespace (caller must send the trimmed name)', () => {
    expect(validateRouteName('  hal_a  ')).toBeNull();
  });
});

describe('waypoint msg conversion', () => {
  it('names wp1..wpN and keeps x/y/yaw', () => {
    const msgs = toWaypointMsgs([P(1, 2, 0.5), P(-3, 4, -Math.PI)]);
    expect(msgs).toEqual([
      { name: 'wp1', x: 1, y: 2, yaw: 0.5 },
      { name: 'wp2', x: -3, y: 4, yaw: -Math.PI },
    ]);
    expect(Object.keys(msgs[0]).sort()).toEqual(['name', 'x', 'y', 'yaw']);
  });

  it('empty list', () => {
    expect(toWaypointMsgs([])).toEqual([]);
    expect(fromWaypointMsgs([])).toEqual([]);
  });

  it('drops extra fields (e.g. editor ids) when converting to msgs', () => {
    const msgs = toWaypointMsgs([{ id: 'wp-x', x: 1, y: 1, yaw: 0 } as Pose2D]);
    expect(msgs[0]).not.toHaveProperty('id');
  });

  it('fromWaypointMsgs strips names; round trip is identity', () => {
    const pts = [P(0, 0, 0), P(1.25, -2.5, 3)];
    const back = fromWaypointMsgs(toWaypointMsgs(pts));
    expect(back).toEqual(pts);
    expect(back[0]).not.toHaveProperty('name');
  });

  it('builds a SetRoute request with the srv field names', () => {
    const req = { name: 'hal_a', waypoints: toWaypointMsgs([P(1, 2)]), overwrite: false };
    expect(Object.keys(req).sort()).toEqual(['name', 'overwrite', 'waypoints']);
  });
});

describe('hasUnsavedChanges', () => {
  const saved = [P(0, 0, 0), P(1, 0, 1)];

  it('identical and both empty → false', () => {
    expect(hasUnsavedChanges(saved, saved.map((p) => ({ ...p })))).toBe(false);
    expect(hasUnsavedChanges([], [])).toBe(false);
  });

  it('different lengths → true', () => {
    expect(hasUnsavedChanges(saved, saved.slice(0, 1))).toBe(true);
    expect(hasUnsavedChanges([], saved)).toBe(true);
    expect(hasUnsavedChanges(saved, [])).toBe(true);
  });

  it('tiny float noise is within epsilon', () => {
    expect(hasUnsavedChanges(saved, [P(1e-6, -1e-6, 1e-6), P(1 + 1e-6, 0, 1 - 1e-6)])).toBe(false);
  });

  it('real moves / yaw changes → true', () => {
    expect(hasUnsavedChanges(saved, [P(0.05, 0, 0), saved[1]])).toBe(true);
    expect(hasUnsavedChanges(saved, [saved[0], P(1, 0.05, 1)])).toBe(true);
    expect(hasUnsavedChanges(saved, [saved[0], P(1, 0, 1.1)])).toBe(true);
  });

  it('reordering counts as a change', () => {
    expect(hasUnsavedChanges(saved, [saved[1], saved[0]])).toBe(true);
  });

  it('yaw wrap: π vs -π and 0 vs 2π are equal', () => {
    expect(hasUnsavedChanges([P(0, 0, Math.PI)], [P(0, 0, -Math.PI)])).toBe(false);
    expect(hasUnsavedChanges([P(0, 0, 0)], [P(0, 0, 2 * Math.PI)])).toBe(false);
    expect(hasUnsavedChanges([P(0, 0, Math.PI - 1e-4)], [P(0, 0, -Math.PI + 1e-4)])).toBe(false);
    expect(hasUnsavedChanges([P(0, 0, -3 * Math.PI)], [P(0, 0, Math.PI)])).toBe(false);
  });

  it('NaN coordinate vs a number counts as a change', () => {
    expect(hasUnsavedChanges([P(1, 0, 0)], [P(NaN, 0, 0)])).toBe(true);
    expect(hasUnsavedChanges([P(1, 0, 0)], [P(1, NaN, 0)])).toBe(true);
    expect(hasUnsavedChanges([P(1, 0, 0)], [P(1, 0, NaN)])).toBe(true);
  });
});

describe('sortRouteNames', () => {
  it('case-insensitive, numeric-aware, does not mutate input', () => {
    const input = ['route10', 'Beta', 'route2', 'alpha', 'route1'];
    const copy = [...input];
    expect(sortRouteNames(input)).toEqual(['alpha', 'Beta', 'route1', 'route2', 'route10']);
    expect(input).toEqual(copy);
  });

  it('empty list', () => {
    expect(sortRouteNames([])).toEqual([]);
  });
});

describe('serviceErrorText', () => {
  it('maps common rosbridge failures to Dutch text', () => {
    expect(serviceErrorText('/list_routes', new Error('Not connected'))).toBe('Niet verbonden met de robot');
    expect(serviceErrorText('/get_route', new Error('Service call timed out'))).toMatch(/\/get_route niet beschikbaar.*route_manager/);
    expect(serviceErrorText('/start_route', 'boom', 'mission_manager')).toBe('Fout bij /start_route: boom');
  });
});

describe('allFinite', () => {
  it('true for normal and empty lists, false for any NaN/Infinity', async () => {
    const { allFinite } = await import('../src/features/routes/routeLibraryLogic');
    expect(allFinite([])).toBe(true);
    expect(allFinite([P(0, 0, 0), P(-1.5, 2, -Math.PI)])).toBe(true);
    for (const bad of [P(NaN, 0, 0), P(0, Infinity, 0), P(0, 0, -Infinity), P(0, 0, NaN)])
      expect(allFinite([P(1, 1, 0), bad])).toBe(false);
  });
});
