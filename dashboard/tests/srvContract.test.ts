import { readFileSync, existsSync } from 'node:fs';
import { resolve } from 'node:path';
import { describe, expect, it } from 'vitest';

// Checks that the .srv/.msg files in my_bot_interfaces match the request/response
// shapes the dashboard uses (documented in src/types.ts "saved routes").
const IFACES = resolve(__dirname, '../../my_bot_interfaces');

/** Field names of a .srv/.msg section (constants and comments skipped). */
function fields(text: string): string[] {
  return text
    .split('\n')
    .map((l) => l.replace(/#.*/, '').trim())
    .filter((l) => l && !l.includes('='))
    .map((l) => l.split(/\s+/)[1]);
}

function srv(name: string): { req: string[]; res: string[] } {
  const p = resolve(IFACES, 'srv', `${name}.srv`);
  expect(existsSync(p), `${p} missing`).toBe(true);
  const [req, res = ''] = readFileSync(p, 'utf8').split(/^---\s*$/m);
  return { req: fields(req), res: fields(res) };
}

const sorted = (a: string[]) => [...a].sort();

describe('my_bot_interfaces contract', () => {
  it('Waypoint.msg matches RouteWaypointMsg', () => {
    const msg = readFileSync(resolve(IFACES, 'msg/Waypoint.msg'), 'utf8');
    expect(sorted(fields(msg))).toEqual(['name', 'x', 'y', 'yaw']);
  });

  it('Turn.msg matches TurnMsg', () => {
    const msg = readFileSync(resolve(IFACES, 'msg/Turn.msg'), 'utf8');
    expect(sorted(fields(msg))).toEqual(['angle', 'direction', 'distance', 'turning']);
  });

  it.each([
    ['ListRoutes', [], ['message', 'names', 'success']],
    ['GetRoute', ['name'], ['map_id', 'message', 'success', 'waypoints']],
    ['SetRoute', ['name', 'overwrite', 'waypoints'], ['message', 'success']],
    ['DeleteRoute', ['name'], ['message', 'success']],
    ['StartRoute', ['route_name'], ['accepted', 'message']],
  ])('%s.srv fields', (name, req, res) => {
    const s = srv(name);
    expect(sorted(s.req)).toEqual(req);
    expect(sorted(s.res)).toEqual(res);
  });

  it('SetRoute/GetRoute waypoints are my_bot_interfaces/Waypoint[]', () => {
    for (const n of ['SetRoute', 'GetRoute']) {
      const t = readFileSync(resolve(IFACES, 'srv', `${n}.srv`), 'utf8');
      expect(t).toMatch(/^\s*(my_bot_interfaces\/)?Waypoint\[\]\s+waypoints/m);
    }
  });

  it('CMakeLists registers the services', () => {
    const cm = readFileSync(resolve(IFACES, 'CMakeLists.txt'), 'utf8');
    for (const n of ['ListRoutes', 'GetRoute', 'SetRoute', 'DeleteRoute', 'StartRoute']) expect(cm).toContain(`srv/${n}.srv`);
  });
});
