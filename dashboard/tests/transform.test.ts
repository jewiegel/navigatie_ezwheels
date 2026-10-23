import { describe, expect, it } from 'vitest';
import {
  fitView,
  mapInfoFromGrid,
  occupancyToRgba,
  screenToWorld,
  worldToScreen,
  zoomAt,
} from '../src/features/map/transform';
import type { MapInfo, OccupancyGrid, ViewTransform } from '../src/types';

const info = (yaw = 0, over: Partial<MapInfo> = {}): MapInfo => ({
  resolution: 0.05,
  width: 200,
  height: 100,
  origin: { x: -3, y: 2, yaw },
  frameId: 'map',
  ...over,
});

const views: ViewTransform[] = [
  { zoom: 1, panX: 0, panY: 0 },
  { zoom: 2.5, panX: 40, panY: -30 },
  { zoom: 0.3, panX: -120, panY: 77 },
];

describe('mapInfoFromGrid', () => {
  it('flattens metadata and extracts origin yaw', () => {
    const yaw = 0.7;
    const grid: OccupancyGrid = {
      header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'map' },
      info: {
        map_load_time: { sec: 0, nanosec: 0 },
        resolution: 0.1,
        width: 3,
        height: 4,
        origin: {
          position: { x: 1, y: -2, z: 0 },
          orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) },
        },
      },
      data: [],
    };
    const mi = mapInfoFromGrid(grid);
    expect(mi).toMatchObject({ resolution: 0.1, width: 3, height: 4, frameId: 'map' });
    expect(mi.origin.x).toBe(1);
    expect(mi.origin.y).toBe(-2);
    expect(mi.origin.yaw).toBeCloseTo(yaw, 12);
  });
});

describe('screenToWorld / worldToScreen', () => {
  for (const yaw of [0, 0.4, -2.1, Math.PI]) {
    for (const v of views) {
      it(`round-trips for yaw=${yaw} view=${JSON.stringify(v)}`, () => {
        const mi = info(yaw);
        for (const p of [{ x: 0, y: 0 }, { x: -3, y: 2 }, { x: 4.2, y: -1.7 }, { x: 10, y: 5 }]) {
          const back = screenToWorld(worldToScreen(p, mi, v), mi, v);
          expect(back.x).toBeCloseTo(p.x, 9);
          expect(back.y).toBeCloseTo(p.y, 9);
        }
        for (const s of [{ sx: 0, sy: 0 }, { sx: 123, sy: 45 }]) {
          const back = worldToScreen(screenToWorld(s, mi, v), mi, v);
          expect(back.sx).toBeCloseTo(s.sx, 9);
          expect(back.sy).toBeCloseTo(s.sy, 9);
        }
      });
    }
  }

  it('maps the world origin of the map to the bottom-left image corner (y flip)', () => {
    const mi = info();
    const v = { zoom: 2, panX: 10, panY: 20 };
    const s = worldToScreen({ x: mi.origin.x, y: mi.origin.y }, mi, v);
    expect(s.sx).toBeCloseTo(10);
    expect(s.sy).toBeCloseTo(20 + mi.height * 2);
  });

  it('origin + one resolution step moves one cell right / one cell up', () => {
    const mi = info();
    const v = { zoom: 3, panX: 0, panY: 0 };
    const o = worldToScreen({ x: -3, y: 2 }, mi, v);
    const r = worldToScreen({ x: -3 + 0.05, y: 2 }, mi, v);
    const u = worldToScreen({ x: -3, y: 2 + 0.05 }, mi, v);
    expect(r.sx - o.sx).toBeCloseTo(3);
    expect(r.sy - o.sy).toBeCloseTo(0);
    expect(u.sx - o.sx).toBeCloseTo(0);
    expect(u.sy - o.sy).toBeCloseTo(-3); // up in world = up on screen
  });

  it('top-left screen corner of the image is the top-left of the map', () => {
    const mi = info();
    const w = screenToWorld({ sx: 0, sy: 0 }, mi, { zoom: 1, panX: 0, panY: 0 });
    expect(w.x).toBeCloseTo(-3);
    expect(w.y).toBeCloseTo(2 + 100 * 0.05);
  });

  it('applies origin yaw: with yaw=π/2, map +x axis points world +y', () => {
    const mi = info(Math.PI / 2, { origin: { x: 0, y: 0, yaw: Math.PI / 2 } });
    const v = { zoom: 1, panX: 0, panY: 0 };
    // one cell right of the bottom-left corner in the image
    const w = screenToWorld({ sx: 1, sy: mi.height }, mi, v);
    expect(w.x).toBeCloseTo(0);
    expect(w.y).toBeCloseTo(0.05);
  });
});

describe('occupancyToRgba', () => {
  const px = (img: Uint8ClampedArray, w: number, col: number, row: number) =>
    Array.from(img.slice((row * w + col) * 4, (row * w + col) * 4 + 4));

  it('flips rows: grid row 0 becomes image bottom row', () => {
    // 2x2 grid: row0 = [-1, 0], row1 = [100, 50]
    const img = occupancyToRgba([-1, 0, 100, 50], 2, 2);
    expect(img.length).toBe(16);
    const unknown = px(occupancyToRgba([-1], 1, 1), 1, 0, 0);
    const free = px(occupancyToRgba([0], 1, 1), 1, 0, 0);
    const occ = px(occupancyToRgba([100], 1, 1), 1, 0, 0);
    expect(px(img, 2, 0, 1)).toEqual(unknown); // bottom image row = grid row 0
    expect(px(img, 2, 1, 1)).toEqual(free);
    expect(px(img, 2, 0, 0)).toEqual(occ); // top image row = grid row 1
  });

  it('-1, 0 and 100 map to distinct opaque colours', () => {
    const img = occupancyToRgba([-1, 0, 100], 3, 1);
    const [u, f, o] = [0, 1, 2].map((i) => px(img, 3, i, 0));
    expect(new Set([u, f, o].map(String)).size).toBe(3);
    for (const c of [u, f, o]) expect(c[3]).toBe(255);
  });

  it('accepts Int8Array data', () => {
    const a = occupancyToRgba(new Int8Array([-1, 0, 100]), 3, 1);
    const b = occupancyToRgba([-1, 0, 100], 3, 1);
    expect(Array.from(a)).toEqual(Array.from(b));
  });
});

describe('zoomAt', () => {
  it('keeps the world point under the cursor fixed', () => {
    const mi = info(0.3);
    const v = { zoom: 1.5, panX: 12, panY: -8 };
    const cursor = { sx: 300, sy: 140 };
    const before = screenToWorld(cursor, mi, v);
    for (const f of [2, 0.5, 1.1]) {
      const nv = zoomAt(v, cursor, f, 0.01, 100);
      expect(nv.zoom).toBeCloseTo(1.5 * f);
      const after = screenToWorld(cursor, mi, nv);
      expect(after.x).toBeCloseTo(before.x, 9);
      expect(after.y).toBeCloseTo(before.y, 9);
    }
  });

  it('clamps to min/max and still keeps the cursor point fixed', () => {
    const mi = info();
    const v = { zoom: 1, panX: 0, panY: 0 };
    const cursor = { sx: 50, sy: 60 };
    const nv = zoomAt(v, cursor, 1000, 0.1, 8);
    expect(nv.zoom).toBe(8);
    const a = screenToWorld(cursor, mi, v), b = screenToWorld(cursor, mi, nv);
    expect(b.x).toBeCloseTo(a.x);
    expect(b.y).toBeCloseTo(a.y);
    expect(zoomAt(v, cursor, 1e-6, 0.1, 8).zoom).toBe(0.1);
  });
});

describe('fitView', () => {
  for (const [cw, ch] of [[800, 600], [300, 900], [1920, 400]]) {
    it(`fits the whole map inside ${cw}x${ch} and centres it`, () => {
      const mi = info();
      const v = fitView(mi, cw, ch, 24);
      const corners = [
        { sx: v.panX, sy: v.panY },
        { sx: v.panX + mi.width * v.zoom, sy: v.panY + mi.height * v.zoom },
      ];
      for (const c of corners) {
        expect(c.sx).toBeGreaterThanOrEqual(24 - 1e-9);
        expect(c.sx).toBeLessThanOrEqual(cw - 24 + 1e-9);
        expect(c.sy).toBeGreaterThanOrEqual(24 - 1e-9);
        expect(c.sy).toBeLessThanOrEqual(ch - 24 + 1e-9);
      }
      // tight along one axis
      const tight = Math.min((cw - 48) / mi.width, (ch - 48) / mi.height);
      expect(v.zoom).toBeCloseTo(tight);
      // centred
      expect(corners[0].sx + corners[1].sx).toBeCloseTo(cw);
      expect(corners[0].sy + corners[1].sy).toBeCloseTo(ch);
    });
  }
});
