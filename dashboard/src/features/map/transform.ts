import type { MapInfo, OccupancyGrid, Point, ScreenPoint, ViewTransform } from '../../types';

export function mapInfoFromGrid(grid: OccupancyGrid): MapInfo {
  const { resolution, width, height, origin } = grid.info;
  const { x, y, z, w } = origin.orientation;
  return {
    resolution,
    width,
    height,
    origin: {
      x: origin.position.x,
      y: origin.position.y,
      yaw: Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
    },
    frameId: grid.header.frame_id,
  };
}

export function screenToWorld(s: ScreenPoint, info: MapInfo, view: ViewTransform): Point {
  const px = (s.sx - view.panX) / view.zoom;
  const py = (s.sy - view.panY) / view.zoom;
  const lx = px * info.resolution;
  const ly = (info.height - py) * info.resolution; // data row 0 is the bottom
  const c = Math.cos(info.origin.yaw);
  const sn = Math.sin(info.origin.yaw);
  return { x: info.origin.x + c * lx - sn * ly, y: info.origin.y + sn * lx + c * ly };
}

export function worldToScreen(p: Point, info: MapInfo, view: ViewTransform): ScreenPoint {
  const dx = p.x - info.origin.x;
  const dy = p.y - info.origin.y;
  const c = Math.cos(info.origin.yaw);
  const sn = Math.sin(info.origin.yaw);
  const lx = c * dx + sn * dy;
  const ly = -sn * dx + c * dy;
  const px = lx / info.resolution;
  const py = info.height - ly / info.resolution;
  return { sx: px * view.zoom + view.panX, sy: py * view.zoom + view.panY };
}

// Palette (RGB): unknown, free, occupied. Occupancy 1..99 interpolates free→occupied.
const UNKNOWN = [0x2a, 0x2f, 0x3a];
const FREE = [0xd8, 0xdd, 0xe6];
const OCCUPIED = [0x11, 0x14, 0x1a];

const pack = (r: number, g: number, b: number) => ((255 << 24) | (b << 16) | (g << 8) | r) >>> 0;

// LUT indexed by the int8 value as unsigned byte (v & 0xff). Values outside 0..100 render as unknown.
const LUT = new Uint32Array(256).fill(pack(UNKNOWN[0], UNKNOWN[1], UNKNOWN[2]));
for (let v = 0; v <= 100; v++) {
  const t = v / 100;
  const mix = (i: number) => Math.round(FREE[i] + (OCCUPIED[i] - FREE[i]) * t);
  LUT[v] = pack(mix(0), mix(1), mix(2));
}

/** RGBA image of the grid, image row 0 = top (grid row height-1). */
export function occupancyToRgba(
  data: ArrayLike<number>,
  width: number,
  height: number,
): Uint8ClampedArray {
  const out = new Uint8ClampedArray(width * height * 4);
  // shortcut: assumes little-endian byte order (true for every browser/Node platform in practice)
  const u32 = new Uint32Array(out.buffer);
  for (let j = 0; j < height; j++) {
    const src = j * width;
    const dst = (height - 1 - j) * width;
    for (let i = 0; i < width; i++) u32[dst + i] = LUT[data[src + i] & 0xff];
  }
  return out;
}

/** Zoom by `factor` keeping the point under `screen` fixed. */
export function zoomAt(
  view: ViewTransform,
  screen: ScreenPoint,
  factor: number,
  min: number,
  max: number,
): ViewTransform {
  const zoom = Math.min(max, Math.max(min, view.zoom * factor));
  const k = zoom / view.zoom;
  return {
    zoom,
    panX: screen.sx - (screen.sx - view.panX) * k,
    panY: screen.sy - (screen.sy - view.panY) * k,
  };
}

/** Centre the whole grid in the canvas with `padding` CSS px around it. */
export function fitView(info: MapInfo, canvasW: number, canvasH: number, padding = 24): ViewTransform {
  const zoom = Math.max(
    1e-6,
    Math.min((canvasW - 2 * padding) / info.width, (canvasH - 2 * padding) / info.height),
  );
  return {
    zoom,
    panX: (canvasW - info.width * zoom) / 2,
    panY: (canvasH - info.height * zoom) / 2,
  };
}
