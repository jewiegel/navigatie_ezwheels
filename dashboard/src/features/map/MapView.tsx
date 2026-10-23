import { useCallback, useEffect, useRef, useState } from 'react';
import { useTopic } from '../../ros';
import type {
  MapInfo,
  MapOverlay,
  MapPointerEvent,
  MapPointerKind,
  MapViewApi,
  OccupancyGrid,
  ScreenPoint,
  ViewTransform,
} from '../../types';
import { fitView, mapInfoFromGrid, occupancyToRgba, screenToWorld, worldToScreen, zoomAt } from './transform';

const MIN_ZOOM = 0.02;
const MAX_ZOOM = 64;
const DRAG_PX = 4;
const BG = '#0f1115';

export interface MapViewProps {
  overlays: MapOverlay[];
  mapTopic?: string;
}

/** Nice round scale-bar length (m) that is roughly `targetPx` wide. */
function niceLength(pxPerMeter: number, targetPx = 100): number {
  const raw = targetPx / pxPerMeter;
  const p = 10 ** Math.floor(Math.log10(raw));
  return [5, 2, 1].map((m) => m * p).find((v) => v <= raw) ?? p;
}

export function MapView({ overlays, mapTopic = '/map' }: MapViewProps) {
  const grid = useTopic<OccupancyGrid>(mapTopic, 'nav_msgs/msg/OccupancyGrid', { latched: true });

  const wrapRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const readoutRef = useRef<HTMLSpanElement>(null);
  const scaleBarRef = useRef<HTMLDivElement>(null);
  const scaleTextRef = useRef<HTMLSpanElement>(null);

  // Mutable render state lives in refs so pointer/wheel handling doesn't re-render React.
  const s = useRef({
    info: null as MapInfo | null,
    bitmap: null as HTMLCanvasElement | null,
    view: { zoom: 1, panX: 0, panY: 0 } as ViewTransform,
    w: 0,
    h: 0,
    dpr: 1,
    needsFit: true,
    overlays,
    raf: 0,
    // pointer state
    pointers: new Map<number, ScreenPoint>(),
    downAt: null as ScreenPoint | null,
    dragged: false,
    panning: false,
    captured: null as MapOverlay | null, // overlay that consumed 'down': gets move/up/click until release
    pinch: null as { dist: number; mid: ScreenPoint } | null,
  }).current;
  s.overlays = overlays;

  const [hasMap, setHasMap] = useState(false);

  const draw = useCallback(() => {
    s.raf = 0;
    const canvas = canvasRef.current;
    const ctx = canvas?.getContext('2d');
    if (!canvas || !ctx) return;
    ctx.setTransform(s.dpr, 0, 0, s.dpr, 0, 0);
    ctx.fillStyle = BG;
    ctx.fillRect(0, 0, s.w, s.h);
    const { info, view } = s;
    if (info && s.bitmap) {
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(s.bitmap, view.panX, view.panY, info.width * view.zoom, info.height * view.zoom);
    }
    const api = makeApi();
    for (const o of s.overlays) {
      ctx.save();
      try {
        o.draw(ctx, api);
      } catch (err) {
        console.error(`overlay ${o.id} draw failed`, err);
      }
      ctx.restore();
    }
    // scale bar
    if (info && scaleBarRef.current && scaleTextRef.current) {
      const len = niceLength(api.pxPerMeter);
      scaleBarRef.current.style.width = `${len * api.pxPerMeter}px`;
      scaleTextRef.current.textContent = len >= 1 ? `${len} m` : `${Math.round(len * 100)} cm`;
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const requestRedraw = useCallback(() => {
    if (!s.raf) s.raf = requestAnimationFrame(draw);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [draw]);

  function makeApi(): MapViewApi {
    const { info, view } = s;
    return {
      info,
      view,
      worldToScreen: (p) => (info ? worldToScreen(p, info, view) : { sx: 0, sy: 0 }),
      screenToWorld: (p) => (info ? screenToWorld(p, info, view) : { x: 0, y: 0 }),
      pxPerMeter: info ? view.zoom / info.resolution : 0,
      requestRedraw,
    };
  }

  const setView = (v: ViewTransform) => {
    s.view = v;
    requestRedraw();
  };

  const fit = () => {
    if (s.info && s.w > 0) setView(fitView(s.info, s.w, s.h));
    else s.needsFit = true;
  };

  // New grid → rebuild bitmap; keep the world point at the canvas centre fixed.
  useEffect(() => {
    if (!grid) return;
    const info = mapInfoFromGrid(grid);
    if (info.width <= 0 || info.height <= 0) return;
    const bmp = s.bitmap ?? document.createElement('canvas');
    bmp.width = info.width;
    bmp.height = info.height;
    const img = new ImageData(occupancyToRgba(grid.data, info.width, info.height), info.width, info.height);
    bmp.getContext('2d')!.putImageData(img, 0, 0);
    const prev = s.info;
    s.bitmap = bmp;
    s.info = info;
    if (prev && !s.needsFit) {
      const centre = { sx: s.w / 2, sy: s.h / 2 };
      const wc = screenToWorld(centre, prev, s.view);
      const now = worldToScreen(wc, info, s.view);
      s.view = { ...s.view, panX: s.view.panX + centre.sx - now.sx, panY: s.view.panY + centre.sy - now.sy };
    } else if (s.w > 0) {
      s.view = fitView(info, s.w, s.h);
      s.needsFit = false;
    }
    setHasMap(true);
    requestRedraw();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [grid]);

  useEffect(requestRedraw, [overlays, requestRedraw]);

  // Size canvas to parent, DPR-aware.
  useEffect(() => {
    const wrap = wrapRef.current!;
    const canvas = canvasRef.current!;
    const ro = new ResizeObserver(() => {
      const r = wrap.getBoundingClientRect();
      s.w = r.width;
      s.h = r.height;
      s.dpr = window.devicePixelRatio || 1;
      canvas.width = Math.round(r.width * s.dpr);
      canvas.height = Math.round(r.height * s.dpr);
      if (s.needsFit && s.info && s.w > 0) {
        s.view = fitView(s.info, s.w, s.h);
        s.needsFit = false;
      }
      draw(); // synchronous: resizing clears the canvas
    });
    ro.observe(wrap);
    return () => {
      ro.disconnect();
      if (s.raf) cancelAnimationFrame(s.raf);
      s.raf = 0;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // ---------- input ----------

  const local = (e: { clientX: number; clientY: number }): ScreenPoint => {
    const r = canvasRef.current!.getBoundingClientRect();
    return { sx: e.clientX - r.left, sy: e.clientY - r.top };
  };

  const dispatch = (kind: MapPointerKind, screen: ScreenPoint, e: { button: number; shiftKey: boolean }) => {
    const api = makeApi();
    const ev: MapPointerEvent = { kind, screen, world: api.screenToWorld(screen), button: e.button, shiftKey: e.shiftKey, api };
    if (s.captured) return { consumed: !!s.captured.onPointer?.(ev), ev };
    for (let i = s.overlays.length - 1; i >= 0; i--) {
      if (s.overlays[i].onPointer?.(ev)) return { consumed: true, ev, by: s.overlays[i] };
    }
    return { consumed: false, ev, by: null };
  };

  const updateCursor = (ev: MapPointerEvent) => {
    let c: string | null = null;
    for (let i = s.overlays.length - 1; i >= 0 && !c; i--) c = s.overlays[i].cursor?.(ev) ?? null;
    canvasRef.current!.style.cursor = s.panning && s.dragged ? 'grabbing' : (c ?? 'grab');
  };

  const onPointerDown = (e: React.PointerEvent) => {
    const p = local(e);
    canvasRef.current!.setPointerCapture(e.pointerId);
    s.pointers.set(e.pointerId, p);
    if (s.pointers.size === 2) {
      // Second finger: switch to pinch, cancel any click/drag in progress.
      const [a, b] = [...s.pointers.values()];
      s.pinch = { dist: Math.hypot(a.sx - b.sx, a.sy - b.sy), mid: { sx: (a.sx + b.sx) / 2, sy: (a.sy + b.sy) / 2 } };
      s.panning = false;
      s.dragged = true;
      s.captured = null;
      return;
    }
    if (s.pointers.size > 2) return;
    s.downAt = p;
    s.dragged = false;
    s.captured = null;
    const { by } = dispatch('down', p, e);
    s.captured = by ?? null;
    s.panning = !by && e.button === 0;
  };

  const onPointerMove = (e: React.PointerEvent) => {
    const p = local(e);
    const prev = s.pointers.get(e.pointerId);
    if (prev) s.pointers.set(e.pointerId, p);

    if (s.pinch && s.pointers.size >= 2) {
      const [a, b] = [...s.pointers.values()];
      const dist = Math.hypot(a.sx - b.sx, a.sy - b.sy);
      const mid = { sx: (a.sx + b.sx) / 2, sy: (a.sy + b.sy) / 2 };
      const v = zoomAt(s.view, mid, dist / (s.pinch.dist || 1), MIN_ZOOM, MAX_ZOOM);
      setView({ ...v, panX: v.panX + mid.sx - s.pinch.mid.sx, panY: v.panY + mid.sy - s.pinch.mid.sy });
      s.pinch = { dist, mid };
      return;
    }

    if (s.downAt && Math.hypot(p.sx - s.downAt.sx, p.sy - s.downAt.sy) > DRAG_PX) s.dragged = true;
    const { ev } = dispatch('move', p, e);
    if (s.panning && prev && s.dragged) {
      setView({ ...s.view, panX: s.view.panX + p.sx - prev.sx, panY: s.view.panY + p.sy - prev.sy });
    }
    updateCursor(ev);
    if (readoutRef.current) {
      readoutRef.current.textContent = s.info ? `x ${ev.world.x.toFixed(2)} m · y ${ev.world.y.toFixed(2)} m` : '';
    }
  };

  const onPointerUp = (e: React.PointerEvent) => {
    const p = local(e);
    const wasPinch = !!s.pinch;
    s.pointers.delete(e.pointerId);
    if (wasPinch) {
      if (s.pointers.size < 2) s.pinch = null;
      if (s.pointers.size === 0) s.downAt = null;
      return;
    }
    if (!s.downAt) return;
    const { ev } = dispatch('up', p, e);
    if (!s.dragged) dispatch('click', p, e);
    s.downAt = null;
    s.panning = false;
    s.captured = null;
    updateCursor(ev);
  };

  const onPointerCancel = (e: React.PointerEvent) => {
    s.pointers.delete(e.pointerId);
    if (s.pointers.size < 2) s.pinch = null;
    s.downAt = null;
    s.panning = false;
    s.captured = null;
  };

  const onDoubleClick = (e: React.MouseEvent) => {
    dispatch('dblclick', local(e), e);
  };

  const onContextMenu = (e: React.MouseEvent) => {
    e.preventDefault();
    dispatch('contextmenu', local(e), e);
  };

  // Wheel needs a non-passive native listener to preventDefault page scroll / browser zoom.
  useEffect(() => {
    const canvas = canvasRef.current!;
    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      const dy = e.deltaMode === 1 ? e.deltaY * 16 : e.deltaMode === 2 ? e.deltaY * 400 : e.deltaY;
      // ctrlKey = trackpad pinch: small deltas, so a stronger factor.
      const factor = Math.exp(-dy * (e.ctrlKey ? 0.01 : 0.0015));
      setView(zoomAt(s.view, local(e), factor, MIN_ZOOM, MAX_ZOOM));
    };
    canvas.addEventListener('wheel', onWheel, { passive: false });
    return () => canvas.removeEventListener('wheel', onWheel);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const zoomCentre = (factor: number) => setView(zoomAt(s.view, { sx: s.w / 2, sy: s.h / 2 }, factor, MIN_ZOOM, MAX_ZOOM));

  const btn =
    'grid size-9 place-items-center text-slate-200 transition-colors hover:bg-white/10 hover:text-white focus-visible:outline-2 focus-visible:outline-accent';

  return (
    <div ref={wrapRef} className="absolute inset-0 overflow-hidden bg-surface select-none">
      <canvas
        ref={canvasRef}
        className="absolute inset-0 size-full touch-none"
        style={{ cursor: 'grab' }}
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        onPointerCancel={onPointerCancel}
        onPointerLeave={() => readoutRef.current && (readoutRef.current.textContent = '')}
        onDoubleClick={onDoubleClick}
        onContextMenu={onContextMenu}
      />

      {!hasMap && (
        <div className="pointer-events-none absolute inset-0 grid place-items-center">
          <div className="flex items-center gap-3 rounded-xl border border-border bg-panel/80 px-4 py-3 text-sm text-muted backdrop-blur">
            <span className="size-2 animate-pulse rounded-full bg-accent" />
            Wachten op {mapTopic}…
          </div>
        </div>
      )}

      <div className="absolute right-3 top-3 flex flex-col overflow-hidden rounded-xl border border-border bg-panel/85 shadow-lg backdrop-blur divide-y divide-border">
        <button type="button" className={btn} title="Inzoomen" aria-label="Inzoomen" onClick={() => zoomCentre(1.4)}>
          <svg viewBox="0 0 24 24" className="size-4" fill="none" stroke="currentColor" strokeWidth={2}><path d="M12 5v14M5 12h14" /></svg>
        </button>
        <button type="button" className={btn} title="Uitzoomen" aria-label="Uitzoomen" onClick={() => zoomCentre(1 / 1.4)}>
          <svg viewBox="0 0 24 24" className="size-4" fill="none" stroke="currentColor" strokeWidth={2}><path d="M5 12h14" /></svg>
        </button>
        <button type="button" className={btn} title="Kaart passend maken" aria-label="Kaart passend maken" onClick={fit}>
          <svg viewBox="0 0 24 24" className="size-4" fill="none" stroke="currentColor" strokeWidth={2}>
            <path d="M4 9V4h5M20 9V4h-5M4 15v5h5M20 15v5h-5" />
          </svg>
        </button>
      </div>

      {hasMap && (
        <div className="pointer-events-none absolute bottom-3 left-3 flex items-end gap-3 rounded-lg border border-border bg-panel/85 px-3 py-2 text-[11px] text-muted tabular-nums backdrop-blur">
          <div className="flex flex-col gap-1">
            <span ref={scaleTextRef} />
            <div ref={scaleBarRef} className="h-1.5 border-x-2 border-b-2 border-slate-300" />
          </div>
          <span ref={readoutRef} className="min-w-[9.5rem]" />
        </div>
      )}
    </div>
  );
}
