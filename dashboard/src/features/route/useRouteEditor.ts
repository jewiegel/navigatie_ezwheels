import { useEffect, useMemo, useRef, useState } from 'react';
import type { MapOverlay, MapPointerEvent, MapViewApi, NavStatus, Point, Pose2D, RouteEditor, Waypoint } from '../../types';
import { useAction } from '../../ros';
import { rosTimeNow } from '../../ros/convert';
import {
  addWaypoint,
  hitTest,
  loadWaypoints,
  isGoalBusy,
  moveWaypoint,
  navStatusFromFeedback,
  removeWaypoint,
  reorderWaypoints,
  toNavigateThroughPosesGoal,
  type NavigateThroughPosesFeedback,
  type NavigateThroughPosesGoal,
} from './routeLogic';

const HIT_RADIUS_PX = 14;
const MARKER_R = 11;
const ACCENT = '#4f8cff';

export function useRouteEditor(): RouteEditor {
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [sentCount, setSentCount] = useState(0);
  // Note: mission_manager in this repo also owns a NavigateThroughPoses client; sending here preempts its goal (known trade-off, user's choice).
  const action = useAction<NavigateThroughPosesGoal, NavigateThroughPosesFeedback, unknown>(
    '/navigate_through_poses',
    'nav2_msgs/action/NavigateThroughPoses',
  );
  const nav = useMemo(
    () => navStatusFromFeedback(action.status, sentCount, action.feedback, action.error),
    [action.status, sentCount, action.feedback, action.error],
  );
  const busy = isGoalBusy(nav.goal);

  // The overlay object is stable; it reads current state through refs.
  const state = useRef({ waypoints, nav, busy });
  state.current = { waypoints, nav, busy };
  const drag = useRef<string | null>(null);
  const lastApi = useRef<MapViewApi | null>(null);

  useEffect(() => lastApi.current?.requestRedraw(), [waypoints, nav]);

  const editor = useMemo(() => {
    const edit = (fn: (l: Waypoint[]) => Waypoint[]) => {
      if (!state.current.busy) setWaypoints(fn);
    };
    const hit = (e: MapPointerEvent) => hitTest(state.current.waypoints, e.screen, e.api, HIT_RADIUS_PX);

    const overlay: MapOverlay = {
      id: 'route',
      draw(ctx, api) {
        lastApi.current = api;
        drawRoute(ctx, api, state.current.waypoints, state.current.nav);
      },
      onPointer(e) {
        lastApi.current = e.api;
        if (state.current.busy) return false;
        switch (e.kind) {
          case 'down': {
            const id = hit(e);
            if (id === null || e.button !== 0) return id !== null;
            if (!e.shiftKey) drag.current = id;
            return true;
          }
          case 'move':
            if (drag.current === null) return false;
            edit((l) => moveWaypoint(l, drag.current!, e.world));
            return true;
          case 'up': {
            const was = drag.current !== null;
            drag.current = null;
            return was;
          }
          case 'click': {
            if (e.button !== 0) return false;
            const id = hit(e);
            if (id === null) edit((l) => addWaypoint(l, e.world));
            else if (e.shiftKey) edit((l) => removeWaypoint(l, id));
            return true;
          }
          case 'contextmenu': {
            const id = hit(e);
            if (id === null) return false;
            edit((l) => removeWaypoint(l, id));
            return true;
          }
          default:
            return false;
        }
      },
      cursor(e) {
        if (state.current.busy) return null;
        if (drag.current !== null) return 'grabbing';
        return hit(e) !== null ? 'grab' : 'crosshair';
      },
    };

    return {
      overlay,
      add: (p: Point) => edit((l) => addWaypoint(l, p)),
      move: (id: string, p: Point) => edit((l) => moveWaypoint(l, id, p)),
      remove: (id: string) => edit((l) => removeWaypoint(l, id)),
      reorder: (from: number, to: number) => edit((l) => reorderWaypoints(l, from, to)),
      clear: () => edit(() => []),
      // redraw follows from the waypoints effect
      load: (points: Pose2D[]) => edit(() => loadWaypoints(points)),
    };
  }, []);

  return {
    ...editor,
    waypoints,
    nav,
    send() {
      if (busy || waypoints.length === 0) return;
      setSentCount(waypoints.length);
      action.send(toNavigateThroughPosesGoal(waypoints, lastApi.current?.info?.frameId || 'map', rosTimeNow()));
    },
    cancel: () => action.cancel(),
  };
}

function drawRoute(ctx: CanvasRenderingContext2D, api: MapViewApi, list: Waypoint[], nav: NavStatus) {
  if (list.length === 0) return;
  const pts = list.map((w) => api.worldToScreen(w));
  // waypoints before index `passed` are already reached during the current goal
  const passed =
    isGoalBusy(nav.goal) && nav.remainingWaypoints !== null ? Math.max(0, nav.totalWaypoints - nav.remainingWaypoints) : 0;

  ctx.save();
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';

  for (let k = 0; k < pts.length - 1; k++) {
    const a = pts[k];
    const b = pts[k + 1];
    ctx.globalAlpha = k + 1 <= passed ? 0.3 : 0.9;
    ctx.strokeStyle = ACCENT;
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(a.sx, a.sy);
    ctx.lineTo(b.sx, b.sy);
    ctx.stroke();

    // direction chevron at the segment midpoint, only if there is room
    const len = Math.hypot(b.sx - a.sx, b.sy - a.sy);
    if (len > MARKER_R * 4) {
      const ang = Math.atan2(b.sy - a.sy, b.sx - a.sx);
      ctx.save();
      ctx.translate((a.sx + b.sx) / 2, (a.sy + b.sy) / 2);
      ctx.rotate(ang);
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(-4, -5);
      ctx.lineTo(3, 0);
      ctx.lineTo(-4, 5);
      ctx.stroke();
      ctx.restore();
    }
  }

  ctx.font = '600 11px Inter, system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  list.forEach((w, k) => {
    const s = pts[k];
    ctx.globalAlpha = k < passed ? 0.35 : 1;
    // heading tick, projected through worldToScreen so any y-flip/rotation of the view is respected
    const ahead = api.worldToScreen({ x: w.x + Math.cos(w.yaw), y: w.y + Math.sin(w.yaw) });
    const norm = Math.hypot(ahead.sx - s.sx, ahead.sy - s.sy) || 1;
    const tx = (ahead.sx - s.sx) / norm;
    const ty = (ahead.sy - s.sy) / norm;
    ctx.strokeStyle = ACCENT;
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(s.sx + tx * MARKER_R, s.sy + ty * MARKER_R);
    ctx.lineTo(s.sx + tx * (MARKER_R + 7), s.sy + ty * (MARKER_R + 7));
    ctx.stroke();

    ctx.fillStyle = ACCENT;
    ctx.strokeStyle = '#0f1115';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(s.sx, s.sy, MARKER_R, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = '#ffffff';
    ctx.fillText(String(k + 1), s.sx, s.sy + 0.5);
  });
  ctx.restore();
}
