import type { MissionState } from '../status/statusLogic';
import type { Point } from '../../types';

export interface MissionProgress {
  passed: number;
  total: number;
  /** 0..1, null when the total is unknown. */
  fraction: number | null;
  /** 1-based number of the waypoint being driven to, null when not driving. */
  current: number | null;
}

/**
 * Progress of a route driven by mission_manager. `remaining` comes from /remaining_waypoints
 * (the hospital BT publishes the goals still to go, current target first).
 */
export function missionProgress(state: MissionState, total: number | null, remaining: number | null): MissionProgress {
  const t = total ?? 0;
  if (state === 'ARRIVED') return { passed: t, total: t, fraction: total ? 1 : null, current: null };
  if (!total || remaining === null) return { passed: 0, total: t, fraction: null, current: null };
  const passed = Math.min(total, Math.max(0, total - remaining));
  const driving = state === 'DRIVING' || state === 'BLOCKED' || state === 'WAITING_OPERATOR';
  return { passed, total, fraction: passed / total, current: driving && passed < total ? passed + 1 : null };
}

/**
 * Straight-line distances from the robot along the remaining waypoints (current target first).
 * shortcut: hemelsbreed, not along the planned path; good enough for a progress hint.
 */
export function remainingDistance(robot: Point | null, remaining: Point[]): { toNext: number | null; total: number | null } {
  if (!robot || remaining.length === 0) return { toNext: null, total: null };
  const toNext = Math.hypot(remaining[0].x - robot.x, remaining[0].y - robot.y);
  let total = toNext;
  for (let i = 1; i < remaining.length; i++) total += Math.hypot(remaining[i].x - remaining[i - 1].x, remaining[i].y - remaining[i - 1].y);
  return { toNext, total };
}

/** "45 s", "2 min 05 s", "1 u 03 min". */
export function formatDuration(seconds: number | null): string {
  if (seconds === null || !Number.isFinite(seconds) || seconds < 0) return '–';
  const s = Math.round(seconds);
  if (s < 60) return `${s} s`;
  if (s < 3600) return `${Math.floor(s / 60)} min ${String(s % 60).padStart(2, '0')} s`;
  return `${Math.floor(s / 3600)} u ${String(Math.floor((s % 3600) / 60)).padStart(2, '0')} min`;
}
