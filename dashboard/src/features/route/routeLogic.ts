import type { GoalStatus, MapViewApi, NavStatus, Point, Pose2D, PoseStamped, ScreenPoint, Time, Waypoint } from '../../types';
import { yawToQuaternion } from '../../ros/convert';

export interface Duration {
  sec: number;
  nanosec: number;
}

/** nav2_msgs/action/NavigateThroughPoses feedback (Iron). */
export interface NavigateThroughPosesFeedback {
  current_pose: PoseStamped;
  navigation_time: Duration;
  estimated_time_remaining: Duration;
  number_of_recoveries: number;
  distance_remaining: number;
  number_of_poses_remaining: number;
}

export interface NavigateThroughPosesGoal {
  poses: PoseStamped[];
  behavior_tree: string;
}

let seq = 0;
// crypto.randomUUID is unavailable on plain-http LAN origins, so use a counter.
const newId = () => `wp-${Date.now().toString(36)}-${(seq++).toString(36)}`;

/** yaw_k points towards the next waypoint; the last one keeps the previous segment's heading. */
export function computeYaws(list: Waypoint[]): Waypoint[] {
  const n = list.length;
  if (n < 2) return list.map((w) => ({ ...w }));
  // heading of segment k (k -> k+1), null when zero-length
  const seg = list.slice(0, -1).map((w, k) => {
    const dx = list[k + 1].x - w.x;
    const dy = list[k + 1].y - w.y;
    return Math.hypot(dx, dy) < 1e-9 ? null : Math.atan2(dy, dx);
  });
  const firstValid = seg.find((h) => h !== null) ?? list[0].yaw;
  const yaws: number[] = [];
  for (let k = 0; k < n - 1; k++) yaws.push(seg[k] ?? (k > 0 ? yaws[k - 1] : firstValid));
  yaws.push(yaws[n - 2]);
  return list.map((w, k) => ({ ...w, yaw: yaws[k] }));
}

/** Replace the list with saved poses: fresh ids, yaws kept exactly as given (no recompute). */
export function loadWaypoints(points: Pose2D[]): Waypoint[] {
  return points.map((p) => ({ id: newId(), x: p.x, y: p.y, yaw: p.yaw }));
}

export function addWaypoint(list: Waypoint[], p: Point, id: string = newId()): Waypoint[] {
  return computeYaws([...list, { id, x: p.x, y: p.y, yaw: 0 }]);
}

export function moveWaypoint(list: Waypoint[], id: string, p: Point): Waypoint[] {
  return computeYaws(list.map((w) => (w.id === id ? { ...w, x: p.x, y: p.y } : w)));
}

export function removeWaypoint(list: Waypoint[], id: string): Waypoint[] {
  return computeYaws(list.filter((w) => w.id !== id));
}

export function reorderWaypoints(list: Waypoint[], from: number, to: number): Waypoint[] {
  const n = list.length;
  if (from < 0 || from >= n || to < 0 || to >= n) return computeYaws(list);
  const out = [...list];
  const [w] = out.splice(from, 1);
  out.splice(to, 0, w);
  return computeYaws(out);
}

/** Id of the waypoint closest to `screen` within radiusPx, or null. */
export function hitTest(list: Waypoint[], screen: ScreenPoint, api: Pick<MapViewApi, 'worldToScreen'>, radiusPx: number): string | null {
  let best: string | null = null;
  let bestD = radiusPx;
  for (const w of list) {
    const s = api.worldToScreen(w);
    const d = Math.hypot(s.sx - screen.sx, s.sy - screen.sy);
    if (d <= bestD) {
      bestD = d;
      best = w.id;
    }
  }
  return best;
}

export function toNavigateThroughPosesGoal(list: Waypoint[], frameId = 'map', stamp: Time = { sec: 0, nanosec: 0 }): NavigateThroughPosesGoal {
  return {
    poses: list.map((w) => ({
      header: { stamp, frame_id: frameId },
      pose: { position: { x: w.x, y: w.y, z: 0 }, orientation: yawToQuaternion(w.yaw) },
    })),
    behavior_tree: '',
  };
}

const MESSAGES: Partial<Record<GoalStatus, string>> = {
  sending: 'Route wordt verstuurd…',
  active: 'Onderweg',
  succeeded: 'Route bereikt',
  canceled: 'Route geannuleerd',
  aborted: 'Navigatie mislukt',
  rejected: 'Doel geweigerd door Nav2',
  error: 'Fout bij navigatie',
};

export function navStatusFromFeedback(
  goal: GoalStatus,
  totalWaypoints: number,
  feedback: NavigateThroughPosesFeedback | null,
  error: string | null = null,
): NavStatus {
  const eta = feedback?.estimated_time_remaining;
  return {
    goal,
    totalWaypoints,
    remainingWaypoints: feedback?.number_of_poses_remaining ?? null,
    distanceRemaining: feedback?.distance_remaining ?? null,
    etaSeconds: eta ? eta.sec + eta.nanosec / 1e9 : null,
    recoveries: feedback?.number_of_recoveries ?? null,
    message: error ?? MESSAGES[goal] ?? null,
  };
}

/** True while a goal is in flight (editing disabled). */
export const isGoalBusy = (g: GoalStatus) => g === 'sending' || g === 'active';
