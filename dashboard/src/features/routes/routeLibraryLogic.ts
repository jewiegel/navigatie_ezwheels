import type { Pose2D, RouteWaypointMsg } from '../../types';

export const NAME_RE = /^[A-Za-z0-9_-]+$/;
export const POS_EPS = 1e-3; // m
export const YAW_EPS = 1e-3; // rad

/** Dutch error text, or null when the name is valid. */
export function validateRouteName(name: string): string | null {
  const n = name.trim();
  if (!n) return 'Geef een naam op';
  if (!NAME_RE.test(n)) return 'Alleen letters, cijfers, _ en - toegestaan';
  return null;
}

export function toWaypointMsgs(points: Pose2D[]): RouteWaypointMsg[] {
  return points.map((p, i) => ({ name: `wp${i + 1}`, x: p.x, y: p.y, yaw: p.yaw }));
}

export function fromWaypointMsgs(msgs: RouteWaypointMsg[]): Pose2D[] {
  return msgs.map(({ x, y, yaw }) => ({ x, y, yaw }));
}

function angleDiff(a: number, b: number): number {
  const d = Math.abs(a - b) % (2 * Math.PI);
  return d > Math.PI ? 2 * Math.PI - d : d;
}

/** True when the editor's points differ from the saved snapshot. */
export function hasUnsavedChanges(saved: Pose2D[], current: Pose2D[]): boolean {
  if (saved.length !== current.length) return true;
  return saved.some(
    (s, i) =>
      // !(d <= eps) so NaN counts as a change
      !(Math.abs(s.x - current[i].x) <= POS_EPS) ||
      !(Math.abs(s.y - current[i].y) <= POS_EPS) ||
      !(angleDiff(s.yaw, current[i].yaw) <= YAW_EPS),
  );
}

/** JSON turns NaN/Infinity into null, which /set_route would reject. */
export function allFinite(points: Pose2D[]): boolean {
  return points.every((p) => Number.isFinite(p.x) && Number.isFinite(p.y) && Number.isFinite(p.yaw));
}

export function sortRouteNames(names: string[]): string[] {
  return [...names].sort((a, b) => a.localeCompare(b, 'nl', { numeric: true, sensitivity: 'base' }));
}

/** Turns a useService rejection into a clear Dutch message. */
export function serviceErrorText(service: string, err: unknown, node = 'route_manager'): string {
  const msg = err instanceof Error ? err.message : String(err);
  if (/not connected/i.test(msg)) return 'Niet verbonden met de robot';
  if (/timed out|does not exist|not available|unknown|no such/i.test(msg))
    return `Service ${service} niet beschikbaar — herstart ${node}`;
  return `Fout bij ${service}: ${msg}`;
}
