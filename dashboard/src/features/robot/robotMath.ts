import type { Pose2D, PoseWithCovarianceStamped } from '../../types';
import { quaternionToYaw } from '../../ros/convert';

/** Map-frame pose from an AMCL message, or null if it is malformed. */
export function poseFromMsg(msg: PoseWithCovarianceStamped | null): Pose2D | null {
  const p = msg?.pose?.pose;
  if (!p) return null;
  const pose = { x: p.position.x, y: p.position.y, yaw: quaternionToYaw(p.orientation) };
  return Number.isFinite(pose.x) && Number.isFinite(pose.y) && Number.isFinite(pose.yaw) ? pose : null;
}

/** 1-sigma ellipse of the x/y block of a row-major 6x6 covariance. angle: major axis, radians CCW from +x (world). */
export interface Ellipse {
  major: number; // m
  minor: number; // m
  angle: number;
}

export function covarianceEllipse(cov: readonly number[] | undefined): Ellipse | null {
  if (!cov || cov.length < 8) return null;
  const a = cov[0], b = cov[1], c = cov[7];
  if (![a, b, c].every(Number.isFinite)) return null;
  const mid = (a + c) / 2;
  const r = Math.hypot((a - c) / 2, b);
  return {
    major: Math.sqrt(Math.max(0, mid + r)),
    minor: Math.sqrt(Math.max(0, mid - r)),
    angle: 0.5 * Math.atan2(2 * b, a - c),
  };
}
