import type { Quaternion, Time } from '../types';

/** Rotation of `yaw` radians about +z. */
export function yawToQuaternion(yaw: number): Quaternion {
  return { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) };
}

/** Yaw (rotation about +z) of a quaternion, in (-pi, pi]. */
export function quaternionToYaw(q: Quaternion): number {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

/** Browser wall-clock time as builtin_interfaces/Time. */
export function rosTimeNow(): Time {
  const ms = Date.now();
  return { sec: Math.floor(ms / 1000), nanosec: (ms % 1000) * 1e6 };
}
