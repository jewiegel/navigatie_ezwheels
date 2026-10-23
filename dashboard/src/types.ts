// Shared contract between the dashboard features. Owned by the lead:
// teammates request changes instead of editing this file.

// ---------- geometry ----------

/** Point in the map frame, metres. */
export interface Point {
  x: number;
  y: number;
}

/** Pose in the map frame: metres + yaw in radians (CCW from +x). */
export interface Pose2D extends Point {
  yaw: number;
}

/** Screen point in CSS pixels, relative to the map canvas' top-left corner. */
export interface ScreenPoint {
  sx: number;
  sy: number;
}

/** Pan/zoom state of the map view. zoom = screen px per map cell. */
export interface ViewTransform {
  zoom: number;
  panX: number;
  panY: number;
}

/** Metadata of the currently loaded OccupancyGrid (nav_msgs/MapMetaData, flattened). */
export interface MapInfo {
  resolution: number; // m/cell
  width: number; // cells
  height: number; // cells
  origin: Pose2D; // pose of cell (0,0)'s corner in the map frame
  frameId: string;
}

// ---------- ROS message shapes (rosbridge JSON) ----------

export interface Time {
  sec: number;
  nanosec: number;
}
export interface Header {
  stamp: Time;
  frame_id: string;
}
export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}
export interface RosPose {
  position: { x: number; y: number; z: number };
  orientation: Quaternion;
}
export interface PoseStamped {
  header: Header;
  pose: RosPose;
}
export interface OccupancyGrid {
  header: Header;
  info: {
    map_load_time: Time;
    resolution: number;
    width: number;
    height: number;
    origin: RosPose;
  };
  data: number[] | Int8Array; // row-major, row 0 = bottom (y = origin.y)
}
export interface PoseWithCovarianceStamped {
  header: Header;
  pose: { pose: RosPose; covariance: number[] };
}
export interface Path {
  header: Header;
  poses: PoseStamped[];
}
export interface BatteryState {
  header: Header;
  voltage: number;
  percentage: number; // 0..1, NaN if unknown
  power_supply_status: number;
}
export interface DiagnosticStatus {
  level: number; // 0 OK, 1 WARN, 2 ERROR, 3 STALE
  name: string;
  message: string;
  hardware_id: string;
  values: { key: string; value: string }[];
}
export interface DiagnosticArray {
  header: Header;
  status: DiagnosticStatus[];
}

// ---------- ROS connection (implemented in src/ros/) ----------

export type ConnectionState = 'connecting' | 'connected' | 'disconnected' | 'error';

export interface TopicOptions {
  /** Ask rosbridge for a TRANSIENT_LOCAL subscription and keep it across reconnects (e.g. /map). */
  latched?: boolean;
  /** Server-side throttle (rosbridge throttle_rate), ms. */
  throttleMs?: number;
  /** rosbridge compression, e.g. 'cbor' for big grids. */
  compression?: 'none' | 'cbor' | 'png' | 'cbor-raw';
}

/** Lifecycle of one action goal as seen by the UI. */
export type GoalStatus = 'idle' | 'sending' | 'active' | 'succeeded' | 'canceled' | 'aborted' | 'rejected' | 'error';

export interface ActionHandle<TGoal, TFeedback, TResult> {
  status: GoalStatus;
  feedback: TFeedback | null;
  result: TResult | null;
  error: string | null;
  send(goal: TGoal): void;
  cancel(): void;
}

// Hook signatures exported from src/ros/index.ts:
//   <RosProvider url?: string>          — default url: `ws://${location.hostname}:9090`
//   useRos(): { ros: Ros | null; state: ConnectionState; url: string }
//   useTopic<T>(name, messageType, opts?: TopicOptions): T | null       (latest message)
//   usePublisher<T>(name, messageType): (msg: T) => void
//   useService<TReq, TRes>(name, serviceType): (req: TReq) => Promise<TRes>
//   useAction<TGoal, TFeedback, TResult>(name, actionType): ActionHandle<TGoal, TFeedback, TResult>

// ---------- map overlays (rendered by src/features/map/) ----------

/** What an overlay gets to draw with; provided by MapView on every frame. */
export interface MapViewApi {
  info: MapInfo | null;
  view: ViewTransform;
  worldToScreen(p: Point): ScreenPoint;
  screenToWorld(s: ScreenPoint): Point;
  /** Screen pixels per metre at the current zoom. */
  pxPerMeter: number;
  /** Ask MapView to repaint (e.g. after overlay state changed outside React). */
  requestRedraw(): void;
}

export type MapPointerKind = 'down' | 'move' | 'up' | 'click' | 'dblclick' | 'contextmenu';

export interface MapPointerEvent {
  kind: MapPointerKind;
  screen: ScreenPoint;
  world: Point;
  button: number;
  shiftKey: boolean;
  api: MapViewApi;
}

/**
 * A layer drawn on top of the map. Overlays are drawn in array order (later = on top);
 * pointer events go in reverse order and stop at the first handler returning true
 * (true = "consumed", MapView then does NOT pan).
 */
export interface MapOverlay {
  id: string;
  draw(ctx: CanvasRenderingContext2D, api: MapViewApi): void;
  onPointer?(e: MapPointerEvent): boolean;
  /** CSS cursor while hovering, if this overlay wants one. */
  cursor?(e: MapPointerEvent): string | null;
}

// MapView props (src/features/map/index.ts):
//   overlays: MapOverlay[]; mapTopic?: string (default '/map')

// ---------- route / navigation ----------

export interface Waypoint extends Pose2D {
  id: string;
}

/** Navigation state shared between route editor (producer) and status panel (consumer). */
export interface NavStatus {
  goal: GoalStatus;
  totalWaypoints: number;
  remainingWaypoints: number | null; // from NavigateThroughPoses feedback
  distanceRemaining: number | null; // m
  etaSeconds: number | null;
  recoveries: number | null;
  message: string | null;
}

/** Returned by useRouteEditor() in src/features/route/. */
export interface RouteEditor {
  waypoints: Waypoint[];
  overlay: MapOverlay;
  nav: NavStatus;
  add(p: Point): void;
  move(id: string, p: Point): void;
  remove(id: string): void;
  reorder(from: number, to: number): void;
  clear(): void;
  send(): void;
  cancel(): void;
}

// Feature exports the layout (App.tsx) relies on:
//   src/features/map/index.ts     → MapView
//   src/features/robot/index.ts   → useRobotOverlay(): MapOverlay
//   src/features/route/index.ts   → useRouteEditor(): RouteEditor, RoutePanel({ editor: RouteEditor })
//   src/features/status/index.ts  → StatusPanel({ nav: NavStatus }), ConnectionBadge()
