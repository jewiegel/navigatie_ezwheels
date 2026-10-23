import type { ConnectionState, DiagnosticStatus, GoalStatus } from '../../types';

/** Semantic tone, mapped to theme colours in the components. */
export type Tone = 'ok' | 'warn' | 'err' | 'accent' | 'muted';

export const CONNECTION_LABEL: Record<ConnectionState, { label: string; tone: Tone }> = {
  connected: { label: 'Verbonden', tone: 'ok' },
  connecting: { label: 'Verbinden…', tone: 'warn' },
  disconnected: { label: 'Verbroken', tone: 'muted' },
  error: { label: 'Fout', tone: 'err' },
};

export const GOAL_LABEL: Record<GoalStatus, { label: string; tone: Tone }> = {
  idle: { label: 'Inactief', tone: 'muted' },
  sending: { label: 'Versturen…', tone: 'accent' },
  active: { label: 'Actief', tone: 'accent' },
  succeeded: { label: 'Bereikt', tone: 'ok' },
  canceled: { label: 'Geannuleerd', tone: 'warn' },
  aborted: { label: 'Mislukt', tone: 'err' },
  rejected: { label: 'Geweigerd', tone: 'err' },
  error: { label: 'Fout', tone: 'err' },
};

// ---------- mission_status ----------

export type MissionState = 'IDLE' | 'DRIVING' | 'BLOCKED' | 'WAITING_OPERATOR' | 'ARRIVED' | 'ERROR' | 'UNKNOWN';

const MISSION_LABEL: Record<MissionState, { label: string; tone: Tone }> = {
  IDLE: { label: 'Inactief', tone: 'muted' },
  DRIVING: { label: 'Rijdt', tone: 'accent' },
  BLOCKED: { label: 'Geblokkeerd', tone: 'err' },
  WAITING_OPERATOR: { label: 'Wacht op operator', tone: 'warn' },
  ARRIVED: { label: 'Aangekomen', tone: 'ok' },
  ERROR: { label: 'Fout', tone: 'err' },
  UNKNOWN: { label: 'Onbekend', tone: 'muted' },
};

export interface Mission {
  state: MissionState;
  raw: string; // original state token (useful when UNKNOWN)
  route: string;
  label: string;
  tone: Tone;
}

/** Parses "<STATE> <route>" (route may contain spaces or be empty). */
export function parseMissionStatus(text: string): Mission {
  const trimmed = text.trim();
  const i = trimmed.search(/\s/);
  const raw = (i < 0 ? trimmed : trimmed.slice(0, i)).toUpperCase();
  const route = i < 0 ? '' : trimmed.slice(i).trim();
  const state = (raw in MISSION_LABEL ? raw : 'UNKNOWN') as MissionState;
  return { state, raw, route, ...MISSION_LABEL[state] };
}

// ---------- battery ----------

export const POWER_SUPPLY_LABEL: Record<number, string> = {
  0: 'Onbekend',
  1: 'Laadt op',
  2: 'Ontlaadt',
  3: 'Laadt niet',
  4: 'Vol',
};

/** percentage is 0..1 per sensor_msgs/BatteryState; NaN/out of range → null. */
export function batteryPercent(percentage: number): number | null {
  if (!Number.isFinite(percentage) || percentage < 0) return null;
  // Some drivers publish 0..100 despite the spec.
  const p = percentage > 1 ? percentage : percentage * 100;
  return Math.min(100, Math.round(p));
}

export function batteryLevel(percent: number | null): { label: string; tone: Tone } {
  if (percent === null) return { label: 'Onbekend', tone: 'muted' };
  if (percent <= 15) return { label: 'Kritiek', tone: 'err' };
  if (percent <= 35) return { label: 'Laag', tone: 'warn' };
  return { label: 'Goed', tone: 'ok' };
}

// ---------- diagnostics ----------

export const DIAG_LEVELS = [
  { level: 0, label: 'OK', tone: 'ok' },
  { level: 1, label: 'WARN', tone: 'warn' },
  { level: 2, label: 'ERROR', tone: 'err' },
  { level: 3, label: 'STALE', tone: 'muted' },
] as const satisfies readonly { level: number; label: string; tone: Tone }[];

export function diagLevel(level: number): { label: string; tone: Tone } {
  return DIAG_LEVELS[level] ?? { label: `?${level}`, tone: 'muted' };
}

/**
 * ROS 2 `level` is a `byte`; depending on version rosbridge sends it as a number, a numeric
 * string ("2") or base64 of one byte ("Ag=="). Anything not resolving to 0..3 becomes 3 (STALE).
 */
export function normalizeLevel(raw: unknown): number {
  let n = NaN;
  if (typeof raw === 'number') n = raw;
  else if (typeof raw === 'string') {
    if (/^\s*\d+\s*$/.test(raw)) n = Number(raw);
    else {
      try {
        const bytes = atob(raw);
        if (bytes.length === 1) n = bytes.charCodeAt(0);
      } catch {
        // not base64
      }
    }
  }
  // Range check is load-bearing: atob is lenient (atob('zz') is one byte, 207).
  return Number.isInteger(n) && n >= 0 && n <= 3 ? n : 3;
}

/** Latest status per name, aggregated across messages. Returns a new object (or the same one if nothing to merge). */
export function mergeDiagnostics(
  prev: Record<string, DiagnosticStatus>,
  incoming: readonly DiagnosticStatus[],
): Record<string, DiagnosticStatus> {
  if (incoming.length === 0) return prev;
  const next = { ...prev };
  for (const s of incoming) next[s.name] = { ...s, level: normalizeLevel(s.level as unknown) };
  return next;
}

export function countByLevel(all: Record<string, DiagnosticStatus>): [number, number, number, number] {
  const c: [number, number, number, number] = [0, 0, 0, 0];
  for (const s of Object.values(all)) if (s.level >= 0 && s.level <= 3) c[s.level]++;
  return c;
}

/** Non-OK items, worst first, then by name. */
export function problems(all: Record<string, DiagnosticStatus>): DiagnosticStatus[] {
  return Object.values(all)
    .filter((s) => s.level !== 0)
    .sort((a, b) => {
      // STALE (3) ranks between WARN and ERROR in severity.
      const rank = (l: number) => (l === 2 ? 0 : l === 3 ? 1 : 2);
      return rank(a.level) - rank(b.level) || a.name.localeCompare(b.name);
    });
}
