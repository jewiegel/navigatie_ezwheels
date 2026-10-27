import type { BuzzerPattern, IndicatorState, TurnMsg } from '../../types';

export type Unknown = 'onbekend';
export const UNKNOWN: Unknown = 'onbekend';

const INDICATORS: readonly IndicatorState[] = ['links', 'rechts', 'gevaar', 'uit'];
const BUZZER: readonly BuzzerPattern[] = ['enkel', 'dubbel', 'elke_5s', 'uit'];

/** Raw /indicators string → state; null/unexpected → 'onbekend'. */
export function parseIndicator(raw: unknown): IndicatorState | Unknown {
  return INDICATORS.includes(raw as IndicatorState) ? (raw as IndicatorState) : UNKNOWN;
}

export function parseBuzzer(raw: unknown): BuzzerPattern | Unknown {
  return BUZZER.includes(raw as BuzzerPattern) ? (raw as BuzzerPattern) : UNKNOWN;
}

/** Which lamps are lit (blinking) for an indicator state. */
export function lamps(s: IndicatorState | Unknown): { left: boolean; right: boolean } {
  return { left: s === 'links' || s === 'gevaar', right: s === 'rechts' || s === 'gevaar' };
}

export const INDICATOR_LABEL: Record<IndicatorState | Unknown, string> = {
  links: 'Links',
  rechts: 'Rechts',
  gevaar: 'Alarmlichten',
  uit: 'Uit',
  onbekend: 'Onbekend',
};

/** Meaning per Horn.py: enkel = vertrek/aankomst, dubbel = wachten op operator of fout. */
export const BUZZER_LABEL: Record<BuzzerPattern | Unknown, string> = {
  enkel: 'Enkel (vertrek/aankomst)',
  dubbel: 'Dubbel (wacht op operator/fout)',
  elke_5s: 'Elke 5 s (blokkade)',
  uit: 'Uit',
  onbekend: 'Onbekend',
};

export const isOneShot = (p: BuzzerPattern | Unknown): p is 'enkel' | 'dubbel' => p === 'enkel' || p === 'dubbel';

/** Repeating buzzer mode after pattern p: one-shots don't change it (Horn only sends them while nothing repeats). */
export function nextRepeating(prev: 'elke_5s' | 'uit' | Unknown, p: BuzzerPattern | Unknown): 'elke_5s' | 'uit' | Unknown {
  return p === 'elke_5s' || p === 'uit' ? p : isOneShot(p) ? 'uit' : prev;
}

export interface SignalEvent {
  at: number; // ms epoch
  source: 'indicators' | 'buzzer';
  value: string;
}

/** Newest first, at most max entries. */
export function appendEvent(log: readonly SignalEvent[], e: SignalEvent, max = 5): SignalEvent[] {
  return [e, ...log].slice(0, Math.max(0, max));
}

export function eventText(e: SignalEvent): string {
  return e.source === 'indicators'
    ? `Knipperlichten: ${INDICATOR_LABEL[parseIndicator(e.value)]}`
    : `Buzzer: ${BUZZER_LABEL[parseBuzzer(e.value)]}`;
}

/** "zojuist", "3 s geleden", "2 min geleden", "1 u geleden". */
export function relativeTime(at: number, now: number): string {
  const s = Math.floor((now - at) / 1000);
  if (!Number.isFinite(s)) return UNKNOWN;
  if (s < 1) return 'zojuist';
  if (s < 60) return `${s} s geleden`;
  if (s < 3600) return `${Math.floor(s / 60)} min geleden`;
  return `${Math.floor(s / 3600)} u geleden`;
}

const DIR: Record<number, { label: string; arrow: string }> = {
  0: { label: 'Geen', arrow: '↑' },
  1: { label: 'Links', arrow: '←' },
  2: { label: 'Rechts', arrow: '→' },
};
export const direction = (d: unknown) => DIR[d as number] ?? { label: 'Onbekend', arrow: '?' };

export interface TurnView {
  upcoming: { label: string; arrow: string };
  distanceM: string | null; // null when no upcoming turn
  angleDeg: string | null;
  turning: { label: string; arrow: string };
}

/** Turn.angle is rad (+ = links); shown as absolute degrees next to the direction. */
export function turnView(t: TurnMsg | null): TurnView | null {
  if (!t) return null;
  const has = t.direction === 1 || t.direction === 2;
  const num = (n: number, f: (n: number) => string) => (has && Number.isFinite(n) ? f(n) : null);
  return {
    upcoming: direction(t.direction),
    distanceM: num(t.distance, (n) => n.toFixed(1)),
    angleDeg: num(t.angle, (n) => Math.round(Math.abs((n * 180) / Math.PI)).toString()),
    turning: direction(t.turning),
  };
}

export function hmiState(msg: { data: boolean } | null): { tone: 'ok' | 'err' | 'muted'; label: string } {
  if (typeof msg?.data !== 'boolean') return { tone: 'muted', label: 'Onbekend' };
  return msg.data ? { tone: 'ok', label: 'Verbonden' } : { tone: 'err', label: 'Verbroken' };
}
