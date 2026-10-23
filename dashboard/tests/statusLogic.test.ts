import { describe, expect, it } from 'vitest';
import {
  batteryLevel,
  batteryPercent,
  countByLevel,
  diagLevel,
  mergeDiagnostics,
  normalizeLevel,
  parseMissionStatus,
  problems,
} from '../src/features/status/statusLogic';
import type { DiagnosticStatus } from '../src/types';

const d = (name: string, level: number, message = ''): DiagnosticStatus => ({ name, level, message, hardware_id: '', values: [] });

describe('battery', () => {
  it('converts 0..1 to percent and tolerates 0..100 drivers', () => {
    expect(batteryPercent(0.5)).toBe(50);
    expect(batteryPercent(0)).toBe(0);
    expect(batteryPercent(1)).toBe(100);
    expect(batteryPercent(0.123)).toBe(12);
    expect(batteryPercent(87)).toBe(87);
    expect(batteryPercent(150)).toBe(100);
  });
  it('NaN / negative / infinite → null', () => {
    for (const v of [NaN, -0.1, Infinity]) expect(batteryPercent(v)).toBeNull();
  });
  it('levels: critical ≤15, low ≤35, else good, unknown for null', () => {
    expect(batteryLevel(null).tone).toBe('muted');
    expect(batteryLevel(0).tone).toBe('err');
    expect(batteryLevel(15).tone).toBe('err');
    expect(batteryLevel(16).tone).toBe('warn');
    expect(batteryLevel(35).tone).toBe('warn');
    expect(batteryLevel(36).tone).toBe('ok');
    expect(batteryLevel(100).tone).toBe('ok');
  });
});

describe('diagnostics', () => {
  it('diagLevel maps 0..3 and falls back for unknown', () => {
    expect([0, 1, 2, 3].map((l) => diagLevel(l).label)).toEqual(['OK', 'WARN', 'ERROR', 'STALE']);
    expect(diagLevel(7).tone).toBe('muted');
    expect(diagLevel(-1).tone).toBe('muted');
  });

  it('mergeDiagnostics keeps latest per name, immutable, same ref on empty', () => {
    const prev = { a: d('a', 0) };
    const next = mergeDiagnostics(prev, [d('a', 2, 'hot'), d('b', 1)]);
    expect(prev.a.level).toBe(0);
    expect(next.a.level).toBe(2);
    expect(Object.keys(next).sort()).toEqual(['a', 'b']);
    expect(mergeDiagnostics(next, [])).toBe(next);
  });

  it('countByLevel counts; merge turns out-of-range levels into STALE', () => {
    const all = mergeDiagnostics({}, [d('a', 0), d('b', 1), d('c', 1), d('e', 2), d('f', 3), d('g', 9)]);
    expect(countByLevel(all)).toEqual([1, 2, 1, 2]);
    expect(countByLevel({ g: d('g', 9) })).toEqual([0, 0, 0, 0]);
  });

  it('normalizeLevel accepts number, numeric string and 1-byte base64 (rosbridge byte)', () => {
    expect(normalizeLevel(2)).toBe(2);
    expect(normalizeLevel('2')).toBe(2);
    expect(normalizeLevel(' 1 ')).toBe(1);
    expect(normalizeLevel('Ag==')).toBe(2);
    expect(normalizeLevel('AQ==')).toBe(1);
    expect(normalizeLevel('AA==')).toBe(0);
    expect(normalizeLevel('Aw==')).toBe(3);
  });

  it('normalizeLevel: anything else is STALE (3)', () => {
    for (const v of [7, -1, '9', 'zz', '', null, undefined, 1.5, NaN, 'AgI=', 'BA==', {}, [2]]) {
      expect(normalizeLevel(v)).toBe(3);
    }
  });

  it('mergeDiagnostics normalizes base64 / string levels', () => {
    const all = mergeDiagnostics({}, [
      { ...d('x', 0), level: 'Ag==' as unknown as number },
      { ...d('y', 0), level: '1' as unknown as number },
    ]);
    expect(all.x.level).toBe(2);
    expect(all.y.level).toBe(1);
    expect(countByLevel(all)).toEqual([0, 1, 1, 0]);
    expect(problems(all).map((s) => s.name)).toEqual(['x', 'y']);
  });

  it('problems: non-OK only, ERROR > STALE > WARN, then by name', () => {
    const all = mergeDiagnostics({}, [d('z', 1), d('ok', 0), d('b', 2), d('a', 2), d('s', 3), d('c', 1)]);
    expect(problems(all).map((s) => s.name)).toEqual(['a', 'b', 's', 'c', 'z']);
  });
});

describe('parseMissionStatus', () => {
  it('parses state and route (route may contain spaces)', () => {
    expect(parseMissionStatus('DRIVING Route A naar B')).toMatchObject({ state: 'DRIVING', route: 'Route A naar B', tone: 'accent' });
    expect(parseMissionStatus('  arrived   dock_1 ')).toMatchObject({ state: 'ARRIVED', raw: 'ARRIVED', route: 'dock_1', tone: 'ok' });
  });
  it('state without route, empty and unknown tokens', () => {
    expect(parseMissionStatus('IDLE')).toMatchObject({ state: 'IDLE', route: '' });
    expect(parseMissionStatus('')).toMatchObject({ state: 'UNKNOWN', route: '' });
    expect(parseMissionStatus('FLYING x')).toMatchObject({ state: 'UNKNOWN', raw: 'FLYING', route: 'x' });
  });
  it('does not treat Object prototype keys as states', () => {
    expect(parseMissionStatus('constructor').state).toBe('UNKNOWN');
    expect(parseMissionStatus('toString foo').state).toBe('UNKNOWN');
  });
});
