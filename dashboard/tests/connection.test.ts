import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import type { Ros } from 'roslib';
import { backoffMs, createConnection } from '../src/ros/connection';
import { FakeRos, resetRoslibMocks } from './mocks/roslib';
import type { ConnectionState } from '../src/types';

const URL = 'ws://robot:9090';

function setup(backoff?: (a: number) => number) {
  const states: ConnectionState[] = [];
  const conn = createConnection({
    url: URL,
    createRos: (u) => new FakeRos({ url: u }) as unknown as Ros,
    onState: (s) => states.push(s),
    ...(backoff ? { backoff } : {}),
  });
  return { conn, states, ros: () => FakeRos.last() };
}

beforeEach(() => {
  resetRoslibMocks();
  vi.useFakeTimers();
});
afterEach(() => vi.useRealTimers());

describe('backoffMs', () => {
  it('doubles from 500ms and caps at 10s', () => {
    expect([0, 1, 2, 3, 4, 5, 6, 10, 50].map((a) => backoffMs(a))).toEqual([500, 1000, 2000, 4000, 8000, 10000, 10000, 10000, 10000]);
  });
  it('is monotonic non-decreasing', () => {
    for (let a = 0; a < 40; a++) expect(backoffMs(a + 1)).toBeGreaterThanOrEqual(backoffMs(a));
  });
});

describe('createConnection', () => {
  it('starts connecting with the url, then connected', () => {
    const { conn, states, ros } = setup();
    expect(FakeRos.instances).toHaveLength(1);
    expect(ros().connectCalls).toEqual([URL]);
    expect(conn.state).toBe('connecting');
    expect(states).toEqual(['connecting']);
    ros().simulateOpen();
    expect(conn.state).toBe('connected');
    expect(conn.ros).toBe(ros());
  });

  it('reconnects after close with growing delays, fresh Ros each time', () => {
    const { conn, states } = setup();
    const delays: number[] = [];
    for (let i = 0; i < 6; i++) {
      FakeRos.last().simulateClose();
      expect(conn.state).toBe('disconnected');
      const n = FakeRos.instances.length;
      const expected = backoffMs(i);
      vi.advanceTimersByTime(expected - 1);
      expect(FakeRos.instances.length).toBe(n); // not yet
      vi.advanceTimersByTime(1);
      expect(FakeRos.instances.length).toBe(n + 1);
      expect(conn.state).toBe('connecting');
      delays.push(expected);
    }
    expect(delays).toEqual([500, 1000, 2000, 4000, 8000, 10000]);
    expect(new Set(FakeRos.instances).size).toBe(7);
    expect(states.filter((s) => s === 'disconnected')).toHaveLength(6);
  });

  it('backoff is capped (uses injected backoff with attempt counter)', () => {
    const seen: number[] = [];
    setup((a) => { seen.push(a); return backoffMs(a); });
    for (let i = 0; i < 25; i++) {
      FakeRos.last().simulateClose();
      vi.advanceTimersByTime(10_000);
    }
    expect(seen).toEqual([...Array(25).keys()]);
    expect(FakeRos.instances).toHaveLength(26);
  });

  it('backoff resets after a successful connection', () => {
    const seen: number[] = [];
    setup((a) => { seen.push(a); return 100; });
    FakeRos.last().simulateClose(); vi.advanceTimersByTime(100);
    FakeRos.last().simulateClose(); vi.advanceTimersByTime(100);
    FakeRos.last().simulateOpen();
    FakeRos.last().simulateClose(); vi.advanceTimersByTime(100);
    expect(seen).toEqual([0, 1, 0]);
  });

  it('error + close for one failure schedules a single retry', () => {
    const { conn } = setup();
    FakeRos.last().simulateError();
    expect(conn.state).toBe('error');
    FakeRos.last().simulateClose();
    expect(conn.state).toBe('error');
    vi.advanceTimersByTime(60_000);
    expect(FakeRos.instances).toHaveLength(2);
  });

  it('closes the failed Ros before retrying', () => {
    setup();
    const first = FakeRos.last();
    first.simulateError();
    expect(first.closeCalls).toBeGreaterThanOrEqual(1);
  });

  it('events from a stale Ros instance are ignored', () => {
    const { conn } = setup();
    const old = FakeRos.last();
    old.simulateClose();
    vi.advanceTimersByTime(500);
    const fresh = FakeRos.last();
    fresh.simulateOpen();
    old.emit('connection', {});
    old.emit('close', {});
    expect(conn.state).toBe('connected');
    expect(conn.ros).toBe(fresh);
    vi.advanceTimersByTime(60_000);
    expect(FakeRos.instances).toHaveLength(2);
  });

  it('no reconnect and no state callbacks after explicit close (while connected)', () => {
    const { conn, states } = setup();
    FakeRos.last().simulateOpen();
    const before = states.length;
    conn.close();
    expect(FakeRos.last().closeCalls).toBeGreaterThanOrEqual(1);
    vi.advanceTimersByTime(60_000);
    expect(FakeRos.instances).toHaveLength(1);
    expect(states.length).toBe(before);
  });

  it('close() cancels a pending reconnect', () => {
    const { conn } = setup();
    FakeRos.last().simulateClose();
    conn.close();
    vi.advanceTimersByTime(60_000);
    expect(FakeRos.instances).toHaveLength(1);
    expect(vi.getTimerCount()).toBe(0);
  });

  it('close() is idempotent', () => {
    const { conn } = setup();
    conn.close();
    expect(() => conn.close()).not.toThrow();
    vi.advanceTimersByTime(60_000);
    expect(FakeRos.instances).toHaveLength(1);
  });
});
