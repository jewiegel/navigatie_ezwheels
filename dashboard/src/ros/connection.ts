import { Ros } from 'roslib';
import type { ConnectionState } from '../types';

/** Reconnect delay for the given attempt (0-based): 0.5s, 1s, 2s, ... capped at 10s. */
export const backoffMs = (attempt: number, base = 500, max = 10_000): number =>
  Math.min(max, base * 2 ** attempt);

export interface ConnectionOptions {
  url: string;
  /** Factory for the roslib Ros; inject a fake in tests. Must start connecting to `url`. */
  createRos?: (url: string) => Ros;
  onState?: (state: ConnectionState, ros: Ros) => void;
  backoff?: (attempt: number) => number;
}

export interface Connection {
  readonly ros: Ros | null;
  readonly state: ConnectionState;
  /** Stop for good: closes the socket, cancels pending reconnects, no more onState calls. */
  close(): void;
}

/**
 * Owns one rosbridge connection. Each (re)connect uses a fresh Ros instance, so consumers
 * re-create their topics/services when `ros` changes (no stale roslib reconnect state).
 */
export function createConnection({
  url,
  createRos = (u) => new Ros({ url: u }),
  onState,
  backoff = backoffMs,
}: ConnectionOptions): Connection {
  let ros: Ros | null = null;
  let state: ConnectionState = 'connecting';
  let attempt = 0;
  let timer: ReturnType<typeof setTimeout> | undefined;
  let stopped = false;

  const set = (s: ConnectionState, r: Ros) => {
    if (stopped || r !== ros) return;
    state = s;
    onState?.(s, r);
  };

  const open = () => {
    const r = createRos(url);
    ros = r;
    let dead = false;
    // 'error' and 'close' usually both fire for one failure; schedule only one retry.
    const drop = (s: ConnectionState) => {
      if (dead || stopped || r !== ros) return;
      dead = true;
      set(s, r);
      r.close();
      timer = setTimeout(open, backoff(attempt++));
    };
    r.on('connection', () => {
      attempt = 0;
      set('connected', r);
    });
    r.on('close', () => drop('disconnected'));
    r.on('error', () => drop('error'));
    set('connecting', r);
  };

  open();
  return {
    get ros() {
      return ros;
    },
    get state() {
      return state;
    },
    close() {
      stopped = true;
      clearTimeout(timer);
      ros?.close();
    },
  };
}
