import { createContext, createElement, useCallback, useContext, useEffect, useMemo, useRef, useState } from 'react';
import type { ReactNode } from 'react';
import { Action, Service, Topic } from 'roslib';
import type { Ros } from 'roslib';
import type { ActionHandle, ConnectionState, GoalStatus, TopicOptions } from '../types';
import { createConnection } from './connection';

export { createConnection, backoffMs } from './connection';
export { yawToQuaternion, quaternionToYaw, rosTimeNow } from './convert';

export const SERVICE_TIMEOUT_MS = 10_000;
export const LATCH_RETRY_MS = 3_000;

interface RosContextValue {
  ros: Ros | null;
  state: ConnectionState;
  url: string;
}

const RosContext = createContext<RosContextValue | null>(null);

/** `?ros=ws://host:port` wins over the prop, so a deployed build can point at another robot. */
export function resolveRosUrl(url?: string): string {
  return new URLSearchParams(location.search).get('ros') ?? url ?? `ws://${location.hostname}:9090`;
}

export function RosProvider({ url, children }: { url?: string; children?: ReactNode }) {
  const resolved = useMemo(() => resolveRosUrl(url), [url]);
  const [conn, setConn] = useState<{ ros: Ros | null; state: ConnectionState }>({ ros: null, state: 'connecting' });
  useEffect(() => {
    const c = createConnection({ url: resolved, onState: (state, ros) => setConn({ ros, state }) });
    return () => c.close();
  }, [resolved]);
  const value = useMemo(() => ({ ...conn, url: resolved }), [conn, resolved]);
  return createElement(RosContext.Provider, { value }, children);
}

export function useRos(): RosContextValue {
  const ctx = useContext(RosContext);
  if (!ctx) throw new Error('useRos must be used inside <RosProvider>');
  return ctx;
}

/** The connected Ros, or null while (re)connecting. Effects keyed on it re-run after every reconnect. */
function useConnectedRos(): Ros | null {
  const { ros, state } = useRos();
  return state === 'connected' ? ros : null;
}

/** Latest message on `name` (kept across reconnects; reset when name/type changes). */
export function useTopic<T>(name: string, messageType: string, opts: TopicOptions = {}): T | null {
  const ros = useConnectedRos();
  const { latched, throttleMs, compression } = opts;
  const key = `${name}|${messageType}`;
  const [latest, setLatest] = useState<{ key: string; msg: T } | null>(null);

  useEffect(() => {
    if (!ros) return;
    const topic = new Topic<T>({
      ros,
      name,
      messageType,
      compression,
      throttle_rate: throttleMs,
      reconnect_on_close: false, // we re-subscribe ourselves on the new Ros instance
    });
    let received = false;
    const onMsg = (msg: T) => {
      received = true;
      setLatest({ key, msg });
    };
    topic.subscribe(onMsg);
    // Neither roslib 2.0.1 nor rosbridge <2.2 lets the client request TRANSIENT_LOCAL: rosbridge copies durability
    // from publishers present at subscribe time, so re-subscribe until a latched publisher (e.g. map_server) has delivered.
    const retry = latched
      ? setInterval(() => {
          if (received) return clearInterval(retry);
          topic.unsubscribe(onMsg);
          topic.subscribe(onMsg);
        }, LATCH_RETRY_MS)
      : undefined;
    return () => {
      clearInterval(retry);
      topic.unsubscribe(onMsg);
    };
  }, [ros, name, messageType, key, latched, throttleMs, compression]);

  return latest?.key === key ? latest.msg : null;
}

/** Publish function; messages sent while disconnected are dropped with a warning. */
export function usePublisher<T>(name: string, messageType: string): (msg: T) => void {
  const ros = useConnectedRos();
  const topicRef = useRef<Topic<T> | null>(null);
  useEffect(() => {
    if (!ros) return;
    const topic = new Topic<T>({ ros, name, messageType, reconnect_on_close: false });
    topic.advertise();
    topicRef.current = topic;
    return () => {
      topicRef.current = null;
      topic.unadvertise();
    };
  }, [ros, name, messageType]);
  return useCallback(
    (msg: T) => {
      if (topicRef.current) topicRef.current.publish(msg);
      else console.warn(`[ros] not connected, dropped message on ${name}`);
    },
    [name],
  );
}

/** Call function; rejects when not connected, on service failure, or after SERVICE_TIMEOUT_MS. */
export function useService<TReq, TRes>(name: string, serviceType: string): (req: TReq) => Promise<TRes> {
  const ros = useConnectedRos();
  return useCallback(
    (req: TReq) =>
      new Promise<TRes>((resolve, reject) => {
        if (!ros) return reject(new Error('Not connected to ROS'));
        const timer = setTimeout(() => reject(new Error(`Service ${name} timed out`)), SERVICE_TIMEOUT_MS);
        new Service<TReq, TRes>({ ros, name, serviceType }).callService(
          req,
          (res) => {
            clearTimeout(timer);
            resolve(res);
          },
          (err) => {
            clearTimeout(timer);
            reject(new Error(err || `Service ${name} failed`));
          },
        );
      }),
    [ros, name, serviceType],
  );
}

interface GoalState<F, R> {
  status: GoalStatus;
  feedback: F | null;
  result: R | null;
  error: string | null;
}

/** Loose view of the rosbridge action messages roslib emits on the goal id. */
interface ActionMsg {
  op: string;
  status?: number;
  result?: boolean;
  values?: unknown;
}

// action_msgs/GoalStatus codes.
const CANCELED = 5;
const ABORTED = 6;
const SUCCEEDED = 4;

export function mapActionResult<R>(m: ActionMsg): Pick<GoalState<unknown, R>, 'status' | 'result' | 'error'> {
  const values = m.values && typeof m.values === 'object' ? (m.values as R) : null;
  if (m.status === CANCELED) return { status: 'canceled', result: values, error: null };
  if (m.status === ABORTED) return { status: 'aborted', result: values, error: 'Goal aborted' };
  // rosbridge builds without a status field only report result: true on success.
  if (m.result && (m.status === SUCCEEDED || m.status === undefined)) return { status: 'succeeded', result: values, error: null };
  const error = typeof m.values === 'string' && m.values ? m.values : `Goal failed (status ${m.status ?? '?'})`;
  return { status: /reject/i.test(error) ? 'rejected' : 'error', result: null, error };
}

const IDLE: GoalState<never, never> = { status: 'idle', feedback: null, result: null, error: null };

export function useAction<TGoal, TFeedback, TResult>(
  name: string,
  actionType: string,
): ActionHandle<TGoal, TFeedback, TResult> {
  const ros = useConnectedRos();
  const [s, setS] = useState<GoalState<TFeedback, TResult>>(IDLE);
  const goal = useRef<{ action: Action<TGoal, TFeedback, TResult>; id: string; stop(): void } | null>(null);

  const stopTracking = () => {
    goal.current?.stop();
    goal.current = null;
  };

  // A goal can't be followed across a new Ros instance; report it instead of hanging in 'active'.
  useEffect(() => {
    if (ros) return;
    if (goal.current) {
      stopTracking();
      setS((p) => ({ ...p, status: 'error', error: 'Connection to ROS lost' }));
    }
  }, [ros]);
  useEffect(() => stopTracking, []);

  const send = useCallback(
    (g: TGoal) => {
      stopTracking();
      if (!ros) return setS({ ...IDLE, status: 'error', error: 'Not connected to ROS' });
      const action = new Action<TGoal, TFeedback, TResult>({ ros, name, actionType });
      // roslib's callbacks flatten every non-success into a string, so listen on the goal id ourselves.
      const id = action.sendGoal(g, () => {}, undefined, () => {});
      if (!id) return setS({ ...IDLE, status: 'error', error: 'Goal not sent' });
      const onMsg = (raw: unknown) => {
        const m = raw as ActionMsg;
        if (m.op === 'action_feedback') setS((p) => ({ ...p, status: 'active', feedback: m.values as TFeedback }));
        else if (m.op === 'action_status' && (m.status === 1 || m.status === 2)) setS((p) => ({ ...p, status: 'active' }));
        else if (m.op === 'action_result') {
          stopTracking();
          setS((p) => ({ ...p, ...mapActionResult<TResult>(m) }));
        }
      };
      ros.on(id, onMsg);
      goal.current = { action, id, stop: () => ros.removeAllListeners(id) };
      setS({ ...IDLE, status: 'sending' });
    },
    [ros, name, actionType],
  );

  // Status changes to 'canceled' when the server confirms with the result.
  const cancel = useCallback(() => {
    if (goal.current) goal.current.action.cancelGoal(goal.current.id);
  }, []);

  return { ...s, send, cancel };
}
