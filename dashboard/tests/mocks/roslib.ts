// Minimal roslib stand-ins. FakeRos never opens a socket; tests drive it with
// simulateOpen/simulateClose/simulateError.
type Listener = (...args: unknown[]) => void;

class Emitter {
  private listeners = new Map<string, Listener[]>();
  on(ev: string, fn: Listener) {
    this.listeners.set(ev, [...(this.listeners.get(ev) ?? []), fn]);
    return this;
  }
  once(ev: string, fn: Listener) {
    const w: Listener = (...a) => { this.off(ev, w); fn(...a); };
    return this.on(ev, w);
  }
  off(ev: string, fn?: Listener) {
    if (!fn) this.listeners.delete(ev);
    else this.listeners.set(ev, (this.listeners.get(ev) ?? []).filter((l) => l !== fn));
    return this;
  }
  removeAllListeners(ev?: string) {
    if (ev) this.listeners.delete(ev); else this.listeners.clear();
    return this;
  }
  emit(ev: string, ...args: unknown[]) {
    for (const l of [...(this.listeners.get(ev) ?? [])]) l(...args);
    return true;
  }
  listenerCount(ev: string) {
    return this.listeners.get(ev)?.length ?? 0;
  }
}

export class FakeRos extends Emitter {
  static instances: FakeRos[] = [];
  isConnected = false;
  closed = false;
  connectCalls: string[] = [];
  closeCalls = 0;
  constructor(public opts: { url?: string } = {}) {
    super();
    FakeRos.instances.push(this);
    if (opts.url) this.connectCalls.push(opts.url);
  }
  connect(url: string) {
    this.connectCalls.push(url);
    return Promise.resolve();
  }
  close() {
    this.closeCalls++;
    if (this.closed) return;
    this.closed = true;
    this.isConnected = false;
    this.emit('close', {});
  }
  simulateOpen() { this.isConnected = true; this.emit('connection', {}); }
  simulateClose() { this.isConnected = false; this.closed = true; this.emit('close', {}); }
  simulateError(err: unknown = new Error('boom')) { this.emit('error', err); }
  static reset() { FakeRos.instances = []; }
  static last() { return FakeRos.instances[FakeRos.instances.length - 1]; }
}

export class FakeTopic<T = unknown> extends Emitter {
  static instances: FakeTopic[] = [];
  callbacks: ((m: T) => void)[] = [];
  published: T[] = [];
  advertised = false;
  unsubscribed = 0;
  constructor(public opts: { ros: unknown; name: string; messageType: string; [k: string]: unknown }) {
    super();
    FakeTopic.instances.push(this as FakeTopic);
  }
  subscribe(cb: (m: T) => void) { this.callbacks.push(cb); }
  unsubscribe(cb?: (m: T) => void) {
    this.unsubscribed++;
    this.callbacks = cb ? this.callbacks.filter((c) => c !== cb) : [];
  }
  advertise() { this.advertised = true; }
  unadvertise() { this.advertised = false; }
  publish(m: T) { this.published.push(m); }
  deliver(m: T) { for (const c of [...this.callbacks]) c(m); }
  static reset() { FakeTopic.instances = []; }
}

export class FakeAction<G = unknown, F = unknown, R = unknown> {
  static instances: FakeAction[] = [];
  goals: { id: string; goal: G; onResult: (r: R) => void; onFeedback?: (f: F) => void; onFail?: (e: string) => void }[] = [];
  cancelled: string[] = [];
  private n = 0;
  constructor(public opts: { ros: unknown; name: string; actionType: string }) {
    FakeAction.instances.push(this as FakeAction);
  }
  sendGoal(goal: G, onResult: (r: R) => void, onFeedback?: (f: F) => void, onFail?: (e: string) => void) {
    const id = `goal_${++this.n}`;
    this.goals.push({ id, goal, onResult, onFeedback, onFail });
    return id;
  }
  cancelGoal(id: string) { this.cancelled.push(id); }
  static reset() { FakeAction.instances = []; }
}

export function resetRoslibMocks() {
  FakeRos.reset(); FakeTopic.reset(); FakeAction.reset();
}

/** Factory shape most connection managers accept: (url) => Ros-like. */
export const fakeRosFactory = (url?: string) => new FakeRos({ url });
