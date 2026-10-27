import { useEffect, useRef, useState, type ReactNode } from 'react';
import { useTopic } from '../../ros';
import type { TurnMsg } from '../../types';
import { Badge, Card } from '../status/ui';
import {
  BUZZER_LABEL,
  INDICATOR_LABEL,
  UNKNOWN,
  appendEvent,
  eventText,
  hmiState,
  isOneShot,
  lamps,
  nextRepeating,
  parseBuzzer,
  parseIndicator,
  relativeTime,
  turnView,
  type SignalEvent,
} from './signalsLogic';

interface StringMsg {
  data: string;
}
interface BoolMsg {
  data: boolean;
}

// 1 s period, half on / half off, like hmi_bridge blink_period_s; reduced motion → steady on.
const BLINK_CSS = `@keyframes sig-blink{50%{opacity:.15}}.sig-blink{animation:sig-blink 1s step-end infinite}
@media (prefers-reduced-motion:reduce){.sig-blink{animation:none}}`;

function useNow(ms = 1000): number {
  const [now, setNow] = useState(() => Date.now());
  useEffect(() => {
    const id = setInterval(() => setNow(Date.now()), ms);
    return () => clearInterval(id);
  }, [ms]);
  return now;
}

function Row({ k, children }: { k: string; children: ReactNode }) {
  return (
    <div className="flex items-baseline justify-between gap-3 text-sm">
      <dt className="text-muted">{k}</dt>
      <dd className="text-right tabular-nums">{children}</dd>
    </div>
  );
}

function Lamp({ side, on }: { side: 'Links' | 'Rechts'; on: boolean }) {
  return (
    <span
      role="img"
      aria-label={`Knipperlicht ${side.toLowerCase()} ${on ? 'aan' : 'uit'}`}
      className={`inline-flex h-9 w-9 items-center justify-center rounded-full text-xl font-bold ${
        on ? 'sig-blink bg-warn text-black' : 'bg-border text-muted'
      }`}
    >
      {side === 'Links' ? '←' : '→'}
    </span>
  );
}

export function SignalsPanel() {
  const ind = useTopic<StringMsg>('/indicators', 'std_msgs/msg/String', { latched: true });
  const buz = useTopic<StringMsg>('/buzzer_pattern', 'std_msgs/msg/String'); // volatile: events only, nothing on connect
  const turn = useTopic<TurnMsg>('/turn', 'my_bot_interfaces/msg/Turn', { throttleMs: 200 });
  const hmi = useTopic<BoolMsg>('/hmi/mqtt_connected', 'std_msgs/msg/Bool', { latched: true });
  const now = useNow();

  const [log, setLog] = useState<SignalEvent[]>([]);
  const [repeating, setRepeating] = useState<'elke_5s' | 'uit' | typeof UNKNOWN>(UNKNOWN);
  const [oneShot, setOneShot] = useState<SignalEvent | null>(null);
  const lastIndicator = useRef<string | null>(null);

  // useTopic hands a new object per message, so these fire once per message, not per render.
  useEffect(() => {
    if (!ind || ind.data === lastIndicator.current) return; // latched re-delivery after reconnect
    lastIndicator.current = ind.data;
    setLog((l) => appendEvent(l, { at: Date.now(), source: 'indicators', value: String(ind.data) }));
  }, [ind]);

  useEffect(() => {
    if (!buz) return;
    const e: SignalEvent = { at: Date.now(), source: 'buzzer', value: String(buz.data) };
    const p = parseBuzzer(buz.data);
    setLog((l) => appendEvent(l, e));
    setRepeating((r) => nextRepeating(r, p));
    if (isOneShot(p)) setOneShot(e);
  }, [buz]);

  const indicator = ind ? parseIndicator(ind.data) : UNKNOWN;
  const lit = lamps(indicator);
  const t = turnView(turn);
  const h = hmiState(hmi);

  return (
    <Card title="Signalen" right={<Badge tone={h.tone} label="HMI-verbinding">HMI {h.label.toLowerCase()}</Badge>}>
      <style>{BLINK_CSS}</style>
      <div aria-live="polite" className="sr-only">
        {log[0] ? eventText(log[0]) : ''}
      </div>
      <dl className="space-y-2">
        <Row k="Knipperlichten">
          <span className="inline-flex items-center gap-2">
            <Lamp side="Links" on={lit.left} />
            <span className="min-w-20 text-center">{INDICATOR_LABEL[indicator]}</span>
            <Lamp side="Rechts" on={lit.right} />
          </span>
        </Row>
        <Row k="Buzzer">{repeating === 'elke_5s' ? BUZZER_LABEL.elke_5s : repeating === 'uit' ? 'Uit' : 'Onbekend'}</Row>
        <Row k="Laatste piep">
          {oneShot ? (
            <>
              {BUZZER_LABEL[parseBuzzer(oneShot.value)]} <span className="text-muted">· {relativeTime(oneShot.at, now)}</span>
            </>
          ) : (
            <span className="text-muted">–</span>
          )}
        </Row>
        <Row k="Bocht">
          {t ? (
            <>
              <span aria-hidden="true">{t.upcoming.arrow} </span>
              {t.upcoming.label}
              {t.distanceM !== null && ` over ${t.distanceM} m`}
              {t.angleDeg !== null && ` (${t.angleDeg}°)`}
            </>
          ) : (
            'Onbekend'
          )}
        </Row>
        <Row k="Draait nu">
          {t ? (t.turning.label === 'Geen' ? 'Nee' : `Ja, ${t.turning.label.toLowerCase()}`) : 'Onbekend'}
        </Row>
        <Row k="HMI-verbinding">
          <Badge tone={h.tone}>{h.label}</Badge>
        </Row>
      </dl>
      <h3 className="mt-3 mb-1 text-xs font-semibold text-muted">Laatste wijzigingen</h3>
      {log.length ? (
        <ol className="space-y-0.5 text-xs">
          {log.map((e, i) => (
            <li key={`${e.at}-${i}`} className="flex justify-between gap-2">
              <span>{eventText(e)}</span>
              <time className="text-muted tabular-nums" dateTime={new Date(e.at).toISOString()}>
                {new Date(e.at).toLocaleTimeString('nl-NL')}
              </time>
            </li>
          ))}
        </ol>
      ) : (
        <p className="text-xs italic text-muted">Nog geen signalen</p>
      )}
    </Card>
  );
}
