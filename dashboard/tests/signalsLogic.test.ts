import { describe, expect, it } from 'vitest';
import {
  appendEvent,
  BUZZER_LABEL,
  eventText,
  hmiState,
  INDICATOR_LABEL,
  lamps,
  nextRepeating,
  parseBuzzer,
  parseIndicator,
  relativeTime,
  turnView,
  type SignalEvent,
} from '../src/features/signals/signalsLogic';

describe('parseIndicator / parseBuzzer', () => {
  it.each(['links', 'rechts', 'gevaar', 'uit'])('indicator %s', (s) => expect(parseIndicator(s)).toBe(s));
  it.each(['enkel', 'dubbel', 'elke_5s', 'uit'])('buzzer %s', (s) => expect(parseBuzzer(s)).toBe(s));

  it.each(['', 'LINKS', ' links', 'left', 'onbekend', 'toString', '__proto__', null, undefined, 1, {}])(
    'unknown %j → onbekend',
    (s) => {
      expect(parseIndicator(s)).toBe('onbekend');
      expect(parseBuzzer(s)).toBe('onbekend');
    },
  );

  it('every state has a label', () => {
    for (const s of ['links', 'rechts', 'gevaar', 'uit', 'onbekend'] as const) expect(INDICATOR_LABEL[s]).toBeTruthy();
    for (const s of ['enkel', 'dubbel', 'elke_5s', 'uit', 'onbekend'] as const) expect(BUZZER_LABEL[s]).toBeTruthy();
  });
});

describe('lamps', () => {
  it('maps states to lamps', () => {
    expect(lamps('links')).toEqual({ left: true, right: false });
    expect(lamps('rechts')).toEqual({ left: false, right: true });
    expect(lamps('gevaar')).toEqual({ left: true, right: true });
    expect(lamps('uit')).toEqual({ left: false, right: false });
    expect(lamps('onbekend')).toEqual({ left: false, right: false });
  });
});

describe('nextRepeating', () => {
  it('elke_5s/uit set the mode; one-shots imply nothing repeats; unknown keeps prev', () => {
    expect(nextRepeating('uit', 'elke_5s')).toBe('elke_5s');
    expect(nextRepeating('elke_5s', 'uit')).toBe('uit');
    expect(nextRepeating('elke_5s', 'enkel')).toBe('uit');
    expect(nextRepeating('elke_5s', 'onbekend')).toBe('elke_5s');
  });
});

describe('appendEvent', () => {
  const ev = (i: number): SignalEvent => ({ at: i, source: 'buzzer', value: 'enkel' });

  it('newest first, capped at max, input not mutated', () => {
    let log: SignalEvent[] = [];
    for (let i = 0; i < 8; i++) log = appendEvent(log, ev(i));
    expect(log.map((e) => e.at)).toEqual([7, 6, 5, 4, 3]);
    const before = [...log];
    appendEvent(log, ev(99), 3);
    expect(log).toEqual(before);
  });

  it('custom max, max 1, max 0 and negative', () => {
    expect(appendEvent([ev(1), ev(0)], ev(2), 2).map((e) => e.at)).toEqual([2, 1]);
    expect(appendEvent([ev(0)], ev(1), 1).map((e) => e.at)).toEqual([1]);
    expect(appendEvent([ev(0)], ev(1), 0)).toEqual([]);
    expect(appendEvent([ev(0)], ev(1), -3)).toEqual([]);
  });

  it('eventText uses labels, unknown values readable', () => {
    expect(eventText({ at: 0, source: 'indicators', value: 'gevaar' })).toBe('Knipperlichten: Alarmlichten');
    expect(eventText({ at: 0, source: 'buzzer', value: 'xyz' })).toBe('Buzzer: Onbekend');
  });
});

describe('relativeTime', () => {
  const now = 10_000_000;
  const ago = (s: number) => relativeTime(now - s * 1000, now);
  it.each([
    [0, 'zojuist'],
    [0.999, 'zojuist'],
    [1, '1 s geleden'],
    [59, '59 s geleden'],
    [59.9, '59 s geleden'],
    [60, '1 min geleden'],
    [119, '1 min geleden'],
    [3599, '59 min geleden'],
    [3600, '1 u geleden'],
    [5 * 3600 + 59 * 60, '5 u geleden'],
  ])('%s s → %s', (s, text) => expect(ago(s)).toBe(text));

  it('future timestamps (clock skew) → zojuist; NaN → onbekend', () => {
    expect(relativeTime(now + 5000, now)).toBe('zojuist');
    expect(relativeTime(NaN, now)).toBe('onbekend');
  });
});

describe('turnView', () => {
  it('null msg → null', () => expect(turnView(null)).toBeNull());

  it('upcoming left turn with distance and absolute degrees', () => {
    expect(turnView({ direction: 1, distance: 2.345, angle: Math.PI / 2, turning: 0 })).toEqual({
      upcoming: { label: 'Links', arrow: '←' },
      distanceM: '2.3',
      angleDeg: '90',
      turning: { label: 'Geen', arrow: '↑' },
    });
  });

  it('right turn: negative angle shown as positive degrees; turning right', () => {
    const v = turnView({ direction: 2, distance: 0, angle: -Math.PI / 4, turning: 2 })!;
    expect(v.upcoming.label).toBe('Rechts');
    expect(v.angleDeg).toBe('45');
    expect(v.distanceM).toBe('0.0');
    expect(v.turning.label).toBe('Rechts');
  });

  it('NONE hides distance/angle; unknown direction and NaN handled', () => {
    const none = turnView({ direction: 0, distance: 3, angle: 1, turning: 1 })!;
    expect(none.distanceM).toBeNull();
    expect(none.angleDeg).toBeNull();
    expect(none.turning.label).toBe('Links');
    const odd = turnView({ direction: 7, distance: NaN, angle: NaN, turning: 9 })!;
    expect(odd.upcoming.label).toBe('Onbekend');
    expect(odd.turning.label).toBe('Onbekend');
    expect(turnView({ direction: 1, distance: NaN, angle: Infinity, turning: 0 })).toMatchObject({ distanceM: null, angleDeg: null });
  });
});

describe('hmiState', () => {
  it('maps Bool msg', () => {
    expect(hmiState({ data: true }).tone).toBe('ok');
    expect(hmiState({ data: false }).tone).toBe('err');
    expect(hmiState(null).tone).toBe('muted');
  });
});
