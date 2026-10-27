import { useCallback, useEffect, useId, useState } from 'react';
import type { Pose2D, RouteEditor, RouteWaypointMsg } from '../../types';
import { useRos, useService } from '../../ros';
import { Card } from '../status/ui';
import { isGoalBusy } from '../route/routeLogic';
import {
  allFinite,
  fromWaypointMsgs,
  hasUnsavedChanges,
  serviceErrorText,
  sortRouteNames,
  toWaypointMsgs,
  validateRouteName,
} from './routeLibraryLogic';
import { MissionCard } from './MissionCard';

interface Result {
  success: boolean;
  message: string;
}
type Msg = { tone: 'ok' | 'err'; text: string } | null;

const btn =
  'min-h-9 rounded-lg px-3 py-2 text-sm focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-accent disabled:opacity-40';
const ghost = `${btn} border border-border text-muted hover:bg-white/5 hover:text-slate-100`;

export function RouteLibraryPanel({ editor }: { editor: RouteEditor }) {
  const connected = useRos().state === 'connected';
  const listRoutes = useService<Record<string, never>, Result & { names: string[] }>('/list_routes', 'my_bot_interfaces/srv/ListRoutes');
  const getRoute = useService<{ name: string }, Result & { map_id: string; waypoints: RouteWaypointMsg[] }>('/get_route', 'my_bot_interfaces/srv/GetRoute');
  const setRoute = useService<{ name: string; waypoints: RouteWaypointMsg[]; overwrite: boolean }, Result>('/set_route', 'my_bot_interfaces/srv/SetRoute');
  const deleteRoute = useService<{ name: string }, Result>('/delete_route', 'my_bot_interfaces/srv/DeleteRoute');
  const startRoute = useService<{ route_name: string }, { accepted: boolean; message: string }>('/start_route', 'my_bot_interfaces/srv/StartRoute');
  const cancelMission = useService<Record<string, never>, Result>('/cancel_mission', 'std_srvs/srv/Trigger');

  const [names, setNames] = useState<string[] | null>(null);
  const [loading, setLoading] = useState(false);
  const [listError, setListError] = useState<string | null>(null);
  const [selected, setSelected] = useState<{ name: string; points: Pose2D[] } | null>(null);
  const [nameInput, setNameInput] = useState('');
  const [busy, setBusy] = useState(false);
  const [msg, setMsg] = useState<Msg>(null);
  const [confirmDelete, setConfirmDelete] = useState<string | null>(null);
  const [confirmOverwrite, setConfirmOverwrite] = useState(false);
  const inputId = useId();

  const refresh = useCallback(async () => {
    setLoading(true);
    try {
      const res = await listRoutes({});
      if (!res.success) throw new Error(res.message || 'list_routes mislukt');
      setNames(sortRouteNames(res.names));
      setListError(null);
    } catch (e) {
      setListError(serviceErrorText('/list_routes', e));
    } finally {
      setLoading(false);
    }
  }, [listRoutes]);

  useEffect(() => {
    if (connected) void refresh();
  }, [connected, refresh]);

  /** Runs one service call with shared busy/result handling. */
  const run = async (service: string, fn: () => Promise<{ ok: boolean; text: string }>, node?: string) => {
    setBusy(true);
    try {
      const r = await fn();
      setMsg({ tone: r.ok ? 'ok' : 'err', text: r.text });
      return r.ok;
    } catch (e) {
      setMsg({ tone: 'err', text: serviceErrorText(service, e, node) });
      return false;
    } finally {
      setBusy(false);
    }
  };

  const select = (name: string) =>
    run('/get_route', async () => {
      const res = await getRoute({ name });
      if (!res.success) return { ok: false, text: res.message || `Route ${name} niet geladen` };
      const points = fromWaypointMsgs(res.waypoints);
      editor.load(points);
      setSelected({ name, points });
      setNameInput(name);
      setConfirmOverwrite(false);
      return { ok: true, text: `Route ${name} geladen (${points.length} waypoints)` };
    });

  const trimmed = nameInput.trim();
  const nameError = trimmed ? validateRouteName(trimmed) : null;
  const exists = names?.includes(trimmed) ?? false;
  const dirty = selected ? hasUnsavedChanges(selected.points, editor.waypoints) : false;

  const save = async () => {
    const err = validateRouteName(trimmed) ?? (editor.waypoints.length === 0 ? 'Route heeft geen waypoints' : !allFinite(editor.waypoints) ? 'Route bevat ongeldige coördinaten' : null);
    if (err) return setMsg({ tone: 'err', text: err });
    if (exists && !confirmOverwrite) return setConfirmOverwrite(true);
    setConfirmOverwrite(false);
    const points: Pose2D[] = editor.waypoints.map(({ x, y, yaw }) => ({ x, y, yaw }));
    const ok = await run('/set_route', async () => {
      const res = await setRoute({ name: trimmed, waypoints: toWaypointMsgs(points), overwrite: exists });
      return { ok: res.success, text: res.message || (res.success ? `Route ${trimmed} opgeslagen` : 'Opslaan mislukt') };
    });
    if (ok) setSelected({ name: trimmed, points });
    void refresh();
  };

  const remove = async (name: string) => {
    if (confirmDelete !== name) return setConfirmDelete(name);
    setConfirmDelete(null);
    const ok = await run('/delete_route', async () => {
      const res = await deleteRoute({ name });
      return { ok: res.success, text: res.message || (res.success ? `Route ${name} verwijderd` : 'Verwijderen mislukt') };
    });
    if (ok && selected?.name === name) setSelected(null);
    void refresh();
  };

  const start = () =>
    selected &&
    run(
      '/start_route',
      async () => {
        const res = await startRoute({ route_name: selected.name });
        return { ok: res.accepted, text: res.message || (res.accepted ? `Route ${selected.name} gestart` : 'Start geweigerd') };
      },
      'mission_manager',
    );

  const stop = () =>
    run(
      '/cancel_mission',
      async () => {
        const res = await cancelMission({});
        return { ok: res.success, text: res.message || (res.success ? 'Missie gestopt' : 'Stoppen mislukt') };
      },
      'mission_manager',
    );

  const off = !connected || busy;
  const navBusy = isGoalBusy(editor.nav.goal); // load() is a no-op while Nav2 drives

  return (
    <Card
      title="Opgeslagen routes"
      right={
        <button className={`${ghost} min-w-9 px-2 py-1`} onClick={() => void refresh()} disabled={!connected || loading} aria-label="Routelijst vernieuwen" title="Vernieuwen">
          ↻
        </button>
      }
    >
      {!connected ? (
        <p className="text-sm italic text-muted">Niet verbonden met de robot</p>
      ) : listError ? (
        <p role="alert" className="rounded-lg border border-err/40 bg-err/10 px-3 py-2 text-sm text-err">{listError}</p>
      ) : names === null || (loading && names.length === 0) ? (
        <p className="text-sm italic text-muted" role="status">Routes laden…</p>
      ) : names.length === 0 ? (
        <p className="text-sm italic text-muted">Nog geen opgeslagen routes</p>
      ) : (
        <ul className="max-h-56 space-y-1 overflow-y-auto" aria-label="Opgeslagen routes">
          {names.map((n) => {
            const isSel = selected?.name === n;
            return (
              <li key={n} className={`flex items-center gap-2 rounded-lg ${isSel ? 'bg-accent/15 ring-1 ring-inset ring-accent/40' : ''}`}>
                <button
                  className={`${btn} flex-1 truncate text-left font-mono ${isSel ? 'font-semibold text-accent' : 'text-slate-200 hover:bg-white/5'}`}
                  onClick={() => void select(n)}
                  disabled={off || navBusy}
                  aria-pressed={isSel}
                  aria-label={`Route ${n} laden`}
                >
                  {n}
                </button>
                <button
                  className={`${btn} ${confirmDelete === n ? 'bg-err text-white' : 'text-muted hover:text-err'}`}
                  onClick={() => void remove(n)}
                  onBlur={() => setConfirmDelete((c) => (c === n ? null : c))}
                  disabled={off}
                  aria-label={confirmDelete === n ? `Route ${n} echt verwijderen` : `Route ${n} verwijderen`}
                >
                  {confirmDelete === n ? 'Zeker?' : '✕'}
                </button>
              </li>
            );
          })}
        </ul>
      )}

      {navBusy && <p className="mt-1 text-xs text-warn">Laden kan niet tijdens het rijden — annuleer eerst de route</p>}

      <div className="mt-3 flex gap-2">
        <label htmlFor={inputId} className="sr-only">Routenaam</label>
        <input
          id={inputId}
          value={nameInput}
          onChange={(e) => {
            setNameInput(e.target.value);
            setConfirmOverwrite(false);
          }}
          placeholder="Routenaam"
          aria-invalid={nameError !== null}
          aria-describedby={nameError ? `${inputId}-err` : undefined}
          className="min-h-9 min-w-0 flex-1 rounded-lg border border-border bg-surface px-3 py-2 font-mono text-sm focus-visible:outline-2 focus-visible:outline-accent"
          disabled={!connected}
        />
        <button
          className={`${btn} font-medium text-white ${confirmOverwrite ? 'bg-warn' : 'bg-accent hover:bg-accent/90'}`}
          onClick={() => void save()}
          disabled={off || !trimmed || nameError !== null || editor.waypoints.length === 0}
        >
          {confirmOverwrite ? 'Overschrijven?' : 'Opslaan'}
        </button>
      </div>
      {nameError && <p id={`${inputId}-err`} className="mt-1 text-xs text-err">{nameError}</p>}

      <div className="mt-3 flex items-center gap-2">
        <button
          className={`${btn} flex-1 bg-ok font-medium text-white hover:bg-ok/90`}
          onClick={() => void start()}
          disabled={off || !selected || dirty}
          aria-describedby={dirty ? `${inputId}-dirty` : undefined}
        >
          {selected ? `Start route ${selected.name}` : 'Start route'}
        </button>
        <button className={`${btn} bg-err font-medium text-white hover:bg-err/90`} onClick={() => void stop()} disabled={off}>
          Stop missie
        </button>
      </div>
      {dirty && <p id={`${inputId}-dirty`} className="mt-1 text-xs text-warn">Eerst opslaan — de route is gewijzigd</p>}

      <MissionCard knownTotals={selected ? { [selected.name]: selected.points.length } : {}} />

      <p role="status" className={`mt-3 text-sm ${msg?.tone === 'ok' ? 'text-ok' : 'text-err'} ${msg ? '' : 'sr-only'}`}>
        {msg?.text}
      </p>
    </Card>
  );
}
