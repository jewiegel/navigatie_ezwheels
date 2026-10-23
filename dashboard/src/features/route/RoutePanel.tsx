import type { RouteEditor } from '../../types';
import { isGoalBusy } from './routeLogic';

const RESULT: Record<string, { text: string; cls: string }> = {
  succeeded: { text: 'Route bereikt', cls: 'text-ok border-ok/40 bg-ok/10' },
  canceled: { text: 'Route geannuleerd', cls: 'text-warn border-warn/40 bg-warn/10' },
  aborted: { text: 'Navigatie mislukt', cls: 'text-err border-err/40 bg-err/10' },
  rejected: { text: 'Doel geweigerd door Nav2', cls: 'text-err border-err/40 bg-err/10' },
  error: { text: 'Fout bij navigatie', cls: 'text-err border-err/40 bg-err/10' },
};

function formatEta(s: number | null) {
  if (s === null || !Number.isFinite(s)) return '–';
  const m = Math.floor(s / 60);
  return m > 0 ? `${m} min ${Math.round(s % 60)} s` : `${Math.round(s)} s`;
}

const iconBtn =
  'grid size-7 place-items-center rounded-md text-muted hover:bg-white/5 hover:text-slate-100 disabled:pointer-events-none disabled:opacity-30';

export function RoutePanel({ editor }: { editor: RouteEditor }) {
  const { waypoints, nav } = editor;
  const busy = isGoalBusy(nav.goal);
  const total = nav.totalWaypoints;
  const done = nav.remainingWaypoints === null ? 0 : Math.max(0, total - nav.remainingWaypoints);
  const pct = total > 0 ? (done / total) * 100 : 0;
  const result = RESULT[nav.goal];

  return (
    <section className="rounded-xl border border-border bg-panel p-4">
      <div className="mb-3 flex items-center justify-between">
        <h2 className="text-sm font-semibold">Route</h2>
        <span className="text-xs text-muted">{waypoints.length} waypoints</span>
      </div>

      {waypoints.length === 0 ? (
        <p className="rounded-lg border border-dashed border-border px-3 py-6 text-center text-sm text-muted">
          Klik op de kaart om waypoints te plaatsen
        </p>
      ) : (
        <ol className="max-h-64 space-y-1 overflow-y-auto">
          {waypoints.map((w, i) => (
            <li
              key={w.id}
              className={`flex items-center gap-3 rounded-lg px-2 py-1.5 hover:bg-white/[0.03] ${busy && i < done ? 'opacity-40' : ''}`}
            >
              <span className="grid size-6 shrink-0 place-items-center rounded-full bg-accent text-xs font-semibold text-white">
                {i + 1}
              </span>
              <span className="flex-1 font-mono text-xs tabular-nums text-slate-300">
                x {w.x.toFixed(2)} m · y {w.y.toFixed(2)} m
              </span>
              <button className={iconBtn} disabled={busy || i === 0} onClick={() => editor.reorder(i, i - 1)} aria-label={`Waypoint ${i + 1} omhoog`} title="Omhoog">
                ↑
              </button>
              <button className={iconBtn} disabled={busy || i === waypoints.length - 1} onClick={() => editor.reorder(i, i + 1)} aria-label={`Waypoint ${i + 1} omlaag`} title="Omlaag">
                ↓
              </button>
              <button className={`${iconBtn} hover:!text-err`} disabled={busy} onClick={() => editor.remove(w.id)} aria-label={`Waypoint ${i + 1} verwijderen`} title="Verwijderen">
                ✕
              </button>
            </li>
          ))}
        </ol>
      )}

      {busy && (
        <div className="mt-4 space-y-2">
          <div className="h-2 overflow-hidden rounded-full bg-border">
            <div className="h-full rounded-full bg-accent transition-[width] duration-500" style={{ width: `${pct}%` }} />
          </div>
          <div className="flex justify-between text-xs text-muted">
            <span>
              {done} van {total} waypoints
            </span>
            <span>
              {nav.distanceRemaining === null ? '–' : `${nav.distanceRemaining.toFixed(1)} m`} resterend · ETA {formatEta(nav.etaSeconds)}
            </span>
          </div>
        </div>
      )}

      {!busy && result && (
        <p className={`mt-4 rounded-lg border px-3 py-2 text-sm ${result.cls}`}>
          {result.text}
          {nav.message && nav.message !== result.text && <span className="block text-xs opacity-80">{nav.message}</span>}
        </p>
      )}

      <div className="mt-4 flex gap-2">
        <button
          className="rounded-lg border border-border px-3 py-2 text-sm text-muted hover:bg-white/5 hover:text-slate-100 disabled:opacity-40"
          disabled={busy || waypoints.length === 0}
          onClick={editor.clear}
        >
          Wissen
        </button>
        {busy ? (
          <button className="flex-1 rounded-lg bg-err px-3 py-2 text-sm font-medium text-white hover:bg-err/90" onClick={editor.cancel}>
            Annuleer
          </button>
        ) : (
          <button
            className="flex-1 rounded-lg bg-accent px-3 py-2 text-sm font-medium text-white hover:bg-accent/90 disabled:opacity-40"
            disabled={waypoints.length === 0}
            onClick={editor.send}
          >
            Verstuur naar Nav2
          </button>
        )}
      </div>
    </section>
  );
}
