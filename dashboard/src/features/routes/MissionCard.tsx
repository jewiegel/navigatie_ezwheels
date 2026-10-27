import { useEffect, useRef, useState } from 'react';
import type { PoseWithCovarianceStamped, RouteWaypointMsg } from '../../types';
import { useService, useTopic } from '../../ros';
import { Badge, BAR } from '../status/ui';
import { parseMissionStatus } from '../status/statusLogic';
import { poseFromMsg } from '../robot';
import { formatDuration, missionProgress, remainingDistance } from './missionProgress';

interface PoseArrayMsg {
  poses: { position: { x: number; y: number } }[];
}

/** Progress and state of the route mission_manager is driving (also routes started from the HMI). */
export function MissionCard({ knownTotals }: { knownTotals: Record<string, number> }) {
  const status = useTopic<{ data: string }>('/mission_status', 'std_msgs/msg/String', { latched: true });
  const remaining = useTopic<PoseArrayMsg>('/remaining_waypoints', 'geometry_msgs/msg/PoseArray', { latched: true, throttleMs: 500 });
  const blocked = useTopic<{ data: boolean }>('/navigation/blocked', 'std_msgs/msg/Bool', { latched: true });
  const amcl = useTopic<PoseWithCovarianceStamped>('/amcl_pose', 'geometry_msgs/msg/PoseWithCovarianceStamped', { latched: true });
  const getRoute = useService<{ name: string }, { success: boolean; waypoints: RouteWaypointMsg[] }>('/get_route', 'my_bot_interfaces/srv/GetRoute');

  const mission = parseMissionStatus(status?.data ?? '');
  const route = mission.route || null;

  // Total waypoints of the running route: known from the panel, else fetched once per route name.
  const [fetched, setFetched] = useState<Record<string, number>>({});
  const total = route ? (knownTotals[route] ?? fetched[route] ?? null) : null;
  useEffect(() => {
    if (!route || total !== null) return;
    let alive = true;
    getRoute({ name: route })
      .then((r) => alive && r.success && setFetched((f) => ({ ...f, [route]: r.waypoints.length })))
      .catch(() => {}); // progress then just shows without a total
    return () => {
      alive = false;
    };
  }, [route, total, getRoute]);

  // Driving time, measured from when this page saw the mission start driving.
  const driving = mission.state === 'DRIVING' || mission.state === 'BLOCKED';
  const since = useRef<number | null>(null);
  const [now, setNow] = useState(Date.now());
  if (!driving) since.current = null;
  else if (since.current === null) since.current = Date.now();
  useEffect(() => {
    if (!driving) return;
    const t = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(t);
  }, [driving]);

  const points = remaining?.poses.map((p) => p.position) ?? [];
  const p = missionProgress(mission.state, total, remaining ? points.length : null);
  const pct = p.fraction === null ? null : Math.round(p.fraction * 100);
  const dist = driving ? remainingDistance(poseFromMsg(amcl), points) : { toNext: null, total: null };
  const tone = mission.state === 'ARRIVED' ? 'ok' : blocked?.data ? 'err' : mission.tone;
  const m = (v: number | null) => (v === null ? '–' : `${v.toFixed(1)} m`);

  return (
    <section aria-label="Huidige missie" className="mt-3 rounded-lg border border-border bg-surface p-3">
      <div className="flex items-center justify-between gap-2">
        <h3 className="text-xs font-semibold uppercase tracking-wider text-muted">Huidige missie</h3>
        <Badge tone={mission.tone} label="Missiestatus">{status ? mission.label : 'Onbekend'}</Badge>
      </div>

      {!status ? (
        <p className="mt-2 text-sm italic text-muted">Nog geen missiestatus ontvangen</p>
      ) : mission.state === 'IDLE' ? (
        <p className="mt-2 text-sm italic text-muted">Geen route actief</p>
      ) : (
        <>
          <p className="mt-2 truncate font-mono text-sm text-slate-200" title={route ?? undefined}>{route ?? '–'}</p>

          <div className="mt-2" role="progressbar" aria-label="Voortgang route" aria-valuemin={0} aria-valuemax={p.total || undefined} aria-valuenow={p.total ? p.passed : undefined}>
            <div className="h-2 overflow-hidden rounded-full bg-border">
              <div className={`h-full rounded-full transition-[width] motion-reduce:transition-none ${BAR[tone]}`} style={{ width: `${pct ?? 0}%` }} />
            </div>
          </div>
          <p className="mt-1 text-sm tabular-nums text-slate-300" aria-live="polite">
            {p.total ? `${p.passed} van ${p.total} waypoints bereikt` : 'Aantal waypoints onbekend'}
            {p.current !== null && ` · onderweg naar ${p.current}`}
            {pct !== null && ` (${pct}%)`}
          </p>

          {blocked?.data && (
            <p role="alert" className="mt-2 rounded-md bg-err/15 px-2 py-1 text-sm font-medium text-red-300">Geblokkeerd — robot wacht op vrije doorgang</p>
          )}

          {driving && (
            <dl className="mt-2 grid grid-cols-3 gap-2 text-xs">
              <div>
                <dt className="text-muted">Tot volgend punt</dt>
                <dd className="tabular-nums text-slate-200">{m(dist.toNext)}</dd>
              </div>
              <div>
                <dt className="text-muted">Resterend</dt>
                <dd className="tabular-nums text-slate-200">{m(dist.total)}</dd>
              </div>
              <div>
                <dt className="text-muted">Rijtijd</dt>
                <dd className="tabular-nums text-slate-200">{formatDuration(since.current === null ? null : (now - since.current) / 1000)}</dd>
              </div>
            </dl>
          )}
          {driving && <p className="mt-1 text-[11px] text-muted">Afstanden hemelsbreed via de waypoints</p>}
        </>
      )}
    </section>
  );
}
