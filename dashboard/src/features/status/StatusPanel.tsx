import { useEffect, useState, type ReactNode } from 'react';
import { useRos, useTopic } from '../../ros';
import type { BatteryState, DiagnosticArray, DiagnosticStatus, NavStatus } from '../../types';
import {
  CONNECTION_LABEL,
  DIAG_LEVELS,
  GOAL_LABEL,
  POWER_SUPPLY_LABEL,
  batteryLevel,
  batteryPercent,
  countByLevel,
  diagLevel,
  mergeDiagnostics,
  parseMissionStatus,
  problems,
} from './statusLogic';
import { BAR, Badge, Card, Empty } from './ui';

interface StringMsg {
  data: string;
}
interface BoolMsg {
  data: boolean;
}
interface Odometry {
  twist: { twist: { linear: { x: number }; angular: { z: number } } };
}

const fmt = (n: number, digits = 2) => (Number.isFinite(n) ? n.toFixed(digits) : '–');

function Row({ k, children }: { k: string; children: ReactNode }) {
  return (
    <div className="flex items-baseline justify-between gap-3 text-sm">
      <dt className="text-muted">{k}</dt>
      <dd className="text-right tabular-nums">{children}</dd>
    </div>
  );
}

function ConnectionCard() {
  const { state, url } = useRos();
  const c = CONNECTION_LABEL[state];
  return (
    <Card title="Verbinding" right={<Badge tone={c.tone} label="Verbinding">{c.label}</Badge>}>
      <p className="truncate font-mono text-xs text-muted" title={url}>
        {url}
      </p>
    </Card>
  );
}

function NavigationCard({ nav }: { nav: NavStatus }) {
  const mission = useTopic<StringMsg>('mission_status', 'std_msgs/msg/String');
  const blocked = useTopic<BoolMsg>('navigation/blocked', 'std_msgs/msg/Bool', { latched: true });
  const g = GOAL_LABEL[nav.goal];
  const m = mission ? parseMissionStatus(mission.data) : null;

  return (
    <Card title="Navigatie" right={<Badge tone={g.tone} label="Navigatiedoel">{g.label}</Badge>}>
      {blocked?.data && (
        <div role="alert" className="mb-2 rounded-md bg-err px-3 py-2 text-sm font-semibold text-white">
          Geblokkeerd — pad is versperd
        </div>
      )}
      <dl className="space-y-1">
        <Row k="Missie">
          {m ? (
            <span className="inline-flex items-center gap-2">
              {m.route && <span className="truncate text-muted">{m.route}</span>}
              <Badge tone={m.tone} label="Missiestatus">
                {m.state === 'UNKNOWN' ? m.raw || m.label : m.label}
              </Badge>
            </span>
          ) : (
            <span className="text-muted">–</span>
          )}
        </Row>
        {nav.totalWaypoints > 0 && (
          <Row k="Waypoints">
            {nav.remainingWaypoints ?? '–'} / {nav.totalWaypoints} resterend
          </Row>
        )}
        {nav.distanceRemaining !== null && <Row k="Afstand">{fmt(nav.distanceRemaining, 1)} m</Row>}
        {nav.etaSeconds !== null && <Row k="ETA">{Math.round(nav.etaSeconds)} s</Row>}
        {nav.recoveries !== null && nav.recoveries > 0 && <Row k="Herstelacties">{nav.recoveries}</Row>}
      </dl>
      {nav.message && <p className="mt-2 text-xs text-muted">{nav.message}</p>}
    </Card>
  );
}

function BatteryCard() {
  const b = useTopic<BatteryState>('/battery_state', 'sensor_msgs/msg/BatteryState');
  if (!b) {
    return (
      <Card title="Batterij">
        <Empty>Geen batterijdata</Empty>
      </Card>
    );
  }
  const pct = batteryPercent(b.percentage);
  const lvl = batteryLevel(pct);
  return (
    <Card title="Batterij" right={<Badge tone={lvl.tone} label="Batterijniveau">{lvl.label}</Badge>}>
      <div
        role="meter"
        aria-label="Batterijpercentage"
        aria-valuemin={0}
        aria-valuemax={100}
        aria-valuenow={pct ?? undefined}
        className="mb-2 h-2 overflow-hidden rounded-full bg-border"
      >
        <div className={`h-full ${BAR[lvl.tone]} transition-[width]`} style={{ width: `${pct ?? 0}%` }} />
      </div>
      <dl className="space-y-1">
        <Row k="Lading">{pct === null ? '–' : `${pct}%`}</Row>
        <Row k="Spanning">{fmt(b.voltage, 1)} V</Row>
        <Row k="Status">{POWER_SUPPLY_LABEL[b.power_supply_status] ?? 'Onbekend'}</Row>
      </dl>
    </Card>
  );
}

function DiagnosticsCard() {
  const msg = useTopic<DiagnosticArray>('/diagnostics', 'diagnostic_msgs/msg/DiagnosticArray', { throttleMs: 1000 });
  const [all, setAll] = useState<Record<string, DiagnosticStatus>>({});
  const [open, setOpen] = useState(false);
  useEffect(() => {
    if (msg) setAll((prev) => mergeDiagnostics(prev, msg.status));
  }, [msg]);

  const counts = countByLevel(all);
  const bad = problems(all);

  return (
    <Card title="Diagnostics">
      {Object.keys(all).length === 0 ? (
        <Empty>Geen diagnostics ontvangen</Empty>
      ) : (
        <>
          <ul className="flex flex-wrap gap-1.5" aria-label="Aantal per niveau">
            {DIAG_LEVELS.map((d) => (
              <li key={d.level}>
                <Badge tone={counts[d.level] ? d.tone : 'muted'} label="Niveau">
                  {d.label} {counts[d.level]}
                </Badge>
              </li>
            ))}
          </ul>
          {bad.length > 0 && (
            <details className="mt-2" open={open} onToggle={(e) => setOpen(e.currentTarget.open)}>
              <summary className="cursor-pointer text-xs text-muted hover:text-slate-200">
                {bad.length} {bad.length === 1 ? 'melding' : 'meldingen'}
              </summary>
              <ul className="mt-1 max-h-48 space-y-1 overflow-y-auto">
                {bad.map((s) => {
                  const l = diagLevel(s.level);
                  return (
                    <li key={s.name} className="rounded bg-surface px-2 py-1 text-xs">
                      <div className="flex items-center gap-2">
                        <Badge tone={l.tone}>{l.label}</Badge>
                        <span className="truncate font-medium" title={s.name}>
                          {s.name}
                        </span>
                      </div>
                      {s.message && <p className="mt-0.5 text-muted">{s.message}</p>}
                    </li>
                  );
                })}
              </ul>
            </details>
          )}
        </>
      )}
    </Card>
  );
}

function SpeedCard() {
  const odom = useTopic<Odometry>('/odom', 'nav_msgs/msg/Odometry', { throttleMs: 200 });
  const t = odom?.twist.twist;
  return (
    <Card title="Snelheid">
      {t ? (
        <dl className="grid grid-cols-2 gap-2">
          <div>
            <dt className="text-xs text-muted">Lineair</dt>
            <dd className="text-lg font-semibold tabular-nums">
              {fmt(t.linear.x)} <span className="text-xs font-normal text-muted">m/s</span>
            </dd>
          </div>
          <div>
            <dt className="text-xs text-muted">Hoek</dt>
            <dd className="text-lg font-semibold tabular-nums">
              {fmt(t.angular.z)} <span className="text-xs font-normal text-muted">rad/s</span>
            </dd>
          </div>
        </dl>
      ) : (
        <Empty>Geen odometrie</Empty>
      )}
    </Card>
  );
}

export function StatusPanel({ nav }: { nav: NavStatus }) {
  return (
    <div className="flex flex-col gap-3">
      <ConnectionCard />
      <NavigationCard nav={nav} />
      <SpeedCard />
      <BatteryCard />
      <DiagnosticsCard />
    </div>
  );
}
