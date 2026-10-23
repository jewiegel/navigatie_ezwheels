import type { ReactNode } from 'react';
import type { Tone } from './statusLogic';

const TONE: Record<Tone, string> = {
  ok: 'bg-ok/15 text-ok ring-ok/30',
  warn: 'bg-warn/15 text-warn ring-warn/30',
  err: 'bg-err/15 text-err ring-err/30',
  accent: 'bg-accent/15 text-accent ring-accent/30',
  muted: 'bg-slate-500/15 text-slate-300 ring-slate-500/30',
};
export const DOT: Record<Tone, string> = {
  ok: 'bg-ok',
  warn: 'bg-warn',
  err: 'bg-err',
  accent: 'bg-accent',
  muted: 'bg-muted',
};
export const BAR = DOT;

export function Badge({ tone, children, label }: { tone: Tone; children: ReactNode; label?: string }) {
  return (
    <span
      className={`inline-flex items-center rounded-full px-2 py-0.5 text-xs font-medium ring-1 ring-inset ${TONE[tone]}`}
    >
      {label && <span className="sr-only">{label}: </span>}
      {children}
    </span>
  );
}

export function Card({ title, right, children }: { title: string; right?: ReactNode; children: ReactNode }) {
  return (
    <section aria-label={title} className="rounded-lg border border-border bg-panel p-3">
      <header className="mb-2 flex items-center justify-between gap-2">
        <h2 className="text-xs font-semibold uppercase tracking-wider text-muted">{title}</h2>
        {right}
      </header>
      {children}
    </section>
  );
}

export function Empty({ children }: { children: ReactNode }) {
  return <p className="text-sm italic text-muted">{children}</p>;
}
