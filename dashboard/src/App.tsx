import { RosProvider } from './ros';
import { MapView } from './features/map';
import { useRobotOverlay } from './features/robot';
import { RoutePanel, useRouteEditor } from './features/route';
import { ConnectionBadge, StatusPanel } from './features/status';
import { RouteLibraryPanel } from './features/routes';
import { SignalsPanel } from './features/signals';

function Dashboard() {
  const robot = useRobotOverlay();
  const route = useRouteEditor();

  return (
    <div className="flex h-full flex-col">
      <header className="flex h-14 shrink-0 items-center justify-between border-b border-border bg-panel px-5">
        <div className="flex items-center gap-3">
          <div className="size-2.5 rounded-full bg-accent" />
          <h1 className="text-sm font-semibold tracking-wide">EZ-Wheels Dashboard</h1>
        </div>
        <ConnectionBadge />
      </header>

      <div className="flex min-h-0 flex-1 flex-col lg:flex-row">
        <main className="relative min-h-[50vh] flex-1">
          <MapView overlays={[robot, route.overlay]} />
        </main>
        <aside className="flex w-full shrink-0 flex-col gap-4 overflow-y-auto border-t border-border bg-surface p-4 lg:w-96 lg:border-l lg:border-t-0">
          <StatusPanel nav={route.nav} />
          <SignalsPanel />
          <RouteLibraryPanel editor={route} />
          <RoutePanel editor={route} />
        </aside>
      </div>
    </div>
  );
}

export default function App() {
  return (
    <RosProvider>
      <Dashboard />
    </RosProvider>
  );
}
