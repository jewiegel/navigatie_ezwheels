import { useRos } from '../../ros';
import { CONNECTION_LABEL } from './statusLogic';
import { DOT } from './ui';

export function ConnectionBadge() {
  const { state, url } = useRos();
  const { label, tone } = CONNECTION_LABEL[state];
  return (
    <span
      role="status"
      aria-label={`ROS-verbinding: ${label} (${url})`}
      title={url}
      className="inline-flex items-center gap-2 rounded-full border border-border bg-surface px-3 py-1 text-xs font-medium"
    >
      <span className={`size-2 rounded-full ${DOT[tone]} ${state === 'connecting' ? 'animate-pulse' : ''}`} aria-hidden />
      {label}
    </span>
  );
}
