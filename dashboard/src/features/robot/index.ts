import { useMemo } from 'react';
import { useTopic } from '../../ros';
import type { MapOverlay, MapViewApi, Path, Point, PoseStamped, PoseWithCovarianceStamped } from '../../types';
import { covarianceEllipse, poseFromMsg } from './robotMath';

export { covarianceEllipse, poseFromMsg } from './robotMath';

const ROBOT_RADIUS_M = 0.35;
const MIN_RADIUS_PX = 8;
const SIGMA_SCALE = 2; // draw the ~95% (2σ) ellipse

function css(name: string, fallback: string): string {
  const v = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  return v || fallback;
}

export function useRobotOverlay(): MapOverlay {
  const amcl = useTopic<PoseWithCovarianceStamped>('/amcl_pose', 'geometry_msgs/msg/PoseWithCovarianceStamped', {
    latched: true,
  });
  const plan = useTopic<Path>('/plan', 'nav_msgs/msg/Path', { throttleMs: 500 });

  return useMemo<MapOverlay>(() => {
    const pose = poseFromMsg(amcl);
    const ellipse = covarianceEllipse(amcl?.pose.covariance);
    const planPts: Point[] = plan?.poses?.map((p: PoseStamped) => ({ x: p.pose.position.x, y: p.pose.position.y })) ?? [];

    return {
      id: 'robot',
      draw(ctx: CanvasRenderingContext2D, api: MapViewApi) {
        const accent = css('--color-accent', '#38bdf8');
        ctx.save();
        ctx.lineJoin = 'round';
        ctx.lineCap = 'round';

        if (planPts.length > 1) {
          ctx.beginPath();
          planPts.forEach((p, i) => {
            const s = api.worldToScreen(p);
            if (i === 0) ctx.moveTo(s.sx, s.sy);
            else ctx.lineTo(s.sx, s.sy);
          });
          ctx.strokeStyle = accent;
          ctx.globalAlpha = 0.7;
          ctx.lineWidth = 2;
          ctx.stroke();
          ctx.globalAlpha = 1;
        }

        if (!pose) return ctx.restore();
        const c = api.worldToScreen(pose);

        // Project world directions instead of using raw angles: the view flips y.
        const screenAngle = (worldAngle: number) => {
          const s = api.worldToScreen({ x: pose.x + Math.cos(worldAngle), y: pose.y + Math.sin(worldAngle) });
          return Math.atan2(s.sy - c.sy, s.sx - c.sx);
        };

        if (ellipse && ellipse.major > 0) {
          ctx.beginPath();
          ctx.ellipse(
            c.sx, c.sy,
            Math.max(1, ellipse.major * SIGMA_SCALE * api.pxPerMeter),
            Math.max(1, ellipse.minor * SIGMA_SCALE * api.pxPerMeter),
            screenAngle(ellipse.angle), 0, Math.PI * 2,
          );
          ctx.fillStyle = accent;
          ctx.globalAlpha = 0.15;
          ctx.fill();
          ctx.globalAlpha = 0.5;
          ctx.lineWidth = 1;
          ctx.strokeStyle = accent;
          ctx.stroke();
          ctx.globalAlpha = 1;
        }

        const r = Math.max(MIN_RADIUS_PX, ROBOT_RADIUS_M * api.pxPerMeter);
        const h = screenAngle(pose.yaw);
        ctx.beginPath();
        ctx.arc(c.sx, c.sy, r, 0, Math.PI * 2);
        ctx.fillStyle = accent;
        ctx.globalAlpha = 0.85;
        ctx.fill();
        ctx.globalAlpha = 1;
        ctx.lineWidth = 2;
        ctx.strokeStyle = '#ffffff';
        ctx.stroke();

        // heading wedge
        const tip = r * 1.6;
        ctx.beginPath();
        ctx.moveTo(c.sx + Math.cos(h) * tip, c.sy + Math.sin(h) * tip);
        ctx.lineTo(c.sx + Math.cos(h + 2.5) * r * 0.6, c.sy + Math.sin(h + 2.5) * r * 0.6);
        ctx.lineTo(c.sx + Math.cos(h - 2.5) * r * 0.6, c.sy + Math.sin(h - 2.5) * r * 0.6);
        ctx.closePath();
        ctx.fillStyle = '#ffffff';
        ctx.fill();

        ctx.restore();
      },
    };
  }, [amcl, plan]);
}
