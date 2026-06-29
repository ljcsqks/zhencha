import { Power, RotateCw } from "lucide-react";
import type { ActiveCommandSnapshot, SimulationState } from "../types/sim";

interface Props {
  state?: SimulationState;
  activeCommands: ActiveCommandSnapshot[];
  selectedUavId?: string;
  busy?: boolean;
  onSelectUav(uavId: string | undefined): void;
  onSetOnline(uavId: string, online: boolean): Promise<void>;
}

export function UavPanel({ state, activeCommands, selectedUavId, busy = false, onSelectUav, onSetOnline }: Props) {
  const uavs = state?.uavs || [];
  return (
    <section className="panel">
      <h2>UAVs</h2>
      <div className="uav-list">
        {uavs.map((uav) => (
          <article
            key={uav.id}
            className={`uav-row ${uav.status.toLowerCase()} ${selectedUavId === uav.id ? "selected" : ""}`}
            onClick={() => onSelectUav(selectedUavId === uav.id ? undefined : uav.id)}
          >
            <div>
              <strong>{uav.id}</strong>
              <span>{uav.status}</span>
            </div>
            <div className="uav-meta">
              <span>
                ({uav.position.x}, {uav.position.y})
              </span>
              <span>battery {formatPct(uav.battery)}</span>
            </div>
            <ActiveCommandLine command={activeCommands.find((item) => item.uav_id === uav.id)} />
            <IdleAssistLine state={state} uavId={uav.id} status={uav.status} idleReason={uav.idle_reason} />
            <div className="mini-actions">
              <button
                onClick={(event) => {
                  event.stopPropagation();
                  onSetOnline(uav.id, false);
                }}
                disabled={busy || uav.status === "OFFLINE"}
              >
                <Power size={14} /> Offline
              </button>
              <button
                onClick={(event) => {
                  event.stopPropagation();
                  onSetOnline(uav.id, true);
                }}
                disabled={busy || uav.status !== "OFFLINE"}
              >
                <RotateCw size={14} /> Recover
              </button>
            </div>
          </article>
        ))}
        {uavs.length === 0 && <span className="empty">No UAV state yet.</span>}
      </div>
    </section>
  );
}

function IdleAssistLine({
  state,
  uavId,
  status,
  idleReason,
}: {
  state?: SimulationState;
  uavId: string;
  status: string;
  idleReason?: string | null;
}) {
  const scheduler = nestedRecord(state?.diagnostics || {}, ["scheduler"]);
  const reasons = nestedRecord(scheduler, ["idle_reason_per_uav"]);
  const reason = status === "IDLE" ? idleReason || stringValue(reasons[uavId]) : undefined;
  const assistTasks = (Array.isArray((state?.tasks as Record<string, unknown> | undefined)?.assist_tasks)
    ? ((state?.tasks as Record<string, unknown>).assist_tasks as unknown[])
    : []
  ).filter(
    (item) =>
      item &&
      typeof item === "object" &&
      ((item as Record<string, unknown>).helper_uav_id === uavId || (item as Record<string, unknown>).donor_uav_id === uavId),
  );
  if (!reason && assistTasks.length === 0) {
    return null;
  }
  return (
    <small className="uav-command">
      {reason ? `Idle: ${formatIdleReason(reason)}` : ""}
      {assistTasks.length > 0 ? ` assist links ${assistTasks.length}` : ""}
    </small>
  );
}

function ActiveCommandLine({ command }: { command?: ActiveCommandSnapshot }) {
  if (!command) {
    return <small className="uav-command">command -</small>;
  }
  return (
    <small className="uav-command">
      command <span className="mono">{command.command_id}</span> / {command.command} / progress {formatPct(command.progress ?? 0)}
    </small>
  );
}

function nestedRecord(payload: Record<string, unknown>, path: string[]): Record<string, unknown> {
  let current: unknown = payload;
  for (const key of path) {
    if (!current || typeof current !== "object" || Array.isArray(current)) {
      return {};
    }
    current = (current as Record<string, unknown>)[key];
  }
  return current && typeof current === "object" && !Array.isArray(current) ? (current as Record<string, unknown>) : {};
}

function stringValue(value: unknown): string | undefined {
  return typeof value === "string" ? value : undefined;
}

function formatIdleReason(reason: string): string {
  return reason.replaceAll("_", " ");
}

function formatPct(value?: number): string {
  if (typeof value !== "number") {
    return "-";
  }
  return `${Math.round(value * 100)}%`;
}
