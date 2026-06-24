import { Power, RotateCw } from "lucide-react";
import type { ActiveCommandSnapshot, SimulationState } from "../types/sim";

interface Props {
  state?: SimulationState;
  activeCommands: ActiveCommandSnapshot[];
  selectedUavId?: string;
  onSelectUav(uavId: string | undefined): void;
  onSetOnline(uavId: string, online: boolean): Promise<void>;
}

export function UavPanel({ state, activeCommands, selectedUavId, onSelectUav, onSetOnline }: Props) {
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
            <div className="mini-actions">
              <button
                onClick={(event) => {
                  event.stopPropagation();
                  onSetOnline(uav.id, false);
                }}
                disabled={uav.status === "OFFLINE"}
              >
                <Power size={14} /> Offline
              </button>
              <button
                onClick={(event) => {
                  event.stopPropagation();
                  onSetOnline(uav.id, true);
                }}
                disabled={uav.status !== "OFFLINE"}
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

function formatPct(value?: number): string {
  if (typeof value !== "number") {
    return "-";
  }
  return `${Math.round(value * 100)}%`;
}
