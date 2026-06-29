import { Activity, CheckCircle2, CircleAlert, Gauge, ShieldCheck } from "lucide-react";
import type { ReactNode } from "react";
import type { CommandLogEntry } from "../hooks/simulationState";
import type { SimulationState } from "../types/sim";

interface Props {
  state?: SimulationState;
  commandLog: CommandLogEntry[];
}

export function MissionStatusPanel({ state, commandLog }: Props) {
  const started = Boolean(state && (state.tick > 0 || state.running || state.global_coverage > 0 || commandLog.length > 0));
  const statusCounts = countUavStatus(state);
  const scheduler = nestedRecord(state?.diagnostics || {}, ["scheduler"]);
  const rejectedCount = commandLog.filter((entry) => ["rejected", "failed"].includes(String(entry.ack_status || ""))).length;
  const noFlyViolations = numberMetric(state?.metrics?.no_fly_violations);
  const confirmRate = numberMetric(state?.metrics?.confirm_success_rate);
  const targetCount = Object.keys(state?.targets || {}).length + numberMetric(state?.metrics?.target_found_count);
  const assistCreated = numberMetric(scheduler.idle_assist_created_tasks);
  const assistAccepted = numberMetric(scheduler.idle_assist_accepted_tasks);

  return (
    <section className="panel mission-status-panel">
      <div className="panel-heading">
        <h2>Mission Status</h2>
        <span className={`status-pill ${started ? (state?.running ? "ok" : "idle") : "neutral"}`}>
          {started ? (state?.running ? "Running" : "Paused") : "Not started"}
        </span>
      </div>

      {!started && (
        <div className="mission-waiting">
          <Activity size={18} />
          <div>
            <strong>Waiting for mission start</strong>
            <span>Configure the mission, then press Start or Step.</span>
          </div>
        </div>
      )}

      <div className="status-meter-grid">
        <StatusMeter label="Global coverage" value={state?.global_coverage} />
        <StatusMeter label="Priority coverage" value={state?.priority_coverage} />
      </div>

      <dl className="mission-status-list">
        <StatusRow label="Time" value={`${formatNumber(state?.time_s, 1)} s`} icon={<Gauge size={15} />} />
        <StatusRow label="UAVs" value={`${state?.uavs.length ?? 0} total`} />
        <StatusRow label="Active / Idle" value={`${statusCounts.active} / ${statusCounts.idle}`} />
        <StatusRow label="Returning / Offline" value={`${statusCounts.returning} / ${statusCounts.offline}`} />
        <StatusRow label="Total distance" value={`${formatNumber(totalDistance(state), 1)} m`} />
        <StatusRow
          label="No-fly status"
          value={noFlyViolations === 0 ? "Clear" : `${noFlyViolations} violations`}
          icon={noFlyViolations === 0 ? <ShieldCheck size={15} /> : <CircleAlert size={15} />}
          tone={noFlyViolations === 0 ? "ok" : "bad"}
        />
        <StatusRow label="Command failures" value={String(rejectedCount)} tone={rejectedCount === 0 ? "ok" : "warn"} />
        <StatusRow
          label="Target confirmation"
          value={targetCount > 0 ? `${formatPercent(confirmRate)} success` : "No target"}
          icon={targetCount > 0 && confirmRate >= 1 ? <CheckCircle2 size={15} /> : undefined}
        />
        <StatusRow label="Algorithm" value={state?.algorithm_version || "-"} mono />
        {(assistCreated > 0 || assistAccepted > 0) && (
          <StatusRow label="Idle assist" value={`${assistAccepted}/${assistCreated} accepted`} tone="ok" />
        )}
      </dl>
    </section>
  );
}

function StatusMeter({ label, value }: { label: string; value?: number }) {
  const pct = Math.max(0, Math.min(1, value ?? 0));
  return (
    <div className="status-meter">
      <div>
        <span>{label}</span>
        <strong>{formatPercent(pct)}</strong>
      </div>
      <i>
        <b style={{ width: `${pct * 100}%` }} />
      </i>
    </div>
  );
}

function StatusRow({
  label,
  value,
  icon,
  tone,
  mono,
}: {
  label: string;
  value: string;
  icon?: ReactNode;
  tone?: "ok" | "warn" | "bad";
  mono?: boolean;
}) {
  return (
    <div className={tone ? `mission-status-row ${tone}` : "mission-status-row"}>
      <dt>{icon}{label}</dt>
      <dd className={mono ? "mono compact" : undefined}>{value}</dd>
    </div>
  );
}

function countUavStatus(state?: SimulationState) {
  const counts = { active: 0, idle: 0, returning: 0, offline: 0 };
  for (const uav of state?.uavs || []) {
    if (uav.status === "OFFLINE") counts.offline += 1;
    else if (uav.status === "RETURNING") counts.returning += 1;
    else if (uav.status === "IDLE") counts.idle += 1;
    else counts.active += 1;
  }
  return counts;
}

function totalDistance(state?: SimulationState): number {
  return (state?.uavs || []).reduce((sum, uav) => sum + (uav.total_distance_m || 0), 0);
}

function formatPercent(value?: number): string {
  return `${((value || 0) * 100).toFixed(1)}%`;
}

function formatNumber(value?: number, digits = 0): string {
  return typeof value === "number" ? value.toFixed(digits) : "-";
}

function numberMetric(value: unknown): number {
  return typeof value === "number" && Number.isFinite(value) ? value : 0;
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
