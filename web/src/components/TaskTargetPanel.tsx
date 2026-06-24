import type { SimulationState } from "../types/sim";

interface Props {
  state?: SimulationState;
}

export function TaskTargetPanel({ state }: Props) {
  const taskCounts = extractTaskCounts(state?.tasks);
  const targets = Object.entries(state?.targets || {});

  return (
    <section className="panel">
      <h2>Tasks / Targets</h2>
      <div className="task-counts">
        {["pending", "assigned", "in_progress", "completed", "failed", "blocked"].map((key) => (
          <div key={key}>
            <span>{key}</span>
            <strong>{taskCounts[key] ?? 0}</strong>
          </div>
        ))}
      </div>

      <div className="target-list">
        {targets.map(([targetId, raw]) => {
          const target = raw as Record<string, unknown>;
          return (
            <article key={targetId} className="target-row">
              <strong>{targetId}</strong>
              <span>{String(target.status ?? (target.success === true ? "confirmed" : target.success === false ? "failed" : "pending"))}</span>
              <small>uav {String(target.assigned_uav_id ?? target.uav_id ?? "-")}</small>
              <small>
                found {fmt(target.found_time_s ?? target.found_time)} / done {fmt(target.confirm_done_time_s ?? target.confirm_done_time)}
              </small>
              <small>
                response {fmt(target.response_time_s ?? target.target_response_time_s)} / duration{" "}
                {fmt(target.confirm_duration_s ?? target.target_confirm_duration_s)}
              </small>
            </article>
          );
        })}
        {targets.length === 0 && <span className="empty">No targets yet.</span>}
      </div>
    </section>
  );
}

function extractTaskCounts(tasks?: Record<string, unknown>): Record<string, number> {
  const statusCounts = tasks?.status_counts as Record<string, number> | undefined;
  if (statusCounts) {
    return statusCounts;
  }
  return {};
}

function fmt(value: unknown): string {
  return typeof value === "number" ? value.toFixed(1) : value == null ? "-" : String(value);
}
