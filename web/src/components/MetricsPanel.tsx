import { BarChart3 } from "lucide-react";
import type { SimulationState } from "../types/sim";

interface Props {
  state?: SimulationState;
  fullMetrics?: Record<string, unknown>;
  onFetchMetrics(): void;
}

const fields = [
  "algorithm_version",
  "global_coverage",
  "priority_coverage",
  "total_distance_m",
  "redundant_coverage_rate",
  "time_to_95_coverage_s",
  "post_95_extra_distance_m",
  "per_uav_workload_balance",
  "confirm_success_rate",
  "target_response_time_s",
  "target_confirm_duration_s",
  "no_fly_violations",
];

export function MetricsPanel({ state, fullMetrics, onFetchMetrics }: Props) {
  const metrics = { ...(state?.metrics || {}), ...(fullMetrics || {}) };
  const algorithmVersion = String(state?.algorithm_version || metrics.algorithm_version || "-");
  const adaptiveDiagnostics = nestedRecord(metrics, ["diagnostics", "segment_quality"]);
  const schedulerDiagnostics = {
    ...nestedRecord(state?.diagnostics || {}, ["scheduler"]),
    ...nestedRecord(metrics, ["diagnostics", "scheduler_quality"]),
  };
  return (
    <section className="panel">
      <div className="panel-heading">
        <h2>Metrics</h2>
        <button onClick={onFetchMetrics}>
          <BarChart3 size={14} /> Fetch metrics
        </button>
      </div>
      <dl className="metric-grid">
        {fields.map((field) => (
          <div key={field}>
            <dt>{field}</dt>
            <dd>{formatMetric(metrics[field] ?? state?.[field as keyof SimulationState])}</dd>
          </div>
        ))}
      </dl>
      {algorithmVersion === "adaptive_component_sweep_v1" && (
        <div className="diagnostic-strip">
          <strong>Adaptive diagnostics</strong>
          <span>clusters {formatMetric(adaptiveDiagnostics.cluster_count_total)}</span>
          <span>frontload {formatMetric(adaptiveDiagnostics.simple_frontload_enabled)}</span>
          <span>planned {formatMetric(adaptiveDiagnostics.fleet_planned_coverage_ratio)}</span>
          <span>gap {formatMetric(adaptiveDiagnostics.planned_actual_gap_abs)}</span>
          <span>clustered launch {formatMetric(adaptiveDiagnostics.clustered_launch_detected)}</span>
          <span>sector {formatMetric(adaptiveDiagnostics.clustered_sector_orientation)}</span>
          <span>{formatMetric(adaptiveDiagnostics.planned_vs_actual_explanation)}</span>
        </div>
      )}
      <div className="diagnostic-strip">
        <strong>Idle assist</strong>
        <span>created {formatMetric(schedulerDiagnostics.idle_assist_created_tasks)}</span>
        <span>accepted {formatMetric(schedulerDiagnostics.idle_assist_accepted_tasks)}</span>
        <span>donor replans {formatMetric(schedulerDiagnostics.idle_assist_donor_replans)}</span>
        <span>reassigned cells {formatMetric(schedulerDiagnostics.idle_assist_cells_reassigned)}</span>
        <span>waiting {formatMetric(schedulerDiagnostics.idle_uav_wait_time_s)} s</span>
        <span>dynamic repairs {formatMetric(schedulerDiagnostics.dynamic_route_repair_success)}</span>
      </div>
    </section>
  );
}

function formatMetric(value: unknown): string {
  if (typeof value === "number") {
    if (Math.abs(value) <= 1) {
      return value.toFixed(3);
    }
    return value.toFixed(1);
  }
  if (typeof value === "boolean") {
    return value ? "true" : "false";
  }
  return value == null ? "-" : String(value);
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
