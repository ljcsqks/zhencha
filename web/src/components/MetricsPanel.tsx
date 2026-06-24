import { BarChart3 } from "lucide-react";
import type { SimulationState } from "../types/sim";

interface Props {
  state?: SimulationState;
  fullMetrics?: Record<string, unknown>;
  onFetchMetrics(): void;
}

const fields = [
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
