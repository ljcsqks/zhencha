import type { CommandLogEntry } from "../hooks/simulationState";
import type { SimulationState } from "../types/sim";

export type AcceptanceStatus = "PASS" | "WARN" | "FAIL";

export interface AcceptanceCheck {
  id: string;
  label: string;
  status: AcceptanceStatus;
  detail: string;
}

export function evaluateAcceptance(state: SimulationState | undefined, commandLog: CommandLogEntry[]): AcceptanceCheck[] {
  const metrics = state?.metrics || {};
  const diagnostics = isRecord(metrics.diagnostics) ? metrics.diagnostics : {};
  const routeQuality = isRecord(diagnostics.route_quality) ? diagnostics.route_quality : {};
  const allocationQuality = isRecord(diagnostics.allocation_quality) ? diagnostics.allocation_quality : {};
  const targets = state?.targets || {};
  const hasTargets = Object.keys(targets).length > 0 || Number(metrics.target_found_count || 0) > 0;
  const rejectedCount = commandLog.filter((entry) => ["rejected", "failed"].includes(String(entry.ack_status || ""))).length;
  const confirmFailedCount = Object.values(targets).filter((target) => isRecord(target) && target.success === false).length;
  const workloadBalance = Number(allocationQuality.workload_balance ?? metrics.per_uav_workload_balance ?? 0);
  const redundantCoverage = Number(metrics.redundant_coverage_rate ?? 0);
  const post95Distance = Number(metrics.post_95_extra_distance_m ?? 0);
  const maxConnectorLength = Number(routeQuality.max_connector_length ?? 0);
  const idleRatio = idleTimeRatio(diagnostics);
  const stuckCount = (state?.active_commands || []).filter((command) => {
    const progress = typeof command.progress === "number" ? command.progress : null;
    return progress !== null && progress <= 0 && (command.remaining_path || []).length === 0;
  }).length;

  return [
    {
      id: "coverage",
      label: "Global coverage",
      status: boolMetric(metrics.coverage_goal_met, (state?.global_coverage || 0) >= 0.95),
      detail: `${formatPercent(state?.global_coverage)} / threshold`,
    },
    {
      id: "priority",
      label: "Priority coverage",
      status: boolMetric(metrics.priority_goal_met, (state?.priority_coverage || 0) >= 0.98),
      detail: `${formatPercent(state?.priority_coverage)} / priority goal`,
    },
    {
      id: "no_fly",
      label: "No-fly violations",
      status: Number(metrics.no_fly_violations || 0) === 0 ? "PASS" : "FAIL",
      detail: `${Number(metrics.no_fly_violations || 0)} violations`,
    },
    {
      id: "target_confirm",
      label: "Target confirmation",
      status: !hasTargets ? "WARN" : Number(metrics.confirm_success_rate || 0) >= 1 && confirmFailedCount === 0 ? "PASS" : "FAIL",
      detail: hasTargets ? `success rate ${formatPercent(Number(metrics.confirm_success_rate || 0))}` : "no target confirmation in this run",
    },
    {
      id: "resume",
      label: "Interrupted task resume",
      status: !hasTargets ? "WARN" : Number(metrics.interrupted_task_resume_rate ?? 1) >= 1 ? "PASS" : "FAIL",
      detail: hasTargets ? `resume rate ${formatPercent(Number(metrics.interrupted_task_resume_rate ?? 1))}` : "not applicable",
    },
    {
      id: "command_rejected",
      label: "Rejected/failed commands",
      status: rejectedCount === 0 ? "PASS" : "WARN",
      detail: `${rejectedCount} rejected or failed ack records`,
    },
    {
      id: "confirm_failed",
      label: "Confirm failed count",
      status: confirmFailedCount === 0 ? "PASS" : "FAIL",
      detail: `${confirmFailedCount} failed target records`,
    },
    {
      id: "active_stuck",
      label: "Stuck active commands",
      status: stuckCount === 0 ? "PASS" : "WARN",
      detail: `${stuckCount} active commands with no progress hint`,
    },
    {
      id: "workload_balance",
      label: "Workload balance",
      status: workloadBalance >= 0.92 ? "PASS" : "WARN",
      detail: `${workloadBalance.toFixed(3)} target >= 0.920`,
    },
    {
      id: "post_95_distance",
      label: "Post-95 extra distance",
      status: post95Distance <= 1000 ? "PASS" : "WARN",
      detail: `${post95Distance.toFixed(1)} m after coverage goal`,
    },
    {
      id: "redundancy",
      label: "Redundant coverage",
      status: redundantCoverage <= 0.35 ? "PASS" : "WARN",
      detail: `${formatPercent(redundantCoverage)} target <= 35.0%`,
    },
    {
      id: "max_connector",
      label: "Max connector length",
      status: maxConnectorLength <= 10 ? "PASS" : "WARN",
      detail: `${maxConnectorLength.toFixed(1)} cells`,
    },
    {
      id: "idle_ratio",
      label: "Idle time ratio",
      status: idleRatio <= 0.2 ? "PASS" : "WARN",
      detail: `${formatPercent(idleRatio)} idle / active+idle`,
    },
  ];
}

function boolMetric(value: unknown, fallback: boolean): AcceptanceStatus {
  if (typeof value === "boolean") {
    return value ? "PASS" : "FAIL";
  }
  return fallback ? "PASS" : "FAIL";
}

function formatPercent(value: unknown): string {
  return typeof value === "number" ? `${(value * 100).toFixed(1)}%` : "-";
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null;
}

function idleTimeRatio(diagnostics: Record<string, unknown>): number {
  const perUav = diagnostics.per_uav;
  if (!isRecord(perUav)) {
    return 0;
  }
  let idle = 0;
  let active = 0;
  for (const value of Object.values(perUav)) {
    if (!isRecord(value)) {
      continue;
    }
    idle += Number(value.idle_time_s || 0);
    active += Number(value.active_time_s || 0);
  }
  const total = idle + active;
  return total > 0 ? idle / total : 0;
}
