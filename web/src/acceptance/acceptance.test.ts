import { describe, expect, it } from "vitest";
import { evaluateAcceptance } from "./acceptance";
import type { SimulationState } from "../types/sim";

const baseState: SimulationState = {
  time_s: 10,
  tick: 10,
  running: false,
  run_id: "run_1",
  scenario_name: "demo_target_confirm",
  global_coverage: 0.96,
  priority_coverage: 1,
  uavs: [],
  commands: [],
  command_acks: [],
  events: [],
  pending_events: [],
  recent_events: [],
  event_log: [],
  advisory_summary: {},
  tasks: {},
  targets: { target_1: { success: true, interrupted_task_id: "task_1", resumed_time_s: 9 } },
  changed_cells: [],
  metrics: {
    coverage_goal_met: true,
    priority_goal_met: true,
    no_fly_violations: 0,
    confirm_success_rate: 1,
    interrupted_task_resume_rate: 1,
    redundant_coverage_rate: 0.2,
    post_95_extra_distance_m: 50,
    diagnostics: {
      route_quality: { max_connector_length: 3 },
      allocation_quality: { workload_balance: 0.95 },
      per_uav: {
        uav_01: { idle_time_s: 5, active_time_s: 95 },
      },
    },
  },
};

describe("evaluateAcceptance", () => {
  it("marks core mission checks as pass when thresholds are satisfied", () => {
    const checks = evaluateAcceptance(baseState, []);

    expect(checks.find((check) => check.id === "coverage")?.status).toBe("PASS");
    expect(checks.find((check) => check.id === "no_fly")?.status).toBe("PASS");
    expect(checks.find((check) => check.id === "target_confirm")?.status).toBe("PASS");
    expect(checks.find((check) => check.id === "workload_balance")?.status).toBe("PASS");
    expect(checks.find((check) => check.id === "post_95_distance")?.detail).toContain("50.0");
    expect(checks.find((check) => check.id === "redundancy")?.status).toBe("PASS");
    expect(checks.find((check) => check.id === "max_connector")?.detail).toContain("3.0");
    expect(checks.find((check) => check.id === "idle_ratio")?.detail).toContain("5.0%");
  });

  it("warns or fails for rejected commands, failed confirmations, and stuck commands", () => {
    const checks = evaluateAcceptance(
      {
        ...baseState,
        metrics: {
          ...baseState.metrics,
          no_fly_violations: 2,
          confirm_success_rate: 0,
          redundant_coverage_rate: 0.55,
          diagnostics: {
            route_quality: { max_connector_length: 18 },
            allocation_quality: { workload_balance: 0.75 },
            per_uav: { uav_01: { idle_time_s: 50, active_time_s: 50 } },
          },
        },
        targets: { target_1: { success: false } },
        active_commands: [{ uav_id: "uav_01", command_id: "cmd_stuck", command: "FOLLOW_PATH", path: [], remaining_path: [], progress: 0 }],
      },
      [{ command_id: "cmd_bad", command: "FOLLOW_PATH", uav_id: "uav_01", ack_status: "rejected" }],
    );

    expect(checks.find((check) => check.id === "no_fly")?.status).toBe("FAIL");
    expect(checks.find((check) => check.id === "target_confirm")?.status).toBe("FAIL");
    expect(checks.find((check) => check.id === "command_rejected")?.status).toBe("WARN");
    expect(checks.find((check) => check.id === "active_stuck")?.status).toBe("WARN");
    expect(checks.find((check) => check.id === "workload_balance")?.status).toBe("WARN");
    expect(checks.find((check) => check.id === "redundancy")?.status).toBe("WARN");
  });
});
