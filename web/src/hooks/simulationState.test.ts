import { describe, expect, it } from "vitest";
import { mergeSimulationState, shouldStepAfterEvent, type SimulationClientState } from "./simulationState";
import type { SimulationState } from "../types/sim";

const baseState: SimulationState = {
  time_s: 0,
  tick: 0,
  running: false,
  run_id: "run_a",
  scenario_name: "area_search_1uav",
  global_coverage: 0,
  priority_coverage: 0,
  uavs: [
    {
      id: "uav_01",
      position: { x: 1, y: 2 },
      status: "IDLE",
      battery: 1,
      task_id: null,
      total_distance_m: 0,
      effective_search_distance_m: 0,
    },
  ],
  commands: [],
  command_acks: [],
  events: [],
  pending_events: [],
  recent_events: [],
  event_log: [],
  advisory_summary: {},
  tasks: {},
  targets: {},
  changed_cells: [],
  metrics: {},
  map: {
    width_cells: 4,
    height_cells: 4,
    resolution_m: 10,
    terrain: [["FREE"]],
    passable: [[true]],
    coverage_count: [[0]],
    search_confidence: [[0]],
    search_priority: [[1]],
  },
};

describe("mergeSimulationState", () => {
  it("stores full map state and resets logs when run_id changes", () => {
    const previous: SimulationClientState = {
      currentState: baseState,
      fullMapState: baseState,
      runId: "old_run",
      commandLog: [{ time_s: 0, command_id: "old", command: "FOLLOW_PATH", uav_id: "uav_01" }],
      commandById: {},
      ackByCommandId: {},
      commandOrder: ["old"],
      eventLog: [{ event_id: "old_event", type: "TARGET_FOUND", status: "handled" }],
      eventById: {},
      eventOrder: ["old_event"],
      uavTrajectories: { uav_01: [{ x: 0, y: 0 }] },
      needsFullMapRefresh: false,
    };

    const next = mergeSimulationState(previous, baseState);

    expect(next.runId).toBe("run_a");
    expect(next.fullMapState).toBe(baseState);
    expect(next.commandLog).toHaveLength(0);
    expect(next.eventLog).toHaveLength(0);
    expect(next.uavTrajectories.uav_01).toEqual([{ x: 1, y: 2 }]);
  });

  it("preserves the full map for lite states and requests refresh after map changes", () => {
    const previous = mergeSimulationState(undefined, baseState);
    const liteState: SimulationState = {
      ...baseState,
      tick: 1,
      time_s: 1,
      map: undefined,
      changed_cells: [{ x: 2, y: 2 }],
      uavs: [{ ...baseState.uavs[0], position: { x: 2, y: 3 }, status: "SEARCHING" }],
    };

    const next = mergeSimulationState(previous, liteState);

    expect(next.fullMapState).toBe(baseState);
    expect(next.needsFullMapRefresh).toBe(true);
    expect(next.uavTrajectories.uav_01).toEqual([
      { x: 1, y: 2 },
      { x: 2, y: 3 },
    ]);
  });

  it("joins command acknowledgements and caps command log length", () => {
    const previous = mergeSimulationState(undefined, baseState);
    const manyCommands = Array.from({ length: 305 }, (_, index) => ({
      time_s: index,
      command_id: `cmd_${index}`,
      command: "FOLLOW_PATH",
      uav_id: "uav_01",
      task_id: null,
      executable: true,
      advisory: false,
      path: [],
      reason: null,
    }));
    const next = mergeSimulationState(previous, {
      ...baseState,
      tick: 2,
      time_s: 2,
      map: undefined,
      commands: manyCommands,
      command_acks: [{ command_id: "cmd_304", uav_id: "uav_01", status: "accepted", reason: "ok" }],
    });

    expect(next.commandLog).toHaveLength(300);
    expect(Object.keys(next.commandById)).toHaveLength(300);
    expect(next.commandOrder).toHaveLength(300);
    expect(next.commandLog.at(-1)).toMatchObject({
      command_id: "cmd_304",
      ack_status: "accepted",
      reason: "ok",
    });
  });

  it("applies coverage patches to the retained full map state", () => {
    const previous = mergeSimulationState(undefined, baseState);

    const next = mergeSimulationState(previous, {
      ...baseState,
      tick: 3,
      time_s: 3,
      map: undefined,
      coverage_changed_cells: [{ x: 0, y: 0, coverage_count: 4, search_confidence: 1 }],
    });

    expect(next.fullMapState?.map?.coverage_count[0][0]).toBe(4);
    expect(next.fullMapState?.map?.search_confidence[0][0]).toBe(1);
  });

  it("updates historical commands when acknowledgements arrive in later ticks", () => {
    const withCommand = mergeSimulationState(undefined, {
      ...baseState,
      commands: [
        {
          command_id: "cmd_late",
          command: "FOLLOW_PATH",
          uav_id: "uav_01",
          task_id: "task_001",
          executable: true,
          advisory: false,
          path: [],
          reason: "assigned",
          issued_at: 1,
        },
      ],
    });

    const withLateAck = mergeSimulationState(withCommand, {
      ...baseState,
      tick: 4,
      time_s: 4,
      map: undefined,
      commands: [],
      command_acks: [
        {
          command_id: "cmd_late",
          uav_id: "uav_01",
          status: "completed",
          progress: 1,
          reason: "path_completed",
          updated_at: 4,
        },
      ],
    });

    expect(withLateAck.commandLog.find((entry) => entry.command_id === "cmd_late")).toMatchObject({
      ack_status: "completed",
      progress: 1,
      reason: "path_completed",
      updated_at: 4,
    });
  });

  it("renders ack-only command records when the original command was not seen", () => {
    const next = mergeSimulationState(undefined, {
      ...baseState,
      command_acks: [{ command_id: "ack_only", uav_id: "uav_01", status: "failed", reason: "uav_offline" }],
    });

    expect(next.commandLog).toEqual([
      expect.objectContaining({
        command_id: "ack_only",
        command: "ACK_ONLY",
        ack_status: "failed",
      }),
    ]);
  });

  it("deduplicates events and upgrades queued events to handled", () => {
    const queued = mergeSimulationState(undefined, {
      ...baseState,
      event_log: [{ event_id: "event_1", type: "TARGET_FOUND", status: "queued", queued_at_s: 1 }],
    });
    const handled = mergeSimulationState(queued, {
      ...baseState,
      tick: 5,
      event_log: [
        { event_id: "event_1", type: "TARGET_FOUND", status: "queued", queued_at_s: 1 },
        { event_id: "event_1", type: "TARGET_FOUND", status: "handled", queued_at_s: 1, handled_at_s: 2 },
      ],
    });

    expect(handled.eventLog).toHaveLength(1);
    expect(handled.eventLog[0]).toMatchObject({ event_id: "event_1", status: "handled", handled_at_s: 2 });
  });

  it("does not request a manual step when injecting events while running", () => {
    expect(shouldStepAfterEvent({ ...baseState, running: true })).toBe(false);
    expect(shouldStepAfterEvent({ ...baseState, running: false })).toBe(true);
  });
});
