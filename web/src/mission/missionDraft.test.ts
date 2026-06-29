import { describe, expect, it } from "vitest";
import { addDraftUav, canEditMissionDraft, createMissionDraftFromState, moveDraftUav, removeDraftUav, updateDraftUav } from "./missionDraft";
import type { SimulationState } from "../types/sim";

const state: SimulationState = {
  time_s: 0,
  tick: 0,
  running: false,
  run_id: "run_1",
  scenario_name: "draft_seed",
  global_coverage: 0,
  priority_coverage: 0,
  uavs: [
    {
      id: "uav_01",
      position: { x: 2, y: 3 },
      status: "IDLE",
      battery: 1,
      sensor_radius_cells: 2,
      speed_mps: 10,
      home_position: { x: 2, y: 3 },
    },
    {
      id: "uav_02",
      position: { x: 4, y: 5 },
      status: "IDLE",
      battery: 0.9,
      sensor_radius_cells: 3,
      speed_mps: 8,
      home_position: { x: 4, y: 5 },
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
    width_cells: 50,
    height_cells: 40,
    resolution_m: 10,
    terrain: [],
    passable: [],
    coverage_count: [],
    search_confidence: [],
    search_priority: [],
  },
};

describe("mission draft helpers", () => {
  it("creates a mission draft from the current full state", () => {
    const draft = createMissionDraftFromState(state);

    expect(draft.draftMapConfig).toEqual({ width_cells: 50, height_cells: 40, resolution_m: 10 });
    expect(draft.draftSearchRegion).toEqual({ x: 0, y: 0, width: 50, height: 40 });
    expect(draft.draftUavs).toEqual([
      expect.objectContaining({ id: "uav_01", initial_position: { x: 2, y: 3 }, sensor_radius_cells: 2, speed_mps: 10 }),
      expect.objectContaining({ id: "uav_02", initial_position: { x: 4, y: 5 }, sensor_radius_cells: 3, speed_mps: 8 }),
    ]);
  });

  it("adds, moves, updates, and removes draft UAVs immutably", () => {
    const draft = createMissionDraftFromState(state);
    const withAdded = addDraftUav(draft, { x: 9, y: 10 });

    expect(withAdded.draftUavs.at(-1)).toEqual({
      id: "uav_03",
      home_position: { x: 9, y: 10 },
      initial_position: { x: 9, y: 10 },
      sensor_radius_cells: 2,
      speed_mps: 10,
      battery: 1,
    });
    expect(draft.draftUavs).toHaveLength(2);

    const moved = moveDraftUav(withAdded, "uav_03", { x: 11, y: 12 });
    expect(moved.draftUavs.at(-1)?.initial_position).toEqual({ x: 11, y: 12 });
    expect(moved.draftUavs.at(-1)?.home_position).toEqual({ x: 11, y: 12 });

    const updated = updateDraftUav(moved, "uav_03", { battery: 0.55, speed_mps: 7.5, sensor_radius_cells: 5 });
    expect(updated.draftUavs.at(-1)).toMatchObject({ battery: 0.55, speed_mps: 7.5, sensor_radius_cells: 5 });

    const removed = removeDraftUav(updated, "uav_03");
    expect(removed.draftUavs.map((uav) => uav.id)).toEqual(["uav_01", "uav_02"]);
  });

  it("only allows draft editing before the mission starts", () => {
    expect(canEditMissionDraft({ ...state, running: false })).toBe(true);
    expect(canEditMissionDraft({ ...state, running: true })).toBe(false);
    expect(canEditMissionDraft({ ...state, running: false, tick: 1 })).toBe(false);
  });
});
