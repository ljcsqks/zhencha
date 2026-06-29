import type { DraftUav, GridPosition, MissionDraft, SimulationState } from "../types/sim";

export function createMissionDraftFromState(state?: SimulationState): MissionDraft {
  const map = state?.map;
  const width = map?.width_cells || 50;
  const height = map?.height_cells || 50;
  const resolution = map?.resolution_m || 10;
  return {
    draftUavs: (state?.uavs || []).map((uav) => ({
      id: uav.id,
      home_position: uav.home_position || uav.position,
      initial_position: uav.position,
      sensor_radius_cells: uav.sensor_radius_cells ?? 2,
      speed_mps: uav.speed_mps ?? 10,
      battery: uav.battery ?? 1,
    })),
    draftObstacles: [],
    draftSearchRegion: { x: 0, y: 0, width, height },
    draftPriorityRegions: [],
    draftMapConfig: { width_cells: width, height_cells: height, resolution_m: resolution },
  };
}

export function canEditMissionDraft(state?: Pick<SimulationState, "running">): boolean {
  return !state?.running;
}

export function addDraftUav(draft: MissionDraft, position: GridPosition): MissionDraft {
  const nextId = nextUavId(draft.draftUavs);
  return {
    ...draft,
    draftUavs: [
      ...draft.draftUavs,
      {
        id: nextId,
        home_position: { ...position },
        initial_position: { ...position },
        sensor_radius_cells: 2,
        speed_mps: 10,
        battery: 1,
      },
    ],
  };
}

export function moveDraftUav(draft: MissionDraft, uavId: string, position: GridPosition): MissionDraft {
  return {
    ...draft,
    draftUavs: draft.draftUavs.map((uav) =>
      uav.id === uavId
        ? { ...uav, home_position: { ...position }, initial_position: { ...position } }
        : uav,
    ),
  };
}

export function updateDraftUav(draft: MissionDraft, uavId: string, patch: Partial<Omit<DraftUav, "id">>): MissionDraft {
  return {
    ...draft,
    draftUavs: draft.draftUavs.map((uav) => (uav.id === uavId ? { ...uav, ...patch } : uav)),
  };
}

export function removeDraftUav(draft: MissionDraft, uavId: string): MissionDraft {
  return {
    ...draft,
    draftUavs: draft.draftUavs.filter((uav) => uav.id !== uavId),
  };
}

function nextUavId(uavs: DraftUav[]): string {
  const used = new Set(uavs.map((uav) => uav.id));
  let index = 1;
  while (used.has(`uav_${String(index).padStart(2, "0")}`)) {
    index += 1;
  }
  return `uav_${String(index).padStart(2, "0")}`;
}
